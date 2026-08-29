//
// Wi-Fi side of the network wake path.
//
// Joins a WPA2 network and listens on UDP for a trigger packet. On a match it
// calls wake_request_from_network(), which drives the keyboard FSM in wake.cpp.
//
// Bluetooth and Wi-Fi share one CYW43439 radio, and the Bluetooth side of this
// firmware carries a latency-sensitive audio stream. Rather than have the two
// contend, only one is up at a time and the controller wins:
//
//   controller connected  -> Wi-Fi down. The host is awake and being played on,
//                            so nothing needs to wake it, and the audio path
//                            gets the radio to itself.
//   controller idle       -> Wi-Fi up, listening for a trigger. This is exactly
//                            when the host is likely asleep.
//
// The other constraint is the 1 s watchdog on the main loop. Bringing the
// station interface up or down and joining a network all walk a series of
// ioctls to the chip and wait for each, which can overrun a second, so the
// watchdog is widened for the duration of those calls and restored after.
//

#include "wifi_wake.h"

#ifdef ENABLE_WIFI_WAKE

#include <cstdio>
#include <cstring>

#include "pico/cyw43_arch.h"
#include "hardware/watchdog.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/netif.h"

#include "bt.h"
#include "wake.h"
#include "wifi_config.h"
#include "config.h"
#include "tusb.h"

#ifndef WIFI_WAKE_UDP_PORT
#define WIFI_WAKE_UDP_PORT 9
#endif

#ifndef WIFI_WAKE_MAGIC
#define WIFI_WAKE_MAGIC "WAKE_G14"
#endif

#ifndef WIFI_WAKE_HOSTNAME
#define WIFI_WAKE_HOSTNAME "ds5dongle"
#endif

// Longest datagram worth inspecting. The trigger is a short string.
#define RX_BUF_SIZE 256

#define POLL_INTERVAL_MS    500
// Kept short: the Bluetooth side is quietened for the duration of an attempt,
// so a controller is slower to attach while one is running.
#define JOIN_TIMEOUT_MS     8000
#define RETRY_DELAY_MS      10000
#define IGNORED_REPORT_INTERVAL_MS 5000

// Let USB enumerate and the initial Bluetooth inquiry settle before touching
// the radio for the first time.
#define BOOT_SETTLE_MS      15000
// After the controller attaches, bt.cpp calls tud_connect() and the host starts
// enumerating. Wait well past that before performing the blocking teardown.
#define TEARDOWN_DELAY_MS   5000
// How long the controller has to stay away before Wi-Fi is worth bringing up.
// A fallback for hosts that never report a suspend, so it cannot be so long
// that a wake becomes impossible on those. The cost of it being short is that
// a controller switched back on takes about a second longer to attach.
#define BRINGUP_IDLE_MS     30000

// The USB link dies with the host, so nothing can be watched live across a
// suspend. Printing the running totals instead means they can be read back
// after the host wakes - in particular whether it suspended the bus at all.
#define STATUS_INTERVAL_MS  10000

// A controller can stay connected while nobody is using it - it is paired and
// idle on the desk. Sitting untouched this long means nobody is playing, and by
// then the host has almost certainly gone to sleep, which is exactly when a
// wake packet has to be able to arrive. So the radio goes back to Wi-Fi even
// though the Bluetooth link is still up. Any input at all reverses it.
#ifndef WIFI_WAKE_BT_IDLE_MIN
#define WIFI_WAKE_BT_IDLE_MIN 30
#endif
#define BT_IDLE_MS          ((uint32_t)WIFI_WAKE_BT_IDLE_MIN * 60u * 1000u)

// Must match the value main.cpp arms the watchdog with.
#define WATCHDOG_NORMAL_MS  1000
#define WATCHDOG_RELAXED_MS 8000

// Report calls to the chip that run long enough to matter to the main loop.
#define SLOW_CALL_WARN_MS   100

namespace {

// Widens the watchdog for the lifetime of the object. Only meaningful when the
// watchdog is actually armed - with ENABLE_SERIAL main.cpp never starts it, and
// arming one here would introduce a reset the stock firmware does not have.
class WatchdogRelax {
public:
    WatchdogRelax() {
#if !ENABLE_SERIAL
        watchdog_enable(WATCHDOG_RELAXED_MS, true);
#endif
    }
    ~WatchdogRelax() {
#if !ENABLE_SERIAL
        watchdog_enable(WATCHDOG_NORMAL_MS, true);
#endif
    }
};

// Times a call to the chip and reports it when slow.
#define TIMED(label, call)                                                       \
    do {                                                                         \
        const uint64_t _t0 = time_us_64();                                       \
        call;                                                                    \
        const uint32_t _ms = (uint32_t)((time_us_64() - _t0) / 1000);            \
        printf("[wifi-wake] %s%s took %lu ms\n",                                 \
               (_ms >= SLOW_CALL_WARN_MS) ? "SLOW: " : "", label,                \
               (unsigned long)_ms);                                              \
    } while (0)

enum class LinkState { Idle, Connecting, Connected };

// The console is unavailable exactly when the interesting things happen, so
// the LED reports them instead:
//
//   one short flash every 5 s   Wi-Fi is associated and listening
//   three short flashes         a trigger packet arrived and was accepted
//   one long flash (1.5 s)      the keystroke actually reached the USB stack
//   two medium flashes          a trigger arrived but wake is off in the config
//   four rapid flashes          the dongle was not on the USB bus; it attached
//                               and the trigger will be retried
//
// The three-flash and the long flash are deliberately separate events. The
// first only says the trigger was taken; the second says a keydown was
// accepted for transmission. A wake that fails between them looks nothing like
// one that fails after, and without the console that distinction is otherwise
// invisible.
//
// A flash is always a change from whatever the LED was already showing, and the
// previous state is restored when the pattern ends. bt.cpp holds the LED on
// while a controller is connected, so against that a flash reads as a blink
// out rather than in - either way it is visible, and either way the indicator
// it borrowed the LED from gets it back.
struct LedPattern { uint16_t on_ms; uint16_t off_ms; uint8_t flashes; };
constexpr LedPattern LED_HEARTBEAT { 30,   0,   1 };
constexpr LedPattern LED_TRIGGERED { 80,   90,  3 };
constexpr LedPattern LED_KEY_SENT  { 1500, 0,   1 };
constexpr LedPattern LED_REFUSED   { 400,  250, 2 };
constexpr LedPattern LED_ATTACHING { 60,   60,  4 };
#define LED_HEARTBEAT_INTERVAL_MS 5000

LedPattern led_pattern {};
uint8_t led_flashes_left = 0;
bool led_flashing = false;
bool led_baseline = false;
absolute_time_t led_next_change;
absolute_time_t led_next_heartbeat;

void led_play(const LedPattern &p) {
    if (led_flashes_left != 0) return;   // let the running pattern finish
    led_baseline = cyw43_arch_gpio_get(CYW43_WL_GPIO_LED_PIN);
    led_pattern = p;
    led_flashes_left = p.flashes;
    led_flashing = false;
    led_next_change = get_absolute_time();
}

bool credentials_ok = false;
char ssid[WIFI_CFG_SSID_LEN];
char password[WIFI_CFG_PASS_LEN];

bool radio_up = false;          // is the station interface currently up
bool paused = true;             // Bluetooth owns the radio; do no Wi-Fi work
bool booted = false;            // has BOOT_SETTLE_MS elapsed
bool mac_reported = false;
bool idle_timing = false;
absolute_time_t last_bt_input;
absolute_time_t settle_deadline;
absolute_time_t teardown_at;
absolute_time_t bringup_at;

LinkState link_state = LinkState::Idle;
absolute_time_t next_action;
absolute_time_t join_deadline;

// Set from the lwIP receive callback, consumed by wifi_wake_task(). The
// callback runs in the cyw43 poll context, so it must not touch TinyUSB.
volatile bool trigger_pending = false;

// A trigger that arrives before the dongle is on the USB bus has to wait for
// enumeration rather than being thrown away.
#define TRIGGER_RETRIES  10
#define TRIGGER_RETRY_MS 400
uint8_t retries_left = 0;
absolute_time_t retry_at;

// How long to give a started trigger before concluding the endpoint is stuck.
#define KEY_WAIT_MS 2500
uint32_t keys_before_trigger = 0;
absolute_time_t recover_at;
bool recovery_armed = false;

// The keyboard interface comes up unusable on the enumeration that follows a
// cold boot: tud_hid_n_ready() is false from the first status line onwards,
// with nothing ever sent on the endpoint, and only a detach/attach clears it.
// Discovering that costs the first wake packet -- exactly the one that matters,
// since by then the host is asleep and nobody is watching. So check while
// nothing is at stake and fix it up front. Bounded, because a re-enumeration
// itself produces a fresh mount and must not feed itself.
#define PRIME_DELAY_MS   6000
#define PRIME_MAX_TRIES  2
absolute_time_t prime_at;
bool prime_armed = false;
uint8_t primes_left = PRIME_MAX_TRIES;
bool usb_was_mounted = false;

bool buffer_contains(const uint8_t *hay, size_t hay_len,
                     const char *needle, size_t needle_len) {
    if (needle_len == 0 || hay_len < needle_len) return false;
    for (size_t i = 0; i + needle_len <= hay_len; i++) {
        if (memcmp(hay + i, needle, needle_len) == 0) return true;
    }
    return false;
}

void udp_recv_callback(void *, struct udp_pcb *, struct pbuf *p,
                       const ip_addr_t *addr, u16_t port) {
    if (p == nullptr) return;

    // lwIP is single-threaded here, so the callback cannot re-enter itself.
    static uint8_t buf[RX_BUF_SIZE];
    const u16_t copied = pbuf_copy_partial(p, buf, sizeof(buf), 0);
    pbuf_free(p);

    static const char magic[] = WIFI_WAKE_MAGIC;
    if (buffer_contains(buf, copied, magic, sizeof(magic) - 1)) {
        printf("[wifi-wake] trigger from %s:%u\n", ipaddr_ntoa(addr), port);
        trigger_pending = true;
        return;
    }

    // Port 9 is also the conventional Wake-on-LAN discard port, so broadcast
    // magic packets meant for other machines land here as a matter of course.
    // Report them rate limited: this runs in the cyw43 poll context, which also
    // carries the Bluetooth path.
    static uint32_t ignored = 0;
    static absolute_time_t next_report;   // zero-initialised: reports immediately
    ignored++;
    if (absolute_time_diff_us(get_absolute_time(), next_report) <= 0) {
        printf("[wifi-wake] ignored %lu non-matching packet(s), last %u bytes from %s:%u\n",
               (unsigned long)ignored, copied, ipaddr_ntoa(addr), port);
        ignored = 0;
        next_report = make_timeout_time_ms(IGNORED_REPORT_INTERVAL_MS);
    }
}

bool listener_start() {
    struct udp_pcb *pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
    if (pcb == nullptr) {
        printf("[wifi-wake] failed to allocate udp pcb\n");
        return false;
    }
    if (udp_bind(pcb, IP_ANY_TYPE, WIFI_WAKE_UDP_PORT) != ERR_OK) {
        printf("[wifi-wake] bind to port %d failed\n", WIFI_WAKE_UDP_PORT);
        udp_remove(pcb);
        return false;
    }
    // Accept subnet broadcasts, so a sender does not need to know the address
    // DHCP handed out.
    ip_set_option(pcb, SOF_BROADCAST);
    udp_recv(pcb, udp_recv_callback, nullptr);
    printf("[wifi-wake] listening on udp %d (unicast and broadcast) for \"%s\"\n",
           WIFI_WAKE_UDP_PORT, WIFI_WAKE_MAGIC);
    return true;
}

void radio_bring_up() {
    WatchdogRelax relax;
    printf("[wifi-wake] bringing Wi-Fi up (host %s)\n",
           wake_host_is_suspended() ? "suspended" : "awake");
    // Slow the page scan for as long as Wi-Fi holds the radio.
    bt_set_page_scan_fast(false);
    TIMED("cyw43_arch_enable_sta_mode", cyw43_arch_enable_sta_mode());

    if (netif_default != nullptr) {
        netif_set_hostname(netif_default, WIFI_WAKE_HOSTNAME);
    }
    if (!mac_reported) {
        uint8_t mac[6];
        if (cyw43_wifi_get_mac(&cyw43_state, CYW43_ITF_STA, mac) == 0) {
            printf("[wifi-wake] mac %02x:%02x:%02x:%02x:%02x:%02x, dhcp hostname \"%s\"\n",
                   mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], WIFI_WAKE_HOSTNAME);
            mac_reported = true;
        }
    }

    radio_up = true;
    link_state = LinkState::Idle;
    next_action = get_absolute_time();
}

void radio_bring_down() {
    WatchdogRelax relax;
    printf("[wifi-wake] taking Wi-Fi down\n");
    TIMED("cyw43_arch_disable_sta_mode", cyw43_arch_disable_sta_mode());
    // Bluetooth gets the radio back at full rate.
    bt_set_page_scan_fast(true);
    bt_set_discoverable(true);
    radio_up = false;
    link_state = LinkState::Idle;
}

void begin_connect() {
    WatchdogRelax relax;
    // Give the scan as much of the radio as possible: a scan has to dwell on
    // each channel, and it is the part that fails when Bluetooth is receiving
    // continuously.
    bt_set_discoverable(false);
    printf("[wifi-wake] connecting to \"%s\" (bluetooth scan quietened)\n", ssid);
    int err;
    TIMED("cyw43_arch_wifi_connect_async",
          err = cyw43_arch_wifi_connect_async(ssid, password, CYW43_AUTH_WPA2_AES_PSK));
    if (err != 0) {
        printf("[wifi-wake] could not start association (%d)\n", err);
        link_state = LinkState::Idle;
        next_action = make_timeout_time_ms(RETRY_DELAY_MS);
        return;
    }
    link_state = LinkState::Connecting;
    join_deadline = make_timeout_time_ms(JOIN_TIMEOUT_MS);
    next_action = make_timeout_time_ms(POLL_INTERVAL_MS);
}

void link_task() {
    if (absolute_time_diff_us(get_absolute_time(), next_action) > 0) return;

    if (link_state == LinkState::Idle) {
        begin_connect();
        return;
    }

    const int status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
    next_action = make_timeout_time_ms(POLL_INTERVAL_MS);

    if (status == CYW43_LINK_UP) {
        if (link_state != LinkState::Connected) {
            link_state = LinkState::Connected;
            // Associated: inquiry scan can come back, but page scan stays slow
            // while Wi-Fi holds the link.
            bt_set_discoverable(true);
            printf("[wifi-wake] connected, ip %s\n",
                   ipaddr_ntoa(netif_ip_addr4(netif_default)));
        }
        return;
    }

    if (status < 0) {
        // -2 is CYW43_LINK_NONET: the scan saw no matching SSID.
        printf("[wifi-wake] association failed (%d), retrying\n", status);
        bt_set_discoverable(true);   // do not stay quiet through the retry gap
        link_state = LinkState::Idle;
        next_action = make_timeout_time_ms(RETRY_DELAY_MS);
        return;
    }

    if (link_state == LinkState::Connected) {
        printf("[wifi-wake] link lost, reconnecting\n");
        link_state = LinkState::Idle;
        next_action = make_timeout_time_ms(RETRY_DELAY_MS);
        return;
    }

    if (absolute_time_diff_us(get_absolute_time(), join_deadline) <= 0) {
        printf("[wifi-wake] association timed out, retrying\n");
        WatchdogRelax relax;
        bt_set_discoverable(true);
        cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
        link_state = LinkState::Idle;
        next_action = make_timeout_time_ms(RETRY_DELAY_MS);
    }
}

// Decides which stack owns the radio.
//
// The two directions are deliberately not symmetric. bt.cpp calls tud_connect()
// the moment the controller attaches, so the host begins enumerating right then
// and the main loop has to stay responsive - which is the worst possible moment
// to be several hundred milliseconds inside an ioctl to the CYW43. So a
// controller arriving stops all Wi-Fi work immediately, and the blocking
// teardown is deferred until enumeration has long since finished. A controller
// leaving is not urgent at all, so that direction just waits.
void arbitrate_radio() {
    const absolute_time_t now = get_absolute_time();

    // A suspended USB bus is the one piece of direct evidence that the host is
    // asleep, which is exactly and only when a wake packet has to be able to
    // arrive. Everything else here is a fallback for hosts that never suspend
    // the bus - some in modern standby do not - where idleness has to stand in
    // for a signal that never comes.
    bool bluetooth_wins;
    if (wake_host_is_suspended()) {
        bluetooth_wins = false;
    } else if (!bt_is_connected()) {
        bluetooth_wins = true;   // released below once the absence has lasted
    } else {
        const int64_t idle_us = absolute_time_diff_us(last_bt_input, now);
        bluetooth_wins = idle_us < (int64_t)BT_IDLE_MS * 1000;
    }

    if (bluetooth_wins && bt_is_connected()) {
        idle_timing = false;
        if (!paused) {
            // Takes effect on this same loop iteration: link_task() is skipped
            // from here on, so nothing new is issued to the chip. bt.cpp calls
            // tud_connect() as a controller attaches and the host starts
            // enumerating right then, so the blocking teardown waits.
            paused = true;
            teardown_at = make_timeout_time_ms(TEARDOWN_DELAY_MS);
            printf("[wifi-wake] controller in use -> Wi-Fi work stopped\n");
        }
        if (radio_up && absolute_time_diff_us(now, teardown_at) <= 0) {
            radio_bring_down();
        }
        return;
    }

    // Nothing is using Bluetooth. Wi-Fi may take the radio, after a wait whose
    // length depends on how sure we are: a suspended host is certain, a
    // controller that has already gone quiet has proved itself, and a merely
    // absent one has to stay absent for a while first.
    paused = false;
    if (!idle_timing) {
        idle_timing = true;
        const bool certain = wake_host_is_suspended() || bt_is_connected();
        bringup_at = certain ? now : make_timeout_time_ms(BRINGUP_IDLE_MS);
        return;
    }
    if (!radio_up && absolute_time_diff_us(now, bringup_at) <= 0) {
        radio_bring_up();
    }
}

} // namespace

void wifi_wake_init(void) {
    credentials_ok = wifi_config_load(ssid, sizeof(ssid), password, sizeof(password));
    if (!credentials_ok) {
        printf("[wifi-wake] no credentials in this image; network wake is inactive.\n"
               "            Patch the .uf2 with tools/uf2-wifi-config.html.\n");
        return;
    }

    // The pcb can be bound before the station interface exists; nothing arrives
    // until the netif comes up, and it survives the interface going up and down.
    listener_start();

    last_bt_input = get_absolute_time();
    settle_deadline = make_timeout_time_ms(BOOT_SETTLE_MS);
    printf("[wifi-wake] armed; Wi-Fi comes up when the host suspends, "
           "or after %d s with no controller, or %d min with no input\n",
           BRINGUP_IDLE_MS / 1000, WIFI_WAKE_BT_IDLE_MIN);
}

void wifi_wake_led_tick(void) {
    if (!credentials_ok) return;
    const absolute_time_t now = get_absolute_time();

    // A keystroke reaching the USB stack outranks the heartbeat.
    static uint32_t last_keys_seen = 0;
    const uint32_t keys = wake_key_sent_count();
    if (keys != last_keys_seen) {
        last_keys_seen = keys;
        led_play(LED_KEY_SENT);
    }

    if (led_flashes_left == 0) {
        // Idle: a short flash every few seconds says the listener is armed.
        if (radio_up && link_state == LinkState::Connected &&
            !get_config().disable_pico_led &&
            absolute_time_diff_us(now, led_next_heartbeat) <= 0) {
            led_next_heartbeat = make_timeout_time_ms(LED_HEARTBEAT_INTERVAL_MS);
            led_play(LED_HEARTBEAT);
        }
        return;
    }

    if (absolute_time_diff_us(now, led_next_change) > 0) return;

    if (!led_flashing) {
        led_flashing = true;
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, !led_baseline);
        led_next_change = make_timeout_time_ms(led_pattern.on_ms);
        return;
    }

    led_flashing = false;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_baseline);
    led_flashes_left--;
    if (led_flashes_left > 0) {
        led_next_change = make_timeout_time_ms(led_pattern.off_ms);
    }
}

void wifi_wake_note_bt_input(const uint8_t *hid_input, uint16_t len) {
    // A DualSense streams input reports at the polling rate for as long as it
    // is connected, so a report arriving says nothing about whether anyone is
    // holding it. Only a change in what it reports does.
    //
    // Layout (after main.cpp's `data + 3` skip), same bytes wake.cpp reads:
    //   1..4  stick axes      5..6  L2 / R2
    //   7     d-pad + face    8     shoulders, share, options, L3, R3
    //   9     PS, touchpad, mute
    //
    // Buttons compare exactly. The analog axes need a deadzone, because a stick
    // at rest jitters by a count or two, and prev only advances on a real
    // change so that jitter can never accumulate into one.
    static const int AXIS_DEADZONE = 8;
    static uint8_t prev[10];
    static bool have_prev = false;

    if (len < 10) return;

    bool changed = false;
    if (!have_prev) {
        have_prev = true;
        changed = true;
    } else if (hid_input[7] != prev[7] || hid_input[8] != prev[8] || hid_input[9] != prev[9]) {
        changed = true;
    } else {
        for (int i = 1; i <= 6; i++) {
            int d = (int)hid_input[i] - (int)prev[i];
            if (d < 0) d = -d;
            if (d > AXIS_DEADZONE) { changed = true; break; }
        }
    }

    if (changed) {
        memcpy(prev, hid_input, sizeof(prev));
        last_bt_input = get_absolute_time();
    }
}

void report_status() {
    static absolute_time_t next_report;
    if (absolute_time_diff_us(get_absolute_time(), next_report) > 0) return;
    next_report = make_timeout_time_ms(STATUS_INTERVAL_MS);

    const int64_t idle_ms = absolute_time_diff_us(last_bt_input, get_absolute_time()) / 1000;
    char hid[64];
    wake_hid_summary(hid, sizeof(hid));
    printf("[wifi-wake] status: wifi=%s link=%s host=%s suspends=%lu resumes=%lu "
           "controller=%s usb=%s hid=%s(%s) state=%s keys=%lu idle=%llds\n",
           radio_up ? "up" : "down",
           link_state == LinkState::Connected ? "connected"
               : (link_state == LinkState::Connecting ? "connecting" : "idle"),
           wake_host_is_suspended() ? "suspended" : "awake",
           (unsigned long)wake_suspend_count(), (unsigned long)wake_resume_count(),
           bt_is_connected() ? "connected" : "none",
           tud_mounted() ? "attached" : "detached",
           wake_kbd_ready() ? "ready" : (wake_kbd_endpoint_open() ? "busy" : "closed"),
           hid,
           wake_state_str(),
           (unsigned long)wake_key_sent_count(),
           (long long)(idle_ms / 1000));
}



// Watches the USB attach edge and, once settled, makes sure the wake keyboard is
// actually usable -- forcing one re-enumeration if it is not.
void prime_keyboard(void) {
    const bool mounted = tud_mounted();
    if (mounted && !usb_was_mounted) {
        prime_at = make_timeout_time_ms(PRIME_DELAY_MS);
        prime_armed = true;
    }
    usb_was_mounted = mounted;

    if (!prime_armed || !mounted) return;
    if (absolute_time_diff_us(get_absolute_time(), prime_at) > 0) return;
    prime_armed = false;

    if (wake_kbd_ready()) return;          // nothing to fix
    // With wake off the keyboard is deliberately absent from the configuration
    // descriptor, so "not ready" is correct and re-enumerating fixes nothing.
    if (!get_config().enable_wake) return;
    if (primes_left == 0) return;          // already tried; don't loop on it
    if (bt_is_connected()) return;         // a re-enumeration would blip the pad
    primes_left--;

    char why[64];
    wake_hid_summary(why, sizeof(why));
    printf("[wifi-wake] wake keyboard unusable after enumeration (%s); "
           "re-enumerating now so the first trigger isn't wasted "
           "-- the USB serial log drops here, reconnect it\n", why);
    sleep_ms(50);
    wake_usb_reconnect();
}

void wifi_wake_task(void) {
    if (!credentials_ok) return;
    report_status();
    prime_keyboard();

    if (trigger_pending) {
        trigger_pending = false;
        // Report which way it went on the LED, because during a host sleep this
        // is the only thing anyone can see.
        switch (wake_request_from_network()) {
        case WAKE_NET_STARTED:
            led_play(LED_TRIGGERED);
            retries_left = 0;
            // If no keystroke follows, the keyboard endpoint is stuck on a
            // transfer the host never collected, and only re-enumerating
            // clears it. Watch for that rather than sitting on it.
            keys_before_trigger = wake_key_sent_count();
            recover_at = make_timeout_time_ms(KEY_WAIT_MS);
            recovery_armed = true;
            break;
        case WAKE_NET_DISABLED:
            led_play(LED_REFUSED);
            retries_left = 0;
            break;
        case WAKE_NET_NOT_ATTACHED:
            // Enumeration takes a moment; come back and finish the job.
            led_play(LED_ATTACHING);
            retries_left = TRIGGER_RETRIES;
            retry_at = make_timeout_time_ms(TRIGGER_RETRY_MS);
            break;
        }
    } else if (recovery_armed &&
               absolute_time_diff_us(get_absolute_time(), recover_at) <= 0) {
        recovery_armed = false;
        if (wake_key_sent_count() == keys_before_trigger) {
            char why[64];
            wake_hid_summary(why, sizeof(why));
            printf("[wifi-wake] no keystroke after trigger (hid=%s %s state=%s); "
                   "re-enumerating -- the USB serial log drops here, reconnect it\n",
                   wake_kbd_ready() ? "ready" : "not-ready", why, wake_state_str());
            led_play(LED_ATTACHING);
            // Give stdio a moment to push that line out before the bus goes away.
            sleep_ms(50);
            wake_usb_reconnect();
            retries_left = TRIGGER_RETRIES;
            retry_at = make_timeout_time_ms(TRIGGER_RETRY_MS);
        }
    } else if (retries_left > 0 &&
               absolute_time_diff_us(get_absolute_time(), retry_at) <= 0) {
        retries_left--;
        retry_at = make_timeout_time_ms(TRIGGER_RETRY_MS);
        if (wake_request_from_network() == WAKE_NET_STARTED) {
            led_play(LED_TRIGGERED);
            retries_left = 0;
        }
    }

    if (!booted) {
        if (absolute_time_diff_us(get_absolute_time(), settle_deadline) > 0) return;
        booted = true;
        printf("[wifi-wake] settled\n");
    }

    arbitrate_radio();

    // Nothing reaches the chip while Bluetooth owns it.
    if (radio_up && !paused) link_task();
}

#endif // ENABLE_WIFI_WAKE
