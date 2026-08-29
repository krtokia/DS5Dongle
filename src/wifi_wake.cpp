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
// Long enough that a controller reconnecting does not cost a radio transition,
// and that USB has settled either way.
#define BRINGUP_IDLE_MS     30000

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
    printf("[wifi-wake] bringing Wi-Fi up\n");
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

    // Who should hold the radio. A controller in use always wins; a controller
    // that has been untouched long enough does not, because by then the host is
    // almost certainly asleep and a wake packet is the only thing that matters.
    bool bluetooth_wins;
    if (!bt_is_connected()) {
        bluetooth_wins = false;
    } else {
        const int64_t idle_us = absolute_time_diff_us(last_bt_input, now);
        bluetooth_wins = idle_us < (int64_t)BT_IDLE_MS * 1000;
    }

    if (bluetooth_wins) {
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

    paused = false;
    if (!idle_timing) {
        idle_timing = true;
        // A controller that has gone quiet for long enough is already proof
        // enough; only an absent one needs the settling wait.
        bringup_at = bt_is_connected() ? now : make_timeout_time_ms(BRINGUP_IDLE_MS);
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
    printf("[wifi-wake] armed; Wi-Fi comes up %d s after the controller leaves, "
           "or %d min after its last input\n",
           BRINGUP_IDLE_MS / 1000, WIFI_WAKE_BT_IDLE_MIN);
}

void wifi_wake_note_bt_input(void) {
    last_bt_input = get_absolute_time();
}

void wifi_wake_task(void) {
    if (!credentials_ok) return;

    if (trigger_pending) {
        trigger_pending = false;
        wake_request_from_network();
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
