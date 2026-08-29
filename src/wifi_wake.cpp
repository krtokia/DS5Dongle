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
#define JOIN_TIMEOUT_MS     20000
#define RETRY_DELAY_MS      5000
#define IGNORED_REPORT_INTERVAL_MS 5000

// Let USB enumerate and the initial Bluetooth inquiry settle before touching
// the radio for the first time.
#define BOOT_SETTLE_MS      10000
// Do not follow every momentary Bluetooth state change; a controller that
// drops and reconnects should not cost two radio transitions.
#define SWITCH_DEBOUNCE_MS  3000

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
bool booted = false;            // has BOOT_SETTLE_MS elapsed
bool mac_reported = false;
absolute_time_t settle_deadline;
absolute_time_t switch_deadline;
bool switch_pending = false;

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
    printf("[wifi-wake] controller idle -> bringing Wi-Fi up\n");
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
    printf("[wifi-wake] controller connected -> taking Wi-Fi down\n");
    TIMED("cyw43_arch_disable_sta_mode", cyw43_arch_disable_sta_mode());
    radio_up = false;
    link_state = LinkState::Idle;
}

void begin_connect() {
    WatchdogRelax relax;
    printf("[wifi-wake] connecting to \"%s\"\n", ssid);
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
            printf("[wifi-wake] connected, ip %s\n",
                   ipaddr_ntoa(netif_ip_addr4(netif_default)));
        }
        return;
    }

    if (status < 0) {
        printf("[wifi-wake] association failed (%d), retrying\n", status);
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
        cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
        link_state = LinkState::Idle;
        next_action = make_timeout_time_ms(RETRY_DELAY_MS);
    }
}

// Decides which radio should own the chip and performs at most one transition
// per call. Debounced, so a controller that briefly drops does not cost two.
void arbitrate_radio() {
    const bool want_up = !bt_is_connected();
    if (want_up == radio_up) {
        switch_pending = false;
        return;
    }

    if (!switch_pending) {
        switch_pending = true;
        switch_deadline = make_timeout_time_ms(SWITCH_DEBOUNCE_MS);
        return;
    }
    if (absolute_time_diff_us(get_absolute_time(), switch_deadline) > 0) return;

    switch_pending = false;
    if (want_up) {
        radio_bring_up();
    } else {
        radio_bring_down();
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

    settle_deadline = make_timeout_time_ms(BOOT_SETTLE_MS);
    printf("[wifi-wake] armed; Wi-Fi comes up once no controller is connected "
           "(settling for %d ms first)\n", BOOT_SETTLE_MS);
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

    if (radio_up) link_task();
}

#endif // ENABLE_WIFI_WAKE
