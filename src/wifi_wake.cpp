//
// Wi-Fi side of the network wake path.
//
// Joins a WPA2 network and listens on UDP for a trigger packet. On a match it
// calls wake_request_from_network(), which drives the keyboard FSM in wake.cpp.
//
// Two constraints shape this file:
//
//   * The main loop runs under a 1 s watchdog and also pumps Bluetooth audio,
//     so nothing here may block. Association uses the async API and is polled.
//   * Bluetooth and Wi-Fi share one CYW43439 radio. Idle cost is what matters,
//     and while waiting for a trigger this holds no sockets open beyond a
//     single bound UDP pcb and sends nothing at all.
//

#include "wifi_wake.h"

#ifdef ENABLE_WIFI_WAKE

#include <cstdio>
#include <cstring>

#include "pico/cyw43_arch.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/netif.h"

#include "wake.h"
#include "wifi_config.h"
#include "debug.h"

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

#define POLL_INTERVAL_MS   500
#define JOIN_TIMEOUT_MS    20000
#define RETRY_DELAY_MS     5000
#define IGNORED_REPORT_INTERVAL_MS 5000

namespace {

enum class LinkState { Idle, Connecting, Connected };

LinkState link_state = LinkState::Idle;
absolute_time_t next_action;
absolute_time_t join_deadline;

char ssid[WIFI_CFG_SSID_LEN];
char password[WIFI_CFG_PASS_LEN];
bool have_credentials = false;

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
    // Report them rate limited rather than per packet: this runs in the cyw43
    // poll context, which also carries the Bluetooth audio path.
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

bool listener_start(void) {
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

void begin_connect(void) {
    printf("[wifi-wake] connecting to \"%s\"\n", ssid);
    const int err = cyw43_arch_wifi_connect_async(ssid, password, CYW43_AUTH_WPA2_AES_PSK);
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

void link_task(void) {
    if (!have_credentials) return;
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
        cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
        link_state = LinkState::Idle;
        next_action = make_timeout_time_ms(RETRY_DELAY_MS);
    }
}

} // namespace

void wifi_wake_init(void) {
    have_credentials = wifi_config_load(ssid, sizeof(ssid), password, sizeof(password));
    if (!have_credentials) {
        printf("[wifi-wake] no credentials in this image; network wake is inactive.\n"
               "            Patch the .uf2 with tools/uf2-wifi-config.html.\n");
        return;
    }

    cyw43_arch_enable_sta_mode();
    netif_set_hostname(netif_default, WIFI_WAKE_HOSTNAME);

    uint8_t mac[6];
    if (cyw43_wifi_get_mac(&cyw43_state, CYW43_ITF_STA, mac) == 0) {
        printf("[wifi-wake] mac %02x:%02x:%02x:%02x:%02x:%02x, dhcp hostname \"%s\"\n",
               mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], WIFI_WAKE_HOSTNAME);
    }

    listener_start();
    next_action = get_absolute_time();
}

void wifi_wake_task(void) {
    if (trigger_pending) {
        trigger_pending = false;
        wake_request_from_network();
    }
    link_task();
}

#endif // ENABLE_WIFI_WAKE
