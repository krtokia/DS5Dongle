//
// Created by awalol on 2026/4/30.
//

#include "wake.h"

#ifdef ENABLE_WAKE_HID

#include <cstdio>
#include <cstring>
#include "bt.h"
#include "tusb.h"
#include "device/dcd.h"
#include "pico/sync.h"
#include "pico/time.h"
#include "ps_shortcut.h"
#include "config.h"

// Defined in usb_descriptors.cpp: what the host was last offered.
bool wake_desc_advertised_kbd(void);
uint16_t wake_desc_config_reads(void);


// TinyUSB hands out HID instance indices in the order hidd_open() claims the
// interfaces, so the wake keyboard's index depends on what else is in the
// configuration (the CDC log interface sits between them in serial builds).
// Look it up instead of assuming. Only the wake keyboard declares the boot
// keyboard protocol, and tud_hid_n_interface_protocol() is non-zero only once
// hidd_open() has claimed the interface AND opened its endpoint -- so "not
// found" and "found but not ready" are two genuinely different faults.
#define WAKE_KBD_NONE         0xFF
#define WAKE_KEYCODE_F15      0x68
// Post-resume timings tuned for "wake-and-resleep" Windows behavior: the host
// resumes USB, but if no HID input is consumed during the brief wake window
// the system can re-suspend within ~1 s. Bigger settles + a second F15 give
// Windows multiple polling cycles to pick the keystroke up.
#define WAKE_SETTLE_US        150000   // 150 ms — let host finish USB re-init
#define WAKE_KEY_HOLD_US       80000   // 80 ms keydown -> keyup gap
#define WAKE_KEY_UP_SETTLE_US 200000   // 200 ms between attempts (or before DONE)
#define WAKE_REQUEST_TIMEOUT_US 5000000
#define WAKE_KEY_ATTEMPTS     2

static uint8_t kbd_instance(void) {
    for (uint8_t i = 0; i < CFG_TUD_HID; i++) {
        if (tud_hid_n_interface_protocol(i) == HID_ITF_PROTOCOL_KEYBOARD) return i;
    }
    return WAKE_KBD_NONE;
}

static bool kbd_ready(void) {
    const uint8_t i = kbd_instance();
    return i != WAKE_KBD_NONE && tud_hid_n_ready(i);
}

// rpt is always an 8-byte boot keyboard report.
static bool kbd_send(const uint8_t *rpt) {
    const uint8_t i = kbd_instance();
    return i != WAKE_KBD_NONE && tud_hid_n_report(i, 0, rpt, 8);
}
#define WAKE_DISCONNECT_DEBOUNCE_US 3000000  // 3s: only disconnect (and thereby power off) the
                                             // controller after a sustained suspend; ignore brief
                                             // hub-induced suspends while the host is awake.
#define WAKE_RECONNECT_GRACE_US   5000000  // 5s: after a deliberate USB reconnect, ignore the
                                           // suspend it causes (it is not a host sleep).
                                           // Cleared early when the device re-mounts.

#ifdef WAKE_DEBUG
#  define WAKE_DBG(fmt, ...) printf("[wake] " fmt "\n", ##__VA_ARGS__)
static const char *wake_state_name(int s) {
    switch (s) {
    case 0: return "IDLE";
    case 1: return "PENDING_PRESS";
    case 2: return "REQUESTED";
    case 3: return "KEY_DOWN";
    case 4: return "KEY_UP_SENT";
    case 5: return "DONE";
    default: return "?";
    }
}
#else
#  define WAKE_DBG(fmt, ...) ((void)0)
static const char *wake_state_name(int s) {
    switch (s) {
    case 0: return "IDLE";
    case 1: return "PENDING_PRESS";
    case 2: return "REQUESTED";
    case 3: return "KEY_DOWN";
    case 4: return "KEY_UP_SENT";
    case 5: return "DONE";
    default: return "?";
    }
}
#endif

typedef enum {
    WAKE_IDLE,
    WAKE_PENDING_PRESS,
    WAKE_REQUESTED,
    WAKE_KEY_DOWN,
    WAKE_KEY_UP_SENT,
    WAKE_DONE,
} wake_state_t;

static critical_section_t wake_cs;
static volatile bool host_suspended = false;
static volatile uint32_t suspend_count = 0;
static volatile uint32_t resume_count = 0;
static volatile uint32_t key_sent_count = 0;
static volatile bool host_resumed_event = false;
static wake_state_t state = WAKE_IDLE;
static uint64_t state_entered_us = 0;
static uint8_t key_attempts = 0;
// Last-seen DualSense button bytes. Idle defaults: byte 7 = 0x08 (D-pad
// released), bytes 8 / 9 = 0 (no shoulders, no PS / touchpad / mute).
static uint8_t prev_b7 = 0x08;
static uint8_t prev_b8 = 0x00;
static uint8_t prev_b9 = 0x00;
// Debounced controller disconnect: time of the pending suspend (0 = none pending).
static volatile uint64_t suspend_at_us = 0;
// During a deliberate USB reconnect, ignore the suspend it triggers until this time.
static volatile uint64_t reconnect_until_us = 0;

static void enter_state(wake_state_t s) {
    state = s;
    state_entered_us = time_us_64();
}

static void request_host_wake(const char *reason) {
    bool ok = tud_remote_wakeup();

    // Linux quirk: Sometimes Linux fails to set the REMOTE_WAKEUP feature
    // flag before the second suspend, causing TinyUSB to refuse to wake.
    // If we are suspended but ok is false, we force the wake signal.
    if (!ok && host_suspended) {
        WAKE_DBG("%s: tud_remote_wakeup()=0 but suspended. Forcing DCD wake.", reason);
        dcd_remote_wakeup(0);
        ok = true;
    }

    if (ok) {
        critical_section_enter_blocking(&wake_cs);
        state = WAKE_REQUESTED;
        state_entered_us = time_us_64();
        critical_section_exit(&wake_cs);
        WAKE_DBG("%s -> REQUESTED", reason);
    }
#ifdef WAKE_DEBUG
    else {
        static uint64_t last_log = 0;
        const uint64_t now = time_us_64();
        if (now - last_log > 5000000) {
            WAKE_DBG("%s, tud_remote_wakeup()=0 (USB bus not in suspend) -- 5s heartbeat", reason);
            last_log = now;
        }
    }
#endif
}

void wake_init(void) {
    critical_section_init(&wake_cs);
}

// Called right before a deliberate USB reconnect (FUNC_RECONNECT): arm a grace window so the
// suspend the reconnect causes is ignored, and drop any already-pending disconnect.
void wake_note_usb_reconnect(void) {
    reconnect_until_us = time_us_64() + WAKE_RECONNECT_GRACE_US;
    suspend_at_us = 0;
}

extern "C" void tud_suspend_cb(bool remote_wakeup_en) {
    WAKE_DBG("tud_suspend_cb remote_wakeup_en=%d prev_state=%s",
             (int)remote_wakeup_en, wake_state_name(state));
    // A deliberate Reconnect USB (FUNC_RECONNECT) tears the bus down and back up, which looks
    // like a suspend but is not a host sleep -- ignore it so it cannot disconnect the controller.
    // See wake_note_usb_reconnect().
    if (time_us_64() < reconnect_until_us) {
        WAKE_DBG("suspend during reconnect grace -> ignored");
        return;
    }
    // The disconnect runs on every genuine suspend, independent of enable_wake. The DS5 powers
    // off when its Bluetooth link is disconnected, saving battery during a real sleep/shutdown.
    // A spurious hub suspend is filtered by the debounce below, not a gate -- it resumes and
    // tud_resume_cb / tud_mount_cb cancel the pending disconnect first.
    suspend_at_us = time_us_64();
    host_suspended = true;
    host_resumed_event = false;
    suspend_count++;
    
    // Everything below is the wake-UP path (press a key to wake the host) -- enable_wake only.
    if (!get_config().enable_wake) return;

    // Unconditionally re-arm on suspend. If a previous wake attempt hung
    // (e.g. Linux ignored a keystroke and left the endpoint busy forever),
    // we must abort and reset so the NEXT wake attempt can trigger.
    state = WAKE_PENDING_PRESS;
    state_entered_us = time_us_64();
    prev_b7 = 0x08; prev_b8 = 0x00; prev_b9 = 0x00;
    key_attempts = 0;
    WAKE_DBG("-> PENDING_PRESS");
}

void wake_on_bt_connect(void) {
    if (!get_config().enable_wake) return;
    critical_section_enter_blocking(&wake_cs);
    const bool should_wake = host_suspended &&
        (state == WAKE_IDLE || state == WAKE_DONE || state == WAKE_PENDING_PRESS);
    critical_section_exit(&wake_cs);

    if (should_wake) {
        request_host_wake("BT reconnect while suspended");
    }
}

extern "C" void tud_resume_cb(void) {
    WAKE_DBG("tud_resume_cb state=%s", wake_state_name(state));
    resume_count++;
    host_suspended = false;
    host_resumed_event = true;
    suspend_at_us = 0;   // resumed before the debounce elapsed -> cancel the disconnect
}

extern "C" void tud_mount_cb(void) {
    WAKE_DBG("tud_mount_cb state=%s", wake_state_name(state));
    host_suspended = false;
    host_resumed_event = true;
    suspend_at_us = 0;
    reconnect_until_us = 0;   // reconnect finished re-enumerating; end the grace early
}

void wake_on_bt_input(const uint8_t *hid_input, uint16_t len) {
    if (!get_config().enable_wake) return;
    if (len < 10) return;
    // DualSense BT 0x31 input report layout (after main.cpp's `data + 3` skip):
    //   byte 7 low nibble: D-pad direction (0x08 idle); high nibble: face buttons
    //   byte 8: L1, R1, L2 click, R2 click, share, options, L3, R3
    //   byte 9: PS (bit 0), touchpad-click (bit 1), mute (bit 2)
    //
    // We trigger on ANY change in those three button bytes, not strictly on
    // the PS bit. Reasons:
    //   1. The DualSense's BT radio enters a low-power sniff mode after a
    //      period of inactivity. The PS button alone often does not wake
    //      the radio out of sniff -- shoulder buttons reliably do. So the
    //      first BT report after S3 is most likely whichever button the
    //      user happened to press to wake the radio. PS itself counts as
    //      "any button" too, so the single-press UX still works.
    //   2. We additionally call tud_remote_wakeup() speculatively even from
    //      WAKE_IDLE / WAKE_DONE state. TinyUSB returns true only when the
    //      host actually USB-suspended the bus; otherwise it's a no-op. This
    //      protects against the case where tud_suspend_cb didn't fire (e.g.
    //      a hub between the host and the dongle masking the suspend signal
    //      from downstream). On success the FSM transitions to REQUESTED and
    //      proceeds with the keystroke as normal.
    const uint8_t b7 = hid_input[7];
    const uint8_t b8 = hid_input[8];
    const uint8_t b9 = hid_input[9];

    critical_section_enter_blocking(&wake_cs);
    const bool changed = (b7 != prev_b7) || (b8 != prev_b8) || (b9 != prev_b9);
    const bool armable = (state == WAKE_IDLE || state == WAKE_DONE || state == WAKE_PENDING_PRESS);
    prev_b7 = b7; prev_b8 = b8; prev_b9 = b9;
    critical_section_exit(&wake_cs);

    if (changed && armable) {
        request_host_wake("button event");
    }
}

// Entry point for a network trigger (see wifi_wake.cpp).
//
// Unlike wake_on_bt_input(), this always proceeds. A wake packet is an explicit
// request from the user rather than incidental input, so there is no risk of
// firing during normal play - which is why the button path must gate on the bus
// actually being suspended and this one must not. That gate is also what limits
// the button path to hosts that suspend the bus: a host in modern standby (S0
// low power idle) can leave the bus up, and then tud_remote_wakeup() refuses and
// no key is ever sent.
//
// Forcing WAKE_REQUESTED reuses the rest of the FSM as-is, including its retry
// and settle handling: its handler fires the keystroke once the host is not
// suspended, which covers both a host that resumed in response to the remote
// wakeup below and a host that never suspended the bus at all.
wake_net_result_t wake_request_from_network(void) {
    // The keyboard interface is only enumerated while enable_wake is on
    // (see usb_descriptors.cpp), so without it there is nothing to send on.
    if (!get_config().enable_wake) {
        WAKE_DBG("network trigger ignored: enable_wake is off");
        return WAKE_NET_DISABLED;
    }

    // The dongle only attaches to USB once a controller connects (bt.cpp), so
    // after a boot with no controller it is not on the bus at all and there is
    // no endpoint to write to. Attach now: that is worth doing on its own,
    // since a host notices a device appearing.
    if (!tud_mounted()) {
        WAKE_DBG("network trigger: not attached, connecting");
        tud_connect();
        return WAKE_NET_NOT_ATTACHED;
    }

    if (host_suspended) {
        if (!tud_remote_wakeup()) {
            // Same fallback as request_host_wake(): the host can leave the bus
            // suspended without having set the REMOTE_WAKEUP feature.
            WAKE_DBG("network trigger: tud_remote_wakeup()=0 while suspended, forcing DCD wake");
            dcd_remote_wakeup(0);
        }
    }

    critical_section_enter_blocking(&wake_cs);
    key_attempts = 0;
    enter_state(WAKE_REQUESTED);
    critical_section_exit(&wake_cs);
    WAKE_DBG("network trigger -> REQUESTED");
    return WAKE_NET_STARTED;
}

bool wake_kbd_ready(void) {
    return kbd_ready();
}

// tud_hid_n_ready() folds three conditions together and reports only their AND,
// which is why a keyboard that never sends is indistinguishable from one that is
// merely between transfers. Split it: this says the host opened the interface and
// TinyUSB has an IN endpoint for it, so "endpoint stuck on an uncollected
// transfer" can be told apart from "interface never configured at all".
bool wake_kbd_endpoint_open(void) {
    return kbd_instance() != WAKE_KBD_NONE;
}

// Writes what every HID instance currently looks like, e.g. "kbd=1 proto=[0,1]".
// Instance protocols come straight out of hidd_open(), so this is the ground
// truth for whether the keyboard interface exists on the bus at all.
void wake_hid_summary(char *buf, size_t len) {
    const uint8_t kbd = kbd_instance();
    int n = snprintf(buf, len, "kbd=");
    if (kbd == WAKE_KBD_NONE) {
        n += snprintf(buf + n, (size_t)((int)len - n), "none");
    } else {
        n += snprintf(buf + n, (size_t)((int)len - n), "%u", (unsigned)kbd);
    }
    n += snprintf(buf + n, (size_t)((int)len - n), " proto=[");
    for (uint8_t i = 0; i < CFG_TUD_HID && n < (int)len; i++) {
        n += snprintf(buf + n, (size_t)((int)len - n), i ? ",%u" : "%u",
                      (unsigned)tud_hid_n_interface_protocol(i));
    }
    snprintf(buf + n, (size_t)((int)len - n), "] cfgdesc=%s/%u",
             wake_desc_advertised_kbd() ? "kbd-in" : "kbd-out",
             (unsigned)wake_desc_config_reads());
}

const char *wake_state_str(void) {
    return wake_state_name(state);
}

void wake_usb_reconnect(void) {
    WAKE_DBG("forcing USB reconnect");
    wake_note_usb_reconnect();
    tud_disconnect();
    sleep_ms(150);
    tud_connect();
}

bool wake_host_is_suspended(void) {
    return host_suspended;
}

uint32_t wake_suspend_count(void)  { return suspend_count; }
uint32_t wake_resume_count(void)   { return resume_count; }
uint32_t wake_key_sent_count(void) { return key_sent_count; }

void wake_on_bt_disconnect(void) {
    critical_section_enter_blocking(&wake_cs);
    state = WAKE_IDLE;
    prev_b7 = 0x08; prev_b8 = 0x00; prev_b9 = 0x00;
    critical_section_exit(&wake_cs);
    ps_shortcut_reset();
}

void wake_task(void) {
    const uint64_t now = time_us_64();

    // Commit the deferred controller disconnect once we have stayed suspended past the debounce
    // window (a genuine host sleep/shutdown). Runs regardless of enable_wake -- it is a
    // battery-save, not part of the wake-UP path. A transient hub suspend will already have
    // been cancelled by tud_resume_cb / tud_mount_cb before this fires.
    if (suspend_at_us != 0 && host_suspended &&
        now - suspend_at_us >= WAKE_DISCONNECT_DEBOUNCE_US) {
        bt_disconnect();
        suspend_at_us = 0;
        WAKE_DBG("suspend debounce elapsed -> bt_disconnect()");
    }

    // The wake-UP FSM below only runs when wake is enabled.
    if (!get_config().enable_wake) return;

    critical_section_enter_blocking(&wake_cs);
    const wake_state_t s = state;
    const uint64_t entered = state_entered_us;
    critical_section_exit(&wake_cs);

    switch (s) {
        case WAKE_IDLE:
        case WAKE_PENDING_PRESS:
        case WAKE_DONE:
            return;

        case WAKE_REQUESTED: {
            if (host_resumed_event || !host_suspended) {
                host_resumed_event = false;
                if (now - entered < WAKE_SETTLE_US) return;
                if (!kbd_ready()) {
#ifdef WAKE_DEBUG
                    static uint64_t last_log = 0;
                    if (now - last_log > 1000000) {
                        WAKE_DBG("REQUESTED waiting: hid_n_ready=0 (heartbeat 1Hz)");
                        last_log = now;
                    }
#endif
                    return;
                }
                uint8_t rpt[8] = { 0, 0, WAKE_KEYCODE_F15, 0, 0, 0, 0, 0 };
                const bool sent = kbd_send(rpt);
                WAKE_DBG("REQUESTED: sent keydown 0x%02X -> %d", WAKE_KEYCODE_F15, (int)sent);
                if (sent) {
                    key_sent_count++;
                    critical_section_enter_blocking(&wake_cs);
                    enter_state(WAKE_KEY_DOWN);
                    critical_section_exit(&wake_cs);
                }
            } else if (now - entered > WAKE_REQUEST_TIMEOUT_US) {
                WAKE_DBG("REQUESTED timeout 5s -> DONE (no resume signaling; may have already woken)");
                critical_section_enter_blocking(&wake_cs);
                enter_state(WAKE_DONE);
                critical_section_exit(&wake_cs);
            }
            return;
        }

        case WAKE_KEY_DOWN: {
            if (now - entered < WAKE_KEY_HOLD_US) return;
            if (!kbd_ready()) {
#ifdef WAKE_DEBUG
                static uint64_t last_log = 0;
                if (now - last_log > 1000000) {
                    WAKE_DBG("KEY_DOWN waiting: hid_n_ready=0 (heartbeat 1Hz)");
                    last_log = now;
                }
#endif
                return;
            }
            uint8_t up[8] = { 0 };
            const bool sent = kbd_send(up);
            WAKE_DBG("KEY_DOWN: sent keyup -> %d", (int)sent);
            if (sent) {
                critical_section_enter_blocking(&wake_cs);
                enter_state(WAKE_KEY_UP_SENT);
                critical_section_exit(&wake_cs);
            }
            return;
        }

        case WAKE_KEY_UP_SENT: {
            if (now - entered < WAKE_KEY_UP_SETTLE_US) return;
            key_attempts++;
            if (key_attempts < WAKE_KEY_ATTEMPTS) {
                // Retry: do NOT re-enter WAKE_REQUESTED (which gates on a
                // fresh tud_resume_cb event). We already established the
                // host woke once; just send another keydown directly. If the
                // host has dipped back into suspend, tud_hid_n_ready will be
                // false and we'll heartbeat from KEY_DOWN until it returns.
                if (!kbd_ready()) {
#ifdef WAKE_DEBUG
                    static uint64_t last_log = 0;
                    if (now - last_log > 1000000) {
                        WAKE_DBG("KEY_UP_SENT retry waiting: hid_n_ready=0 (heartbeat 1Hz)");
                        last_log = now;
                    }
#endif
                    return;
                }
                uint8_t rpt[8] = { 0, 0, WAKE_KEYCODE_F15, 0, 0, 0, 0, 0 };
                const bool sent = kbd_send(rpt);
                WAKE_DBG("KEY_UP_SENT: retrying F15 (attempt %d/%d) -> %d",
                         (int)key_attempts + 1, (int)WAKE_KEY_ATTEMPTS, (int)sent);
                if (sent) {
                    critical_section_enter_blocking(&wake_cs);
                    enter_state(WAKE_KEY_DOWN);
                    critical_section_exit(&wake_cs);
                }
            } else {
                WAKE_DBG("KEY_UP_SENT settle done -> DONE");
                critical_section_enter_blocking(&wake_cs);
                enter_state(WAKE_DONE);
                key_attempts = 0;
                critical_section_exit(&wake_cs);
            }
            return;
        }
    }
}

#endif // ENABLE_WAKE_HID
