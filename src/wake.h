//
// Created by awalol on 2026/4/30.
//

#ifndef DS5_BRIDGE_WAKE_H
#define DS5_BRIDGE_WAKE_H

#include <cstddef>
#include <cstdint>

#ifdef ENABLE_WAKE_HID
void wake_init(void);
void wake_on_bt_connect(void);
void wake_on_bt_input(const uint8_t *hid_input, uint16_t len);
void wake_on_bt_disconnect(void);
void wake_task(void);
void wake_note_usb_reconnect(void);
// What a network trigger managed to do, so the caller can report it and retry.
typedef enum {
    WAKE_NET_STARTED = 0,     // the keystroke sequence is under way
    WAKE_NET_DISABLED,        // wake is off in the config; nothing to send on
    WAKE_NET_NOT_ATTACHED,    // not on the USB bus yet; an attach was started
} wake_net_result_t;
wake_net_result_t wake_request_from_network(void);
// Whether the USB host has suspended the bus, i.e. the PC is asleep.
bool wake_host_is_suspended(void);
// How many times the host has suspended and resumed the bus since boot. These
// survive the USB link dropping, so they can be read back after the host wakes
// to find out whether it ever suspended at all.
uint32_t wake_suspend_count(void);
uint32_t wake_resume_count(void);
// Keydowns actually accepted by the USB stack, as opposed to wake attempts
// started. The gap between the two is where a wake can silently fail.
uint32_t wake_key_sent_count(void);
// Why a keystroke is or is not going out: the three conditions behind
// tud_hid_n_ready() for the wake keyboard, and where the FSM is sitting.
bool wake_kbd_ready(void);
bool wake_kbd_endpoint_open(void);
void wake_hid_summary(char *buf, size_t len);
const char *wake_state_str(void);
// Forces a clean re-enumeration. Recovers a wake keyboard endpoint left busy
// by a transfer the host never collected.
void wake_usb_reconnect(void);
#else
static inline void wake_init(void) {}
static inline void wake_on_bt_connect(void) {}
static inline void wake_on_bt_input(const uint8_t *, uint16_t) {}
static inline void wake_on_bt_disconnect(void) {}
static inline void wake_task(void) {}
static inline void wake_note_usb_reconnect(void) {}
typedef enum { WAKE_NET_STARTED = 0, WAKE_NET_DISABLED, WAKE_NET_NOT_ATTACHED } wake_net_result_t;
static inline wake_net_result_t wake_request_from_network(void) { return WAKE_NET_DISABLED; }
static inline bool wake_host_is_suspended(void) { return false; }
static inline uint32_t wake_suspend_count(void) { return 0; }
static inline uint32_t wake_resume_count(void) { return 0; }
static inline uint32_t wake_key_sent_count(void) { return 0; }
static inline bool wake_kbd_ready(void) { return false; }
static inline bool wake_kbd_endpoint_open(void) { return false; }
static inline void wake_hid_summary(char *buf, size_t len) { if (len) buf[0] = 0; }
static inline const char *wake_state_str(void) { return "n/a"; }
static inline void wake_usb_reconnect(void) {}
#endif

#endif //DS5_BRIDGE_WAKE_H
