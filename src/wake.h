//
// Created by awalol on 2026/4/30.
//

#ifndef DS5_BRIDGE_WAKE_H
#define DS5_BRIDGE_WAKE_H

#include <cstdint>

#ifdef ENABLE_WAKE_HID
void wake_init(void);
void wake_on_bt_connect(void);
void wake_on_bt_input(const uint8_t *hid_input, uint16_t len);
void wake_on_bt_disconnect(void);
void wake_task(void);
void wake_note_usb_reconnect(void);
void wake_request_from_network(void);
// Whether the USB host has suspended the bus, i.e. the PC is asleep.
bool wake_host_is_suspended(void);
// How many times the host has suspended and resumed the bus since boot. These
// survive the USB link dropping, so they can be read back after the host wakes
// to find out whether it ever suspended at all.
uint32_t wake_suspend_count(void);
uint32_t wake_resume_count(void);
#else
static inline void wake_init(void) {}
static inline void wake_on_bt_connect(void) {}
static inline void wake_on_bt_input(const uint8_t *, uint16_t) {}
static inline void wake_on_bt_disconnect(void) {}
static inline void wake_task(void) {}
static inline void wake_note_usb_reconnect(void) {}
static inline void wake_request_from_network(void) {}
static inline bool wake_host_is_suspended(void) { return false; }
static inline uint32_t wake_suspend_count(void) { return 0; }
static inline uint32_t wake_resume_count(void) { return 0; }
#endif

#endif //DS5_BRIDGE_WAKE_H
