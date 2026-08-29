//
// Wake the host over the network: join a Wi-Fi network and listen for a UDP
// trigger packet, then hand off to the wake FSM in wake.cpp.
//

#ifndef DS5_BRIDGE_WIFI_WAKE_H
#define DS5_BRIDGE_WIFI_WAKE_H

#include <cstdint>

#ifdef ENABLE_WIFI_WAKE
// Called once after cyw43_arch_init(). Non-blocking: association is started
// asynchronously and driven from wifi_wake_task().
void wifi_wake_init(void);

// Call from the main loop. Never blocks - the firmware runs under a 1 s
// watchdog, so association is polled rather than waited on.
void wifi_wake_task(void);

// Called for every input report from the controller. Reports arrive constantly
// whether or not anyone is holding it, so this looks at what changed rather
// than at the fact that one arrived.
void wifi_wake_note_bt_input(const uint8_t *hid_input, uint16_t len);

// Drives the onboard LED. Call last in the main loop: while a pattern is
// playing it takes the LED from the other users, and the rest of the time it
// leaves it alone. This is the only way to see what the firmware is doing
// while the host is asleep, since the USB console dies with it.
void wifi_wake_led_tick(void);
#else
static inline void wifi_wake_init(void) {}
static inline void wifi_wake_task(void) {}
static inline void wifi_wake_note_bt_input(const uint8_t *, uint16_t) {}
static inline void wifi_wake_led_tick(void) {}
#endif

#endif //DS5_BRIDGE_WIFI_WAKE_H
