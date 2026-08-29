//
// Wake the host over the network: join a Wi-Fi network and listen for a UDP
// trigger packet, then hand off to the wake FSM in wake.cpp.
//

#ifndef DS5_BRIDGE_WIFI_WAKE_H
#define DS5_BRIDGE_WIFI_WAKE_H

#ifdef ENABLE_WIFI_WAKE
// Called once after cyw43_arch_init(). Non-blocking: association is started
// asynchronously and driven from wifi_wake_task().
void wifi_wake_init(void);

// Call from the main loop. Never blocks - the firmware runs under a 1 s
// watchdog, so association is polled rather than waited on.
void wifi_wake_task(void);
#else
static inline void wifi_wake_init(void) {}
static inline void wifi_wake_task(void) {}
#endif

#endif //DS5_BRIDGE_WIFI_WAKE_H
