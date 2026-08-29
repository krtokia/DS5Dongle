// The Wi-Fi credentials live in a fixed-layout block in flash rather than in
// ordinary string constants, so that a built .uf2 can be given credentials
// after the fact (see tools/uf2-wifi-config.html). That keeps the image
// produced by CI free of secrets even though the repository is public.
//
// Two details make the block patchable:
//
//   * it is `volatile`, so the compiler must emit real loads instead of
//     folding the compile-time initialiser into the code that reads it;
//   * every field is fixed-width, so the patcher only has to locate the magic
//     and overwrite a known number of bytes.

#include <stdbool.h>

#include "wifi_config.h"

#ifndef WIFI_SSID_DEFAULT
#define WIFI_SSID_DEFAULT ""
#endif

#ifndef WIFI_PASSWORD_DEFAULT
#define WIFI_PASSWORD_DEFAULT ""
#endif

// `const` places this in flash; `volatile` defeats constant propagation.
// It is referenced by wifi_config_load() below, so --gc-sections keeps it.
static const volatile wifi_config_t wifi_config = {
    .magic    = WIFI_CFG_MAGIC,   // exactly 16 chars, so no NUL is stored
    .version  = 1,
    .ssid     = WIFI_SSID_DEFAULT,
    .password = WIFI_PASSWORD_DEFAULT,
    .flags    = 0,
};

// Byte-at-a-time through a volatile pointer: this is what forces the values to
// actually be read back out of flash at runtime.
static void copy_field(char *dst, size_t dst_size,
                       const volatile char *src, size_t src_size) {
    size_t n = (dst_size - 1 < src_size) ? dst_size - 1 : src_size;
    size_t i;
    for (i = 0; i < n; i++) {
        char c = src[i];
        if (c == '\0') {
            break;
        }
        dst[i] = c;
    }
    dst[i] = '\0';
}

bool wifi_config_load(char *ssid, size_t ssid_size,
                      char *password, size_t password_size) {
    copy_field(ssid, ssid_size, wifi_config.ssid, WIFI_CFG_SSID_LEN);
    copy_field(password, password_size, wifi_config.password, WIFI_CFG_PASS_LEN);
    return ssid[0] != '\0';
}
