#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

// Size of the character fields inside the on-flash configuration block.
// 32 bytes is the maximum length of an SSID, 63 that of a WPA2 passphrase;
// both fields are padded out so the block layout stays trivially parseable
// by tools/uf2-wifi-config.html.
#define WIFI_CFG_MAGIC_LEN 16
#define WIFI_CFG_SSID_LEN  64
#define WIFI_CFG_PASS_LEN  64

// The magic that marks the start of the block inside the firmware image.
// Kept in one place so the C code and the JS patcher cannot drift apart.
#define WIFI_CFG_MAGIC "PICO2W-WIFI-CFG1"

// Layout of the block as it appears in flash (and therefore in the .uf2):
//
//   offset  size  field
//   ------  ----  -----------------------------------------------------
//        0    16  magic     "PICO2W-WIFI-CFG1", no NUL terminator
//       16     4  version   little-endian uint32, currently 1
//       20    64  ssid      NUL-padded UTF-8
//       84    64  password  NUL-padded UTF-8
//      148     4  flags     little-endian uint32, reserved, currently 0
//   ------  ----
//      152  total
//
// Everything is fixed-width and byte-aligned so that patching the block is a
// straight memcpy over a region located by searching for the magic.
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    char     magic[WIFI_CFG_MAGIC_LEN];
    uint32_t version;
    char     ssid[WIFI_CFG_SSID_LEN];
    char     password[WIFI_CFG_PASS_LEN];
    uint32_t flags;
} wifi_config_t;

// Copies the credentials out of flash into caller-owned buffers, always
// NUL-terminating. Returns false if no SSID has been configured, in which
// case the firmware cannot join a network.
bool wifi_config_load(char *ssid, size_t ssid_size, char *password, size_t password_size);

#ifdef __cplusplus
}
#endif

#endif // WIFI_CONFIG_H
