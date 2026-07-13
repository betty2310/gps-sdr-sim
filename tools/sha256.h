#ifndef GPS_SDR_SIM_SHA256_H
#define GPS_SDR_SIM_SHA256_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SHA256_HEX_SIZE 65

void sha256_bytes_hex(const void *data, size_t size,
                      char output[SHA256_HEX_SIZE]);
int sha256_file_hex(const char *path, char output[SHA256_HEX_SIZE]);

#ifdef __cplusplus
}
#endif

#endif
