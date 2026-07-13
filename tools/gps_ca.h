#ifndef GPS_SDR_SIM_GPS_CA_H
#define GPS_SDR_SIM_GPS_CA_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GPS_CA_CHIPS 1023

int gps_ca_generate_binary(int prn, int *chips);
int gps_ca_generate_bipolar(int prn, int8_t *chips);

#ifdef __cplusplus
}
#endif

#endif
