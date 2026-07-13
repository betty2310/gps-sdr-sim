#include "gps_ca.h"

#include <stddef.h>

static const int delays[32] = {
    5,   6,   7,   8,   17,  18,  139, 140, 141, 251, 252,
    254, 255, 256, 257, 258, 469, 470, 471, 472, 473, 474,
    509, 512, 513, 514, 515, 516, 859, 860, 861, 862};

int gps_ca_generate_binary(int prn, int *chips) {
  int g1[GPS_CA_CHIPS];
  int g2[GPS_CA_CHIPS];
  int r1[10];
  int r2[10];
  int i;
  int j;

  if (chips == NULL || prn < 1 || prn > 32)
    return -1;

  for (i = 0; i < 10; ++i)
    r1[i] = r2[i] = -1;

  for (i = 0; i < GPS_CA_CHIPS; ++i) {
    int c1;
    int c2;

    g1[i] = r1[9];
    g2[i] = r2[9];
    c1 = r1[2] * r1[9];
    c2 = r2[1] * r2[2] * r2[5] * r2[7] * r2[8] * r2[9];
    for (j = 9; j > 0; --j) {
      r1[j] = r1[j - 1];
      r2[j] = r2[j - 1];
    }
    r1[0] = c1;
    r2[0] = c2;
  }

  j = GPS_CA_CHIPS - delays[prn - 1];
  for (i = 0; i < GPS_CA_CHIPS; ++i, ++j)
    chips[i] = (1 - g1[i] * g2[j % GPS_CA_CHIPS]) / 2;
  return 0;
}

int gps_ca_generate_bipolar(int prn, int8_t *chips) {
  int binary[GPS_CA_CHIPS];
  int i;

  if (chips == NULL || gps_ca_generate_binary(prn, binary) != 0)
    return -1;
  for (i = 0; i < GPS_CA_CHIPS; ++i)
    chips[i] = (int8_t)(binary[i] * 2 - 1);
  return 0;
}
