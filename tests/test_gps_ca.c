#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "tools/gps_ca.h"

static void assert_prefix(int prn, const char *expected) {
  int chips[GPS_CA_CHIPS];
  int8_t bipolar[GPS_CA_CHIPS];
  size_t i;

  assert(gps_ca_generate_binary(prn, chips) == 0);
  assert(gps_ca_generate_bipolar(prn, bipolar) == 0);
  for (i = 0; i < strlen(expected); ++i) {
    assert(chips[i] == expected[i] - '0');
    assert(bipolar[i] == (int8_t)(chips[i] * 2 - 1));
  }
}

int main(void) {
  int chips[GPS_CA_CHIPS];

  assert_prefix(1, "11001000001110010100100111100101");
  assert_prefix(2, "11100100001110000011111010011001");
  assert_prefix(10, "11010001001010001001110000110110");
  assert_prefix(32, "11110010100000110011101100011001");
  assert(gps_ca_generate_binary(0, chips) != 0);
  assert(gps_ca_generate_binary(33, chips) != 0);
  return 0;
}
