#include "gpssim.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int expect_true(int cond, const char *msg) {
  if (!cond) {
    fprintf(stderr, "FAIL: %s\n", msg);
    return 1;
  }
  return 0;
}

int main(void) {
  ephem_t eph[EPHEM_ARRAY_SIZE][MAX_SAT];
  ephem_t picked;
  gpstime_t t_now;
  gpstime_t template_toe;
  double rx_llh[3];
  double rx_xyz[3];
  double delta_sec;
  double elev_deg;
  int found_ephem = FALSE;
  int failures = 0;

  memset(eph, 0, sizeof(eph));

  rx_llh[0] = 21.004458 / R2D;
  rx_llh[1] = 105.846200 / R2D;
  rx_llh[2] = 22.0;
  llh2xyz(rx_llh, rx_xyz);

  t_now.week = 2415;
  t_now.sec = 273600.0;
  template_toe = incGpsTime(t_now, -SYNTH_REVIVE_DEFAULT_LOOKBACK_SEC);

  synthEphemeris(&eph[0][0], rx_xyz, 0.0, PI / 2.0, template_toe,
                 template_toe);

  failures += expect_true(
      scanEphemerisForRevive(eph, EPHEM_ARRAY_SIZE, 1, t_now, rx_xyz, &picked,
                             &template_toe, &delta_sec, &elev_deg,
                             &found_ephem) == TRUE,
      "scanner finds overhead revive template");
  failures += expect_true(found_ephem == TRUE, "scanner saw ephemeris");
  failures += expect_true(fabs(delta_sec - SYNTH_REVIVE_DEFAULT_LOOKBACK_SEC) <
                              1.0e-6,
                          "scanner picks default lookback");
  failures += expect_true(elev_deg >= SYNTH_REVIVE_MIN_ELEVATION_DEG,
                          "scanner elevation accepted");
  failures += expect_true(picked.vflg == 1, "scanner returns template");

  return failures == 0 ? 0 : 1;
}
