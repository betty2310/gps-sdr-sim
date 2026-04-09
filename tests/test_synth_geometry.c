#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "gpssim.h"

static double positionError(const double *a, const double *b) {
  double diff[3];

  subVect(diff, a, b);
  return normVect(diff);
}

static void expectSyntheticMatch(double lat_deg, double lon_deg, double h_m,
                                 double az_deg, double el_deg,
                                 double max_error_m) {
  double llh[3] = {lat_deg * D2R, lon_deg * D2R, h_m};
  double rx_xyz[3];
  double target_sat[3];
  double recon_sat[3];
  double vel[3], clk[2];
  double error_m;
  ephem_t eph;
  gpstime_t toe = {.week = 2412, .sec = 372832.0};

  llh2xyz(llh, rx_xyz);
  azel2satpos(rx_xyz, az_deg * D2R, el_deg * D2R, target_sat);
  synthEphemeris(&eph, rx_xyz, az_deg * D2R, el_deg * D2R, toe, toe);
  satpos(eph, toe, recon_sat, vel, clk);

  error_m = positionError(recon_sat, target_sat);
  if (error_m > max_error_m) {
    fprintf(stderr,
            "Synthetic geometry mismatch for az=%.1f el=%.1f: %.3f m > %.3f m\n",
            az_deg, el_deg, error_m, max_error_m);
    exit(1);
  }
}

int main(void) {
  double llh[3] = {21.0047844 * D2R, 105.8460541 * D2R, 5.0};
  double rx_xyz[3];
  double sat_ecef[3];

  expectSyntheticMatch(21.0047844, 105.8460541, 5.0, 20.0, 60.0, 1.0);
  expectSyntheticMatch(21.0047844, 105.8460541, 5.0, 90.0, 45.0, 1.0);
  expectSyntheticMatch(21.0047844, 105.8460541, 5.0, 30.0, 45.0, 1.0);

  llh2xyz(llh, rx_xyz);
  if (synthAzelReachable(rx_xyz, 27.0 * D2R, 35.0 * D2R, sat_ecef) != FALSE) {
    fprintf(stderr,
            "Expected az=27.0 el=35.0 to be outside the synthetic orbit envelope\n");
    exit(1);
  }

  return (0);
}
