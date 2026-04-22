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

static int expect_near(double got, double want, double tol, const char *msg) {
  if (fabs(got - want) > tol) {
    fprintf(stderr, "FAIL: %s got %.17g want %.17g\n", msg, got, want);
    return 1;
  }
  return 0;
}

int main(void) {
  ephem_t templ;
  ephem_t revived;
  gpstime_t t_now;
  double delta = 7205.0;
  double tk_at_now;
  double toe_delta_sec;
  double pos_old[3], vel_old[3], clk_old[2];
  double pos_new[3], vel_new[3], clk_new[2];
  double diff[3];
  int failures = 0;

  memset(&templ, 0, sizeof(templ));
  templ.vflg = 1;
  templ.toe.week = 2415;
  templ.toe.sec = 266400.0;
  templ.toc = templ.toe;
  templ.sqrta = sqrt(GPS_ORBIT_RADIUS);
  templ.ecc = 0.01;
  templ.inc0 = GPS_INCLINATION;
  templ.m0 = 0.25;
  templ.aop = -0.4;
  templ.omg0 = 1.1;
  templ.omgdot = -8.0e-9;
  templ.idot = 1.0e-10;
  templ.deltan = 2.0e-9;
  templ.cuc = 1.0e-6;
  templ.cus = -2.0e-6;
  templ.cic = 3.0e-7;
  templ.cis = -4.0e-7;
  templ.crc = 120.0;
  templ.crs = -90.0;
  templ.af0 = 2.0e-5;
  templ.af1 = -1.0e-12;
  templ.af2 = 2.0e-18;
  templ.tgd = -6.0e-9;
  templ.iode = 17;
  templ.iodc = 17;
  templ.A = templ.sqrta * templ.sqrta;
  templ.n = sqrt(GM_EARTH / (templ.A * templ.A * templ.A)) + templ.deltan;
  templ.sq1e2 = sqrt(1.0 - templ.ecc * templ.ecc);
  templ.omgkdot = templ.omgdot - OMEGA_EARTH;

  t_now = incGpsTime(templ.toe, delta);
  reviveEphemerisFromTemplate(&revived, &templ, delta, t_now);
  tk_at_now = subGpsTime(t_now, revived.toe);
  toe_delta_sec = subGpsTime(revived.toe, templ.toe);

  failures += expect_near(revived.toe.sec, 273600.0, 1.0e-9,
                          "toe is retimed to broadcast quantum");
  failures += expect_near(revived.toc.sec, revived.toe.sec, 1.0e-9,
                          "toc equals toe");
  failures += expect_near(revived.omg0,
                          wrapToPi(templ.omg0 +
                                   OMEGA_EARTH * toe_delta_sec -
                                   templ.omgkdot * tk_at_now),
                          1.0e-10, "Omega0 compensated");
  failures += expect_near(revived.af0,
                          templ.af0 + templ.af1 * delta +
                              0.5 * templ.af2 * delta * delta,
                          1.0e-15, "af0 propagated");
  failures += expect_true(revived.iode == ((templ.iode + 2) & 0xFF),
                          "iode bumped by elapsed 2h cycles");
  failures += expect_near(revived.m0, wrapToPi(templ.m0 - templ.n * tk_at_now),
                          1.0e-12, "M0 compensates TOE rounding");
  failures += expect_true(revived.sqrta == templ.sqrta, "sqrtA unchanged");
  failures += expect_true(revived.ecc == templ.ecc, "ecc unchanged");
  failures += expect_near(revived.inc0, templ.inc0 - templ.idot * tk_at_now,
                          1.0e-15, "inc0 compensates TOE rounding");

  satpos(templ, templ.toe, pos_old, vel_old, clk_old);
  satpos(revived, t_now, pos_new, vel_new, clk_new);
  subVect(diff, pos_new, pos_old);
  failures += expect_true(normVect(diff) < 1.0,
                          "revived position matches template position");

  return failures == 0 ? 0 : 1;
}
