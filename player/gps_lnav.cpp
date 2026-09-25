#include "gps_lnav.hpp"
#include <algorithm>
#include <cmath>

namespace ubx {
namespace {
// Bit positions in the 240 data bits (ten 24-bit words, parity removed),
// from IS-GPS-200N figures 20-1/20-2/20-3. No host-calendar week inference.
uint32_t bits(const std::array<uint32_t, 10> &words, unsigned pos, unsigned n) {
  uint32_t result = 0;
  for (unsigned i = pos; i < pos + n; ++i)
    result = (result << 1) | ((words[i / 24] >> (29 - i % 24)) & 1);
  return result;
}
double scaled(const std::array<uint32_t, 10> &w, unsigned p, unsigned n,
              int exponent, bool sign = true) {
  uint32_t raw = bits(w, p, n);
  int64_t value = raw;
  if (sign && (raw & (uint32_t(1) << (n - 1))))
    value -= int64_t(1) << n;
  return std::ldexp(static_cast<double>(value), exponent);
}
gpstime_t nearWeek(int week, double tow, gpstime_t reference) {
  gpstime_t result{week, tow};
  const double difference = subGpsTime(result, reference);
  if (difference > 302400)
    --result.week;
  if (difference < -302400)
    ++result.week;
  return result;
}
} // namespace

bool ingestLnav(Navigation &nav, const std::array<uint32_t, 10> &words,
                gpstime_t time, double received) {
  const unsigned sf = bits(words, 43, 3);
  const unsigned how = bits(words, 24, 17);
  if (bits(words, 0, 8) != 0x8b || sf < 1 || sf > 5 || how > 100799 ||
      time.week < 0 || !std::isfinite(time.sec) || !std::isfinite(received))
    return false;
  nav.words[sf - 1] = words;
  nav.received[sf - 1] = received;
  nav.how[sf - 1] = how;
  nav.coverage |= 1u << (sf - 1);
  if (sf > 3)
    return true;
  nav.complete = false;
  if ((nav.coverage & 7) != 7)
    return true;
  const auto &a = nav.words[0], &b = nav.words[1], &c = nav.words[2];
  const int iodc = (bits(a, 70, 2) << 8) | bits(a, 168, 8);
  const int iode = bits(b, 48, 8);
  if (iode != (iodc & 255) || iode != static_cast<int>(bits(c, 216, 8)))
    return true;
  double earliest =
      *std::min_element(nav.received.begin(), nav.received.begin() + 3);
  double latest =
      *std::max_element(nav.received.begin(), nav.received.begin() + 3);
  if (latest - earliest > 90)
    return true;
  int week = (time.week / 1024) * 1024 + bits(a, 48, 10);
  if (week - time.week > 512)
    week -= 1024;
  if (time.week - week > 512)
    week += 1024;
  if (std::abs(week - time.week) > 1)
    return true;
  // The SF1 week labels its broadcast transmission epoch. A one-week-old
  // packet with an otherwise similar TOE must not be shifted into this week
  // and appear fresh. Allow the short cache span across a genuine rollover.
  gpstime_t sf1_epoch{week, nav.how[0] * 6.0};
  if (std::fabs(subGpsTime(sf1_epoch, time)) > 96)
    return true;
  ephem_t e{};
  e.iodc = iodc;
  e.iode = iode;
  e.codeL2 = bits(a, 58, 2);
  e.svhlth = bits(a, 64, 6);
  nav.ura = bits(a, 60, 4);
  nav.extended_fit = bits(b, 232, 1);
  e.tgd = scaled(a, 160, 8, -31);
  // -128 is the unavailable TGD sentinel, never silently treated as zero.
  if (bits(a, 160, 8) == 128)
    return true;
  e.toc = nearWeek(week, bits(a, 176, 16) * 16.0, time);
  e.af2 = scaled(a, 192, 8, -55);
  e.af1 = scaled(a, 200, 16, -43);
  e.af0 = scaled(a, 216, 22, -31);
  e.crs = scaled(b, 56, 16, -5);
  e.deltan = scaled(b, 72, 16, -43) * PI;
  e.m0 = scaled(b, 88, 32, -31) * PI;
  e.cuc = scaled(b, 120, 16, -29);
  e.ecc = scaled(b, 136, 32, -33, false);
  e.cus = scaled(b, 168, 16, -29);
  e.sqrta = scaled(b, 184, 32, -19, false);
  e.toe = nearWeek(week, bits(b, 216, 16) * 16.0, time);
  e.cic = scaled(c, 48, 16, -29);
  e.omg0 = scaled(c, 64, 32, -31) * PI;
  e.cis = scaled(c, 96, 16, -29);
  e.inc0 = scaled(c, 112, 32, -31) * PI;
  e.crc = scaled(c, 144, 16, -5);
  e.aop = scaled(c, 160, 32, -31) * PI;
  e.omgdot = scaled(c, 192, 24, -43) * PI;
  e.idot = scaled(c, 224, 14, -43) * PI;
  if (e.toe.sec >= SECONDS_IN_WEEK || e.toc.sec >= SECONDS_IN_WEEK ||
      e.sqrta < 4000 || e.sqrta > 6000 || e.ecc >= 1)
    return true;
  e.A = e.sqrta * e.sqrta;
  e.n = std::sqrt(GM_EARTH / (e.A * e.A * e.A)) + e.deltan;
  e.sq1e2 = std::sqrt(1 - e.ecc * e.ecc);
  e.omgkdot = e.omgdot - OMEGA_EARTH;
  gps2date(&e.toc, &e.t);
  e.vflg = 1;
  nav.eph = e;
  nav.complete = true;
  return true;
}

bool usableNavigation(const Navigation &nav, gpstime_t epoch, double now,
                      double max_receive_age) {
  if (!nav.complete || nav.eph.svhlth || nav.ura == 15)
    return false;
  for (unsigned n = 0; n < 3; ++n)
    if (now < nav.received[n] || now - nav.received[n] > max_receive_age)
      return false;
  // Conservative four-hour fit window even when the broadcast allows longer.
  return std::fabs(subGpsTime(epoch, nav.eph.toe)) <= 7200 &&
         std::fabs(subGpsTime(epoch, nav.eph.toc)) <= 7200;
}
} // namespace ubx
