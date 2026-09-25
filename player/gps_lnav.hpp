#ifndef GPS_LNAV_HPP
#define GPS_LNAV_HPP

#include <array>
#include <cstdint>
extern "C" {
#include "gpssim.h"
}

namespace ubx {

struct Navigation {
  ephem_t eph{};
  bool complete = false;
  int ura = 15;
  bool extended_fit = false;
  unsigned coverage = 0;
  std::array<std::array<uint32_t, 10>, 5> words{};
  std::array<double, 5> received{};
  std::array<unsigned, 5> how{};
};

// SFRBX GPS L1 C/A words have already had data inversion corrected by F9P.
// Keep their parity bits as received; do not treat them as a timed RF
// bitstream.
bool ingestLnav(Navigation &nav, const std::array<uint32_t, 10> &words,
                gpstime_t receiver_time, double received);
bool usableNavigation(const Navigation &nav, gpstime_t epoch, double now,
                      double max_receive_age = 90.0);

} // namespace ubx
#endif
