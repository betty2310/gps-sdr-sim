#ifndef RTCM3_NAV_HPP
#define RTCM3_NAV_HPP

#include <cstddef>
#include <cstdint>

extern "C" {
#include "gpssim.h"
}

#define RTCM3_NAV_ERR_SIZE 256

typedef struct {
  const char *host;
  int port;
  int timeout_ms;
  const char *mount_point;
  const char *credentials;
} rtcm3_nav_options_t;

typedef struct {
  int fd;
  std::uint8_t scanner_buf[16384];
  std::size_t scanner_len;
  std::uint64_t crc_errs;
  std::uint64_t bytes_dropped;
  ephem_t gps_rtcm_eph[MAX_SAT];
  int valid[MAX_SAT];
  int valid_count;
  int observed[MAX_SAT];
  int observed_count;
  double last_obs_tow_s;
  unsigned int msg1004_count;
} rtcm3_nav_stream_t;

void rtcm3_nav_init(rtcm3_nav_stream_t *stream);
void rtcm3_nav_close(rtcm3_nav_stream_t *stream);
int rtcm3_nav_open(rtcm3_nav_stream_t *stream, const rtcm3_nav_options_t *opt,
                   char *err, std::size_t err_len);
int rtcm3_nav_pump(rtcm3_nav_stream_t *stream, int timeout_ms, int *updated,
                   char *err, std::size_t err_len);
void rtcm3_nav_copy_ephemeris(const rtcm3_nav_stream_t *stream, ephem_t *dst);
int rtcm3_nav_has_prn(const rtcm3_nav_stream_t *stream, int prn);
int rtcm3_nav_valid_prns(const rtcm3_nav_stream_t *stream);
int rtcm3_nav_has_observation_prn(const rtcm3_nav_stream_t *stream, int prn);
int rtcm3_nav_observed_prns(const rtcm3_nav_stream_t *stream);
double rtcm3_nav_last_obs_tow(const rtcm3_nav_stream_t *stream);
void rtcm3_nav_copy_observed(const rtcm3_nav_stream_t *stream, int *dst);

#endif
