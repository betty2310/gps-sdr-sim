#ifndef X300_LIVE_TIME_HPP
#define X300_LIVE_TIME_HPP

#include "ubx_receiver.hpp"
#include "x300_timing.hpp"
#include "x300_pps_time.hpp"
#include <iomanip>
#include <sstream>

namespace x300 {
struct ClockObservation {
  double host_before = 0, hardware = 0, host_after = 0;
  double midpoint() const { return (host_before + host_after) / 2; }
  double width() const { return host_after - host_before; }
};
inline ClockObservation observeClock(Radio &radio) {
  ClockObservation best;
  bool found = false;
  // Query RTT bounds the host/hardware correlation ambiguity, not GPS error.
  // Use the narrowest of eight brackets to reduce host/network scheduling noise.
  for (unsigned n = 0; n < 8; ++n) {
    ClockObservation o;
    o.host_before = radio.monotonic();
    o.hardware = radio.now().get_real_secs();
    o.host_after = radio.monotonic();
    if (!std::isfinite(o.host_before) || !std::isfinite(o.host_after) ||
        !std::isfinite(o.hardware) || o.width() < 0 || o.width() > 0.05)
      continue;
    if (!found || o.width() < best.width()) {
      best = o;
      found = true;
    }
  }
  if (!found)
    throw std::runtime_error("UHD/host time query bracket exceeds 50 ms");
  return best;
}
struct LiveStartPlan {
  bool valid = false;
  double hardware_start = 0;
  gpstime_t gps_zero{-1, 0};
  gpstime_t gps_zero_before_model_offset{-1, 0};
  ClockObservation correlation;
  double delivery_delay = 0, tx_path_delay = 0, sky_path_delay = 0;
  double model_time_offset = 0;
  ubx::GpsNowEstimate estimate;
  ubx::TimeSolution anchor;
  uint64_t time_rejections = 0, clock_resets = 0;
  PpsReference pps;
};
// An effective delay measured against sky includes receiver, transport and RF
// effects. It is not a measurement of USB latency. Positive correction advances
// the model epoch of the SAME hardware sample zero; it never changes Fs, moves
// the deadline, or slews already-rendered samples.
inline gpstime_t offsetModelEpoch(gpstime_t epoch, double correction) {
  if (!std::isfinite(correction) || std::fabs(correction) > 1)
    throw std::runtime_error("model time offset must be finite and within +/-1 second");
  epoch.sec += correction;
  epoch = sampleTime(epoch, 0, 1);
  if (epoch.week < 0)
    throw std::runtime_error("model time offset precedes GPS epoch");
  return epoch;
}
inline LiveStartPlan planLiveStart(const ubx::Snapshot &s, ClockObservation o,
                                   double lead, double delivery = 0,
                                   double tx_path = 0, double sky_path = 0,
                                   double model_offset = 0) {
  if (s.replay)
    throw std::runtime_error("recorded UBX cannot supply fresh live time");
  auto problem = ubx::readiness(s, o.host_after, s.time.gps, {}, false);
  if (!problem.empty())
    throw std::runtime_error(problem);
  if (!std::isfinite(lead) || lead < 0.02 || lead > 60 ||
      !std::isfinite(delivery) || delivery < 0 || delivery > 10 ||
      !std::isfinite(tx_path) || tx_path < 0 || tx_path > 1 ||
      !std::isfinite(sky_path) || sky_path < 0 || sky_path > 1 ||
      !std::isfinite(o.hardware) || !std::isfinite(o.host_before) ||
      !std::isfinite(o.host_after) || !std::isfinite(o.width()) ||
      o.width() < 0 || o.width() > 0.05)
    throw std::runtime_error("invalid live time estimate inputs");
  LiveStartPlan p;
  p.hardware_start = std::ceil(o.hardware + o.width() / 2 + lead);
  p.correlation = o;
  p.delivery_delay = delivery;
  p.tx_path_delay = tx_path;
  p.sky_path_delay = sky_path;
  p.model_time_offset = model_offset;
  p.anchor = s.time;
  p.time_rejections = s.time_rejections;
  p.clock_resets = s.clock_resets;
  p.estimate = ubx::estimateGpsNow(s, o.host_after, delivery);
  const double hardware_at_query = o.hardware + o.width() / 2;
  p.gps_zero = p.estimate.gps;
  p.gps_zero.sec += p.hardware_start - hardware_at_query + tx_path - sky_path;
  p.gps_zero = sampleTime(p.gps_zero, 0, 1);
  p.gps_zero_before_model_offset = p.gps_zero;
  p.gps_zero = offsetModelEpoch(p.gps_zero, model_offset);
  p.valid = true;
  return p;
}
inline void checkLiveHealth(const ubx::Snapshot &s, const LiveStartPlan &p,
                            double now, gpstime_t epoch,
                            const std::vector<int> &prns,
                            bool require_navigation = true) {
  auto problem = ubx::readiness(s, now, epoch, prns, require_navigation);
  if (!problem.empty())
    throw std::runtime_error(problem);
  if (s.time_rejections != p.time_rejections ||
      s.clock_resets != p.clock_resets)
    throw std::runtime_error(
        "receiver time discontinuity/reset; new segment required");
  if (p.pps.verified) {
    checkPpsMetadata(s, now);
    if (!samePulseConfig(p.pps.config, s.pulse_config) ||
        s.time.leap_seconds != p.pps.leap_seconds ||
        !s.time_pulse.present || now < s.time_pulse.received ||
        now - s.time_pulse.received > 1.5)
      throw std::runtime_error("PPS metadata changed or pulse-label stream stopped");
    return;
  }
  double dt = subGpsTime(s.time.gps, p.anchor.gps);
  if (std::fabs(dt - (s.time.received - p.anchor.received)) > 0.25)
    throw std::runtime_error(
        "receiver time/arrival timeline departed from frozen start estimate");
}
inline LiveStartPlan planPpsStart(const ubx::Snapshot &s, ClockObservation o,
                                 const PpsReference &reference, double lead,
                                 double tx_path = 0, double sky_path = 0) {
  if (!reference.verified || s.replay || !std::isfinite(lead) ||
      lead < .02 || lead > 60 || !std::isfinite(tx_path) || tx_path < 0 ||
      tx_path > 1 || !std::isfinite(sky_path) || sky_path < 0 || sky_path > 1 ||
      !std::isfinite(o.hardware) || !std::isfinite(o.host_before) ||
      !std::isfinite(o.host_after) || o.width() < 0 || o.width() > .05 ||
      o.hardware - reference.last_confirmed_hardware > 1.5 ||
      o.hardware < reference.last_confirmed_hardware)
    throw std::runtime_error("invalid or stale PPS-associated start inputs");
  LiveStartPlan p;
  p.pps = reference;
  p.correlation = o;
  p.hardware_start = std::ceil(o.hardware + o.width() / 2 + lead);
  p.gps_zero = reference.gps;
  p.gps_zero.sec += p.hardware_start - reference.hardware + tx_path - sky_path;
  p.gps_zero = sampleTime(p.gps_zero, 0, 1);
  p.gps_zero_before_model_offset = p.gps_zero;
  p.tx_path_delay = tx_path;
  p.sky_path_delay = sky_path;
  p.anchor = s.time;
  p.time_rejections = s.time_rejections;
  p.clock_resets = s.clock_resets;
  checkLiveHealth(s, p, o.host_after, s.time.gps, {}, false);
  const double event = s.leap_info.seconds_to_event - (o.host_after - s.leap_info.received);
  if (event > 0 && event <= p.hardware_start - o.hardware + 10)
    throw std::runtime_error("planned start crosses leap-second guard; reacquire afterwards");
  p.valid = true;
  return p;
}
inline std::string livePlanJson(const LiveStartPlan &p) {
  std::ostringstream out;
  out << std::setprecision(17) << "{\"valid\":" << (p.valid ? "true" : "false")
      << ",\"delivery_delay_estimate_s\":" << p.delivery_delay
      << ",\"tx_path_delay_s\":" << p.tx_path_delay
      << ",\"sky_path_delay_s\":" << p.sky_path_delay
      << ",\"model_time_offset_s\":" << p.model_time_offset
      << ",\"gps_zero_before_model_offset\":{\"week\":"
      << p.gps_zero_before_model_offset.week << ",\"tow\":"
      << p.gps_zero_before_model_offset.sec << '}'
      << ",\"model_offset_changes_hardware_deadline\":false"
      << ",\"delays_calibrated\":false,\"absolute_gps_uncertainty_s\":null"
      << ",\"arrival_variation_s\":" << p.estimate.arrival_variation
      << ",\"time_estimate\":" << (p.pps.verified ? "null" : ubx::timeEstimateJson(p.estimate))
      << ",\"pps_epoch_association\":" << ppsReferenceJson(p.pps)
      << ",\"hardware_query\":{\"host_before_s\":" << p.correlation.host_before
      << ",\"hardware_s\":" << p.correlation.hardware
      << ",\"host_after_s\":" << p.correlation.host_after
      << ",\"bracket_width_s\":" << p.correlation.width() << '}'
      << ",\"hardware_query_half_width_s\":" << p.correlation.width() / 2
      << ",\"anchor_gps_week\":" << p.anchor.gps.week
      << ",\"anchor_gps_tow\":" << p.anchor.gps.sec
      << ",\"anchor_host_received_s\":" << p.anchor.received
      << ",\"receiver_tacc_ns\":" << p.anchor.accuracy_ns << '}';
  return out.str();
}
} // namespace x300
#endif
