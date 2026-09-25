#ifndef X300_TIMING_HPP
#define X300_TIMING_HPP

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <uhd/types/metadata.hpp>
#include <vector>

extern "C" {
#include "gpssim.h"
}

namespace x300 {

// Scenario time is assigned to sample zero. A local PPS supplies no GPS label.
inline gpstime_t sampleTime(gpstime_t zero, uint64_t samples, double rate) {
  if (!std::isfinite(rate) || rate <= 0 || !std::isfinite(zero.sec))
    throw std::invalid_argument("invalid sample timeline");
  long double seconds = zero.sec + static_cast<long double>(samples) / rate;
  long double weeks = floorl(seconds / SECONDS_IN_WEEK);
  if (weeks + zero.week > std::numeric_limits<int>::max() ||
      weeks + zero.week < std::numeric_limits<int>::min())
    throw std::overflow_error("GPS week overflow");
  zero.week += static_cast<int>(weeks);
  zero.sec = static_cast<double>(seconds - weeks * SECONDS_IN_WEEK);
  if (zero.sec >= SECONDS_IN_WEEK) {
    zero.sec = 0;
    ++zero.week;
  }
  return zero;
}

inline int64_t navFrame(gpstime_t time) {
  time = sampleTime(time, 0, 1);
  return static_cast<int64_t>(time.week) * 20160 +
         static_cast<int64_t>(floor(time.sec / 30.0));
}

// Refresh navigation before a render block can exhaust the last subframe.
// The boundary may fall between samples: include at most one sample past it.
inline uint64_t capAtNavBoundary(gpstime_t begin, double rate, uint64_t count) {
  begin = sampleTime(begin, 0, rate);
  long double seconds = 30.0L - fmodl(begin.sec, 30.0L);
  long double samples = ceill(seconds * rate);
  return samples < count ? static_cast<uint64_t>(samples) : count;
}

// This boundary lets tests inject PPS, transport faults, and short sends
// without discovering a device. Only the sender thread accesses the radio.
class Radio {
public:
  virtual ~Radio() = default;
  virtual uhd::time_spec_t now() = 0;
  virtual uhd::time_spec_t lastPps() = 0;
  virtual void latchNextPps() = 0;
  virtual void checkLocks() = 0;
  virtual double monotonic() = 0;
  // Coarse independent second-label check only; never a fractional RF clock.
  virtual double utcNow() {
    return std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
  }
  virtual void sleep(double seconds) = 0;
  virtual size_t send(const int16_t *iq, size_t count,
                      const uhd::tx_metadata_t &metadata, double timeout) = 0;
  virtual bool receiveEvent(uhd::async_metadata_t &event, double timeout) = 0;
};

struct SyncState {
  bool verified = false;
  double last_pps_seconds = 0;
  double last_pps_observed_at = 0;
  double last_hardware_seconds = 0;
  double last_checked_at = 0;
};

inline SyncState synchronize(Radio &radio, double timeout = 5.0) {
  radio.checkLocks();
  const auto before = radio.lastPps();
  const double deadline = radio.monotonic() + timeout;
  while (radio.lastPps() == before) {
    if (radio.monotonic() >= deadline)
      throw std::runtime_error("no PPS edge detected before sync timeout");
    radio.sleep(0.005);
  }
  // Just after an observed edge, arm the following edge with a full second
  // of margin. Wait by monotonic time: old and newly latched PPS can both be 0.
  radio.latchNextPps();
  radio.sleep(1.05);
  const double latched = radio.lastPps().get_real_secs();
  const double current = radio.now().get_real_secs();
  if (fabs(latched) > 1.0e-6 || current < 0 || current > 0.5)
    throw std::runtime_error("PPS time-zero latch could not be verified");
  const double next_deadline = radio.monotonic() + timeout;
  while (fabs(radio.lastPps().get_real_secs()) < 1.0e-6) {
    if (radio.monotonic() >= next_deadline)
      throw std::runtime_error("PPS disappeared after time-zero latch");
    radio.sleep(0.005);
  }
  const double next = radio.lastPps().get_real_secs();
  if (fabs(next - 1.0) > 1.0e-6)
    throw std::runtime_error("PPS interval is inconsistent with hardware time");
  radio.checkLocks();
  const double hardware_now = radio.now().get_real_secs();
  const double observed_at = radio.monotonic();
  return {true, next, observed_at, hardware_now, observed_at};
}

inline void checkHealth(Radio &radio, SyncState &sync) {
  radio.checkLocks();
  double pps = radio.lastPps().get_real_secs();
  double current = radio.now().get_real_secs();
  double observed_at = radio.monotonic();
  if (!sync.verified || !std::isfinite(pps) || !std::isfinite(current) ||
      current < pps || pps < sync.last_pps_seconds || current - pps > 1.5)
    throw std::runtime_error("PPS lost or hardware time discontinuity");
  // Monotonic host time is only a watchdog; it never drives signal time.
  if (fabs((current - sync.last_hardware_seconds) -
           (observed_at - sync.last_checked_at)) > 0.25)
    throw std::runtime_error("hardware time jumped or stopped");
  sync.last_hardware_seconds = current;
  sync.last_checked_at = observed_at;
  if (pps != sync.last_pps_seconds) {
    double interval = pps - sync.last_pps_seconds;
    if (fabs(interval - round(interval)) > 1.0e-6)
      throw std::runtime_error("PPS interval changed relative to sample clock");
    sync.last_pps_seconds = pps;
    sync.last_pps_observed_at = observed_at;
  } else if (observed_at - sync.last_pps_observed_at > 1.5) {
    throw std::runtime_error("PPS stopped during transmission");
  }
}

struct TxStats {
  uint64_t generated = 0;
  uint64_t accepted = 0;
  uint64_t underflows = 0;
  uint64_t sequence_errors = 0;
  uint64_t time_errors = 0;
  uint64_t other_errors = 0;
  uint64_t data_send_calls = 0;
  uint64_t partial_sample_sends = 0;
  uint64_t zero_sample_sends = 0;
  uint64_t prestart_backpressure_waits = 0;
  double start_seconds = 0;
  double minimum_lead_seconds = std::numeric_limits<double>::infinity();
  double maximum_render_seconds = 0;
  double maximum_send_seconds = 0;
  size_t max_samples_per_send = 0;
  double prebuffer_seconds = 0;
  double first_send_hardware_seconds = 0;
  bool send_attempted = false;
  bool interrupted = false;
  bool burst_ack = false;
  bool start_margin_met = false;
  std::string failure;
  std::string first_event;
  bool first_event_has_time = false;
  double first_event_seconds = 0;
  // Host observation time, not the RF time or sample index of a lost packet.
  std::optional<double> first_event_observed_host_monotonic_seconds;
  uint64_t accepted_at_first_error = 0;
  uint64_t generated_iq_fnv1a64 = UINT64_C(14695981039346656037);
};

inline void hashIq(const std::vector<int16_t> &iq, TxStats &stats) {
  // Explicit SC16 little-endian bytes, independent of host byte order.
  for (int16_t component : iq) {
    uint16_t value = static_cast<uint16_t>(component);
    for (unsigned shift : {0u, 8u}) {
      stats.generated_iq_fnv1a64 ^= (value >> shift) & 255;
      stats.generated_iq_fnv1a64 *= UINT64_C(1099511628211);
    }
  }
}

inline void accountEvent(const uhd::async_metadata_t &event, TxStats &stats) {
  using M = uhd::async_metadata_t;
  const unsigned code = static_cast<unsigned>(event.event_code);
  if (code == M::EVENT_CODE_BURST_ACK) {
    stats.burst_ack = true;
    return;
  }
  if (code == 0)
    return;
  if (code & (M::EVENT_CODE_UNDERFLOW | M::EVENT_CODE_UNDERFLOW_IN_PACKET))
    ++stats.underflows;
  if (code & (M::EVENT_CODE_SEQ_ERROR | M::EVENT_CODE_SEQ_ERROR_IN_BURST))
    ++stats.sequence_errors;
  if (code & M::EVENT_CODE_TIME_ERROR)
    ++stats.time_errors;
  const unsigned known =
      M::EVENT_CODE_UNDERFLOW | M::EVENT_CODE_UNDERFLOW_IN_PACKET |
      M::EVENT_CODE_SEQ_ERROR | M::EVENT_CODE_SEQ_ERROR_IN_BURST |
      M::EVENT_CODE_TIME_ERROR;
  if (code & ~known)
    ++stats.other_errors;
  if (stats.first_event.empty()) {
    stats.first_event = event.strevent();
    stats.first_event_has_time = event.has_time_spec;
    if (event.has_time_spec)
      stats.first_event_seconds = event.time_spec.get_real_secs();
    stats.accepted_at_first_error = stats.accepted;
  }
  if (stats.failure.empty())
    stats.failure = "UHD continuity failure: " + stats.first_event;
}

inline void drainEvents(Radio &radio, TxStats &stats, double timeout = 0) {
  uhd::async_metadata_t event;
  while (radio.receiveEvent(event, timeout)) {
    accountEvent(event, stats);
    if (!stats.first_event.empty() &&
        !stats.first_event_observed_host_monotonic_seconds)
      stats.first_event_observed_host_monotonic_seconds = radio.monotonic();
    timeout = 0;
  }
}

struct Frame {
  std::vector<int16_t> iq;
};

using Render = std::function<bool(Frame &)>; // false means finite source EOF
using Stop = std::function<bool()>;

// Both waveform modes use this sender. Rendering owns all generator state;
// transport owns the radio, counters, and first-sample metadata.
inline bool transmit(
    Radio &radio, SyncState sync, double rate, double start_lead,
    size_t max_send, size_t capacity, const Render &render, const Stop &stop,
    TxStats &stats, const std::function<void()> &armed = [] {},
    std::optional<double> fixed_start = std::nullopt,
    const std::function<void()> &source_health = [] {}) {
  std::deque<Frame> queue;
  std::mutex mutex;
  std::condition_variable ready;
  std::atomic<bool> cancel{false};
  bool done = false;
  std::string producer_error;
  std::thread producer;
  bool attempted_send = false;
  auto produce = [&](Frame &frame) {
    const auto begin = std::chrono::steady_clock::now();
    bool available = render(frame);
    stats.maximum_render_seconds = std::max(
        stats.maximum_render_seconds,
        std::chrono::duration<double>(std::chrono::steady_clock::now() - begin)
            .count());
    if (available) {
      if (frame.iq.empty() || frame.iq.size() % 2 != 0)
        throw std::runtime_error("renderer produced an invalid IQ frame");
      stats.generated += frame.iq.size() / 2;
      hashIq(frame.iq, stats);
    }
    return available;
  };
  try {
    if (!std::isfinite(rate) || rate <= 0 || !std::isfinite(start_lead) ||
        start_lead < 0.02 || start_lead > 60 || !max_send || !capacity)
      throw std::invalid_argument("invalid buffered transmitter configuration");
    stats.max_samples_per_send = max_send;
    if (fixed_start) {
      if (!std::isfinite(*fixed_start) || *fixed_start <= 0)
        throw std::invalid_argument("invalid fixed hardware start");
      stats.start_seconds = *fixed_start;
    }
    source_health();
    const double prebuffer_begin = radio.monotonic();
    for (size_t n = 0; n < capacity && !stop(); ++n) {
      Frame frame;
      if (!produce(frame)) {
        done = true;
        break;
      }
      queue.push_back(std::move(frame));
    }
    stats.prebuffer_seconds = radio.monotonic() - prebuffer_begin;
    if (queue.empty() || stop())
      throw std::runtime_error("stopped or empty source before timed TX start");
    checkHealth(radio, sync);
    source_health();
    // An arbitrary scenario epoch is mapped to a future local hardware second.
    // All prebuffering happens first; no stale GPS/host-wall-clock start
    // exists.
    if (!fixed_start)
      stats.start_seconds = ceil(radio.now().get_real_secs() + start_lead);
    if (stats.start_seconds - radio.now().get_real_secs() < 0.02)
      throw std::runtime_error("fixed TX deadline missed before send; discard dated IQ and restart");
    armed();
    producer = std::thread([&] {
      try {
        for (;;) {
          {
            std::unique_lock<std::mutex> lock(mutex);
            ready.wait(lock, [&] {
              return cancel || done || queue.size() < capacity;
            });
            if (cancel || done)
              return;
          }
          Frame frame;
          bool available = produce(frame);
          {
            std::lock_guard<std::mutex> lock(mutex);
            if (available)
              queue.push_back(std::move(frame));
            else
              done = true;
          }
          ready.notify_all();
          if (!available)
            return;
        }
      } catch (const std::exception &e) {
        std::lock_guard<std::mutex> lock(mutex);
        producer_error = e.what();
        done = true;
        ready.notify_all();
      } catch (...) {
        std::lock_guard<std::mutex> lock(mutex);
        producer_error = "unknown rendering failure";
        done = true;
        ready.notify_all();
      }
    });
    uhd::tx_metadata_t md;
    md.start_of_burst = true;
    md.end_of_burst = false;
    md.has_time_spec = true;
    md.time_spec = uhd::time_spec_t(stats.start_seconds);
    double next_health = radio.monotonic();
    while (!stop() && stats.failure.empty()) {
      Frame frame;
      {
        std::unique_lock<std::mutex> lock(mutex);
        ready.wait_for(lock, std::chrono::milliseconds(10),
                       [&] { return done || !queue.empty(); });
        if (!producer_error.empty())
          throw std::runtime_error(producer_error);
        if (!queue.empty()) {
          frame = std::move(queue.front());
          queue.pop_front();
        } else if (done) {
          break;
        }
      }
      ready.notify_all();
      drainEvents(radio, stats);
      if (!stats.failure.empty())
        break;
      if (radio.monotonic() >= next_health) {
        checkHealth(radio, sync);
        source_health();
        next_health = radio.monotonic() + 0.25;
      }
      size_t sent = 0, count = frame.iq.size() / 2;
      while (sent < count && !stop() && stats.failure.empty()) {
        double timeout = 0.25;
        if (md.has_time_spec) {
          source_health();
          double margin = stats.start_seconds - radio.now().get_real_secs();
          if (margin < 0.02)
            throw std::runtime_error(
                "timed TX start became stale; no samples sent");
          stats.start_margin_met = true;
          timeout = margin + 0.25;
        }
        size_t request = std::min(max_send, count - sent);
        attempted_send = true;
        if (!stats.send_attempted)
          stats.first_send_hardware_seconds = radio.now().get_real_secs();
        stats.send_attempted = true;
        ++stats.data_send_calls;
        const double send_begin = radio.monotonic();
        size_t n = radio.send(frame.iq.data() + sent * 2, request, md, timeout);
        stats.maximum_send_seconds = std::max(
            stats.maximum_send_seconds, radio.monotonic() - send_begin);
        if (n > request)
          throw std::runtime_error("UHD send returned an invalid sample count");
        if (n == 0) {
          ++stats.zero_sample_sends;
          drainEvents(radio, stats);
          // SIGINT may interrupt a blocking send. Preserve any actual UHD
          // error, but do not turn an operator stop into a no-progress fault.
          if (!stats.failure.empty() || stop())
            break;
          checkHealth(radio, sync);
          source_health();
          next_health = radio.monotonic() + 0.25;
          const double margin = stats.start_seconds - sync.last_hardware_seconds;
          if (stats.accepted > 0 && margin > 0) {
            // The timed first packet can fill a finite device/transport FIFO
            // long before H_start. Later packets then legitimately time out:
            // nothing drains until H_start, irrespective of the host prebuffer.
            // Keep the same IQ suffix and metadata, with bounded waits so PPS,
            // UBX health, async errors and operator stop remain observable.
            ++stats.prestart_backpressure_waits;
            radio.sleep(std::min(0.001, margin)); // avoid spinning on an early 0
            continue;
          }
          throw std::runtime_error(
              "UHD send made no valid forward progress at hardware " +
              std::to_string(sync.last_hardware_seconds) + " s (start " +
              std::to_string(stats.start_seconds) + " s, accepted " +
              std::to_string(stats.accepted) + " samples)");
        }
        if (n < request)
          ++stats.partial_sample_sends;
        sent += n;
        stats.accepted += n;
        md.start_of_burst = false;
        md.has_time_spec = false;
        // UHD accepts packets before the device reports transport loss.
        // Poll every send, including successful/partial ones, so a fault
        // stops submissions within the frame instead of up to 100 ms later.
        drainEvents(radio, stats);
      }
      if (count) {
        double lead = stats.start_seconds +
                      static_cast<double>(stats.accepted) / rate -
                      radio.now().get_real_secs();
        stats.minimum_lead_seconds = std::min(stats.minimum_lead_seconds, lead);
      }
    }
  } catch (const std::exception &e) {
    if (stats.failure.empty())
      stats.failure = e.what();
  } catch (...) {
    if (stats.failure.empty())
      stats.failure = "unknown transmitter failure";
  }
  cancel = true;
  ready.notify_all();
  if (producer.joinable())
    producer.join();
  if (stats.failure.empty() && !producer_error.empty())
    stats.failure = producer_error;
  stats.interrupted = stop();
  if (attempted_send) {
    try {
      uhd::tx_metadata_t end;
      end.start_of_burst = false;
      end.has_time_spec = false;
      end.end_of_burst = true;
      // EOB consumes transport space too. A short burst or operator stop can
      // leave the FIFO full before H_start; its zero return cannot tell us
      // whether the metadata was sent, so allow the complete startup wait.
      const double end_timeout =
          std::max(0.25, stats.start_seconds - radio.now().get_real_secs() + 0.25);
      radio.send(nullptr, 0, end, end_timeout);
      // A short finite burst can still be queued before its future start.
      double until_end = stats.start_seconds +
                         static_cast<double>(stats.accepted) / rate -
                         radio.now().get_real_secs();
      const double deadline =
          radio.monotonic() + std::max(0.0, until_end) + 1.0;
      double next_health = radio.monotonic();
      do {
        if (radio.monotonic() >= next_health) {
          checkHealth(radio, sync);
          source_health();
          next_health = radio.monotonic() + 0.25;
        }
        drainEvents(radio, stats, 0.05);
      } while (!stats.burst_ack && radio.monotonic() < deadline);
      if (!stats.burst_ack && stats.failure.empty())
        stats.failure = "no end-of-burst acknowledgement before timeout";
    } catch (const std::exception &e) {
      if (stats.failure.empty())
        stats.failure = std::string("TX shutdown failed: ") + e.what();
    }
  }
  return stats.failure.empty() && stats.accepted > 0;
}

} // namespace x300
#endif
