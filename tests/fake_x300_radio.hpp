#ifndef TEST_FAKE_X300_RADIO_HPP
#define TEST_FAKE_X300_RADIO_HPP

#include "player/x300_timing.hpp"
#include <cassert>

using Event = uhd::async_metadata_t;

// One controllable physical PPS source and a separately latched device clock.
// Every radio operation must stay on the sender thread, even during rendering.
class FakeRadio final : public x300::Radio {
public:
  double wall = 100.25;
  double origin = 0;
  double utc_offset = 0;
  std::function<void(double)> on_advance;
  double pending_latch = -1;
  bool locked = true;
  bool pps_present = true;
  double frozen_pps = 100;
  bool bad_latch = false;
  bool lose_pps_after_latch = false;
  bool acknowledge = true;
  bool ended = false;
  bool ack_sent = false;
  bool zero_send = false;
  bool lose_lock_on_send = false;
  size_t partial_limit = 3;
  size_t latch_calls = 0;
  size_t send_calls = 0;
  double start = 0;
  double rate = 100;
  double delay_per_send = 0;
  // Optional timed device FIFO. No samples drain before the burst timestamp;
  // afterward capacity becomes available at the configured sample rate.
  size_t fifo_capacity = 0;
  bool stalled_fifo = false;
  size_t send_timeouts = 0;
  size_t accepted_at_first_timeout = 0;
  double first_timeout_hardware = 0;
  bool fifo_underflow_reported = false;
  std::deque<double> query_delays;
  unsigned inject_event = 0;
  bool event_sent = false;
  bool event_has_time = true;
  std::vector<int16_t> received;
  std::vector<uhd::tx_metadata_t> metadata;
  std::thread::id owner = std::this_thread::get_id();

  void ownerOnly() { assert(std::this_thread::get_id() == owner); }
  void advance(double seconds) {
    ownerOnly();
    wall += seconds;
    if (pending_latch >= 0 && wall >= pending_latch) {
      origin = pending_latch + (bad_latch ? 0.125 : 0);
      pending_latch = -1;
      if (lose_pps_after_latch) {
        pps_present = false;
        frozen_pps = 0;
      }
    }
    if (on_advance) on_advance(wall);
  }
  uhd::time_spec_t now() override {
    ownerOnly();
    if (!query_delays.empty()) {
      const double delay = query_delays.front();
      query_delays.pop_front();
      advance(delay / 2);
      const uhd::time_spec_t value(wall - origin);
      advance(delay / 2);
      return value;
    }
    return uhd::time_spec_t(wall - origin);
  }
  uhd::time_spec_t lastPps() override {
    ownerOnly();
    return uhd::time_spec_t(pps_present ? floor(wall) - origin : frozen_pps);
  }
  void latchNextPps() override {
    ownerOnly();
    ++latch_calls;
    pending_latch = floor(wall) + 1;
  }
  void checkLocks() override {
    ownerOnly();
    if (!locked)
      throw std::runtime_error("reference unlocked");
  }
  double monotonic() override {
    ownerOnly();
    return wall;
  }
  double utcNow() override { ownerOnly(); return utc_offset + wall; }
  void sleep(double seconds) override { advance(seconds); }
  bool waitForRoom(size_t samples, double timeout) {
    if (!fifo_capacity)
      return true;
    assert(samples <= fifo_capacity);
    const double consumed = static_cast<double>(received.size() / 2) +
                            samples - fifo_capacity;
    if (consumed <= 0)
      return true;
    const double wait = stalled_fifo ? std::numeric_limits<double>::infinity()
        : std::max(0.0, start + consumed / rate - (wall - origin));
    advance(std::min(wait, timeout));
    if (wait <= timeout)
      return true;
    if (send_timeouts++ == 0) {
      accepted_at_first_timeout = received.size() / 2;
      first_timeout_hardware = wall - origin;
    }
    return false;
  }
  size_t send(const int16_t *iq, size_t count, const uhd::tx_metadata_t &md,
              double timeout) override {
    ownerOnly();
    assert(timeout > 0);
    ++send_calls;
    metadata.push_back(md);
    if (!count) {
      assert(md.end_of_burst && !md.has_time_spec && !md.start_of_burst);
      // UHD's zero-length EOB still needs a transport packet. A zero return
      // cannot distinguish its successful submission from a timeout.
      ended = waitForRoom(1, timeout);
      return 0;
    }
    assert(!md.end_of_burst);
    if (md.has_time_spec)
      start = md.time_spec.get_real_secs();
    advance(delay_per_send);
    if (zero_send)
      return 0;
    size_t n = std::min(count, partial_limit);
    if (!waitForRoom(n, timeout))
      return 0;
    received.insert(received.end(), iq, iq + n * 2);
    if (lose_lock_on_send)
      locked = false;
    return n;
  }
  bool receiveEvent(Event &event, double timeout) override {
    ownerOnly();
    event = {};
    if (inject_event && !event_sent && !received.empty()) {
      event_sent = true;
      event.event_code = static_cast<Event::event_code_t>(inject_event);
      event.has_time_spec = event_has_time;
      event.time_spec = uhd::time_spec_t(start + 0.025);
      return true;
    }
    advance(timeout);
    if (fifo_capacity && !ended && !received.empty() &&
        !fifo_underflow_reported &&
        wall - origin > start + received.size() / 2.0 / rate + 1e-6) {
      fifo_underflow_reported = true;
      event.event_code = Event::EVENT_CODE_UNDERFLOW;
      event.has_time_spec = true;
      event.time_spec = uhd::time_spec_t(start + received.size() / 2.0 / rate);
      return true;
    }
    if (ended && acknowledge && !ack_sent &&
        now().get_real_secs() >= start + received.size() / 2.0 / rate) {
      event.event_code = Event::EVENT_CODE_BURST_ACK;
      ack_sent = true;
      return true;
    }
    return false;
  }
};

#endif
