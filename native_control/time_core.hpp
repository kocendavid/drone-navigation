#pragma once

#ifdef _WIN32
#include <windows.h>
#include <winsock2.h>
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>

namespace isaac_control {
inline constexpr double missing = std::numeric_limits<double>::quiet_NaN();
inline int64_t qpc_ns() {
#ifdef _WIN32
  LARGE_INTEGER counter, frequency;
  QueryPerformanceCounter(&counter);
  QueryPerformanceFrequency(&frequency);
  return (counter.QuadPart / frequency.QuadPart) * 1000000000LL +
         (counter.QuadPart % frequency.QuadPart) * 1000000000LL /
             frequency.QuadPart;
#else
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
#endif
}
inline double wall_time(bool qpc) {
#ifdef _WIN32
  if (!qpc)
    return static_cast<double>(GetTickCount64()) / 1000.;
#else
  (void)qpc;
#endif
  return static_cast<double>(qpc_ns()) / 1e9;
}

struct ClockState {
  int64_t stamp = -1, decision_stamp = -1;
  double total = 0, mark = 0, received = missing, delta = missing;
  double decision_wall = missing, decision_age = missing, decision_elapsed = 0;
  int32_t confirming = 3, reason = 1, discontinuity = 0, has_decision = 0;
};

class TrajectoryClock {
public:
  std::mutex mutex;
  ClockState state;
  bool fresh_locked(double now) const {
    return std::isfinite(state.received) && now >= state.received &&
           now - state.received <= .25 && state.confirming == 0;
  }
  bool fresh(double now) {
    std::lock_guard<std::mutex> lock(mutex);
    return fresh_locked(now);
  }
  ClockState snapshot() {
    std::lock_guard<std::mutex> lock(mutex);
    return state;
  }
  void invalidate() {
    std::lock_guard<std::mutex> lock(mutex);
    state.received = missing;
    state.confirming = 3;
    state.mark = state.total;
  }
  void observe(int64_t stamp, double now) {
    std::lock_guard<std::mutex> lock(mutex);
    if (stamp < 0) {
      state.received = missing;
      return;
    }
    const auto previous = state.stamp;
    state.stamp = stamp;
    if (previous < 0 || previous == stamp)
      return;
    const double delta = static_cast<double>(stamp - previous) / 1e9;
    state.delta = delta;
    if (!(delta > 0 && delta <= .25)) {
      state.discontinuity = delta < 0 ? 1 : 2;
      state.confirming = 3;
      state.received = missing;
      state.mark = state.total;
      return;
    }
    if (state.confirming) {
      --state.confirming;
      state.mark = state.total;
    } else
      state.total += delta;
    state.received = now;
  }
  double consume(double now) {
    std::lock_guard<std::mutex> lock(mutex);
    const double elapsed = state.total - state.mark;
    state.mark = state.total;
    state.reason = !std::isfinite(state.received) ? 1
                   : state.confirming             ? 2
                   : !fresh_locked(now)           ? 3
                   : (!std::isfinite(elapsed) || elapsed < 0 || elapsed > .25)
                       ? 4
                       : 0;
    state.has_decision = 1;
    state.decision_wall = now;
    state.decision_stamp = state.stamp;
    state.decision_age = now - state.received;
    state.decision_elapsed = elapsed;
    return state.reason == 0 ? elapsed : missing;
  }
};

class SimulatorClock {
public:
  std::string session;
  int64_t native = -1, sim = -1;
  int32_t confirmed = 0, epoch = 0;
  double received = missing,
         last_sent = -std::numeric_limits<double>::infinity();
  void observe(const std::string &id, int64_t px4, int64_t isaac, double sent,
               double now, double qpc) {
    if (id.empty() || px4 < 0 || isaac < 0 || !std::isfinite(sent) ||
        qpc < sent || qpc - sent > .25 || sent <= last_sent)
      return;
    last_sent = sent;
    if (native >= 0) {
      if (id == session && px4 == native && isaac == sim)
        return;
      const int64_t delta = isaac - sim;
      // Bounds also prevent overflow from malformed clock packets.
      const double offset_change =
          (static_cast<double>(px4) - native) * 1000. - delta;
      if (id != session || delta <= 0 || delta > 250000000 ||
          std::abs(offset_change) > 2000000) {
        confirmed = 0;
        ++epoch;
      }
    }
    session = id;
    native = px4;
    sim = isaac;
    received = now;
    ++confirmed;
  }
  int64_t convert(int64_t stamp, double now) const {
    if (native < 0 || confirmed < 3 ||
        !(now >= received && now - received <= .25))
      return -1;
    const double delta = (static_cast<double>(stamp) - native) * 1000.;
    if (delta < -250000000 || delta > 100000000)
      return -1;
    return sim + static_cast<int64_t>(delta);
  }
};

struct Sync {
  int64_t native, offset;
  double received;
};
class Timeline {
public:
  std::mutex mutex;
  SimulatorClock clock;
  std::deque<Sync> sync;
#ifdef _WIN32
  SOCKET socket = INVALID_SOCKET;
  bool winsock_started = false;
  static constexpr SOCKET invalid_socket = INVALID_SOCKET;
#else
  int socket = -1;
  static constexpr int invalid_socket = -1;
#endif
  explicit Timeline(int port) {
    if (port < 0)
      return;
#ifdef _WIN32
    WSADATA data;
    if (WSAStartup(MAKEWORD(2, 2), &data) != 0)
      return;
    winsock_started = true;
#endif
    socket = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socket == invalid_socket)
      return;
    sockaddr_in peer{};
    peer.sin_family = AF_INET;
    peer.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    peer.sin_port = htons(static_cast<uint16_t>(port));
    if (::bind(socket, reinterpret_cast<sockaddr *>(&peer), sizeof(peer)) !=
        0) {
      close_socket();
      return;
    }
#ifdef _WIN32
    u_long enabled = 1;
    ioctlsocket(socket, FIONBIO, &enabled);
#else
    fcntl(socket, F_SETFL, O_NONBLOCK);
#endif
  }
  ~Timeline() {
    close_socket();
#ifdef _WIN32
    if (winsock_started)
      WSACleanup();
#endif
  }
  void close_socket() {
    if (socket == invalid_socket)
      return;
#ifdef _WIN32
    closesocket(socket);
#else
    ::close(socket);
#endif
    socket = invalid_socket;
  }
  void poll(double now) {
    std::lock_guard<std::mutex> lock(mutex);
    if (socket == invalid_socket)
      return;
    char payload[1025];
    for (int count = 0; count < 32; ++count) {
      const int size =
          static_cast<int>(::recv(socket, payload, sizeof(payload), 0));
      if (size < 0)
        break;
      if (size > 1024)
        continue;
      try {
        auto row = nlohmann::json::parse(payload, payload + size);
        if (!row.at("px4_us").is_number_integer() ||
            !row.at("sim_ns").is_number_integer() ||
            row.at("px4_us").is_boolean() || row.at("sim_ns").is_boolean())
          continue;
        clock.observe(row.at("session").get<std::string>(),
                      row.at("px4_us").get<int64_t>(),
                      row.at("sim_ns").get<int64_t>(),
                      row.at("sent_qpc_s").get<double>(), now,
                      static_cast<double>(qpc_ns()) / 1e9);
      } catch (const nlohmann::json::exception &) {
        continue;
      }
    }
  }
  void observe_sync(int64_t stamp, int64_t offset, int protocol,
                    int64_t roundtrip, double now) {
    std::lock_guard<std::mutex> lock(mutex);
    if (protocol != 2 || stamp <= 0 || roundtrip < 0 || roundtrip > 20000)
      return;
    const int64_t native = stamp + offset;
    if (native <= 0)
      return;
    if (!sync.empty() && native < sync.back().native)
      sync.clear();
    if (sync.empty() || native != sync.back().native ||
        offset != sync.back().offset) {
      sync.push_back({native, offset, now});
      if (sync.size() > 12)
        sync.pop_front();
    }
  }
  // Reason: 0 aligned, 1 invalid timestamp, 2 unavailable, 3 ambiguous epoch.
  int sample(int64_t stamp, int64_t sample, double now, int64_t &result) {
    poll(now);
    std::lock_guard<std::mutex> lock(mutex);
    if (sample <= 0 || sample > stamp || stamp - sample > 250000)
      return 1;
    int64_t minimum = std::numeric_limits<int64_t>::max(), maximum = -1;
    for (size_t index = 0; index < sync.size(); ++index) {
      const auto &row = sync[index];
      const int64_t published = stamp + row.offset;
      if (now < row.received || now - row.received > 2. ||
          published < row.native ||
          (index + 1 < sync.size() && published >= sync[index + 1].native))
        continue;
      const auto converted = clock.convert(sample + row.offset, now);
      if (converted < 0)
        continue;
      minimum = std::min(minimum, converted);
      maximum = std::max(maximum, converted);
      result = converted;
    }
    if (maximum < 0)
      return 2;
    return maximum - minimum > 2000000 ? 3 : 0;
  }
};

struct Reference {
  int64_t stamp;
  std::array<double, 2> xy, velocity;
};
class History {
public:
  std::deque<Reference> rows;
  int64_t epoch = 0;
  bool has_epoch = false;
  void record(int64_t stamp, const double *xy, const double *velocity,
              int64_t generation) {
    if (!has_epoch || generation != epoch ||
        (!rows.empty() && stamp < rows.back().stamp))
      rows.clear();
    epoch = generation;
    has_epoch = true;
    Reference row{stamp, {xy[0], xy[1]}, {velocity[0], velocity[1]}};
    if (!rows.empty() && stamp == rows.back().stamp)
      rows.back() = row;
    else {
      rows.push_back(row);
      if (rows.size() > 128)
        rows.pop_front();
    }
  }
  // Exact, interpolated, predicted, unavailable, gap, too old.
  int at(int64_t stamp, double *xy) const {
    if (rows.empty() || stamp < rows.front().stamp)
      return 3;
    auto previous = rows.begin();
    for (auto row = rows.begin(); row != rows.end(); ++row) {
      if (stamp == row->stamp) {
        std::copy(row->xy.begin(), row->xy.end(), xy);
        return 0;
      }
      if (row->stamp > stamp) {
        const auto gap = row->stamp - previous->stamp;
        if (gap > 250000000)
          return 4;
        const double fraction =
            static_cast<double>(stamp - previous->stamp) / gap;
        for (int i = 0; i < 2; ++i)
          xy[i] = previous->xy[i] + fraction * (row->xy[i] - previous->xy[i]);
        return 1;
      }
      previous = row;
    }
    const double ahead = static_cast<double>(stamp - previous->stamp) / 1e9;
    if (ahead > .1)
      return 5;
    for (int i = 0; i < 2; ++i)
      xy[i] = previous->xy[i] + previous->velocity[i] * ahead;
    return 2;
  }
};
} // namespace isaac_control
