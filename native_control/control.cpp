#include "time_core.hpp"

#include <atomic>
#include <condition_variable>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <vector>

#ifdef _WIN32
#define IC_API extern "C" __declspec(dllexport)
#else
#define IC_API extern "C" __attribute__((visibility("default")))
#endif

using namespace isaac_control;
using Steady = std::chrono::steady_clock;
static thread_local std::string last_error;

struct PublicationStats {
  uint64_t published = 0, expired = 0, rejected = 0, repeats = 0;
};
struct BenchRow {
  int64_t issued;
  double duration_us;
};

class ControlPublisher {
public:
  std::shared_ptr<rclcpp::Context> context;
  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr mode;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr target;
  rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr
      sync_subscription;
  std::thread ros_thread, publication_thread;
  std::mutex mutex;
  std::condition_variable changed;
#ifdef _WIN32
  HANDLE wake_event = nullptr, high_resolution_timer = nullptr;
#endif
  bool stopped = false, has_command = false, bench = false, qpc_wall;
  std::array<double, 8> command{};
  int64_t issued_ns = 0, last_publication_ns = 0;
  Timeline *timeline;
  TrajectoryClock *clock;
  PublicationStats stats;
  bool flight_topics;
  size_t bench_count = 0;
  int64_t bench_start = 0;
  Steady::time_point bench_due;
  std::vector<BenchRow> bench_rows;
  bool trace_publications = false;

  ControlPublisher(int domain, std::string prefix, Timeline *source,
                   TrajectoryClock *trajectory, bool uses_qpc)
      : qpc_wall(uses_qpc), timeline(source), clock(trajectory) {
    if (domain < 0 || domain > 232 || prefix.empty())
      throw std::invalid_argument("Invalid domain/topics");
    flight_topics = prefix == "/fmu/in";
    context = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions init;
    init.set_domain_id(static_cast<size_t>(domain));
    context->init(0, nullptr, init);
    static std::atomic<unsigned> number{0};
    rclcpp::NodeOptions options;
    options.context(context).use_global_arguments(false);
    node = std::make_shared<rclcpp::Node>(
        "isaac_native_control_" + std::to_string(++number), options);
    auto qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    mode = node->create_publisher<px4_msgs::msg::OffboardControlMode>(
        prefix + "/offboard_control_mode", qos);
    target = node->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        prefix + "/trajectory_setpoint", qos);
    flight_topics =
        std::string(mode->get_topic_name()) ==
            "/fmu/in/offboard_control_mode" ||
        std::string(target->get_topic_name()) == "/fmu/in/trajectory_setpoint";
    if (timeline) {
      sync_subscription =
          node->create_subscription<px4_msgs::msg::TimesyncStatus>(
              "/fmu/out/timesync_status", rclcpp::SensorDataQoS(),
              [this](const px4_msgs::msg::TimesyncStatus &message) {
                timeline->observe_sync(
                    static_cast<int64_t>(message.timestamp),
                    message.estimated_offset, message.source_protocol,
                    message.round_trip_time, wall_time(qpc_wall));
              });
    }
    rclcpp::ExecutorOptions exec_options;
    exec_options.context = context;
    executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(
        exec_options);
    executor->add_node(node);
#ifdef _WIN32
    wake_event = CreateEventW(nullptr, FALSE, FALSE, nullptr);
    high_resolution_timer =
        CreateWaitableTimerExW(nullptr, nullptr, 0x00000002, TIMER_ALL_ACCESS);
    if (!wake_event || !high_resolution_timer)
      throw std::runtime_error("High-resolution timer unavailable");
#endif
    ros_thread = std::thread([this] {
      try {
        executor->spin();
      } catch (const std::exception &) {
        std::lock_guard<std::mutex> lock(mutex);
        stopped = true;
        notify();
      }
    });
    publication_thread = std::thread([this] {
      try {
        run();
      } catch (const std::exception &) {
        std::lock_guard<std::mutex> lock(mutex);
        stopped = true;
        notify();
      }
    });
  }
  ~ControlPublisher() {
    {
      std::lock_guard<std::mutex> lock(mutex);
      stopped = true;
      notify();
    }
    if (publication_thread.joinable())
      publication_thread.join();
    executor->cancel();
    if (ros_thread.joinable())
      ros_thread.join();
    executor->remove_node(node);
    context->shutdown("native publisher closed");
#ifdef _WIN32
    CloseHandle(wake_event);
    CloseHandle(high_resolution_timer);
#endif
  }
  void notify() {
    changed.notify_all();
#ifdef _WIN32
    SetEvent(wake_event);
#endif
  }
  void wait(std::unique_lock<std::mutex> &lock, int64_t nanoseconds) {
#ifdef _WIN32
    LARGE_INTEGER due;
    due.QuadPart = -std::max<int64_t>(1, (nanoseconds + 99) / 100);
    if (!SetWaitableTimerEx(high_resolution_timer, &due, 0, nullptr, nullptr,
                            nullptr, 0))
      throw std::runtime_error("High-resolution timer arm failed");
    HANDLE handles[] = {wake_event, high_resolution_timer};
    lock.unlock();
    const auto result = WaitForMultipleObjects(2, handles, FALSE, INFINITE);
    lock.lock();
    if (result == WAIT_FAILED)
      throw std::runtime_error("Publication wait failed");
#else
    changed.wait_for(lock, std::chrono::nanoseconds(nanoseconds));
#endif
  }
  bool publish_locked(const std::array<double, 8> &frame, bool repeat) {
    if (flight_topics &&
        (node->count_publishers("/fmu/in/offboard_control_mode") > 1 ||
         node->count_publishers("/fmu/in/trajectory_setpoint") > 1)) {
      ++stats.rejected;
      return false;
    }
    const auto publication_started = trace_publications ? qpc_ns() : 0;
    px4_msgs::msg::OffboardControlMode m{};
    px4_msgs::msg::TrajectorySetpoint t{};
    m.timestamp = 0;
    m.position = true;
    t.timestamp = 0;
    for (int i = 0; i < 3; ++i)
      t.position[i] = static_cast<float>(frame[i]);
    const bool fresh = !clock || clock->fresh(wall_time(qpc_wall));
    t.velocity = {static_cast<float>(fresh ? frame[3] : 0),
                  static_cast<float>(fresh ? frame[4] : 0),
                  static_cast<float>(missing)};
    t.acceleration = {static_cast<float>(fresh ? frame[5] : 0),
                      static_cast<float>(fresh ? frame[6] : 0),
                      static_cast<float>(missing)};
    t.jerk.fill(static_cast<float>(missing));
    t.yaw = static_cast<float>(frame[7]);
    t.yawspeed = static_cast<float>(missing);
    mode->publish(m);
    target->publish(t);
    last_publication_ns = qpc_ns();
    if (trace_publications && bench_rows.size() < 10000)
      bench_rows.push_back(
          {publication_started,
           static_cast<double>(last_publication_ns - publication_started) /
               1000.});
    ++stats.published;
    if (repeat)
      ++stats.repeats;
    return true;
  }
  bool submit(const double *values, int64_t issued) {
    std::lock_guard<std::mutex> lock(mutex);
    const int64_t now = qpc_ns();
    bool valid = !stopped && !bench && issued > issued_ns && now >= issued &&
                 now - issued <= 100000000;
    for (int i : {0, 1, 2, 3, 4, 7})
      valid = valid && std::isfinite(values[i]);
    valid = valid && ((std::isfinite(values[5]) && std::isfinite(values[6])) ||
                      (std::isnan(values[5]) && std::isnan(values[6])));
    if (!valid) {
      ++stats.rejected;
      has_command = false;
      notify();
      return false;
    }
    std::copy(values, values + 8, command.begin());
    issued_ns = issued;
    has_command = true;
    // New validated references publish immediately; no extra 50 ms IPC/timer
    // delay.
    const bool result = publish_locked(command, false);
    if (!result)
      has_command = false;
    notify();
    return result;
  }
  void revoke() {
    std::lock_guard<std::mutex> lock(mutex);
    has_command = false;
    notify();
  }
  void run() {
    std::unique_lock<std::mutex> lock(mutex);
    while (!stopped) {
      if (bench) {
        const auto remaining =
            bench_start +
            static_cast<int64_t>(bench_rows.size() + 1) * 50000000 - qpc_ns();
        if (remaining > 0) {
          wait(lock, remaining);
          continue;
        }
        const auto began = qpc_ns();
        std::array<double, 8> frame{
            static_cast<double>(bench_rows.size()), 0, -1, .6, 0, .25, 0, 0};
        publish_locked(frame, false);
        bench_rows.push_back(
            {began, static_cast<double>(qpc_ns() - began) / 1000.});
        bench_due += std::chrono::milliseconds(50);
        if (bench_rows.size() == bench_count)
          bench = false;
      } else if (has_command) {
        const auto now = qpc_ns();
        if (now - issued_ns > 100000000 || now < issued_ns) {
          has_command = false;
          ++stats.expired;
          continue;
        }
        const auto remaining = 50000000 - (now - last_publication_ns);
        if (remaining <= 0) {
          if (!publish_locked(command, true))
            has_command = false;
        } else {
          wait(lock, remaining);
        }
      } else {
        changed.wait(lock, [this] { return stopped || has_command || bench; });
      }
    }
  }
};

IC_API int ic_abi_version() { return 1; }
IC_API const char *ic_source_hash() { return IC_SOURCE_HASH; }
IC_API const char *ic_error() { return last_error.c_str(); }
IC_API void *ic_clock_new() { return new TrajectoryClock(); }
IC_API void ic_clock_delete(TrajectoryClock *clock) { delete clock; }
IC_API void ic_clock_observe(TrajectoryClock *clock, int64_t stamp,
                             double now) {
  clock->observe(stamp, now);
}
IC_API void ic_clock_invalidate(TrajectoryClock *clock) { clock->invalidate(); }
IC_API double ic_clock_consume(TrajectoryClock *clock, double now) {
  return clock->consume(now);
}
IC_API int ic_clock_fresh(TrajectoryClock *clock, double now) {
  return clock->fresh(now) ? 1 : 0;
}
IC_API void ic_clock_state(TrajectoryClock *clock, ClockState *out) {
  *out = clock->snapshot();
}

IC_API void *ic_timeline_new(int port) { return new Timeline(port); }
IC_API void ic_timeline_delete(Timeline *timeline) { delete timeline; }
IC_API void ic_timeline_poll(Timeline *timeline, double now) {
  timeline->poll(now);
}
IC_API int ic_timeline_epoch(Timeline *timeline) {
  std::lock_guard<std::mutex> lock(timeline->mutex);
  return timeline->clock.epoch;
}
IC_API void ic_timeline_pair(Timeline *timeline, const char *session,
                             int64_t native, int64_t sim, double sent,
                             double now, double qpc) {
  std::lock_guard<std::mutex> lock(timeline->mutex);
  timeline->clock.observe(session, native, sim, sent, now, qpc);
}
IC_API void ic_timeline_sync(Timeline *timeline, int64_t stamp, int64_t offset,
                             int protocol, int64_t roundtrip, double now) {
  timeline->observe_sync(stamp, offset, protocol, roundtrip, now);
}
IC_API int ic_timeline_sample(Timeline *timeline, int64_t stamp, int64_t sample,
                              double now, int64_t *out) {
  return timeline->sample(stamp, sample, now, *out);
}

IC_API void *ic_history_new() { return new History(); }
IC_API void ic_history_delete(History *history) { delete history; }
IC_API void ic_history_clear(History *history) { history->rows.clear(); }
IC_API void ic_history_record(History *history, int64_t stamp, const double *xy,
                              const double *velocity, int64_t epoch) {
  history->record(stamp, xy, velocity, epoch);
}
IC_API int ic_history_at(History *history, int64_t stamp, double *out) {
  return history->at(stamp, out);
}

IC_API void *ic_publisher_new(int domain, const char *prefix,
                              Timeline *timeline, TrajectoryClock *clock,
                              int qpc_wall) {
  try {
    return new ControlPublisher(domain, prefix, timeline, clock, qpc_wall != 0);
  } catch (const std::exception &error) {
    last_error = error.what();
    return nullptr;
  }
}
IC_API void ic_publisher_delete(ControlPublisher *publisher) {
  delete publisher;
}
IC_API int ic_publisher_submit(ControlPublisher *publisher, const double *frame,
                               int64_t issued) {
  try {
    return publisher->submit(frame, issued) ? 1 : 0;
  } catch (const std::exception &error) {
    publisher->revoke();
    last_error = error.what();
    return -1;
  }
}
IC_API void ic_publisher_revoke(ControlPublisher *publisher) {
  publisher->revoke();
}
IC_API size_t ic_publisher_readers(ControlPublisher *publisher, int target) {
  return target ? publisher->target->get_subscription_count()
                : publisher->mode->get_subscription_count();
}
IC_API void ic_publisher_stats(ControlPublisher *publisher,
                               PublicationStats *out) {
  std::lock_guard<std::mutex> lock(publisher->mutex);
  *out = publisher->stats;
}
IC_API int ic_publisher_alive(ControlPublisher *publisher) {
  std::lock_guard<std::mutex> lock(publisher->mutex);
  return publisher->stopped ? 0 : 1;
}
IC_API int64_t ic_publisher_benchmark(ControlPublisher *publisher,
                                      size_t count) {
  std::lock_guard<std::mutex> lock(publisher->mutex);
  if (count == 0 || publisher->flight_topics || publisher->has_command ||
      publisher->bench)
    return -1;
  publisher->bench_rows.clear();
  publisher->trace_publications = false;
  publisher->bench_rows.reserve(count);
  publisher->bench_count = count;
  publisher->bench_start = qpc_ns();
  publisher->bench_due = Steady::now() + std::chrono::milliseconds(50);
  publisher->bench = true;
  publisher->notify();
  return publisher->bench_start;
}
IC_API size_t ic_publisher_benchmark_count(ControlPublisher *publisher) {
  std::lock_guard<std::mutex> lock(publisher->mutex);
  return publisher->bench_rows.size();
}
IC_API int ic_publisher_trace(ControlPublisher *publisher) {
  std::lock_guard<std::mutex> lock(publisher->mutex);
  if (publisher->flight_topics || publisher->has_command || publisher->bench)
    return 0;
  publisher->bench_rows.clear();
  publisher->bench_rows.reserve(10000);
  publisher->trace_publications = true;
  return 1;
}
IC_API int ic_publisher_benchmark_row(ControlPublisher *publisher, size_t index,
                                      int64_t *issued, double *duration) {
  std::lock_guard<std::mutex> lock(publisher->mutex);
  if (index >= publisher->bench_rows.size())
    return 0;
  *issued = publisher->bench_rows[index].issued;
  *duration = publisher->bench_rows[index].duration_us;
  return 1;
}
