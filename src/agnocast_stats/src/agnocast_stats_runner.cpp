// PoC runner for ros2 topic hz_agnocast / delay_agnocast.
//
// Subscribes to a single agnocast topic via agnocast::GenericSubscription (no MessageT,
// no per-type plugin), drains MQ events to count arrivals (hz mode) or reads header.stamp
// via runtime-resolved offsets (delay mode), and prints rolling statistics on stdout.

#include "agnocast/agnocast.hpp"

#include "rclcpp/rclcpp.hpp"

#include <mqueue.h>
#include <poll.h>
#include <signal.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <string>

namespace
{

std::atomic<bool> g_stop{false};

void handle_sigint(int) { g_stop.store(true, std::memory_order_relaxed); }

struct Args
{
  std::string topic_name;
  std::string mode;     // "hz" or "delay"
  std::string type_name;  // required for delay
  double window_sec = 5.0;
};

bool parse_args(int argc, char ** argv, Args & out)
{
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto need = [&](const char * label) -> const char * {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "missing value for %s\n", label);
        return nullptr;
      }
      return argv[++i];
    };
    if (a == "--topic") {
      const char * v = need("--topic");
      if (!v) return false;
      out.topic_name = v;
    } else if (a == "--mode") {
      const char * v = need("--mode");
      if (!v) return false;
      out.mode = v;
    } else if (a == "--type") {
      const char * v = need("--type");
      if (!v) return false;
      out.type_name = v;
    } else if (a == "--window") {
      const char * v = need("--window");
      if (!v) return false;
      out.window_sec = std::atof(v);
    } else {
      std::fprintf(stderr, "unknown arg: %s\n", a.c_str());
      return false;
    }
  }
  if (out.topic_name.empty() || out.mode.empty()) {
    std::fprintf(stderr,
      "usage: agnocast_stats_runner --topic <name> --mode hz|delay "
      "[--type <pkg/msg/Type>] [--window <sec>]\n");
    return false;
  }
  if (out.mode == "delay" && out.type_name.empty()) {
    std::fprintf(stderr, "--mode delay requires --type <pkg/msg/Type>\n");
    return false;
  }
  if (out.mode != "hz" && out.mode != "delay" && out.mode != "echo") {
    std::fprintf(stderr, "--mode must be hz, delay, or echo\n");
    return false;
  }
  if (out.mode == "echo" && out.type_name.empty()) {
    std::fprintf(stderr, "--mode echo requires --type <pkg/msg/Type>\n");
    return false;
  }
  if (out.window_sec <= 0.0) out.window_sec = 5.0;
  return true;
}

double now_sec()
{
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

double now_wall_sec()
{
  struct timespec ts {};
  clock_gettime(CLOCK_REALTIME, &ts);
  return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

struct RollingStats
{
  // Keep all samples in the current window; for a PoC this is fine.
  std::deque<std::pair<double, double>> samples;  // (insert_time, value)
  double window_sec;
  void add(double t, double v)
  {
    samples.emplace_back(t, v);
    while (!samples.empty() && samples.front().first < t - window_sec) {
      samples.pop_front();
    }
  }
  bool summarize(double & mean, double & vmin, double & vmax, double & stddev, size_t & n) const
  {
    n = samples.size();
    if (n == 0) return false;
    double s = 0.0;
    vmin = samples.front().second;
    vmax = samples.front().second;
    for (const auto & p : samples) {
      s += p.second;
      vmin = std::min(vmin, p.second);
      vmax = std::max(vmax, p.second);
    }
    mean = s / static_cast<double>(n);
    double sq = 0.0;
    for (const auto & p : samples) {
      double d = p.second - mean;
      sq += d * d;
    }
    stddev = std::sqrt(sq / static_cast<double>(n));
    return true;
  }
};

}  // namespace

int main(int argc, char ** argv)
{
  Args args;
  if (!parse_args(argc, argv, args)) return 1;

  signal(SIGINT, handle_sigint);
  signal(SIGTERM, handle_sigint);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("agnocast_stats_runner");

  // SensorDataQoS works for typical agnocast usage (depth 5, best effort -> reliability is
  // ignored by agnocast anyway). Use a depth-of-1 by default for take-style consumption.
  rclcpp::QoS qos(1);

  std::optional<agnocast::StampOffsets> offsets;
  if (args.mode == "delay") {
    offsets = agnocast::resolve_header_stamp_offsets(args.type_name);
    if (!offsets) {
      std::fprintf(
        stderr,
        "failed to resolve header.stamp offsets for type '%s' (does it contain `header`?)\n",
        args.type_name.c_str());
      rclcpp::shutdown();
      return 2;
    }
    std::fprintf(
      stderr, "[delay] resolved offsets: sec=%zu nanosec=%zu (type=%s)\n",
      offsets->sec_offset, offsets->nanosec_offset, args.type_name.c_str());
  } else if (args.mode == "echo") {
    // Probe the type once so a malformed name fails fast before any payload arrives.
    auto probe = agnocast::dump_message_yaml(args.type_name, nullptr);
    (void)probe;
    std::fprintf(
      stderr, "[echo] subscribed to '%s' as %s\n", args.topic_name.c_str(),
      args.type_name.c_str());
  }

  agnocast::GenericSubscription sub(node.get(), args.topic_name, qos);

  RollingStats stats{};
  stats.window_sec = args.window_sec;

  // Track the previous arrival time so we can compute inter-arrival deltas for hz statistics.
  double prev_recv_steady = -1.0;
  double next_print = now_sec() + 1.0;

  if (args.mode != "echo") {
    std::fprintf(
      stderr, "[%s] subscribed to '%s' (window=%.2fs). Waiting for messages...\n",
      args.mode.c_str(), args.topic_name.c_str(), args.window_sec);
  }

  while (rclcpp::ok() && !g_stop.load(std::memory_order_relaxed)) {
    // Wait up to 500 ms; periodic wakeup so we still print stats when no messages arrive.
    struct pollfd pfd
    {
    };
    pfd.fd = sub.mq_fd();
    pfd.events = POLLIN;
    int pret = ::poll(&pfd, 1, 500);
    if (pret > 0 && (pfd.revents & POLLIN)) {
      // Drain notifications. MQ_attr.mq_maxmsg is 1 in agnocast, but we loop defensively.
      char buf[64];
      while (true) {
        ssize_t r = mq_receive(sub.mq_fd(), buf, sizeof(buf), nullptr);
        if (r < 0) {
          if (errno == EAGAIN) break;
          std::fprintf(stderr, "mq_receive failed: %s\n", std::strerror(errno));
          break;
        }
        // For each notification, drain all kmod-pending entries. One MQ notification can
        // correspond to several queued entries because mq_send is coalesced (EAGAIN path on
        // publisher side) when bursts happen.
        sub.drain([&](const void * payload, int64_t entry_id) {
          const double recv_steady = now_sec();
          if (args.mode == "hz") {
            if (prev_recv_steady >= 0.0) {
              stats.add(recv_steady, recv_steady - prev_recv_steady);
            }
            prev_recv_steady = recv_steady;
          } else if (args.mode == "delay") {
            const auto * base = static_cast<const uint8_t *>(payload);
            const int32_t sec =
              *reinterpret_cast<const int32_t *>(base + offsets->sec_offset);
            const uint32_t nsec =
              *reinterpret_cast<const uint32_t *>(base + offsets->nanosec_offset);
            const double stamp_wall =
              static_cast<double>(sec) + static_cast<double>(nsec) * 1e-9;
            const double delay = now_wall_sec() - stamp_wall;
            stats.add(recv_steady, delay);
          } else {
            // echo: dump full struct via introspection walker.
            auto yaml = agnocast::dump_message_yaml(args.type_name, payload);
            std::printf("--- entry_id=%ld\n", static_cast<long>(entry_id));
            if (yaml) {
              std::fwrite(yaml->data(), 1, yaml->size(), stdout);
            } else {
              std::printf("<failed to resolve type '%s'>\n", args.type_name.c_str());
            }
            std::fflush(stdout);
          }
        });
      }
    }

    if (args.mode == "echo") {
      // echo prints inline; skip the periodic statistics block entirely.
      continue;
    }

    if (now_sec() >= next_print) {
      double mean = 0;
      double vmin = 0;
      double vmax = 0;
      double sd = 0;
      size_t n = 0;
      if (stats.summarize(mean, vmin, vmax, sd, n)) {
        if (args.mode == "hz") {
          // mean is the average inter-arrival period; rate = 1/mean
          const double rate = (mean > 0.0) ? 1.0 / mean : 0.0;
          std::printf("average rate: %.3f\n", rate);
          std::printf(
            "\tmin: %.4fs max: %.4fs std dev: %.5fs window: %zu\n", vmin, vmax, sd, n);
        } else {
          std::printf("average delay: %.6fs\n", mean);
          std::printf(
            "\tmin: %.6fs max: %.6fs std dev: %.6fs window: %zu\n", vmin, vmax, sd, n);
        }
        std::fflush(stdout);
      } else {
        std::fprintf(stderr, "no messages yet\n");
      }
      next_print = now_sec() + 1.0;
    }
  }

  rclcpp::shutdown();
  return 0;
}
