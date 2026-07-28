// Copyright 2026 Burf Robotics
//
// Talks to MyRobotLab on pi5-inmoov DIRECTLY over the network from skynet. Nothing is
// installed on the pi5.
//
// MRL's Nixie API is plain HTTP -- `GET /api/service/{service}/{method}/{jsonArg}` --
// so there is no need for a WebSocket at all. Each path segment is JSON-decoded by MRL,
// hence numbers go on the URL bare and strings would need quoting (see mrl_client.py on
// the pi5, which learned that the hard way).
//
// Two access patterns:
//   * Blocking helpers (ping / getLimits / moveToBlocking) for setup and teardown.
//     These are only ever called from non-realtime lifecycle transitions.
//   * setTargets(), which hands a whole pose to a background worker thread. write() must
//     never block the ros2_control update loop on LAN I/O, and MRL needs one HTTP request
//     PER SERVO, so the worker fans them out concurrently with curl_multi over keep-alive
//     connections.
//
// The worker also owns the droop refresh: InMoov servos sag if they are not re-commanded,
// so a pose that has not changed is still re-asserted every hold_refresh_s. That is
// lifted straight from inmoov_quest_bridge.py, which exists because the head sank.

#ifndef INMOOV_HARDWARE__MRL_CLIENT_HPP_
#define INMOOV_HARDWARE__MRL_CLIENT_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace inmoov_hardware
{

struct ServoLimits
{
  double min{0.0};
  double max{180.0};
  double rest{90.0};
};

struct MrlClientConfig
{
  std::string host{"pi5-inmoov.local"};
  int port{8888};
  /// Only push a servo once it has moved at least this many degrees. Stops us spamming
  /// MRL with sub-degree noise from a 50 Hz controller.
  double move_epsilon_deg{1.0};
  /// ...but re-assert every servo at least this often anyway, or held poses droop.
  double hold_refresh_s{0.5};
  /// How often the worker wakes to consider sending. 20 Hz is well above what MRL's
  /// serial servo bus can usefully absorb.
  double worker_rate_hz{20.0};
  /// Per-request timeout. Kept short so one wedged servo cannot stall the whole pose.
  double request_timeout_s{1.0};
  /// When true NOTHING is sent to MRL; every would-be write is handed to the log sink
  /// instead. This is the dry-run flag.
  bool dry_run{false};
};

class MrlClient
{
public:
  using LogFn = std::function<void (const std::string &)>;

  MrlClient();
  ~MrlClient();

  MrlClient(const MrlClient &) = delete;
  MrlClient & operator=(const MrlClient &) = delete;

  void configure(const MrlClientConfig & cfg, LogFn info, LogFn warn);

  const MrlClientConfig & config() const {return cfg_;}

  /// True if MRL answers runtime/getServiceNames with a non-empty list.
  bool ping();

  /// Read a servo's calibrated min/max/rest live (getMin/getMax/getRest). nullopt if any
  /// of the three cannot be read, so the caller can fall back to the YAML defaults.
  std::optional<ServoLimits> getLimits(const std::string & service);

  /// Read a servo's current position (getPos). See the comment in inmoov_system.cpp
  /// about what this value actually means -- it is MRL's own idea of where the servo is,
  /// not an encoder.
  std::optional<double> getPosition(const std::string & service);

  /// Blocking single move, used for the rest pose on deactivate and for setup probes.
  bool moveToBlocking(const std::string & service, double degrees);

  /// Start the background writer. Idempotent.
  void start();
  /// Stop the background writer and join it. Idempotent.
  void stop();

  /// Hand the worker the current desired pose (service -> degrees). Non-blocking:
  /// takes a short mutex and returns. Safe to call from write() at controller rate.
  void setTargets(const std::map<std::string, double> & targets);

  /// Force every currently-known target to be re-sent on the next worker pass,
  /// regardless of epsilon. Used when going to the rest pose.
  void forceResend();

  /// Total successful moveTo calls since start(); used by the smoke tests.
  uint64_t sentCount() const {return sent_count_.load();}
  /// Requests that returned an error or timed out.
  uint64_t errorCount() const {return error_count_.load();}

private:
  std::string urlFor(const std::string & service, const std::string & method,
    const std::string & json_arg) const;
  /// Blocking GET. Returns the body, or nullopt on any transport/HTTP error.
  std::optional<std::string> httpGet(const std::string & url, void * curl_easy_handle);

  void workerLoop();

  MrlClientConfig cfg_;
  LogFn info_;
  LogFn warn_;

  mutable std::mutex mtx_;
  std::map<std::string, double> targets_;                       // desired
  std::map<std::string, double> last_sent_deg_;                  // what MRL last got
  std::map<std::string, std::chrono::steady_clock::time_point> last_sent_at_;
  bool force_resend_{false};

  std::thread worker_;
  std::atomic<bool> running_{false};
  std::condition_variable cv_;

  std::atomic<uint64_t> sent_count_{0};
  std::atomic<uint64_t> error_count_{0};

  void * blocking_handle_{nullptr};   // CURL* for the blocking helpers
  std::mutex blocking_mtx_;
};

}  // namespace inmoov_hardware

#endif  // INMOOV_HARDWARE__MRL_CLIENT_HPP_
