// Copyright 2026 Burf Robotics
#include "inmoov_hardware/mrl_client.hpp"

#include <curl/curl.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace inmoov_hardware
{

namespace
{

size_t writeCb(char * ptr, size_t size, size_t nmemb, void * userdata)
{
  auto * out = static_cast<std::string *>(userdata);
  out->append(ptr, size * nmemb);
  return size * nmemb;
}

std::string fmt(double v, int decimals = 2)
{
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.*f", decimals, v);
  return std::string(buf);
}

/// curl_global_init is not thread-safe and must happen once per process.
struct CurlGlobal
{
  CurlGlobal() {curl_global_init(CURL_GLOBAL_DEFAULT);}
  ~CurlGlobal() {curl_global_cleanup();}
};
CurlGlobal g_curl_global;

}  // namespace

MrlClient::MrlClient() = default;

MrlClient::~MrlClient()
{
  stop();
  if (blocking_handle_ != nullptr) {
    curl_easy_cleanup(static_cast<CURL *>(blocking_handle_));
    blocking_handle_ = nullptr;
  }
}

void MrlClient::configure(const MrlClientConfig & cfg, LogFn info, LogFn warn)
{
  cfg_ = cfg;
  info_ = std::move(info);
  warn_ = std::move(warn);
}

std::string MrlClient::urlFor(
  const std::string & service, const std::string & method, const std::string & json_arg) const
{
  std::ostringstream os;
  os << "http://" << cfg_.host << ":" << cfg_.port << "/api/service/" << service << "/" << method;
  if (!json_arg.empty()) {
    os << "/" << json_arg;
  }
  return os.str();
}

std::optional<std::string> MrlClient::httpGet(const std::string & url, void * handle)
{
  auto * curl = static_cast<CURL *>(handle);
  std::string body;
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCb);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &body);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, static_cast<long>(cfg_.request_timeout_s * 1000));
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS, 2000L);
  // Keep-alive: MRL wants one request per servo, so reusing the connection is the
  // difference between a pose costing ~20 TCP handshakes and ~0.
  curl_easy_setopt(curl, CURLOPT_TCP_KEEPALIVE, 1L);
  curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);

  const CURLcode rc = curl_easy_perform(curl);
  if (rc != CURLE_OK) {
    return std::nullopt;
  }
  long status = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
  if (status != 200) {
    return std::nullopt;
  }
  return body;
}

bool MrlClient::ping()
{
  std::lock_guard<std::mutex> lk(blocking_mtx_);
  if (blocking_handle_ == nullptr) {
    blocking_handle_ = curl_easy_init();
  }
  const auto body = httpGet(urlFor("runtime", "getServiceNames", ""), blocking_handle_);
  // A live MRL returns a JSON array of service names; anything shorter than "[]" plus a
  // name is not a working runtime.
  return body.has_value() && body->size() > 4;
}

std::optional<ServoLimits> MrlClient::getLimits(const std::string & service)
{
  std::lock_guard<std::mutex> lk(blocking_mtx_);
  if (blocking_handle_ == nullptr) {
    blocking_handle_ = curl_easy_init();
  }
  const auto mn = httpGet(urlFor(service, "getMin", ""), blocking_handle_);
  const auto mx = httpGet(urlFor(service, "getMax", ""), blocking_handle_);
  const auto rs = httpGet(urlFor(service, "getRest", ""), blocking_handle_);
  if (!mn || !mx || !rs) {
    return std::nullopt;
  }
  try {
    ServoLimits l;
    l.min = std::stod(*mn);
    l.max = std::stod(*mx);
    l.rest = std::stod(*rs);
    if (l.min > l.max) {
      return std::nullopt;
    }
    return l;
  } catch (const std::exception &) {
    // MRL answers "null" for a servo that exists but has no calibration -- treat that as
    // "no live data" and let the caller keep the YAML defaults.
    return std::nullopt;
  }
}

std::optional<double> MrlClient::getPosition(const std::string & service)
{
  std::lock_guard<std::mutex> lk(blocking_mtx_);
  if (blocking_handle_ == nullptr) {
    blocking_handle_ = curl_easy_init();
  }
  const auto p = httpGet(urlFor(service, "getPos", ""), blocking_handle_);
  if (!p) {
    return std::nullopt;
  }
  try {
    return std::stod(*p);
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

bool MrlClient::moveToBlocking(const std::string & service, double degrees)
{
  if (cfg_.dry_run) {
    if (info_) {
      info_("[DRY RUN] moveTo " + service + " -> " + fmt(degrees, 1));
    }
    return true;
  }
  std::lock_guard<std::mutex> lk(blocking_mtx_);
  if (blocking_handle_ == nullptr) {
    blocking_handle_ = curl_easy_init();
  }
  const auto r = httpGet(urlFor(service, "moveTo", fmt(degrees, 1)), blocking_handle_);
  if (r) {
    sent_count_.fetch_add(1);
    return true;
  }
  error_count_.fetch_add(1);
  return false;
}

void MrlClient::start()
{
  if (running_.exchange(true)) {
    return;
  }
  worker_ = std::thread(&MrlClient::workerLoop, this);
}

void MrlClient::stop()
{
  if (!running_.exchange(false)) {
    return;
  }
  cv_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
}

void MrlClient::setTargets(const std::map<std::string, double> & targets)
{
  std::lock_guard<std::mutex> lk(mtx_);
  for (const auto & [svc, deg] : targets) {
    targets_[svc] = deg;
  }
}

void MrlClient::forceResend()
{
  std::lock_guard<std::mutex> lk(mtx_);
  force_resend_ = true;
}

void MrlClient::workerLoop()
{
  // One easy handle per service, kept alive for the life of the worker. curl_multi then
  // drives them all concurrently, so a whole pose costs roughly one round trip rather
  // than one per servo.
  CURLM * multi = curl_multi_init();
  std::map<std::string, CURL *> handles;
  std::map<CURL *, std::string> bodies;

  const auto period = std::chrono::duration<double>(1.0 / cfg_.worker_rate_hz);

  while (running_.load()) {
    std::map<std::string, double> to_send;
    {
      std::unique_lock<std::mutex> lk(mtx_);
      cv_.wait_for(lk, period, [this] {return !running_.load();});
      const auto now = std::chrono::steady_clock::now();
      const bool force = force_resend_;
      force_resend_ = false;

      for (const auto & [svc, deg] : targets_) {
        const auto it_last = last_sent_deg_.find(svc);
        const bool changed = it_last == last_sent_deg_.end() ||
          std::abs(it_last->second - deg) >= cfg_.move_epsilon_deg;

        // hold_refresh_s <= 0 disables the anti-droop pulse entirely. These are hobby servos
        // with no thermal protection, and re-asserting a held pose forever keeps every one of
        // them energised; letting a limb sag is the safer failure. The explicit guard is
        // REQUIRED: without it a 0 interval means "always stale" and pulses harder, not less.
        bool stale = false;
        if (cfg_.hold_refresh_s > 0.0) {
          stale = true;
          const auto it_at = last_sent_at_.find(svc);
          if (it_at != last_sent_at_.end()) {
            stale = std::chrono::duration<double>(now - it_at->second).count() >=
              cfg_.hold_refresh_s;
          }
        }
        // Send if the value moved OR the held pose is due its anti-droop refresh pulse.
        if (force || changed || stale) {
          to_send[svc] = deg;
          last_sent_deg_[svc] = deg;
          last_sent_at_[svc] = now;
        }
      }
    }

    if (to_send.empty()) {
      continue;
    }

    if (cfg_.dry_run) {
      std::ostringstream os;
      os << "[DRY RUN] would send " << to_send.size() << " servo(s):";
      for (const auto & [svc, deg] : to_send) {
        os << " " << svc << "=" << fmt(deg, 1);
      }
      if (info_) {
        info_(os.str());
      }
      sent_count_.fetch_add(to_send.size());
      continue;
    }

    // Queue every move on the multi handle.
    std::vector<CURL *> in_flight;
    for (const auto & [svc, deg] : to_send) {
      auto it = handles.find(svc);
      if (it == handles.end()) {
        it = handles.emplace(svc, curl_easy_init()).first;
      }
      CURL * eh = it->second;
      bodies[eh].clear();
      const std::string url = urlFor(svc, "moveTo", fmt(deg, 1));
      // curl copies the URL string when CURLOPT_URL is set, so the temporary is fine.
      curl_easy_setopt(eh, CURLOPT_URL, url.c_str());
      curl_easy_setopt(eh, CURLOPT_WRITEFUNCTION, writeCb);
      curl_easy_setopt(eh, CURLOPT_WRITEDATA, &bodies[eh]);
      curl_easy_setopt(eh, CURLOPT_TIMEOUT_MS,
        static_cast<long>(cfg_.request_timeout_s * 1000));
      curl_easy_setopt(eh, CURLOPT_CONNECTTIMEOUT_MS, 2000L);
      curl_easy_setopt(eh, CURLOPT_TCP_KEEPALIVE, 1L);
      curl_easy_setopt(eh, CURLOPT_NOSIGNAL, 1L);
      curl_multi_add_handle(multi, eh);
      in_flight.push_back(eh);
    }

    int still_running = 0;
    do {
      const CURLMcode mc = curl_multi_perform(multi, &still_running);
      if (mc != CURLM_OK) {
        break;
      }
      if (still_running > 0) {
        curl_multi_poll(multi, nullptr, 0, 50, nullptr);
      }
    } while (still_running > 0 && running_.load());

    // Drain results so a failing servo is counted rather than silently ignored.
    int msgs_left = 0;
    while (CURLMsg * m = curl_multi_info_read(multi, &msgs_left)) {
      if (m->msg != CURLMSG_DONE) {
        continue;
      }
      long status = 0;
      curl_easy_getinfo(m->easy_handle, CURLINFO_RESPONSE_CODE, &status);
      if (m->data.result == CURLE_OK && status == 200) {
        sent_count_.fetch_add(1);
      } else {
        error_count_.fetch_add(1);
      }
    }
    for (CURL * eh : in_flight) {
      curl_multi_remove_handle(multi, eh);
    }

    const uint64_t errs = error_count_.load();
    if (errs > 0 && errs % 50 == 1 && warn_) {
      warn_("MRL write errors so far: " + std::to_string(errs) + " (is MRL up on " +
        cfg_.host + ":" + std::to_string(cfg_.port) + "?)");
    }
  }

  for (auto & [svc, eh] : handles) {
    (void)svc;
    curl_easy_cleanup(eh);
  }
  curl_multi_cleanup(multi);
}

}  // namespace inmoov_hardware
