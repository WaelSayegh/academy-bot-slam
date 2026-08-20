#ifndef ACADBOT_COURIER__JOB_REGISTRY_HPP_
#define ACADBOT_COURIER__JOB_REGISTRY_HPP_

#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

namespace acadbot_courier
{

enum class JobState
{
  PENDING,
  ACTIVE,
  FINISHED
};

struct Job
{
  std::string id;
  std::string pickup;
  std::string dropoff;
  JobState state{JobState::PENDING};
  std::string outcome;
  std::chrono::steady_clock::time_point created_at;
};

class JobRegistry
{
public:
  explicit JobRegistry(double ttl_sec);

  std::string create(const std::string & pickup, const std::string & dropoff);
  bool claim(const std::string & id);
  void finish(const std::string & id, const std::string & outcome);
  bool has_active() const;
  std::optional<std::string> active_id() const;
  std::optional<std::string> pending_id() const;
  void expire_stale(double ttl_sec);
  std::optional<Job> get(const std::string & id) const;

private:
  bool expired_locked(const Job & job, double ttl_sec) const;

  mutable std::mutex mutex_;
  std::unordered_map<std::string, Job> jobs_;
  std::uint64_t next_id_{1};
  double ttl_sec_;
};

}  // namespace acadbot_courier

#endif  // ACADBOT_COURIER__JOB_REGISTRY_HPP_
