#include "acadbot_courier/job_registry.hpp"

#include <iomanip>
#include <sstream>
#include <utility>

namespace acadbot_courier
{

JobRegistry::JobRegistry(double ttl_sec)
: ttl_sec_(ttl_sec)
{
}

std::string JobRegistry::create(const std::string & pickup, const std::string & dropoff)
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::ostringstream id;
  id << "job_" << std::setw(4) << std::setfill('0') << next_id_++;

  Job job;
  job.id = id.str();
  job.pickup = pickup;
  job.dropoff = dropoff;
  job.created_at = std::chrono::steady_clock::now();
  jobs_.emplace(job.id, job);
  return job.id;
}

bool JobRegistry::claim(const std::string & id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto found = jobs_.find(id);
  if (found == jobs_.end() || found->second.state != JobState::PENDING) {
    return false;
  }
  if (expired_locked(found->second, ttl_sec_)) {
    jobs_.erase(found);
    return false;
  }
  for (const auto & entry : jobs_) {
    if (entry.second.state == JobState::ACTIVE) {
      return false;
    }
  }
  found->second.state = JobState::ACTIVE;
  return true;
}

void JobRegistry::finish(const std::string & id, const std::string & outcome)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto found = jobs_.find(id);
  if (found == jobs_.end()) {
    return;
  }
  found->second.state = JobState::FINISHED;
  found->second.outcome = outcome;
}

bool JobRegistry::has_active() const
{
  return active_id().has_value();
}

std::optional<std::string> JobRegistry::active_id() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & entry : jobs_) {
    if (entry.second.state == JobState::ACTIVE) {
      return entry.first;
    }
  }
  return std::nullopt;
}

std::optional<std::string> JobRegistry::pending_id() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & entry : jobs_) {
    if (entry.second.state == JobState::PENDING && !expired_locked(entry.second, ttl_sec_)) {
      return entry.first;
    }
  }
  return std::nullopt;
}

void JobRegistry::expire_stale(double ttl_sec)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto it = jobs_.begin(); it != jobs_.end();) {
    if (it->second.state == JobState::PENDING && expired_locked(it->second, ttl_sec)) {
      it = jobs_.erase(it);
    } else {
      ++it;
    }
  }
}

std::optional<Job> JobRegistry::get(const std::string & id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto found = jobs_.find(id);
  if (found == jobs_.end()) {
    return std::nullopt;
  }
  return found->second;
}

bool JobRegistry::expired_locked(const Job & job, double ttl_sec) const
{
  const std::chrono::duration<double> age = std::chrono::steady_clock::now() - job.created_at;
  return age.count() > ttl_sec;
}

}  // namespace acadbot_courier
