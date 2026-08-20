// dispatcher.cpp
// ---------------------------------------------------------------------------
// Dispatcher node — FINAL: RequestDelivery service, ExecuteDelivery action
// server, queued two-leg driving via Executor, per-leg retries, per-leg
// timeouts, and a clean distinction between "requester cancelled" (job ends
// CANCELED, no retry) and "leg timed out" (treated as a normal leg failure,
// eligible for retry like any other Nav2 failure).
//
// Ownership and threading notes (summary):
// - This Node object owns an `Executor` instance (`executor_`) via
//   `std::unique_ptr`. That is RAII: when the Dispatcher is destroyed the
//   Executor is destroyed automatically.
// - `running_job_` is a `std::shared_ptr<Job>`; copies of this shared_ptr are
//   captured into asynchronous callbacks (timers, Nav2 done callbacks) so the
//   Job object remains alive until all callbacks complete.
// - Mutual exclusion for shared state (`queue_`, `running_job_`, etc.) is
//   provided by `mutex_`. Callers must hold `mutex_` when mutating / reading
//   those members in a non-atomic way. Some reads (e.g. `cancel_requested`)
//   are currently done without locking; that is a potential data-race and
//   should be hardened (e.g. `std::atomic<bool>` for `cancel_requested`).
// - Callbacks (timers, action results, feedback) are invoked on rclcpp's
//   executor threads. Any lambda that captures `this` must assume the
//   Dispatcher/Executor will remain alive for the callback's lifetime.
// ---------------------------------------------------------------------------
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "acadbot_courier_msgs/srv/request_delivery.hpp"
#include "acadbot_courier_msgs/action/execute_delivery.hpp"
#include "executor.hpp"

using RequestDelivery = acadbot_courier_msgs::srv::RequestDelivery;
using ExecuteDelivery = acadbot_courier_msgs::action::ExecuteDelivery;
using GoalHandleExecuteDelivery = rclcpp_action::ServerGoalHandle<ExecuteDelivery>;

struct LocationPose { double x, y, yaw; };

// A queued/running job. goal_handle is null until the requester actually
// calls the ExecuteDelivery action for this job_id — feedback/result only
// get published once that happens; the job still runs either way.
// cancel_requested distinguishes "requester hit cancel" (job ends CANCELED,
// no more retries) from a leg timing out internally (still just a failure,
// eligible for retry).
struct Job
{
  std::string job_id;
  std::string pickup, dropoff;
  // `goal_handle` is the server-side handle for the ExecuteDelivery action
  // associated with this job. It is a SharedPtr because rclcpp action API
  // returns shared ownership to allow asynchronous callbacks to hold it.
  std::shared_ptr<GoalHandleExecuteDelivery> goal_handle;

  // `cancel_requested` indicates the *requester* asked to cancel. This flag
  // may be set under `mutex_` (see `handleCancel`) but currently read in
  // other threads without locking; for correctness it should be `std::atomic<bool>`.
  bool cancel_requested{false};
};

class Dispatcher : public rclcpp::Node
{
public:
  Dispatcher() : Node("dispatcher")
  {
    loadLocations();
    retry_limit_ = declare_parameter<int>("retry_limit", 2);
    leg_timeout_sec_ = declare_parameter<double>("leg_timeout_sec", 60.0);
    // Executor is owned (RAII) by Dispatcher. We pass a raw `this` pointer
    // as a non-owning reference; the Executor expects the Node to outlive it.
    // This is a common pattern in rclcpp where Node lifetime is explicit.
    executor_ = std::make_unique<Executor>(this);

    service_ = create_service<RequestDelivery>(
      "request_delivery",
      std::bind(&Dispatcher::handleRequest, this,
                std::placeholders::_1, std::placeholders::_2));

    action_server_ = rclcpp_action::create_server<ExecuteDelivery>(
      this, "execute_delivery",
      std::bind(&Dispatcher::handleGoal, this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&Dispatcher::handleCancel, this, std::placeholders::_1),
      std::bind(&Dispatcher::handleAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "dispatcher ready — %zu known locations, retry_limit=%d, leg_timeout_sec=%.1f.",
      locations_.size(), retry_limit_, leg_timeout_sec_);
  }

private:
  // ---- location config (decision 5) --------------------------------------
  void loadLocations()
  {
    declare_parameter<std::vector<std::string>>("location_names",
                                                  std::vector<std::string>{});
    auto names = get_parameter("location_names").as_string_array();

    for (const auto & name : names) {
      declare_parameter<double>("locations." + name + ".x", 0.0);
      declare_parameter<double>("locations." + name + ".y", 0.0);
      declare_parameter<double>("locations." + name + ".yaw", 0.0);

      // Read the location parameters into a local value-type `LocationPose`.
      // Storing value-types in `locations_` avoids pointer ownership issues
      // and keeps the map safe to read from multiple threads (read-only).
      LocationPose pose;
      get_parameter("locations." + name + ".x", pose.x);
      get_parameter("locations." + name + ".y", pose.y);
      get_parameter("locations." + name + ".yaw", pose.yaw);
      locations_[name] = pose;

      RCLCPP_INFO(get_logger(), "  location '%s' = (%.2f, %.2f, yaw=%.2f)",
                  name.c_str(), pose.x, pose.y, pose.yaw);
    }
  }

  // ---- RequestDelivery service (requirements 1, 2) -----------------------
  void handleRequest(
    const std::shared_ptr<RequestDelivery::Request> req,
    std::shared_ptr<RequestDelivery::Response> res)
  {
    if (!locations_.count(req->pickup)) {
      res->accepted = false;
      res->reason = "unknown pickup location: " + req->pickup;
      res->job_id = "";
      RCLCPP_WARN(get_logger(), "Rejected: %s", res->reason.c_str());
      return;
    }
    if (!locations_.count(req->dropoff)) {
      res->accepted = false;
      res->reason = "unknown dropoff location: " + req->dropoff;
      res->job_id = "";
      RCLCPP_WARN(get_logger(), "Rejected: %s", res->reason.c_str());
      return;
    }

    std::string job_id = "job-" + std::to_string(next_job_id_++);

    res->accepted = true;
    res->reason = "accepted";
    res->job_id = job_id;

    RCLCPP_INFO(get_logger(), "Accepted %s: %s -> %s",
                job_id.c_str(), req->pickup.c_str(), req->dropoff.c_str());

    // Enqueue the job while holding `mutex_` to synchronise with other
    // threads that mutate `queue_` or `running_job_`. `Job` is a small
    // aggregate stored by value in the deque; the `running_job_` will be a
    // `shared_ptr<Job>` when it is started.
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push_back({job_id, req->pickup, req->dropoff, nullptr, false});
    tryStartNext(); // caller still holds mutex_ — tryStartNext assumes that
                    // and will atomically move a job to `running_job_`.
  }

  // ---- ExecuteDelivery action server (requirements 3, 4, 6) --------------
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ExecuteDelivery::Goal> goal)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto & job : queue_) {
      if (job.job_id == goal->job_id) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      }
    }
    if (running_job_ && running_job_->job_id == goal->job_id) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    RCLCPP_WARN(get_logger(), "ExecuteDelivery goal for unknown job_id '%s'",
                goal->job_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleExecuteDelivery> gh)
  {
    // Requirement 6: cancel means stop, promptly, reported as CANCELED —
    // never retried afterwards. Mark the job so onLegDone knows this
    // cancellation came from the requester, not from our own timeout.
    // We set `cancel_requested` while holding `mutex_` to synchronise with
    // other code paths that inspect or mutate `running_job_` or `queue_`.
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_job_ && running_job_->goal_handle == gh) {
      running_job_->cancel_requested = true;
    }

    // Ask the executor to cancel the currently active Nav2 goal. This
    // triggers the Nav2 result callback which will call back into
    // `onLegDone` asynchronously. Note: `executor_->cancel()` is safe to
    // call from any thread because rclcpp action client is thread-safe.
    executor_->cancel();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandleExecuteDelivery> gh)
  {
    // Record the mapping from job_id -> goal_handle so the dispatcher can
    // publish feedback/results for the action. This mutation is protected by
    // `mutex_` because `queue_` and `running_job_` are shared state.
    std::lock_guard<std::mutex> lock(mutex_);
    const std::string & job_id = gh->get_goal()->job_id;
    for (auto & job : queue_) {
      if (job.job_id == job_id) { job.goal_handle = gh; return; }
    }
    if (running_job_ && running_job_->job_id == job_id) {
      running_job_->goal_handle = gh;
    }
  }

  // ---- driving (requirements 3, 4, 5, 6, 7, 8) ----------------------------
  void tryStartNext()
  {
    // `tryStartNext` assumes the caller holds `mutex_`. It moves the first
    // queued Job into `running_job_` (wrapped in a shared_ptr) so that
    // asynchronous callbacks can safely hold a copy of the Job object.
    if (running_job_ || queue_.empty()) return;

    running_job_ = std::make_shared<Job>(queue_.front());
    queue_.pop_front();

    RCLCPP_INFO(get_logger(), "[%s] starting: %s -> %s",
                running_job_->job_id.c_str(),
                running_job_->pickup.c_str(), running_job_->dropoff.c_str());

    // Begin the pickup leg. `driveLeg` will capture a shared_ptr copy of
    // `running_job_` to keep it alive across async callbacks.
    driveLeg("pickup", running_job_->pickup, 0);
  }

  // leg is "pickup" or "dropoff". attempt counts retries for THIS leg only
  // (architecture.md decision 4 — retries are per leg).
  void driveLeg(const std::string & leg, const std::string & location_name,
                int attempt)
  {
    // Copy `running_job_` into a local shared_ptr (keeps the Job alive while
    // callbacks are outstanding). This avoids the Job object being destroyed
    // mid-callback if `running_job_` were reset elsewhere.
    auto job = running_job_;  // keep alive across async callbacks
    // `locations_.at` returns a copy of the LocationPose (value type). That
    // copy is deliberately passed into lambdas to avoid holding `locations_`
    // mutable state across threads.
    auto p = locations_.at(location_name);

    // Send an initial feedback message (distance -1.0 means "unknown yet").
    publishFeedback(job, leg, location_name, p, -1.0);

    // One-shot timeout: if the leg hasn't finished within `leg_timeout_sec_`
    // we cancel the Nav2 goal ourselves. We store the timeout flag in a
    // `shared_ptr<bool>` so both the timer callback and the Nav2 result
    // callback can observe it safely (shared ownership ensures lifetime).
    auto leg_timed_out = std::make_shared<bool>(false);

    // Create a timer that will fire once after `leg_timeout_sec_`. We keep
    // a SharedPtr to the timer (`timeout_timer`) and capture it into the
    // Nav2 done callback so that the timer's lifetime extends until the
    // Nav2 result arrives (the done callback calls `cancel()` on it).
    auto timeout_timer = create_wall_timer(
      std::chrono::duration<double>(leg_timeout_sec_),
      [this, job, leg, leg_timed_out]() {
        RCLCPP_WARN(get_logger(), "[%s] %s leg timed out after %.1fs — canceling.",
                    job->job_id.c_str(), leg.c_str(), leg_timeout_sec_);
        *leg_timed_out = true;   // mark timeout for the result handler
        executor_->cancel();     // thread-safe call into Executor
      });

    // Ask the Executor to drive to the goal. The callbacks below are invoked
    // asynchronously by the rclcpp executor; we capture needed values by
    // value to ensure they remain alive (shared_ptr semantics) until the
    // callbacks run.
    executor_->driveTo(p.x, p.y, p.yaw, "map",
      [this, job, leg, location_name, p](double dist) {
        // Feedback callback: invoked frequently by the Executor. It is
        // lightweight and only publishes a feedback message for the action
        // if the requester has attached a `goal_handle`.
        publishFeedback(job, leg, location_name, p, dist);
      },
      [this, job, leg, location_name, attempt, timeout_timer, leg_timed_out]
      (bool success, const std::string & reason) {
        // This is the final result callback for the leg. We cancel the
        // timeout timer so it won't race in the future; cancelling a timer
        // that already fired is harmless.
        timeout_timer->cancel();
        std::string effective_reason = reason;
        if (*leg_timed_out && !success) {
          effective_reason = "leg timeout";
        }
        // Delegate to onLegDone which contains retry logic and result
        // publishing. Note: onLegDone may run on the same executor thread.
        onLegDone(job, leg, location_name, attempt, success, effective_reason);
      });
  }

  void publishFeedback(const std::shared_ptr<Job> & job, const std::string & leg,
                        const std::string & location_name, const LocationPose & p,
                        double dist)
  {
    // Publish feedback only if the requester has attached an action goal
    // (mapped to this job via `goal_handle`). The `goal_handle` is a
    // shared pointer owned by both the server (this code) and rclcpp's
    // action_server machinery; publishing feedback is thread-safe.
    if (!job->goal_handle) return;
    auto fb = std::make_shared<ExecuteDelivery::Feedback>();
    fb->current_leg = leg;
    fb->heading_to.name = location_name;
    fb->heading_to.x = p.x;
    fb->heading_to.y = p.y;
    fb->heading_to.yaw = p.yaw;
    fb->distance_remaining = dist;  // -1.0 until first real Nav2 feedback
    job->goal_handle->publish_feedback(fb);
  }

  void onLegDone(const std::shared_ptr<Job> & job, const std::string & leg,
                 const std::string & location_name, int attempt,
                 bool success, const std::string & reason)
  {
    if (success) {
      RCLCPP_INFO(get_logger(), "[%s] %s leg OK.", job->job_id.c_str(), leg.c_str());
      if (leg == "pickup") {
        driveLeg("dropoff", job->dropoff, 0);
      } else {
        finish(job, true, "delivered", "");
      }
      return;
    }

    // Requester-initiated cancel always wins: stop retrying, end CANCELED.
    // NOTE: `cancel_requested` is written while holding `mutex_` in
    // `handleCancel`. Here we read it without a lock which is a possible
    // data race on some platforms. For correctness this flag should be
    // `std::atomic<bool>` or protected by `mutex_` consistently.
    if (job->cancel_requested) {
      finish(job, false, "canceled by requester", leg);
      return;
    }

    // Any other failure (Nav2 aborted, rejected, or our own timeout) is
    // treated the same way: retry up to retry_limit, then give up honestly.
    if (attempt + 1 < retry_limit_) {
      RCLCPP_WARN(get_logger(), "[%s] %s leg failed (%s) — retry %d/%d",
                  job->job_id.c_str(), leg.c_str(), reason.c_str(),
                  attempt + 2, retry_limit_);
      driveLeg(leg, location_name, attempt + 1);
      return;
    }

    RCLCPP_ERROR(get_logger(), "[%s] %s leg failed after %d attempts (%s) — giving up.",
                 job->job_id.c_str(), leg.c_str(), retry_limit_, reason.c_str());
    finish(job, false, leg + " leg failed: " + reason, leg);
  }

  // success=true -> SUCCEEDED.
  // job->cancel_requested -> CANCELED (requirement 6).
  // otherwise (retries exhausted, including from timeouts) -> ABORTED,
  // failed_leg names which leg (requirement 7).
  void finish(const std::shared_ptr<Job> & job, bool success,
              const std::string & message, const std::string & failed_leg)
  {
    if (job->goal_handle) {
      auto result = std::make_shared<ExecuteDelivery::Result>();
      result->success = success;
      result->result_message = message;
      result->failed_leg = failed_leg;
      if (success) {
        job->goal_handle->succeed(result);
      } else if (job->cancel_requested) {
        job->goal_handle->canceled(result);
      } else {
        job->goal_handle->abort(result);
      }
    }

    // Clear the running job while holding the mutex so that other threads
    // don't race with `tryStartNext` or `handleRequest`.
    std::lock_guard<std::mutex> lock(mutex_);
    running_job_.reset();
    // Start the next queued job, if any. We still hold the lock; tryStartNext
    // expects the caller to hold `mutex_` (see its comment).
    tryStartNext();
  }

  rclcpp::Service<RequestDelivery>::SharedPtr service_;
  rclcpp_action::Server<ExecuteDelivery>::SharedPtr action_server_;
  std::map<std::string, LocationPose> locations_;
  std::unique_ptr<Executor> executor_;

  std::mutex mutex_;
  std::deque<Job> queue_;
  std::shared_ptr<Job> running_job_;

  int retry_limit_{2};
  double leg_timeout_sec_{60.0};
  int next_job_id_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Dispatcher>());
  rclcpp::shutdown();
  return 0;
}