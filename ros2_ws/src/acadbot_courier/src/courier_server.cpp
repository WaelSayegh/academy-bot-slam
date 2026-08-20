#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "acadbot_courier_interfaces/srv/accept_job.hpp"
#include "acadbot_courier_interfaces/action/execute_delivery.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "courier_bt_nodes.hpp"

#include <unordered_map>
#include <string>
#include <vector>
#include <mutex>
#include <atomic>   
#include <future>
#include <thread>
#include <chrono>
#include <algorithm>

using namespace std::chrono_literals;
using AcceptJob = acadbot_courier_interfaces::srv::AcceptJob;
using ExecuteDelivery = acadbot_courier_interfaces::action::ExecuteDelivery;
using GoalHandleDel = rclcpp_action::ServerGoalHandle<ExecuteDelivery>;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

struct Location {
    double x, y, yaw;
};

struct Job {
    std::string pickup, dropoff;
};

class CourierServer : public rclcpp::Node {
    public:
        CourierServer() : Node("courier_server") {
            frame_id_ = declare_parameter<std::string>("frame_id", "map");
            retry_count_ = declare_parameter<int>("retry_count", 2);
            retry_delay_sec_ = declare_parameter<double>("retry_delay_sec", 3.0);
            feedback_period_sec_ = declare_parameter<double>("feedback_period_sec", 1.0);
            leg_timeout_sec_ = declare_parameter<double>("leg_timeout_sec", 0.0);
            busy_reject_ = declare_parameter<bool>("busy_reject", true);
            mission_mode_ = declare_parameter<std::string>("mission_mode", "bt");

            auto names = declare_parameter<std::vector<std::string>>("location_names", std::vector<std::string>{});
            auto poses = declare_parameter<std::vector<double>>("location_poses", std::vector<double>{});
            for (size_t i = 0; i < names.size() && 3  * (i + 1) <= poses.size(); ++i) {
                locations_[names[i]] = {poses[3 * i], poses[3 * i + 1], poses[3 * i + 2]};
            }

            accept_service_ = create_service<AcceptJob>("courier/accept_job", std::bind(&CourierServer::handle_accept_job, this, std::placeholders::_1, std::placeholders::_2));

            nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

            feedback_timer_ = create_wall_timer(
                std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(feedback_period_sec_)),
                std::bind(&CourierServer::publish_status_feedback, this)
            );

            delivery_server_ = rclcpp_action::create_server<ExecuteDelivery>(
                this,
                "courier/execute_delivery",
                std::bind(&CourierServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&CourierServer::handle_cancel, this, std::placeholders::_1),
                std::bind(&CourierServer::handle_accepted, this, std::placeholders::_1)
            );

            RCLCPP_INFO(get_logger(), "Courier server initialized with %zu locations, %d retries", locations_.size(), retry_count_);
            for (const auto& [name, loc] : locations_) {
                RCLCPP_INFO(get_logger(), "Location %s: x=%.2f, y=%.2f, yaw=%.2f", name.c_str(), loc.x, loc.y, loc.yaw);
            }
        }
    
    private:
        std::string frame_id_;
        int retry_count_;
        double retry_delay_sec_;
        double feedback_period_sec_;
        double leg_timeout_sec_;
        bool busy_reject_;
        std::string mission_mode_;
        std::unordered_map<std::string, Location> locations_;

        rclcpp::Service<AcceptJob>::SharedPtr accept_service_;
        std::unordered_map<uint32_t, Job> jobs_;
        uint32_t next_job_id_{1};
        bool job_active_{false};
        std::atomic<uint32_t> active_job_id_{0};

        rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
        rclcpp::TimerBase::SharedPtr startup_timer_;

        rclcpp_action::Server<ExecuteDelivery>::SharedPtr delivery_server_;
        GoalHandleNav::SharedPtr active_nav_goal_;
        rclcpp::TimerBase::SharedPtr feedback_timer_;
        std::shared_ptr<GoalHandleDel> feedback_goal_;
        std::string fb_leg_, fb_target_;
        float fb_distance_{-1.0f};
        std::string fb_note_;
        std::mutex state_mutex_;
        std::atomic<bool> cancelling_{false};

        void handle_accept_job(const std::shared_ptr<AcceptJob::Request> request, std::shared_ptr<AcceptJob::Response> response) {
            auto pickup = locations_.find(request->pickup_name);
            auto dropoff = locations_.find(request->dropoff_name);

            if (pickup == locations_.end()) {
                response->accepted = false;
                response->reason = "unknown location '" + request->pickup_name + "'";
                return;
            }

            if (dropoff == locations_.end()) {
                response->accepted = false;
                response->reason = "unknown location '" + request->dropoff_name + "'";
                return;
            }

            if (request->pickup_name == request->dropoff_name) {
                response->accepted = false;
                response->reason = "pickup and dropoff locations are the same";
                return;
            }

            if (busy_reject_ && job_active_) {
                response->accepted = false;
                response->reason = "robot is busy with job " + std::to_string(active_job_id_.load());
                return;
            }

            const uint32_t id = next_job_id_++;
            jobs_[id] = {request->pickup_name, request->dropoff_name};
            response->accepted = true;
            response->job_id = id;

            RCLCPP_INFO(get_logger(), "Accepted job %u: %s -> %s", id, request->pickup_name.c_str(), request->dropoff_name.c_str());
        }

        const char* result_code_name(rclcpp_action::ResultCode code) {
            switch (code) {
                case rclcpp_action::ResultCode::SUCCEEDED: return "SUCCEEDED";
                case rclcpp_action::ResultCode::CANCELED: return "CANCELED";
                case rclcpp_action::ResultCode::ABORTED: return "ABORTED";
                case rclcpp_action::ResultCode::UNKNOWN: return "UNKNOWN";
                default: return "???";
            }
        }
        
        rclcpp_action::ResultCode drive_to(const std::string& name, const std::shared_ptr<GoalHandleDel>& goal_handle, const std::string& leg, uint32_t attempt) {
            auto it = locations_.find(name);
            if (it == locations_.end()) {
                RCLCPP_ERROR(get_logger(), "Unknown location '%s'", name.c_str());
                return rclcpp_action::ResultCode::ABORTED;
            }
            const Location& loc = it->second;

            NavigateToPose::Goal goal;
            goal.pose.header.frame_id = frame_id_;
            goal.pose.header.stamp = now();
            goal.pose.pose.position.x = loc.x;
            goal.pose.pose.position.y = loc.y;
            tf2::Quaternion q;
            q.setRPY(0, 0, loc.yaw);
            goal.pose.pose.orientation.x = q.x();
            goal.pose.pose.orientation.y = q.y();
            goal.pose.pose.orientation.z = q.z();
            goal.pose.pose.orientation.w = q.w();

            RCLCPP_INFO(get_logger(), "Driving to %s (%.2f,  %.2f, %.2f)", name.c_str(), loc.x, loc.y, loc.yaw);

            auto promise = std::make_shared<std::promise<rclcpp_action::ResultCode>>();
            auto future = promise->get_future();

            rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
            options.goal_response_callback = [this, promise](GoalHandleNav::SharedPtr gh) {
                if (!gh) {
                    promise->set_value(rclcpp_action::ResultCode::ABORTED);
                } else {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    active_nav_goal_ = gh;
                }
            };
            options.feedback_callback = [this](GoalHandleNav::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> fb) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                fb_distance_ = fb->distance_remaining;
                RCLCPP_INFO(get_logger(), "[%s] -> %s, %.2f m left", fb_leg_.c_str(), fb_target_.c_str(), fb->distance_remaining);
            };
            options.result_callback = [promise](const GoalHandleNav::WrappedResult& result) {
                promise->set_value(result.code);
            };

            if (!nav_client_->wait_for_action_server(10s)) {
                RCLCPP_WARN(get_logger(), "Nav2 'navigate_to_pose' server is not ready; aborting leg %s", leg.c_str());
                return rclcpp_action::ResultCode::ABORTED;
            }
            
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                fb_leg_ = leg;
                fb_target_ = name;
                fb_distance_ = -1.0f;
                feedback_goal_ = goal_handle;
                fb_note_ = "attempt " + std::to_string(attempt) + " of " + std::to_string(retry_count_);
            }

            nav_client_->async_send_goal(goal, options);

            const auto send_time = std::chrono::steady_clock::now();
            while (future.wait_for(250ms) != std::future_status::ready) {
                if (cancelling_) nav_client_->async_cancel_goal(active_nav_goal_);
                if (leg_timeout_sec_ > 0.0 && std::chrono::duration<double>(std::chrono::steady_clock::now() - send_time).count() > leg_timeout_sec_) {
                    RCLCPP_WARN(get_logger(), "Leg %s timed out after %.2f seconds; cancelling goal", leg.c_str(), leg_timeout_sec_);
                    nav_client_->async_cancel_goal(active_nav_goal_);
                    return rclcpp_action::ResultCode::ABORTED;  
                }
            }

            auto code = future.get();
            return code;
        }

        void publish_status_feedback() {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!feedback_goal_ || fb_distance_ < 0.0f) return;
            auto cf = std::make_shared<ExecuteDelivery::Feedback>();
            cf->leg = fb_leg_;
            cf->target_location = fb_target_;
            cf->distance_remaining = fb_distance_;
            cf->note = fb_note_;
            feedback_goal_->publish_feedback(cf);
        }

        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const ExecuteDelivery::Goal> goal) {
            if (busy_reject_ && job_active_) {
                RCLCPP_WARN(get_logger(), "Rejecting delivery goal: robot is busy with job %u", active_job_id_.load());
                return rclcpp_action::GoalResponse::REJECT;
            }
            if (jobs_.find(goal->job_id) == jobs_.end()) {
                RCLCPP_WARN(get_logger(), "Rejecting delivery goal: unknown job id %u", goal->job_id);
                return rclcpp_action::GoalResponse::REJECT;
            }
            RCLCPP_INFO(get_logger(), "Accepted delivery goal for job %u:", goal->job_id);
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleDel> goal_handle) {
            RCLCPP_INFO(get_logger(), "Received request to cancel delivery goal for job %u", goal_handle->get_goal()->job_id);
            cancelling_ = true; 
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (active_nav_goal_) {
                nav_client_->async_cancel_goal(active_nav_goal_);
            }
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleDel> goal_handle) {
            job_active_ = true;
            cancelling_ = false;
            std::thread{std::bind(&CourierServer::execute_delivery, this, std::placeholders::_1), goal_handle}.detach();
        }

        void execute_delivery(const std::shared_ptr<GoalHandleDel> goal_handle) {
            if (mission_mode_ == "bt") {
                execute_delivery_bt(goal_handle);
            } else {
                execute_delivery_loop(goal_handle);
            }
        }

        void execute_delivery_loop(const std::shared_ptr<GoalHandleDel> goal_handle) {
            const uint32_t job_id = goal_handle->get_goal()->job_id;
            const Job& job = jobs_[job_id];
            active_job_id_ = job_id;

            const std::string legs[2] = {job.pickup, job.dropoff};  
            const std::string leg_label[2] = {"pickup", "dropoff"};

            rclcpp_action::ResultCode outcome = rclcpp_action::ResultCode::UNKNOWN;
            std::string failed_leg;
            uint32_t attempts = 0;
            
            for (int i = 0; i < 2; ++i) {
                uint32_t attempt = 0;

                while (attempt < static_cast<uint32_t>(retry_count_) && !cancelling_) {
                    ++attempt;
                    ++attempts;
                    RCLCPP_INFO(get_logger(), "[job %u] %s leg -> %s (attempts %u of %d)", job_id, leg_label[i].c_str(), legs[i].c_str(), attempt, retry_count_);
                    outcome = drive_to(legs[i], goal_handle, leg_label[i], attempt);

                    if (outcome == rclcpp_action::ResultCode::SUCCEEDED) break;
                    if (attempt < static_cast<uint32_t>(retry_count_)) {
                        RCLCPP_WARN(get_logger(), "[job %u] %s leg -> %s failed (%s), retrying in %.1f seconds", job_id, leg_label[i].c_str(), legs[i].c_str(), result_code_name(outcome), retry_delay_sec_);
                        const int slices = std::max(1, static_cast<int>(retry_delay_sec_ * 10));
                        for (int s = 0; s < slices && !cancelling_; ++s) {
                            std::this_thread::sleep_for(100ms);
                        }
                    }
                }
                if (outcome != rclcpp_action::ResultCode::SUCCEEDED && !cancelling_) {
                    failed_leg = leg_label[i];
                    break;
                }
                if (cancelling_) break;
            }

            auto result = std::make_shared<ExecuteDelivery::Result>();
            result->success = (outcome == rclcpp_action::ResultCode::SUCCEEDED && !cancelling_);

            result->failed_leg = failed_leg;
            result->attempts = attempts;

            if (cancelling_) {
                goal_handle->canceled(result);
            } else if (result->success) {
                goal_handle->succeed(result);
            } else {
                goal_handle->abort(result);
            }

            finish_delivery();
        }

        void execute_delivery_bt(const std::shared_ptr<GoalHandleDel> goal_handle) {
            const uint32_t job_id = goal_handle->get_goal()->job_id;
            const Job& job = jobs_[job_id];
            active_job_id_ = job_id;

            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                feedback_goal_ = goal_handle;
                fb_distance_ = -1.0f;
            }

            RCLCPP_INFO(get_logger(), "[job %u] starting behavior-tree mission (mission_mode 'bt')", job_id);

            auto context = std::make_shared<acadbot_courier::CourierNavContext>();
            context->node = shared_from_this();
            context->frame_id = frame_id_;
            for (const auto& [name, loc] : locations_) {
                context->locations[name] = {loc.x, loc.y, loc.yaw};
            }
            context->is_cancelled = [this]() { return cancelling_.load(); };
            context->on_feedback = [this](const std::string& leg, const std::string& target,
                                          float distance, const std::string& note) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                fb_leg_ = leg;
                fb_target_ = target;
                fb_distance_ = distance;
                fb_note_ = note;
            };

            BT::BehaviorTreeFactory factory;
            acadbot_courier::registerCourierNodes(factory, context);

            auto blackboard = BT::Blackboard::create();
            auto pose_str = [&](const std::string & name) -> std::string {
                auto it = locations_.find(name);
                if (it == locations_.end()) { return "0,0,0"; }
                const auto & loc = it->second;
                return std::to_string(loc.x) + "," + std::to_string(loc.y) + "," +
                       std::to_string(loc.yaw);
            };
            blackboard->set<std::string>("pickup_name", job.pickup);
            blackboard->set<std::string>("dropoff_name", job.dropoff);
            blackboard->set<std::string>("pickup_pose", pose_str(job.pickup));
            blackboard->set<std::string>("dropoff_pose", pose_str(job.dropoff));
            blackboard->set<int>("retry_count", retry_count_);
            const int timeout_msec = leg_timeout_sec_ > 0.0
                                         ? static_cast<int>(leg_timeout_sec_ * 1000.0)
                                         : 120000;
            blackboard->set<unsigned int>("leg_timeout_msec",
                                          static_cast<unsigned int>(timeout_msec));
            blackboard->set<unsigned int>("retry_delay_msec",
                                          static_cast<unsigned int>(retry_delay_sec_ * 1000.0));
            blackboard->set<int>("attempts", 0);
            blackboard->set<std::string>("failed_leg", "");

            const std::string bt_path =
                ament_index_cpp::get_package_share_directory("acadbot_courier") +
                "/config/courier_bt.xml";
            auto tree = factory.createTreeFromFile(bt_path, blackboard);

            BT::NodeStatus status = BT::NodeStatus::RUNNING;
            while (rclcpp::ok() && !cancelling_.load() && status == BT::NodeStatus::RUNNING) {
                status = tree.tickOnce();
                if (status == BT::NodeStatus::RUNNING) {
                    std::this_thread::sleep_for(100ms);
                }
            }
            if (cancelling_.load()) {
                tree.haltTree();
            }

            auto result = std::make_shared<ExecuteDelivery::Result>();
            if (status == BT::NodeStatus::SUCCESS && !cancelling_.load()) {
                result->success = true;
                result->failed_leg = "";
            } else {
                result->success = false;
                static_cast<void>(blackboard->get("failed_leg", result->failed_leg));
            }
            int attempts = 0;
            static_cast<void>(blackboard->get("attempts", attempts));
            result->attempts = static_cast<uint32_t>(attempts);

            if (cancelling_.load()) {
                goal_handle->canceled(result);
            } else if (result->success) {
                goal_handle->succeed(result);
            } else {
                goal_handle->abort(result);
            }

            finish_delivery();
        }

        void finish_delivery() {
            job_active_ = false;

            std::lock_guard<std::mutex> lock(state_mutex_);
            feedback_goal_.reset();
            fb_distance_ = -1.0f;
        }


};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<CourierServer>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}