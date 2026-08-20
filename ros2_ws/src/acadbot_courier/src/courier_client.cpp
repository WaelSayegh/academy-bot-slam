#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "acadbot_courier_interfaces/srv/accept_job.hpp"
#include "acadbot_courier_interfaces/action/execute_delivery.hpp"

#include <chrono>
#include <csignal>
#include <cstdint>
#include <future>
#include <memory>
#include <string>

using namespace std::chrono_literals;

using AcceptJob = acadbot_courier_interfaces::srv::AcceptJob;
using ExecuteDelivery = acadbot_courier_interfaces::action::ExecuteDelivery;

namespace {

    struct DeliveryResult {
        rclcpp_action::ResultCode status = rclcpp_action::ResultCode::UNKNOWN;
        bool success = false;
        std::string failed_leg;
        uint32_t attempts = 0;
    };

    // Ctrl-C handling: rclcpp's default SIGINT handler calls shutdown() which
    // makes the wait loops below hang. Override it so we can cancel the action
    // cleanly and exit.
    volatile std::sig_atomic_t g_interrupt_requested = 0;

    void handle_sigint(int) {
        g_interrupt_requested = 1;
    }

}

class CourierClient : public rclcpp::Node {
    public:
        using GoalHandle =rclcpp_action::ClientGoalHandle<ExecuteDelivery>;

        CourierClient() : Node("courier_client") {
            this->declare_parameter<std::string>("pickup_name", "reception");
            this->declare_parameter<std::string>("dropoff_name", "lab_bench");
            this->declare_parameter<double>("cancel_after_sec", 0.0);

            pickup_name_ = this->get_parameter("pickup_name").as_string();
            dropoff_name_ = this->get_parameter("dropoff_name").as_string();
            const double cancel_after = this->get_parameter("cancel_after_sec").as_double();

            accept_client_ = this->create_client<AcceptJob>("/courier/accept_job");
            delivery_client_ = rclcpp_action::create_client<ExecuteDelivery>(
                this, "/courier/execute_delivery"
            );

            if (cancel_after > 0.0) {
                cancel_timer_ = this->create_wall_timer(
                    std::chrono::duration<double>(cancel_after),
                    [this]() {
                        RCLCPP_WARN(this->get_logger(), "cancel_after_sec elapsed - sending cancel request");
                        if (goal_handle_) {
                            delivery_client_->async_cancel_goal(goal_handle_);
                        }
                    }
                );
            }
        }

        bool accept_job(rclcpp::Executor& executor) {
            if (!accept_client_->wait_for_service(10s)) {
                RCLCPP_ERROR(this->get_logger(), "courier accept_job service not available");
                return false;
            }

            auto request = std::make_shared<AcceptJob::Request>();
            request->pickup_name = pickup_name_;
            request->dropoff_name = dropoff_name_;
            
            auto future = accept_client_->async_send_request(request);

            auto timeout = std::chrono::steady_clock::now() + 10s;
            while (future.wait_for(250ms) != std::future_status::ready) {
                if (std::chrono::steady_clock::now() > timeout) {
                    RCLCPP_ERROR(this->get_logger(), "timed out waiting for accept_job response");
                    return false;
                }
                if (g_interrupt_requested) {
                    RCLCPP_WARN(this->get_logger(), "interrupted while waiting for accept_job");
                    return false;
                }
                executor.spin_some();
            }
            executor.spin_some();
            auto response = future.get();
            job_id_ = response->job_id;
            RCLCPP_INFO(this->get_logger(), "accepted_job -> accepted: %s, job_id: %u, reason: '%s'",
                        response->accepted ? "true" : "false", job_id_, response->reason.c_str());
            return response->accepted;
        }

        int run_delivery(rclcpp::Executor& executor) {
            if (!delivery_client_->wait_for_action_server(10s)) {
                RCLCPP_ERROR(this->get_logger(), "ExecuteDelivery action server not available");
                return 1;
            }
            if (job_id_ == 0u) {
                RCLCPP_ERROR(this->get_logger(), "no valiid job_id - job was rejected");
                return 1;
            }
            RCLCPP_INFO(this->get_logger(), "Sending ExecuteDelivery for job %u: %s -> %s", job_id_, pickup_name_.c_str(), dropoff_name_.c_str());

            auto goal = ExecuteDelivery::Goal();
            goal.job_id = job_id_;

            auto result_promise = std::make_shared<std::promise<DeliveryResult>>();
            auto result_future =  result_promise->get_future();

            auto options = rclcpp_action::Client<ExecuteDelivery>::SendGoalOptions();
            options.goal_response_callback =
                [this] (const GoalHandle:: SharedPtr& goal_handle) {
                    if (!goal_handle) {
                        RCLCPP_ERROR(this->get_logger(), "goa rejected by courier server");
                        return;
                    }
                    goal_handle_ = goal_handle;
                    RCLCPP_INFO(this->get_logger(), "goal accepted (job %u)", job_id_);
                };
            options.feedback_callback = 
                [this] (GoalHandle:: SharedPtr,
                        const std:: shared_ptr<const ExecuteDelivery::Feedback> feedback) {
                    RCLCPP_INFO(this->get_logger(), "[%s -> %s] %0.2f m (%s)",
                                        feedback->leg.c_str(), feedback->target_location.c_str(),
                                        feedback->distance_remaining, feedback->note.c_str());
                };
            options.result_callback =
                [result_promise](const GoalHandle::WrappedResult& wrapped) {
                    DeliveryResult result;
                    result.status = wrapped.code;
                    if (wrapped.result) {
                        result.success = wrapped.result->success;
                        result.failed_leg = wrapped.result->failed_leg;
                        result.attempts = wrapped.result->attempts;
                    }
                    result_promise->set_value(result);
                };
            delivery_client_->async_send_goal(goal, options);

            const auto interrupt_start = std::chrono::steady_clock::now();
            bool cancel_sent = false;
            while (result_future.wait_for(250ms) != std::future_status::ready) {
                executor.spin_some();
                if (g_interrupt_requested) {
                    if (!cancel_sent) {
                        cancel_sent = true;
                        RCLCPP_WARN(this->get_logger(), "Ctrl-C received - cancelling delivery");
                        if (goal_handle_) {
                            delivery_client_->async_cancel_goal(goal_handle_);
                        }
                    }
                    if (std::chrono::steady_clock::now() - interrupt_start > 2s) {
                        RCLCPP_WARN(this->get_logger(), "cancel not confirmed within 2 s, exiting");
                        break;
                    }
                }
            }

            DeliveryResult result;
            if (result_future.wait_for(0s) != std::future_status::ready) {
                result.status = rclcpp_action::ResultCode::CANCELED;
                result.success = false;
                result.failed_leg = "interrupted";
            } else {
                result = result_future.get();
            }

            RCLCPP_INFO(this->get_logger(),
                        "Result: success=%s, failed_leg='%s', attempts=%u, status=%d",
                        result.success ? "true" : "false", result.failed_leg.c_str(),
                        result.attempts, static_cast<int>(result.status));

            switch (result.status) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    return 0;
                case rclcpp_action::ResultCode::CANCELED:
                    return 2;
                default:
                    return 1;
            }
        }

    private:                                                                            
       std::string pickup_name_;                                                           
       std::string dropoff_name_;                                                          
       uint32_t job_id_ = 0;                                                               
       GoalHandle::SharedPtr goal_handle_;                                                 
       rclcpp::Client<AcceptJob>::SharedPtr accept_client_;                                
       rclcpp_action::Client<ExecuteDelivery>::SharedPtr delivery_client_;                 
       rclcpp::TimerBase::SharedPtr cancel_timer_;                                         
};

int main(int argc, char ** argv) {                                                    
rclcpp::init(argc, argv);
    // Replace rclcpp's SIGINT handler so Ctrl-C lets us cancel the delivery
    // cleanly instead of hanging the wait loop.
    std::signal(SIGINT, handle_sigint);
    std::signal(SIGTERM, handle_sigint);
    auto node = std::make_shared<CourierClient>();
    rclcpp::executors::SingleThreadedExecutor executor;                                 
    executor.add_node(node);                                                            
                                                                                        
    if (!node->accept_job(executor)) {                                                          
        rclcpp::shutdown();                                                               
        return 1;                                                                         
    }                                                                                   
                                                                                        
    int rc = node->run_delivery(executor);                                              
    rclcpp::shutdown();                                                                 
    return rc;                                                                          
}   