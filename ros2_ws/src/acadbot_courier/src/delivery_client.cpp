#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <future>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "acadbot_courier_msgs/action/deliver.hpp"
#include "acadbot_courier_msgs/srv/request_delivery.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace
{

using Deliver = acadbot_courier_msgs::action::Deliver;
using RequestDelivery = acadbot_courier_msgs::srv::RequestDelivery;
using DeliverGoalHandle = rclcpp_action::ClientGoalHandle<Deliver>;
using namespace std::chrono_literals;

std::atomic<bool> cancel_requested{false};

void handle_signal(int signal_number)
{
  if (cancel_requested.exchange(true)) {
    std::signal(signal_number, SIG_DFL);
    std::raise(signal_number);
  }
}

template<typename FutureT>
bool wait_for_future(FutureT & future)
{
  while (rclcpp::ok() && future.wait_for(100ms) != std::future_status::ready) {
  }
  return future.wait_for(0ms) == std::future_status::ready;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 3) {
    std::cerr << "Usage: ros2 run acadbot_courier delivery_client <pickup> <dropoff>\n";
    return 2;
  }

  rclcpp::init(argc, argv);
  std::signal(SIGINT, handle_signal);
  auto node = std::make_shared<rclcpp::Node>("delivery_client");
  auto service_client = node->create_client<RequestDelivery>("/courier_node/request_delivery");
  auto action_client = rclcpp_action::create_client<Deliver>(node, "/courier_node/deliver");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  int exit_code = 1;
  do {
    while (!cancel_requested.load() && !service_client->wait_for_service(1s)) {
      std::cout << "Waiting for the courier request service...\n";
    }
    if (cancel_requested.load()) {
      std::cerr << "Interrupted before the courier service became available\n";
      break;
    }

    auto request = std::make_shared<RequestDelivery::Request>();
    request->pickup = argv[1];
    request->dropoff = argv[2];
    auto service_future = service_client->async_send_request(request);
    if (!wait_for_future(service_future)) {
      std::cerr << "Interrupted before the service replied\n";
      break;
    }
    const auto response = service_future.get();
    std::cout << response->reason << '\n';
    if (!response->accepted) {
      exit_code = 3;
      break;
    }

    while (!cancel_requested.load() && !action_client->wait_for_action_server(1s)) {
      std::cout << "Waiting for the courier delivery action...\n";
    }
    if (cancel_requested.load()) {
      std::cerr << "Interrupted before the delivery action became available\n";
      break;
    }

    Deliver::Goal goal;
    goal.job_id = response->job_id;
    rclcpp_action::Client<Deliver>::SendGoalOptions options;
    options.feedback_callback =
      [](DeliverGoalHandle::SharedPtr, const std::shared_ptr<const Deliver::Feedback> feedback) {
        std::cout << "job=" << feedback->job_id
                  << " leg=" << feedback->leg
                  << " target=" << feedback->target_location
                  << " distance=" << feedback->distance_remaining << "m"
                  << " attempt=" << feedback->attempt
                  << " state=" << feedback->state
                  << " elapsed=" << feedback->elapsed_sec << "s\n";
      };

    auto goal_future = action_client->async_send_goal(goal, options);
    if (!wait_for_future(goal_future)) {
      std::cerr << "Interrupted before the action server replied\n";
      break;
    }
    const auto goal_handle = goal_future.get();
    if (!goal_handle) {
      std::cerr << "Delivery goal was rejected\n";
      exit_code = 4;
      break;
    }

    auto result_future = action_client->async_get_result(goal_handle);
    bool cancel_sent = false;
    while (rclcpp::ok() && result_future.wait_for(100ms) != std::future_status::ready) {
      if (cancel_requested.load() && !cancel_sent) {
        std::cout << "Cancellation requested; waiting for the terminal result...\n";
        action_client->async_cancel_goal(goal_handle);
        cancel_sent = true;
      }
    }
    if (result_future.wait_for(0ms) != std::future_status::ready) {
      std::cerr << "No terminal delivery result received\n";
      break;
    }

    const auto wrapped_result = result_future.get();
    if (!wrapped_result.result) {
      std::cerr << "Delivery returned an empty result\n";
      break;
    }
    std::cout << "outcome=" << wrapped_result.result->outcome
              << " success=" << (wrapped_result.result->success ? "true" : "false")
              << " failed_leg=" << wrapped_result.result->failed_leg
              << " attempts_used=" << wrapped_result.result->attempts_used
              << " message=" << wrapped_result.result->message << '\n';
    exit_code = wrapped_result.result->success ? 0 : 5;
  } while (false);

  rclcpp::shutdown();
  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  return exit_code;
}
