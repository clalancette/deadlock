#include <cassert>
#include <chrono>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "test_msgs/action/fibonacci.hpp"

using Fibonacci = test_msgs::action::Fibonacci;
using CancelResponse = typename Fibonacci::Impl::CancelGoalService::Response;
using GoalUUID = rclcpp_action::GoalUUID;

class TestServer
{
public:
  std::shared_ptr<Fibonacci::Impl::SendGoalService::Request>
  send_goal_request(rclcpp::Node::SharedPtr node, GoalUUID uuid)
  {
    auto client = node->create_client<Fibonacci::Impl::SendGoalService>(
      "fibonacci/_action/send_goal");
    if (!client->wait_for_service(std::chrono::seconds(20))) {
      throw std::runtime_error("send goal service didn't become available");
    }
    auto request = std::make_shared<Fibonacci::Impl::SendGoalService::Request>();
    request->goal_id.uuid = uuid;
    auto future = client->async_send_request(request);
    if (
      rclcpp::FutureReturnCode::SUCCESS !=
      rclcpp::spin_until_future_complete(node, future))
    {
      throw std::runtime_error("send goal future didn't complete succesfully");
    }
    return request;
  }
  CancelResponse::SharedPtr
  send_cancel_request(rclcpp::Node::SharedPtr node, GoalUUID uuid)
  {
    auto cancel_client = node->create_client<Fibonacci::Impl::CancelGoalService>(
      "fibonacci/_action/cancel_goal");
    if (!cancel_client->wait_for_service(std::chrono::seconds(20))) {
      throw std::runtime_error("cancel goal service didn't become available");
    }
    auto request = std::make_shared<Fibonacci::Impl::CancelGoalService::Request>();
    request->goal_info.goal_id.uuid = uuid;
    auto future = cancel_client->async_send_request(request);
    if (
      rclcpp::FutureReturnCode::SUCCESS !=
      rclcpp::spin_until_future_complete(node, future))
    {
      throw std::runtime_error("cancel goal future didn't complete succesfully");
    }
    return future.get();
  }
};

class TestDeadlockServer : public TestServer
{
public:
  TestDeadlockServer()
  {
    node_ = std::make_shared<rclcpp::Node>("goal_request", "/rclcpp_action/goal_request");
    uuid1_ = {{1, 2, 3, 4, 5, 6, 70, 80, 9, 1, 11, 120, 13, 140, 15, 160}};
    uuid2_ = {{2, 2, 3, 4, 5, 6, 70, 80, 9, 1, 11, 120, 13, 140, 15, 160}};
    action_server_ = rclcpp_action::create_server<Fibonacci>(
      node_, "fibonacci",
      [this](const GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
        // instead of making a deadlock, check if it can acquire the lock in a second
        std::unique_lock<std::recursive_timed_mutex> lock(server_mutex_, std::defer_lock);
        this->TryLockFor(lock, std::chrono::milliseconds(1000));
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<GoalHandle>) {
        // instead of making a deadlock, check if it can acquire the lock in a second
        std::unique_lock<std::recursive_timed_mutex> lock(server_mutex_, std::defer_lock);
        this->TryLockFor(lock, std::chrono::milliseconds(1000));
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<GoalHandle> handle) {
        // instead of making a deadlock, check if it can acquire the lock in a second
        std::unique_lock<std::recursive_timed_mutex> lock(server_mutex_, std::defer_lock);
        this->TryLockFor(lock, std::chrono::milliseconds(1000));
        goal_handle_ = handle;
      });
  }

  void GoalSucceeded()
  {
    std::lock_guard<std::recursive_timed_mutex> lock(server_mutex_);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    auto result = std::make_shared<Fibonacci::Result>();
    result->sequence = {5, 8, 13, 21};
    goal_handle_->succeed(result);
  }

  void GoalCanceled()
  {
    std::lock_guard<std::recursive_timed_mutex> lock(server_mutex_);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    auto result = std::make_shared<Fibonacci::Result>();
    goal_handle_->canceled(result);
  }

  void TryLockFor(
    std::unique_lock<std::recursive_timed_mutex> & lock,
    std::chrono::milliseconds timeout
  )
  {
    assert(lock.try_lock_for(timeout));
  }

  std::recursive_timed_mutex server_mutex_;
  GoalUUID uuid1_, uuid2_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp_action::Server<Fibonacci>> action_server_;

  using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;
  std::shared_ptr<GoalHandle> goal_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  TestDeadlockServer server;
  server.send_goal_request(server.node_, server.uuid1_);
  std::thread t(&TestDeadlockServer::GoalSucceeded, &server);
  rclcpp::sleep_for(std::chrono::milliseconds(50));
  server.send_goal_request(server.node_, server.uuid2_);
  t.join();
  rclcpp::shutdown();

  return 0;
}
