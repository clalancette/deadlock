#include <functional>
#include <memory>
#include <random>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/compute_path_to_pose.hpp"

template<class ActionT>
class TestActionServer : public rclcpp::Node
{
public:
  explicit TestActionServer(
    const std::string & action_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("test_action_server", options)
  {
    this->action_server_ = rclcpp_action::create_server<ActionT>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      action_name,
      std::bind(&TestActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TestActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&TestActionServer::handle_accepted, this, std::placeholders::_1));
  }

protected:
  virtual rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const typename ActionT::Goal>)
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  virtual rclcpp_action::CancelResponse handle_cancel(
    const typename std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>)
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  virtual void execute(
    const typename std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle) = 0;

  void handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle)
  {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&TestActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

private:
  mutable std::recursive_mutex update_mutex_;
  bool server_active_{false};
  typename rclcpp_action::Server<ActionT>::SharedPtr action_server_;
};

class ComputePathToPoseActionServer : public TestActionServer<nav2_msgs::action::ComputePathToPose>
{
public:
  ComputePathToPoseActionServer()
  : TestActionServer("compute_path_to_pose")
  {}

protected:
  void execute(
    const typename std::shared_ptr<
      rclcpp_action::ServerGoalHandle<nav2_msgs::action::ComputePathToPose>> goal_handle)
  override
  {
    std::random_device rd; // obtain a random number from hardware
    std::mt19937 gen(rd()); // seed the generator
    std::uniform_real_distribution<double> distr(0.1, 1.0); // define the range

    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<nav2_msgs::action::ComputePathToPose::Result>();
    result->path.poses.resize(1);
    result->path.poses[0].pose.position.x = goal->pose.pose.position.x;
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(distr(gen) * 1000)));
    goal_handle->succeed(result);
  }
};

int main(int argc, char ** argv)
{
  // initialize ROS
  rclcpp::init(argc, argv);

  // initialize action server and spin on new thread
  auto action_server = std::make_shared<ComputePathToPoseActionServer>();

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(), 2);
  executor.add_node(action_server);
  executor.spin();

  // shutdown ROS
  rclcpp::shutdown();

  return 0;
}
