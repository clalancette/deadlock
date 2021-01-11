#include <cassert>
#include <chrono>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "simple_action_server.hpp"

#include "nav2_msgs/action/compute_path_to_pose.hpp"

class NodeThread
{
public:
  explicit NodeThread(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base)
: node_(node_base)
  {
    thread_ = std::make_unique<std::thread>(
      [&]()
      {
        executor_.add_node(node_);
        executor_.spin();
        executor_.remove_node(node_);
      });
  }

  template<typename NodeT>
  explicit NodeThread(NodeT node)
  : NodeThread(node->get_node_base_interface())
  {}

  ~NodeThread()
  {
    executor_.cancel();
    thread_->join();
  }

protected:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_;
  std::unique_ptr<std::thread> thread_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

class PlannerServer : public rclcpp::Node
{
public:
  PlannerServer()
  : rclcpp::Node("nav2_planner")
  {
    rclcpp_node_ = std::make_shared<rclcpp::Node>("internal_node");
    rclcpp_thread_ = std::make_unique<NodeThread>(rclcpp_node_);

    action_server_ = std::make_unique<nav2_util::SimpleActionServer<nav2_msgs::action::ComputePathToPose>>(
      rclcpp_node_,
      "compute_path_to_pose",
      std::bind(&PlannerServer::computePlan, this));
    action_server_->activate();
  }

  ~PlannerServer()
  {
  }

  void computePlan()
  {
    RCLCPP_INFO(get_logger(), "computePlan start\n");
    auto start_time = steady_clock_.now();

    // Initialize the ComputePathToPose goal and result
    auto goal = action_server_->get_current_goal();
    auto result = std::make_shared<nav2_msgs::action::ComputePathToPose::Result>();

    try {
      if (action_server_ == nullptr || !action_server_->is_server_active()) {
        RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
        return;
      }

      if (action_server_->is_cancel_requested()) {
        RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling planning action.");
        action_server_->terminate_all();
        return;
      }

      // CHANGE: potentially wait here with a sleep
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      if (action_server_->is_preempt_requested()) {
        goal = action_server_->accept_pending_goal();
      }

      // CHANGE: getPlan here (potentially taking some time)

      action_server_->succeeded_current(result);

    } catch (const std::exception & ex) {
      RCLCPP_WARN(get_logger(), "Failed to plan");
      action_server_->terminate_current();
    }
    RCLCPP_INFO(get_logger(), "computePlan end\n");
  }

private:
  std::unique_ptr<nav2_util::SimpleActionServer<nav2_msgs::action::ComputePathToPose>> action_server_;

  rclcpp::Node::SharedPtr rclcpp_node_;
  std::unique_ptr<NodeThread> rclcpp_thread_;

  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlannerServer>();

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
