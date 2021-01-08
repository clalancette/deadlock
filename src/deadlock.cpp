#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/compute_path_to_pose.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  std::string action_name = "/compute_path_to_pose";
  auto node = std::make_shared<rclcpp::Node>("action_client");
  int i = 0;
  bool well_behaved = false;
  while (rclcpp::ok()) {
    i++;
    if (well_behaved) {
      auto goal = nav2_msgs::action::ComputePathToPose::Goal();
      auto action_client = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(node, action_name);
      bool goal_result_available = false;
      rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::WrappedResult result = rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::WrappedResult();;

      goal.pose.pose.position.x = 1;
      goal.pose.pose.position.y = 1;

      auto send_goal_options = typename rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SendGoalOptions();
      send_goal_options.result_callback =
        [&goal_result_available, &result](const typename rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::WrappedResult & _result) {
          result = _result;
          goal_result_available = true;
        };

      RCLCPP_INFO(node->get_logger(), "waiting action server [%8d]", i);
      if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_INFO(node->get_logger(), "action server is not available");
        return 1;
      }
      RCLCPP_INFO(node->get_logger(), "action server is available");

      RCLCPP_INFO(node->get_logger(), "sending goal");
      auto future_goal_handle = action_client->async_send_goal(goal, send_goal_options);
      RCLCPP_INFO(node->get_logger(), "sent goal");

      if (rclcpp::spin_until_future_complete(node, future_goal_handle, std::chrono::seconds(2)) !=
          rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_INFO(node->get_logger(), "send_goal failed");
          return 2;
        }

      while (rclcpp::ok()){
        if (goal_result_available) {
          break;
        }
        rclcpp::spin_some(node);
      }
      switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(node->get_logger(), "SUCCEEDED");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_INFO(node->get_logger(), "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(node->get_logger(), "Goal was canceled");
        break;
      default:
        RCLCPP_INFO(node->get_logger(), "Unknown result code");
        break;
      }
    } else {
      auto goal = nav2_msgs::action::ComputePathToPose::Goal();
      auto action_client = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(node, action_name);
      bool goal_result_available = false;
      rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::WrappedResult result = rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::WrappedResult();;

      goal.pose.pose.position.x = 1;
      goal.pose.pose.position.y = 1;

      auto send_goal_options = typename rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SendGoalOptions();
      send_goal_options.result_callback =
        [&goal_result_available, &result](const typename rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::WrappedResult & _result) {
          result = _result;
          goal_result_available = true;
        };

      RCLCPP_INFO(node->get_logger(), "sending goal [%8d]", i);
      auto future_goal_handle = action_client->async_send_goal(goal, send_goal_options);

      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  return 0;
}
