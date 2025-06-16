#include "rclcpp/rclcpp.hpp"
#include "controller_manager_msgs/srv/load_controller.hpp"

using namespace std::chrono_literals;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("wait_for_service_node");
  rclcpp::Client<controller_manager_msgs::srv::LoadController>::SharedPtr client =
    node->create_client<controller_manager_msgs::srv::LoadController>("controller_manager/load_controller");

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  rclcpp::shutdown();
  return 0;
}