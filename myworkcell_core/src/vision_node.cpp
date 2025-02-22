/**
**  Simple ROS Node
**/
#include <fake_ar_publisher/msg/ar_marker.hpp>
#include <myworkcell_core/srv/localize_part.hpp>
#include <rclcpp/rclcpp.hpp>

class Localizer : public rclcpp::Node
{
public:
  Localizer() : Node("vision_node"), last_msg_{nullptr}
  {
    // Topic subscription
    ar_sub_ = this->create_subscription<fake_ar_publisher::msg::ARMarker>(
        "ar_pose_marker",                                                    // Topic name
        rclcpp::QoS(1),                                                      // Quality of Service: 1 received message to buffer
        std::bind(&Localizer::visionCallback, this, std::placeholders::_1)); // indicate 1 argument

    // Service server
    server_ = this->create_service<myworkcell_core::srv::LocalizePart>(
        "localize_part",
        std::bind(&Localizer::localizePart, this, std::placeholders::_1, std::placeholders::_2)); // indicate 2 arguments
  }

  // topic callback
  void visionCallback(fake_ar_publisher::msg::ARMarker::SharedPtr msg)
  {
    last_msg_ = msg;
    RCLCPP_INFO(get_logger(), "Received pose: x=%f, y=%f, z=%f", msg->pose.pose.position.x, msg->pose.pose.position.y,
                msg->pose.pose.position.z);
  }

  // service callback
  void localizePart(myworkcell_core::srv::LocalizePart::Request::SharedPtr req,
                    myworkcell_core::srv::LocalizePart::Response::SharedPtr res)
  {
    // Read last message
    fake_ar_publisher::msg::ARMarker::SharedPtr p = last_msg_;

    if (!p)
    {
      RCLCPP_ERROR(this->get_logger(), "no data");
      res->success = false;
      return;
    }

    res->pose = p->pose.pose;
    res->success = true;
  }

  fake_ar_publisher::msg::ARMarker::SharedPtr last_msg_;
  rclcpp::Subscription<fake_ar_publisher::msg::ARMarker>::SharedPtr ar_sub_;
  rclcpp::Service<myworkcell_core::srv::LocalizePart>::SharedPtr server_;
};

int main(int argc, char **argv)
{
  //// 1.3 - Packages and Nodes
  // 3. This must be called before anything else ROS-related
  rclcpp::init(argc, argv);

  // 4. Create a ROS node instance
  // auto node = std::make_shared<rclcpp::Node>("vision_node");

  // 5. Print a "Hello, world!" message
  // RCLCPP_INFO(node->get_logger(), "Hello, world!");

  //// 1.4 - TOpics and Messages
  // The Localizer class provides this node's ROS interfaces
  auto node = std::make_shared<Localizer>();

  RCLCPP_INFO(node->get_logger(), "Vision node starting");

  // 6. Do not exit the program automatically - keep the node alive
  // Don't exit the program.
  rclcpp::spin(node);
}