// Description: This node subscribes to 2 current_pose topic of the Duro GPS's 
// creates a new message with the heading of the Duro GPS (heading) and the pose of the Duro GPS (pose) 
// and publishes it to a new topic and tf

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using std::placeholders::_1;

class DualHeadingMsg : public rclcpp::Node
{
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    for (const auto &param : parameters)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Param update: " << param.get_name().c_str() << ": " << param.value_to_string().c_str());
      if (param.get_name() == "child_frame_id")
      {
        dual_transform_.child_frame_id = param.as_string();
      }
      else if (param.get_name() == "frame_id")
      {
        dual_transform_.header.frame_id = param.as_string();
      }
      else if (param.get_name() == "x")
      {
        dual_transform_.transform.translation.x = param.as_double();
      }
      else if (param.get_name() == "sub_head_topic"){
        sub_head_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(param.as_string(), 10, std::bind(&DualHeadingMsg::heading_callback, this, _1));
      }
      else if (param.get_name() == "sub_pose_topic"){
        sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(param.as_string(), 10, std::bind(&DualHeadingMsg::pose_callback, this, _1));
      }
      else if (param.get_name() == "pub_dual_pose_topic"){
        pub_dual_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(param.as_string(), 10);
      }
      else
      {
        result.successful = false;
        result.reason = "failed";
      }

    }
    return result;
  }
  public:
    DualHeadingMsg() : Node("dual_heading_msg")
    {
      this->declare_parameter("child_frame_id", "duro");
      this->declare_parameter("frame_id", "map");
      this->declare_parameter("x", 0.0);
      this->declare_parameter("sub_head_topic", "/lexus3/gps/duro/hea/current_pose");
      this->declare_parameter("sub_pose_topic", "/lexus3/gps/duro/pos/current_pose");
      this->declare_parameter("pub_dual_pose_topic", "/lexus3/gps/duro/current_pose");
      this->get_parameter("sub_head_topic", sub_head_topic);
      this->get_parameter("sub_pose_topic", sub_pose_topic);
      this->get_parameter("pub_dual_pose_topic", pub_dual_pose_topic);



      sub_head_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(sub_head_topic, 10, std::bind(&DualHeadingMsg::heading_callback, this, _1));
      sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(sub_pose_topic, 10, std::bind(&DualHeadingMsg::pose_callback, this, _1));
      pub_dual_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pub_dual_pose_topic, 10);
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      callback_handle_ = this->add_on_set_parameters_callback(std::bind(&DualHeadingMsg::parametersCallback, this, std::placeholders::_1));

      RCLCPP_INFO_STREAM(this->get_logger(), "Node dual_heading_msg started.");
      RCLCPP_INFO_STREAM(this->get_logger(), "Subscribing to " << sub_head_topic << " and " << sub_pose_topic);
    }

  private:
    void heading_callback(const geometry_msgs::msg::PoseStamped heading_msg)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Heading: " << heading_msg.pose.orientation.x << ", " << heading_msg.pose.orientation.y << ", " << heading_msg.pose.orientation.z << ", " << heading_msg.pose.orientation.w);
      dual_pose_.header = heading_msg.header;
      dual_pose_.pose.orientation = heading_msg.pose.orientation;
      dual_transform_.header = heading_msg.header;
      dual_transform_.transform.rotation.x = heading_msg.pose.orientation.x;
      dual_transform_.transform.rotation.y = heading_msg.pose.orientation.y;
      dual_transform_.transform.rotation.z = heading_msg.pose.orientation.z;
      dual_transform_.transform.rotation.w = heading_msg.pose.orientation.w;
    }
    void pose_callback(const geometry_msgs::msg::PoseStamped pose_msg)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Pose: " << pose_msg.pose.position.x);
      dual_pose_.header = pose_msg.header;
      dual_pose_.pose.position = pose_msg.pose.position;
      pub_dual_pose_->publish(dual_pose_);
      dual_transform_.header = pose_msg.header;
      dual_transform_.child_frame_id = "duro";
      dual_transform_.transform.translation.x = pose_msg.pose.position.x;
      dual_transform_.transform.translation.y = pose_msg.pose.position.y;
      dual_transform_.transform.translation.z = pose_msg.pose.position.z;
    }
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_head_, sub_pose_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_dual_pose_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    geometry_msgs::msg::PoseStamped dual_pose_;
    geometry_msgs::msg::TransformStamped  dual_transform_;
    std::string sub_head_topic, sub_pose_topic, pub_dual_pose_topic;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DualHeadingMsg>());
  rclcpp::shutdown();
  return 0;
}