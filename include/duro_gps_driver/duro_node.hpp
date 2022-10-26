// ros headers
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/time_reference.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

// libsbp - Swift Binary Protocol library headers
#include <libsbp/sbp.h>
#include <libsbp/system.h>
#include <libsbp/navigation.h>
#include <libsbp/orientation.h>
#include <libsbp/imu.h>
#include <libsbp/mag.h>

// include folder headers
#include "duro_gps_driver/UTM.h"
#include "duro_gps_driver/fake_orientation.hpp"

class DuroNode : public rclcpp::Node
{
public:
  DuroNode()
  : Node("duro_node")
  {
    navsatfix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("navsatfix", 100);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 100);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 100);
    mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("mag", 100);
    euler_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("rollpitchyaw", 100);
    euler_pub_fake_ = this->create_publisher<geometry_msgs::msg::Vector3>("rollpitchyaw_fake", 100);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("current_pose", 100);
    fake_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("current_pose_fake_orientation", 100);
    status_flag_pub_ = this->create_publisher<std_msgs::msg::UInt8>("status_flag", 100);
    status_string_pub_ = this->create_publisher<std_msgs::msg::String>("status_string", 100);
    time_ref_pub_ = this->create_publisher<sensor_msgs::msg::TimeReference>("time_ref", 100);
    // timer_ = this->create_wall_timer(
    // 500ms, std::bind(&MinimalPublisher::timer_callback, this));
    this->declare_parameter<std::string>("ip_address", "192.168.1.222");
    this->declare_parameter<int>("port", 55555);
    this->declare_parameter<std::string>("gps_receiver_frame", "duro");
    this->declare_parameter<std::string>("imu_frame", "duro");
    this->declare_parameter<std::string>("utm_frame", "map");
    this->declare_parameter<std::string>("orientation_source", "gps");
    this->declare_parameter<std::string>("z_coord_ref_switch", "orig");
    this->declare_parameter<bool>("euler_based_orientation", true);

    this->get_parameter("ip_address", tcp_ip_addr_);
    this->get_parameter("port", tcp_ip_port_);
    this->get_parameter("gps_receiver_frame", gps_receiver_frame_);
    this->get_parameter("imu_frame", imu_frame_);
    this->get_parameter("utm_frame", utm_frame_);
    this->get_parameter("orientation_source", orientation_source_);
    this->get_parameter("z_coord_ref_switch", z_coord_ref_switch_);
    this->get_parameter("euler_based_orientation", euler_based_orientation_);

  }

  void run();

private:
  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsatfix_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr euler_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr euler_pub_fake_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr fake_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr status_flag_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_string_pub_;
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr time_ref_pub_;

  // ROS msgs
  sensor_msgs::msg::NavSatFix navsatfix_msg_;
  nav_msgs::msg::Odometry odom_msg_;
  sensor_msgs::msg::Imu imu_msg_;
  sensor_msgs::msg::MagneticField mag_msg_;
  geometry_msgs::msg::Vector3 euler_vector_msg_;
  geometry_msgs::msg::Vector3 euler_fake_vector_msg_;
  geometry_msgs::msg::PoseStamped pose_msg_;
  geometry_msgs::msg::PoseStamped fake_pose_msg_;
  std_msgs::msg::UInt8 status_flag_msg_;
  std_msgs::msg::String status_string_msg_;
  sensor_msgs::msg::TimeReference time_ref_msg_;

  // ROS node parameters
  std::string tcp_ip_addr_;
  int tcp_ip_port_;
  std::string gps_receiver_frame_;
  std::string imu_frame_;
  std::string utm_frame_;
  std::string orientation_source_;
  std::string z_coord_ref_switch_;
  bool euler_based_orientation_;

  // SBP callbacks node
  // sbp_msg_callbacks_node_t heartbeat_callback_node_;
  static sbp_msg_callbacks_node_t pos_ll_callback_node_;
  static sbp_msg_callbacks_node_t orientation_callback_node_;
  static sbp_msg_callbacks_node_t orientation_euler_callback_node_;
  static sbp_msg_callbacks_node_t time_callback_node_;
  static sbp_msg_callbacks_node_t imu_callback_node_;
  static sbp_msg_callbacks_node_t imu_aux_callback_node_;
  static sbp_msg_callbacks_node_t mag_callback_node_;

  CoordinateTransition coordinate_transition_;
  FakeOri fake_ori_;

  int socket_desc_ = -1;
  double linear_acc_conf_ = -1.0;  //4096; // default acc_range 8g
  double angular_vel_conf_ = -1.0; //262.4; // default gyro_range 125
  bool first_run_imu_conf_ = true;
  bool first_run_z_coord_ = true;
  double z_coord_start_ = 0.0;

  void setup_socket();
  void close_socket();
  void pos_ll_callback(u16 sender_id, u8 len, u8 msg[], void *context);
  void orientation_callback(u16 sender_id, u8 len, u8 msg[], void *context);
  void time_callback(u16 sender_id, u8 len, u8 msg[], void *context);
  void orientation_euler_callback(u16 sender_id, u8 len, u8 msg[], void *context);
  void imu_callback(u16 sender_id, u8 len, u8 msg[], void *context);
  void imu_aux_callback(u16 sender_id, u8 len, u8 msg[], void *context);
  void mag_callback(u16 sender_id, u8 len, u8 msg[], void *context);
  s32 socket_read(u8 *buff, u32 n, void *context);
};