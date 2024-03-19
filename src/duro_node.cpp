// standard c headers
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <cmath>
#include <memory>

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
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"



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

rclcpp::Node::SharedPtr node;
// Publishers
rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsatfix_pub;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub;
rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr euler_pub;
rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr euler_pub_fake;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_with_cov_pub;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr fake_pub;
rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr status_flag_pub;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_string_pub;
rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr time_ref_pub;
rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr time_diff_pub;

// ROS msgs
sensor_msgs::msg::NavSatFix navsatfix_msg;
nav_msgs::msg::Odometry odom_msg;
sensor_msgs::msg::Imu imu_msg;
sensor_msgs::msg::MagneticField mag_msg;
geometry_msgs::msg::Vector3 euler_vector_msg;
geometry_msgs::msg::Vector3 euler_fake_vector_msg;
geometry_msgs::msg::PoseStamped pose_msg;
geometry_msgs::msg::PoseWithCovarianceStamped pose_cov_msg;
geometry_msgs::msg::PoseStamped fake_pose_msg;
std_msgs::msg::UInt8 status_flag_msg;
std_msgs::msg::String status_string_msg;
sensor_msgs::msg::TimeReference time_ref_msg;
std_msgs::msg::Float64 diff_msg; 
geometry_msgs::msg::TransformStamped t, tf_static;
std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

// ROS node parameters
std::string tcp_ip_addr;
int tcp_ip_port;
std::string gps_receiver_frame;
std::string imu_frame;
std::string utm_frame;
std::string orientation_source;
std::string z_coord_ref_switch;
std::string tf_frame_id, tf_child_frame_id;
bool euler_based_orientation;
bool zero_based_pose;
float x_coord_offset, y_coord_offset;
float z_coord_exact_height;

// SBP variables
static sbp_msg_callbacks_node_t pos_ll_callback_node;
static sbp_msg_callbacks_node_t orientation_callback_node;
static sbp_msg_callbacks_node_t orientation_euler_callback_node;
static sbp_msg_callbacks_node_t imu_callback_node;
static sbp_msg_callbacks_node_t imu_aux_callback_node;
static sbp_msg_callbacks_node_t mag_callback_node;
static sbp_msg_callbacks_node_t time_callback_node;
static sbp_msg_callbacks_node_t vel_ned_cov_callback_node;
sbp_state_t sbp_state;

// Parameters
CoordinateTransition coordinate_transition;
FakeOri fake_ori;
int socket_desc = -1;
double linear_acc_conf = -1.0;  //4096; // default acc_range 8g
double angular_vel_conf = -1.0; //262.4; // default gyro_range 125
bool first_run_imu_conf = true;
bool first_run_z_coord = true;
bool first_run_position = true;
bool first_run_tf_static = true;
bool first_run_orientation = true;
double z_coord_start = 0.0;
geometry_msgs::msg::PoseStamped start_pose;

void setup_socket()
{
  struct sockaddr_in server;
  socket_desc = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_desc == -1)
  {
    RCLCPP_ERROR(node->get_logger(), "Could not create socket");
  }

  memset(&server, '0', sizeof(server));
  server.sin_addr.s_addr = inet_addr(tcp_ip_addr.c_str());
  server.sin_family = AF_INET;
  server.sin_port = htons(tcp_ip_port);

  if (connect(socket_desc, (struct sockaddr *)&server, sizeof(server)) < 0)
  {
    RCLCPP_ERROR(node->get_logger(), "Connection error");
  }
}

void close_socket()
{
  close(socket_desc);
}

// first three bits are fix mode
const int FIX_MODE_POSITION = 0;
const u8 FIX_MODE_MASK = 7;

namespace fix_modes
{
  enum FIX_MODE
  {
    INVALID = 0,
    SINGLE_POINT_POSITION,
    DIFFERENTIAL_GNSS,
    FLOAT_RTK,
    FIXED_RTK,
    DEAD_RECKONING,
    SBAS_POSITION
  };
}

// next two bits are internal navigation system mode
const int INS_MODE_POSITION = 3;
const u8 INS_MODE_MASK = 3 << INS_MODE_POSITION;

namespace ins_modes
{
  enum INS_MODE
  {
    NONE = 0,
    INS_USED
  };
}

/*
* Reads the LatLon message from the SBP API, which comes through the pos_llh variable, 
* and publishes in a NavSatFix ROS topic
*/
void pos_ll_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  msg_pos_llh_t *latlonmsg = (msg_pos_llh_t *)msg;
  // navsatfix (latlon) message over ROS
  navsatfix_msg.header.stamp = node->now();
  navsatfix_msg.header.frame_id = gps_receiver_frame;

  int ins_mode = (latlonmsg->flags & INS_MODE_MASK) >> INS_MODE_POSITION; // INS mode seems to remain 0...
  status_flag_msg.data = (latlonmsg->flags & FIX_MODE_MASK) >> FIX_MODE_POSITION;

  status_string_msg.data = "Invalid";

  status_flag_pub->publish(status_flag_msg); // 0: Invalid 1: Single Point Position (SPP) 2: Differential GNSS (DGNSS) 3:
                                        // Float RTK 4: Fixed RTK 5: Dead Reckoning 6: SBAS Position

  if (status_flag_msg.data > fix_modes::INVALID)
  {
    navsatfix_msg.latitude = latlonmsg->lat;
    navsatfix_msg.longitude = latlonmsg->lon;
    navsatfix_msg.altitude = latlonmsg->height;
    // covariance matrix
    double h_covariance = pow(latlonmsg->h_accuracy * 1e-3, 2); // Convert mm to m and take the ^2 for going from std to cov
    double v_covariance = pow(latlonmsg->v_accuracy * 1e-3, 2); // Convert mm to m and take the ^2 for going from std to cov
    navsatfix_msg.position_covariance[0] = h_covariance;                  // x = 0, 0
    navsatfix_msg.position_covariance[4] = h_covariance;                  // y = 1, 1
    navsatfix_msg.position_covariance[8] = v_covariance;                  // z = 2, 2
    navsatfix_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    double x = 0, y = 0;
    coordinate_transition.LatLonToUTMXY(latlonmsg->lat, latlonmsg->lon, x, y);

    odom_msg.header.stamp = node->now();
    odom_msg.header.frame_id = utm_frame;
    odom_msg.child_frame_id = gps_receiver_frame;
    // odom_msg.pose.pose.position.x = x;
    // odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = latlonmsg->height;

    pose_msg.header.stamp = node->now();
    pose_msg.header.frame_id = utm_frame;
    pose_msg.pose.position.x = x;
    pose_msg.pose.position.y = y;

    t.header.stamp = node->get_clock()->now();
    t.header.frame_id = tf_frame_id;
    t.child_frame_id = tf_child_frame_id;

    fake_ori.addXY(x, y);
    // fake_ori.printAll();

    if (first_run_z_coord)
    {
      z_coord_start = latlonmsg->height;
      first_run_z_coord = false;
    }

    // z_coord_ref_switch can be zero / zero_based / orig
    if (z_coord_ref_switch.compare("zero") == 0)
    {
      pose_msg.pose.position.z = 0;
    }
    else if (z_coord_ref_switch.compare("exact") == 0)
    {
      pose_msg.pose.position.z = z_coord_exact_height;
    }
    else if (z_coord_ref_switch.compare("zero_based") == 0)
    {
      pose_msg.pose.position.z = latlonmsg->height - z_coord_start;
      
    }
    else if (z_coord_ref_switch.compare("orig") == 0)
    {
      pose_msg.pose.position.z = latlonmsg->height;
    }
    pose_msg.pose.position.x += x_coord_offset;
    pose_msg.pose.position.y += y_coord_offset;
    fake_pose_msg.header = pose_msg.header;
    fake_pose_msg.pose.position = pose_msg.pose.position;
    tf2::Quaternion fake_quat;
    fake_quat.setRPY(0.0, 0.0, fake_ori.getOri() + M_PI);
    fake_pose_msg.pose.orientation.w = fake_quat.getW();
    fake_pose_msg.pose.orientation.x = fake_quat.getX();
    fake_pose_msg.pose.orientation.y = fake_quat.getY();
    fake_pose_msg.pose.orientation.z = fake_quat.getZ();
    fake_pub->publish(fake_pose_msg);

    if (first_run_position)
    {
      start_pose.pose.position = pose_msg.pose.position;
      first_run_position = false;
    }
    if(first_run_tf_static)
    {
      if (x_coord_offset < -0.1 || x_coord_offset > 0.1)
      {
        tf_static.header.frame_id = "map_utm_zone0";
        tf_static.child_frame_id = "map";
        tf_static.transform.translation.x = x_coord_offset; 
        tf_static.transform.translation.y = y_coord_offset; 
        tf_static.transform.translation.z = 0.0; 
        // tf_static.transform.rotation.x = 0.0;
        // tf_static.transform.rotation.y = 0.0;
        // tf_static.transform.rotation.z = 0.0;
        // tf_static.transform.rotation.w = 1.0;
        tf_static_broadcaster_->sendTransform(tf_static);
      }
    }
    if(zero_based_pose)
    {
      pose_msg.pose.position.x = pose_msg.pose.position.x - start_pose.pose.position.x;
      pose_msg.pose.position.y = pose_msg.pose.position.y - start_pose.pose.position.y;

      tf2::Quaternion current_orientation(pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w);
      tf2::Quaternion start_orientation_offset(-start_pose.pose.orientation.x, -start_pose.pose.orientation.y, -start_pose.pose.orientation.z, start_pose.pose.orientation.w);
      
      // quaternion start_orientation_offset to roll pitch yaw
      double roll_so, pitch_so, yaw_so;
      tf2::Matrix3x3(start_orientation_offset).getRPY(roll_so, pitch_so, yaw_so);

      // rotate pose_msg.pose.position.x and y by start_orientation_offset Z (yaw) around origo
      double rot_sin = sin(yaw_so);
      double rot_cos = cos(yaw_so);

      double posX = pose_msg.pose.position.x;
      double posY = pose_msg.pose.position.y;

      pose_msg.pose.position.x = posX * rot_cos - posY * rot_sin;
      pose_msg.pose.position.y = posX * rot_sin + posY * rot_cos;

      // rotate current_orientation by start_orientation_offset
      current_orientation = start_orientation_offset * current_orientation;

      current_orientation.normalize();

      pose_msg.pose.orientation.w = current_orientation.getW();
      pose_msg.pose.orientation.x = current_orientation.getX();
      pose_msg.pose.orientation.y = current_orientation.getY();
      pose_msg.pose.orientation.z = current_orientation.getZ();
    }
    t.transform.translation.x = pose_msg.pose.position.x; 
    t.transform.translation.y = pose_msg.pose.position.y; 
    t.transform.translation.z = pose_msg.pose.position.z; 
    t.transform.rotation.x = pose_msg.pose.orientation.x;
    t.transform.rotation.y = pose_msg.pose.orientation.y;
    t.transform.rotation.z = pose_msg.pose.orientation.z;
    t.transform.rotation.w = pose_msg.pose.orientation.w;
    tf_broadcaster_->sendTransform(t);

    if (orientation_source.compare("gps")==0)
    {
      pose_cov_msg.pose.pose = pose_msg.pose;
      pose_cov_msg.header = pose_msg.header;
      pose_cov_msg.pose.covariance[0] = 0.01;
      pose_cov_msg.pose.covariance[7] = 0.01;
      pose_cov_msg.pose.covariance[14] = 0.01;
      pose_cov_msg.pose.covariance[21] = 0.01;
      pose_cov_msg.pose.covariance[28] = 0.01;
      pose_cov_msg.pose.covariance[35] = 0.01;

      pose_pub->publish(pose_msg);
      pose_with_cov_pub->publish(pose_cov_msg);
      odom_msg.pose.pose.position.x = pose_msg.pose.position.x;
      odom_msg.pose.pose.position.y = pose_msg.pose.position.y;
      odom_msg.pose.pose.orientation.w = pose_msg.pose.orientation.w;
      odom_msg.pose.pose.orientation.x = pose_msg.pose.orientation.x;
      odom_msg.pose.pose.orientation.y = pose_msg.pose.orientation.y;
      odom_msg.pose.pose.orientation.z = pose_msg.pose.orientation.z;
      odom_pub->publish(odom_msg);
    }
    else if (orientation_source.compare("odom")==0)
    {
      pose_pub->publish(fake_pose_msg);
    }


    fake_ori.setStatus(status_flag_msg.data);
    switch (status_flag_msg.data)
    {
    case fix_modes::INVALID:
      navsatfix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
      status_string_msg.data = "Invalid";
      break;
    case fix_modes::SINGLE_POINT_POSITION:
      navsatfix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      status_string_msg.data = "Single Point Position (SPP)";
      break;
    case fix_modes::DIFFERENTIAL_GNSS:
      navsatfix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
      status_string_msg.data = "Differential GNSS (DGNSS)";
      break;
    case fix_modes::FLOAT_RTK:
      navsatfix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
      status_string_msg.data = "Float RTK";
      break;
    case fix_modes::FIXED_RTK:
      navsatfix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
      status_string_msg.data = "Fixed RTK";
      break;
    case fix_modes::DEAD_RECKONING:
      navsatfix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
      status_string_msg.data = "Dead Reckoning (DR)";
      break;
    case fix_modes::SBAS_POSITION:
      navsatfix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
      status_string_msg.data = "SBAS Position";
      break;
    default:
      navsatfix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
      RCLCPP_WARN(node->get_logger(), "Acquired a navsatfix with a mode that's not implemented. You are likely"
                      "using an unsupported version of libsbp.");
      status_string_msg.data = "Not implemented";
    }

    navsatfix_pub->publish(navsatfix_msg);
  }
  status_string_pub->publish(status_string_msg);
}

void orientation_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  // enable MSG ID 544 in swift console
  // the MSG ID comes from eg #define SBP_MSG_ORIENT_QUAT 0x0220 --> 544
  if (!euler_based_orientation)
  {
    msg_orient_quat_t *orimsg = (msg_orient_quat_t *)msg;

    double w = orimsg->w * pow(2, -31);
    double x = orimsg->x * pow(2, -31);
    double y = orimsg->y * pow(2, -31);
    double z = orimsg->z * pow(2, -31);
    tf2::Quaternion tf_orig(x, y, z, w);
    tf2::Quaternion tf_rot, tf_aligned;
    tf_rot.setRPY(0.0, 0.0, -M_PI_2); // left-handerd / right handed rotation
    tf_aligned = tf_rot * tf_orig;    // left-handerd / right handed rotation
    pose_msg.pose.orientation.w = tf_aligned.w() * -1;
    pose_msg.pose.orientation.x = tf_aligned.y();      // left-handerd / right handed orientation
    pose_msg.pose.orientation.y = tf_aligned.x() * -1; // left-handerd / right handed orientation
    pose_msg.pose.orientation.z = tf_aligned.z();      // left-handerd / right handed orientation

    tf_broadcaster_->sendTransform(t);
    
  }
}

void time_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  msg_gps_time_t *time_gps = (msg_gps_time_t *)msg;
  time_ref_msg.header.frame_id = "ros_time_frame";
  time_ref_msg.header.stamp = node->now();

  //rounded msec + residual nsec -> truncated sec + remainder nsec
  long long int towtemp = time_gps->tow % 1000;
  long long int ttemp = (towtemp * 1000000 + time_gps->ns_residual) % 1000000000;
  time_ref_msg.time_ref.nanosec = ttemp;
  time_ref_msg.time_ref.sec = time_gps->tow / 1000 + time_gps->wn * 604800 + 315964782; 
  time_ref_msg.source = "gps_duro";
  diff_msg.data = (time_ref_msg.header.stamp.sec - time_ref_msg.time_ref.sec) + (time_ref_msg.header.stamp.nanosec - time_ref_msg.time_ref.nanosec) % 1000000000 * 1e-9;
  time_ref_pub->publish(time_ref_msg);
  time_diff_pub->publish(diff_msg);
}

void vel_ned_cov_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  msg_vel_ned_cov_t *vel_ned_cov = (msg_vel_ned_cov_t *)msg;
}


void orientation_euler_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  // enable MSG ID 545 in swift console
  msg_orient_euler_t *orimsg = (msg_orient_euler_t *)msg;
  euler_vector_msg.x = orimsg->roll / 57292374.; // 57292374: raw > microdegrees > rad constant
  euler_vector_msg.y = orimsg->pitch / 57292374.;
  euler_vector_msg.z = orimsg->yaw / 57292374.;
  euler_fake_vector_msg.z = fake_ori.getOri();
  euler_pub->publish(euler_vector_msg);
  euler_pub_fake->publish(euler_fake_vector_msg);
  if (euler_based_orientation)
  {
    tf2::Quaternion fromeuler;
    fromeuler.setRPY(euler_vector_msg.x, euler_vector_msg.y, (euler_vector_msg.z * -1) + M_PI_2); // left-handerd / right handed orientation
    pose_msg.pose.orientation.w = fromeuler.getW();
    pose_msg.pose.orientation.x = fromeuler.getX();
    pose_msg.pose.orientation.y = fromeuler.getY();
    pose_msg.pose.orientation.z = fromeuler.getZ();

    if (first_run_orientation){
      start_pose.pose.orientation = pose_msg.pose.orientation;
      first_run_orientation = false;
    }

  }
}

const double G_TO_M_S2 = 9.80665;       // constans to convert g to m/s^2
const double GRAD_TO_RAD_ACC = 0.01745; // constans to convert to rad/sec
void imu_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  if (linear_acc_conf > 0)
  {
    msg_imu_raw_t *imumsg = (msg_imu_raw_t *)msg;
    imu_msg.header.stamp = node->now();
    imu_msg.header.frame_id = imu_frame;
    imu_msg.linear_acceleration.x = double(imumsg->acc_x) / linear_acc_conf * G_TO_M_S2;
    imu_msg.linear_acceleration.y = double(imumsg->acc_y) / linear_acc_conf * G_TO_M_S2;
    imu_msg.linear_acceleration.z = double(imumsg->acc_z) / linear_acc_conf * G_TO_M_S2;

    imu_msg.angular_velocity.x = double(imumsg->gyr_x) / angular_vel_conf * GRAD_TO_RAD_ACC; // Angular rate around IMU frame X axis
    imu_msg.angular_velocity.y = double(imumsg->gyr_y) / angular_vel_conf * GRAD_TO_RAD_ACC;
    imu_msg.angular_velocity.z = double(imumsg->gyr_z) / angular_vel_conf * GRAD_TO_RAD_ACC;

    imu_msg.orientation.w = pose_msg.pose.orientation.w;
    imu_msg.orientation.x = pose_msg.pose.orientation.x;
    imu_msg.orientation.y = pose_msg.pose.orientation.y;
    imu_msg.orientation.z = pose_msg.pose.orientation.z;
    imu_pub->publish(imu_msg);
  }
}

const int ACC_MODE_POSITION = 0;
const u8 ACC_MODE_MASK = 0xF;
// imu_conf[0:3] in g (acclelerometer)
namespace acc_conf_modes
{
  enum ACC_CONF_MODE
  {
    G2 = 0, // +/- 2g
    G4,     // +/- 4g
    G8,     // +/- 8g
    G16     // +/- 16g
  };
}
const int GYRO_MODE_POSITION = 4;
const u8 GYRO_MODE_MASK = 0xF0;
// imu_conf[4:7] in deg / s
namespace gyro_conf_modes
{
  enum GYRO_CONF_MODE
  {
    DEG_S2000 = 0, // +/- 2000 deg / s
    DEG_S1000,     // +/- 1000 deg / s
    DEG_S500,      // +/- 500 deg / s
    DEG_S250,      // +/- 250 deg / s
    DEG_S125       // +/- 125 deg / s
  };
}

void imu_aux_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  msg_imu_aux_t *imuauxmsg = (msg_imu_aux_t *)msg;
  int acc_mode = (imuauxmsg->imu_conf & ACC_MODE_MASK) >> ACC_MODE_POSITION;
  int gyro_mode = (imuauxmsg->imu_conf & GYRO_MODE_MASK) >> GYRO_MODE_POSITION;
  //ROS_INFO("IMU %x gyro:%d acc:%d", imuauxmsg->imu_conf, gyro_mode, acc_mode);
  switch (acc_mode)
  {
  case acc_conf_modes::G2:
    linear_acc_conf = 16384;
    break;
  case acc_conf_modes::G4:
    linear_acc_conf = 8192;
    break;
  case acc_conf_modes::G8:
    linear_acc_conf = 4096;
    break;
  case acc_conf_modes::G16:
    linear_acc_conf = 2048;
    break;
  }
  switch (gyro_mode)
  {
  case gyro_conf_modes::DEG_S2000:
    angular_vel_conf = 16.4;
    break;
  case gyro_conf_modes::DEG_S1000:
    angular_vel_conf = 32.8;
    break;
  case gyro_conf_modes::DEG_S500:
    angular_vel_conf = 65.6;
    break;
  case gyro_conf_modes::DEG_S250:
    angular_vel_conf = 131.2;
    break;
  case gyro_conf_modes::DEG_S125:
    angular_vel_conf = 262.4;
    break;
  }
  if (first_run_imu_conf)
  {
    RCLCPP_INFO(node->get_logger(), "Duro IMU initalized");
    first_run_imu_conf = false;
  }
}

void mag_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  msg_mag_raw_t *magmsg = (msg_mag_raw_t *)msg;
  mag_msg.header.stamp = node->now();
  mag_msg.header.frame_id = imu_frame;

  mag_msg.magnetic_field.x = magmsg->mag_x * 1e-6; // Magnetic field in the body frame X axis [microteslas]
  mag_msg.magnetic_field.y = magmsg->mag_y * 1e-6;
  mag_msg.magnetic_field.z = magmsg->mag_z * 1e-6;
  mag_pub->publish(mag_msg);
}

s32 socket_read(u8 *buff, u32 n, void *context)
{
  return read(socket_desc, buff, n);
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("duro_node");

  navsatfix_pub = node->create_publisher<sensor_msgs::msg::NavSatFix>("navsatfix", 100);
  odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("odom", 100);
  imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("imu", 100);
  mag_pub = node->create_publisher<sensor_msgs::msg::MagneticField>("mag", 100);
  euler_pub = node->create_publisher<geometry_msgs::msg::Vector3>("rollpitchyaw", 100);
  euler_pub_fake = node->create_publisher<geometry_msgs::msg::Vector3>("rollpitchyaw_fake", 100);
  pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("current_pose", 100);
  pose_with_cov_pub = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("current_pose_with_cov", 100);
  fake_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("current_pose_fake_orientation", 100);
  status_flag_pub = node->create_publisher<std_msgs::msg::UInt8>("status_flag", 100);
  status_string_pub = node->create_publisher<std_msgs::msg::String>("status_string", 100);
  time_ref_pub = node->create_publisher<sensor_msgs::msg::TimeReference>("time_ref", 100);
  time_diff_pub = node->create_publisher<std_msgs::msg::Float64>("time_diff", 100);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

  node->declare_parameter<std::string>("ip_address", "192.168.0.222");
  node->declare_parameter<int>("port", 55555);
  node->declare_parameter<std::string>("gps_receiver_frame", "duro");
  node->declare_parameter<std::string>("imu_frame", "duro");
  node->declare_parameter<std::string>("utm_frame", "map");
  node->declare_parameter<std::string>("orientation_source", "gps");
  node->declare_parameter<std::string>("z_coord_ref_switch", "orig");
  node->declare_parameter<bool>("euler_based_orientation", true);
  node->declare_parameter<float>("x_coord_offset", 0.0);
  node->declare_parameter<float>("y_coord_offset", 0.0);
  node->declare_parameter<float>("z_coord_exact_height", 1.9);
  node->declare_parameter<std::string>("tf_frame_id", "map");
  node->declare_parameter<std::string>("tf_child_frame_id", "gps");
  node->declare_parameter<bool>("zero_based_pose", false);
  

  node->get_parameter("ip_address", tcp_ip_addr);
  node->get_parameter("port", tcp_ip_port);
  node->get_parameter("gps_receiver_frame", gps_receiver_frame);
  node->get_parameter("imu_frame", imu_frame);
  node->get_parameter("utm_frame", utm_frame);
  node->get_parameter("orientation_source", orientation_source);
  node->get_parameter("x_coord_offset", x_coord_offset);
  node->get_parameter("y_coord_offset", y_coord_offset);
  node->get_parameter("z_coord_ref_switch", z_coord_ref_switch);
  node->get_parameter("z_coord_exact_height", z_coord_exact_height);
  node->get_parameter("euler_based_orientation", euler_based_orientation);
  node->get_parameter("tf_frame_id", tf_frame_id); 
  node->get_parameter("tf_child_frame_id", tf_child_frame_id); 
  node->get_parameter("zero_based_pose", zero_based_pose); 
  

  RCLCPP_INFO(node->get_logger(), "Starting GPS Duro...");
  RCLCPP_INFO(node->get_logger(), "Connecting to duro on %s:%d", tcp_ip_addr.c_str(), tcp_ip_port);

  if (z_coord_ref_switch.compare("exact") == 0){
    RCLCPP_INFO_STREAM(node->get_logger(), "Exact height (z): " << z_coord_exact_height);
  }
  if (x_coord_offset < -0.1 || x_coord_offset > 0.1){
    RCLCPP_INFO_STREAM(node->get_logger(), "x and y coordinate offset: " << x_coord_offset << "," << y_coord_offset);
  }
  RCLCPP_INFO_STREAM(node->get_logger(), "TF child frame id: " << tf_child_frame_id);

  setup_socket();
  sbp_state_init(&sbp_state);
  sbp_register_callback(&sbp_state, SBP_MSG_POS_LLH, pos_ll_callback, NULL, &pos_ll_callback_node);
  sbp_register_callback(&sbp_state, SBP_MSG_ORIENT_QUAT, orientation_callback, NULL, &orientation_callback_node);
  sbp_register_callback(&sbp_state, SBP_MSG_ORIENT_EULER, orientation_euler_callback, NULL, &orientation_euler_callback_node);
  sbp_register_callback(&sbp_state, SBP_MSG_IMU_RAW, imu_callback, NULL, &imu_callback_node);
  sbp_register_callback(&sbp_state, SBP_MSG_IMU_AUX, imu_aux_callback, NULL, &imu_aux_callback_node);
  sbp_register_callback(&sbp_state, SBP_MSG_MAG_RAW, mag_callback, NULL, &mag_callback_node);
  sbp_register_callback(&sbp_state, SBP_MSG_GPS_TIME, time_callback, NULL, &time_callback_node);
  sbp_register_callback(&sbp_state, SBP_MSG_VEL_NED_COV, vel_ned_cov_callback, NULL, &vel_ned_cov_callback_node);
  RCLCPP_INFO(node->get_logger(), "Success on %s:%d", tcp_ip_addr.c_str(), tcp_ip_port);
  
  while (rclcpp::ok())
  {
    sbp_process(&sbp_state, &socket_read);
    rclcpp::spin_some(node);
  }
  close_socket();
  rclcpp::shutdown();
  return 0;
}