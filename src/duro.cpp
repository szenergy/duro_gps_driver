// ros headers
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/NavSatStatus.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_broadcaster.h"
#include <tf2_ros/static_transform_broadcaster.h>
// standard c headers
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <cmath>
// libsbp - Swift Binary Protocol library headers
#include <libsbp/sbp.h>
#include <libsbp/system.h>
#include <libsbp/navigation.h>
#include <libsbp/orientation.h>
#include <libsbp/imu.h>
#include <libsbp/mag.h>
// include folder headers
#include "UTM.h"

sensor_msgs::NavSatFix fix;
ros::Publisher nav_fix_pub;
ros::Publisher odom_pub;
ros::Publisher imu_pub;
ros::Publisher mag_pub;
ros::Publisher euler_pub;
ros::Publisher pose_pub;
ros::Publisher status_flag_pub;
ros::Publisher status_stri_pub;

std::string tcp_ip_addr = "";
int tcp_ip_port = -1;
std::string gps_receiver_frame_id;
std::string imu_frame_id;
std::string utm_frame_id;
static sbp_msg_callbacks_node_t heartbeat_callback_node;
static sbp_msg_callbacks_node_t pos_ll_callback_node;
static sbp_msg_callbacks_node_t orientation_callback_node;
static sbp_msg_callbacks_node_t orientation_euler_callback_node;
static sbp_msg_callbacks_node_t time_callback_node;
static sbp_msg_callbacks_node_t imu_callback_node;
static sbp_msg_callbacks_node_t mag_callback_node;
nav_msgs::Odometry odom;
geometry_msgs::PoseStamped pose_msg;
// Debug broadcast tf
// geometry_msgs::TransformStamped static_transformStamped;


CoordinateTransition coordinate_transition;

int socket_desc = -1;

void setup_socket()
{
  struct sockaddr_in server;
  socket_desc = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_desc == -1)
  {
    ROS_ERROR("Could not create socket");
  }

  memset(&server, '0', sizeof(server));
  server.sin_addr.s_addr = inet_addr(tcp_ip_addr.c_str());
  server.sin_family = AF_INET;
  server.sin_port = htons(tcp_ip_port);

  if (connect(socket_desc, (struct sockaddr *)&server, sizeof(server)) < 0)
  {
    ROS_ERROR("Connection error");
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

void pos_ll_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;
  msg_pos_llh_t *latlonmsg = (msg_pos_llh_t *)msg;
  // nav fix (latlon) message over ROS
  fix.header.stamp = ros::Time::now();
  fix.header.frame_id  = gps_receiver_frame_id;

  int ins_mode = (latlonmsg->flags & INS_MODE_MASK) >> INS_MODE_POSITION;  // INS mode seems to remain 0...
  int fix_mode = (latlonmsg->flags & FIX_MODE_MASK) >> FIX_MODE_POSITION;

  std_msgs::String stflags;
  stflags.data = "Invalid";

  std_msgs::UInt8 fix_mode_msg;
  fix_mode_msg.data = fix_mode;
  status_flag_pub.publish(fix_mode_msg);  // 0: Invalid 1: Single Point Position (SPP) 2: Differential GNSS (DGNSS) 3:
                                          // Float RTK 4: Fixed RTK 5: Dead Reckoning 6: SBAS Position

  if (fix_mode > fix_modes::INVALID)
  {
    fix.latitude = latlonmsg->lat;
    fix.longitude = latlonmsg->lon;
    // covariance matrix
    double h_covariance = pow(latlonmsg->h_accuracy * 1e-3, 2);  // Convert mm to m and take the ^2 for going from std to cov
    double v_covariance = pow(latlonmsg->v_accuracy * 1e-3, 2);  // Convert mm to m and take the ^2 for going from std to cov
    fix.position_covariance[0] = h_covariance; // x = 0, 0
    fix.position_covariance[4] = h_covariance; // y = 1, 1
    fix.position_covariance[8] = v_covariance; // z = 2, 2
    fix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    double x = 0, y = 0;
    coordinate_transition.LatLonToUTMXY(latlonmsg->lat, latlonmsg->lon, x, y);
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = utm_frame_id;
    odom.child_frame_id = gps_receiver_frame_id;
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = latlonmsg->height;
    odom_pub.publish(odom);
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = utm_frame_id;
    pose_msg.pose.position.x = x;
    pose_msg.pose.position.y = y;
    pose_msg.pose.position.z = 0; //latlonmsg->height;
    // Debug broadcast tf
    //static_transformStamped.header.stamp = ros::Time::now();
    //static_transformStamped.header.frame_id = "/map";
    //static_transformStamped.child_frame_id = "/duro";
    //static_transformStamped.transform.translation.x = x;
    //static_transformStamped.transform.translation.y = y;
    //static_transformStamped.transform.translation.z = 0;
    pose_pub.publish(pose_msg); //

    switch (fix_mode)
    {
      case fix_modes::INVALID:
        fix.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
        stflags.data = "Invalid";
        break;
      case fix_modes::SINGLE_POINT_POSITION:
        fix.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
        stflags.data = "Single Point Position (SPP)";
        break;
      case fix_modes::DIFFERENTIAL_GNSS:
        fix.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
        stflags.data = "Differential GNSS (DGNSS)";
        break;
      case fix_modes::FLOAT_RTK:
        fix.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
        stflags.data = "Float RTK";
        break;
      case fix_modes::FIXED_RTK:
        fix.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
        stflags.data = "Fixed RTK";
        break;
      case fix_modes::DEAD_RECKONING:
        fix.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
        stflags.data = "Dead Reckoning (DR)";
        break;
      case fix_modes::SBAS_POSITION:
        fix.status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
        stflags.data = "SBAS Position";
        break;
      default:
        fix.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
        ROS_WARN_STREAM("Acquired a fix with a mode that's not implemented. You are likely"
                        "using an unsupported version of libsbp.");
        stflags.data = "Not implemented";
        break;
    }

    nav_fix_pub.publish(fix);
  }
  status_stri_pub.publish(stflags);
}

void orientation_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  // enable MSG ID 544 in swift console
  // the MSG ID comes from eg #define SBP_MSG_ORIENT_QUAT 0x0220 --> 544
  (void)sender_id, (void)len, (void)msg, (void)context;
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
  pose_msg.header.frame_id = "map";
  // Debug broadcast tf
  //static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  //static_transformStamped.transform.rotation.x = x;
  //static_transformStamped.transform.rotation.y = y;
  //static_transformStamped.transform.rotation.z = z;
  //static_transformStamped.transform.rotation.w = w;
  //static_broadcaster.sendTransform(static_transformStamped);
}

void orientation_euler_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  // enable MSG ID 545 in swift console
  (void)sender_id, (void)len, (void)msg, (void)context;
  msg_orient_euler_t *orimsg = (msg_orient_euler_t *)msg;
  geometry_msgs::Vector3 eulervect;
  eulervect.x = orimsg->roll / 57292374.; // 57292374: raw > microdegrees > rad constant
  eulervect.y = orimsg->pitch / 57292374.;
  eulervect.z = orimsg->yaw / 57292374.;
  euler_pub.publish(eulervect);
}

void imu_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;
  msg_imu_raw_t *imumsg = (msg_imu_raw_t *)msg;
  sensor_msgs::Imu imu_ros_msg;
  imu_ros_msg.header.stamp = ros::Time::now();
  imu_ros_msg.header.frame_id = imu_frame_id;
  imu_ros_msg.linear_acceleration.x = imumsg->acc_x;
  imu_ros_msg.linear_acceleration.y = imumsg->acc_y;
  imu_ros_msg.linear_acceleration.z = imumsg->acc_z;

  imu_ros_msg.angular_velocity.x = imumsg->gyr_x; // Angular rate around IMU frame X axis
  imu_ros_msg.angular_velocity.y = imumsg->gyr_y;
  imu_ros_msg.angular_velocity.z = imumsg->gyr_z;
  imu_pub.publish(imu_ros_msg);
}

void mag_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;
  msg_mag_raw_t *magmsg = (msg_mag_raw_t *)msg;
  sensor_msgs::MagneticField mag_ros_msg;
  mag_ros_msg.header.stamp = ros::Time::now();
  mag_ros_msg.header.frame_id = imu_frame_id;

  mag_ros_msg.magnetic_field.x = magmsg->mag_x * 1e-6; // Magnetic field in the body frame X axis [microteslas]
  mag_ros_msg.magnetic_field.y = magmsg->mag_y * 1e-6;
  mag_ros_msg.magnetic_field.z = magmsg->mag_z * 1e-6;
  mag_pub.publish(mag_ros_msg);
}

s32 socket_read(u8 *buff, u32 n, void *context)
{
  (void)context;
  s32 result;

  result = read(socket_desc, buff, n);
  return result;
}

int main(int argc, char **argv)
{
  int opt;
  int result = 0;
  sbp_state_t s;
  ros::init(argc, argv, "duro");
  ros::NodeHandle n;
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);
  pose_pub = n.advertise<geometry_msgs::PoseStamped>("current_pose", 100);
  nav_fix_pub = n.advertise<sensor_msgs::NavSatFix>("fix", 100);
  mag_pub = n.advertise<sensor_msgs::MagneticField>("mag", 100);
  imu_pub = n.advertise<sensor_msgs::Imu>("imu", 100);
  euler_pub = n.advertise<geometry_msgs::Vector3>("rollpitchyaw", 100);
  status_flag_pub = n.advertise<std_msgs::UInt8>("status_flag", 100);
  status_stri_pub = n.advertise<std_msgs::String>("status_string", 100);

  ros::NodeHandle n_private("~");
  n_private.param<std::string>("ip_address", tcp_ip_addr, "192.168.0.222");
  n_private.param<int>("port", tcp_ip_port, 55555);
  n_private.param<std::string>("gps_receiver_frame_id", gps_receiver_frame_id, "duro_link");
  n_private.param<std::string>("imu_frame_id", imu_frame_id, gps_receiver_frame_id);
  n_private.param<std::string>("utm_frame_id", utm_frame_id, "utm");
  ROS_INFO("Connecting to duro on %s:%d", tcp_ip_addr.c_str(), tcp_ip_port);

  setup_socket();
  sbp_state_init(&s);
  sbp_register_callback(&s, SBP_MSG_POS_LLH, &pos_ll_callback, NULL, &pos_ll_callback_node);
  sbp_register_callback(&s, SBP_MSG_ORIENT_QUAT, &orientation_callback, NULL, &orientation_callback_node);
  sbp_register_callback(&s, SBP_MSG_ORIENT_EULER, &orientation_euler_callback, NULL, &orientation_euler_callback_node);
  sbp_register_callback(&s, SBP_MSG_IMU_RAW, &imu_callback, NULL, &imu_callback_node);
  sbp_register_callback(&s, SBP_MSG_MAG_RAW, &mag_callback, NULL, &mag_callback_node);
  //ros::Rate loop_rate(10);

  while (ros::ok())
  {
    sbp_process(&s, &socket_read);
    ros::spinOnce();
  }
  close_socket();
  return result;
}
