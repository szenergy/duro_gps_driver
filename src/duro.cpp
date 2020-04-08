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
ros::Publisher chatter_pub_st;
ros::Publisher imu_pub;
ros::Publisher mag_pub;
ros::Publisher gyro_pub;
ros::Publisher euler_pub;
ros::Publisher pose_pub;
ros::Publisher tmp_pub;
ros::Publisher status_flag_pub;
ros::Publisher status_stri_pub;

std::string tcp_ip_addr = "";
int tcp_ip_port = -1;
static sbp_msg_callbacks_node_t heartbeat_callback_node;
static sbp_msg_callbacks_node_t pos_ll_callback_node;
static sbp_msg_callbacks_node_t orientation_callback_node;
static sbp_msg_callbacks_node_t orientation_euler_callback_node;
static sbp_msg_callbacks_node_t time_callback_node;
static sbp_msg_callbacks_node_t imu_callback_node;
static sbp_msg_callbacks_node_t mag_callback_node;
nav_msgs::Odometry odom;
geometry_msgs::PoseStamped pose_msg;
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
  server.sin_port = htons(55555);

  if (connect(socket_desc, (struct sockaddr *)&server, sizeof(server)) < 0)
  {
    ROS_ERROR("Connection error");
  }
}

void close_socket()
{
  close(socket_desc);
}

void heartbeat_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;
  //fprintf(stdout, "%s\n", __FUNCTION__);
  /*for(int i = 0; i < len; i++)
    printf("%d ", msg[i]);
  printf("len: %d\n", len);*/
}

void pos_ll_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;
  msg_pos_llh_t *latlonmsg = (msg_pos_llh_t *)msg;
  // nav fix (latlon) message over ROS
  fix.header.stamp = ros::Time::now();
  if (latlonmsg->lat != 0.0)
  {
    fix.latitude = latlonmsg->lat;
    fix.longitude = latlonmsg->lon;
    nav_fix_pub.publish(fix);
    double x = 0, y = 0;
    coordinate_transition.LatLonToUTMXY(latlonmsg->lat, latlonmsg->lon, x, y);
    odom.header.stamp = ros::Time::now();
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = latlonmsg->height;
    odom_pub.publish(odom);
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.pose.position.x = x;
    pose_msg.pose.position.y = y;
    pose_msg.pose.position.z = latlonmsg->height;
    pose_pub.publish(pose_msg); //
    std_msgs::UInt8 flags;
    flags.data = latlonmsg->flags;
    status_flag_pub.publish(flags); // 0: Invalid 1: Single Point Position (SPP) 2: Differential GNSS (DGNSS) 3: Float RTK 4: Fixed RTK 5: Dead Reckoning 6: SBAS Position
    std_msgs::String stflags;

    switch (latlonmsg->flags)
    {
    case 9: // 1 // TODO: check
      stflags.data = "Single Point Position (SPP)";
      break;
    case 10: // 2
      stflags.data = "Differential GNSS (DGNSS)";
      break;
    case 11: // 3
      stflags.data = "Float RTK";
      break;
    case 12: // 4
      stflags.data = "Fixed RTK";
      break;
    case 13:
      stflags.data = "Dead Reckoning (DR)";
      break;
    case 14:
      stflags.data = "SBAS Position";
      break;
    default:
      stflags.data = "Invalid";
      break;
    }

    status_stri_pub.publish(stflags);
  }
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
  pose_msg.header.frame_id = "duro";
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

void time_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;
  msg_gps_time_t *timemsg = (msg_gps_time_t *)msg;
  //printf("%d\n", timemsg->tow);
}

void imu_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;
  msg_imu_raw_t *imumsg = (msg_imu_raw_t *)msg;
  geometry_msgs::Vector3 imu_ros_msg;
  imu_ros_msg.x = imumsg->acc_x;
  imu_ros_msg.y = imumsg->acc_y;
  imu_ros_msg.z = imumsg->acc_z;
  imu_pub.publish(imu_ros_msg);
  geometry_msgs::Vector3 gyro_ros_msg;
  gyro_ros_msg.x = imumsg->gyr_x; // Angular rate around IMU frame X axis
  gyro_ros_msg.y = imumsg->gyr_y;
  gyro_ros_msg.z = imumsg->gyr_z;
  gyro_pub.publish(gyro_ros_msg);
}

void mag_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;
  msg_mag_raw_t *magmsg = (msg_mag_raw_t *)msg;
  sensor_msgs::MagneticField mag_ros_msg;       // is geometry_msgs::Vector3 better?
  mag_ros_msg.magnetic_field.x = magmsg->mag_x; // Magnetic field in the body frame X axis [microteslas]
  mag_ros_msg.magnetic_field.y = magmsg->mag_y;
  mag_ros_msg.magnetic_field.z = magmsg->mag_z;
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
  chatter_pub_st = n.advertise<std_msgs::String>("gps/duro/utmzone", 100);
  odom_pub = n.advertise<nav_msgs::Odometry>("gps/duro/odom", 100);
  pose_pub = n.advertise<geometry_msgs::PoseStamped>("gps/duro/current_pose", 100);
  nav_fix_pub = n.advertise<sensor_msgs::NavSatFix>("gps/duro/fix", 100);
  mag_pub = n.advertise<sensor_msgs::MagneticField>("gps/duro/mag", 100);
  imu_pub = n.advertise<geometry_msgs::Vector3>("gps/duro/imu", 100);
  gyro_pub = n.advertise<geometry_msgs::Vector3>("gps/duro/gyro", 100);
  euler_pub = n.advertise<geometry_msgs::Vector3>("gps/duro/rollpitchyaw", 100);
  status_flag_pub = n.advertise<std_msgs::UInt8>("gps/duro/status_flag", 100);
  status_stri_pub = n.advertise<std_msgs::String>("gps/duro/status_string", 100);
  tmp_pub = n.advertise<std_msgs::Float64>("gps/duro/tmp", 100);

  n.getParam("address", tcp_ip_addr);
  n.getParam("port", tcp_ip_port);
  if (tcp_ip_addr.length() < 3)
  {
    ROS_WARN("No or wrong parameters provided, assuming _address:=192.168.1.222 _port:=55555");
    tcp_ip_addr = "192.168.1.222";
    tcp_ip_port = 55555;
  }
  else if (tcp_ip_port == -1)
  {
    tcp_ip_port = 55555;
  }
  else
  {
    ROS_INFO("Starting with _address:=%s _port:=%d", tcp_ip_addr.c_str(), tcp_ip_port);
  }
  setup_socket();
  sbp_state_init(&s);
  sbp_register_callback(&s, SBP_MSG_HEARTBEAT, &heartbeat_callback, NULL, &heartbeat_callback_node);
  sbp_register_callback(&s, SBP_MSG_POS_LLH, &pos_ll_callback, NULL, &pos_ll_callback_node);
  sbp_register_callback(&s, SBP_MSG_ORIENT_QUAT, &orientation_callback, NULL, &orientation_callback_node);
  sbp_register_callback(&s, SBP_MSG_ORIENT_EULER, &orientation_euler_callback, NULL, &orientation_euler_callback_node);
  sbp_register_callback(&s, SBP_MSG_GPS_TIME, &time_callback, NULL, &time_callback_node);
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
