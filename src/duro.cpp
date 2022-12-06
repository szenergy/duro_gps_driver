// ros headers
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/NavSatStatus.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/TimeReference.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2/LinearMath/Quaternion.h"
// standard c headers
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <cmath>
#include <string>
// libsbp - Swift Binary Protocol library headers
#include <libsbp/sbp.h>
#include <libsbp/system.h>
#include <libsbp/navigation.h>
#include <libsbp/orientation.h>
#include <libsbp/imu.h>
#include <libsbp/mag.h>
// include folder headers
#include "UTM.h"
#include "fake_orientation.hpp"

sensor_msgs::NavSatFix fix;
ros::Publisher nav_fix_pub;
ros::Publisher odom_pub;
ros::Publisher imu_pub;
ros::Publisher mag_pub;
ros::Publisher euler_pub;
ros::Publisher euler_pub_fake;
ros::Publisher pose_pub;
ros::Publisher fake_pub;
ros::Publisher status_flag_pub;
ros::Publisher status_stri_pub;
ros::Publisher time_ref_pub;
ros::Publisher time_diff_pub;
ros::Publisher time_gps_str_pub;

std::string tcp_ip_addr;
int tcp_ip_port;
std::string gps_receiver_frame_id;
std::string imu_frame_id;
std::string utm_frame_id;
std::string z_coord_ref_switch;
std::string orientation_source;
float z_coord_exact_height;
bool euler_based_orientation;
static sbp_msg_callbacks_node_t heartbeat_callback_node;
static sbp_msg_callbacks_node_t pos_ll_callback_node;
static sbp_msg_callbacks_node_t orientation_callback_node;
static sbp_msg_callbacks_node_t orientation_euler_callback_node;
static sbp_msg_callbacks_node_t time_callback_node;
static sbp_msg_callbacks_node_t imu_callback_node;
static sbp_msg_callbacks_node_t imu_aux_callback_node;
static sbp_msg_callbacks_node_t mag_callback_node;
nav_msgs::Odometry odom;
geometry_msgs::PoseStamped pose_msg;
geometry_msgs::PoseStamped fake_pose_msg;

CoordinateTransition coordinate_transition;
FakeOri fake_ori;

int socket_desc = -1;
double linear_acc_conf = -1.0;  //4096; // default acc_range 8g
double angular_vel_conf = -1.0; //262.4; // default gyro_range 125
bool first_run_imu_conf = true;
bool first_run_z_coord = true;
double z_coord_start = 0.0;

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
  msg_pos_llh_t *latlonmsg = (msg_pos_llh_t *)msg;
  // nav fix (latlon) message over ROS
  fix.header.stamp = ros::Time::now();
  fix.header.frame_id = gps_receiver_frame_id;

  int ins_mode = (latlonmsg->flags & INS_MODE_MASK) >> INS_MODE_POSITION; // INS mode seems to remain 0...
  int fix_mode = (latlonmsg->flags & FIX_MODE_MASK) >> FIX_MODE_POSITION;

  std_msgs::String stflags;
  stflags.data = "Invalid";

  std_msgs::UInt8 fix_mode_msg;
  fix_mode_msg.data = fix_mode;
  status_flag_pub.publish(fix_mode_msg); // 0: Invalid 1: Single Point Position (SPP) 2: Differential GNSS (DGNSS) 3:
                                         // Float RTK 4: Fixed RTK 5: Dead Reckoning 6: SBAS Position

  if (fix_mode > fix_modes::INVALID)
  {
    fix.latitude = latlonmsg->lat;
    fix.longitude = latlonmsg->lon;
    fix.altitude = latlonmsg->height;
    // covariance matrix
    double h_covariance = pow(latlonmsg->h_accuracy * 1e-3, 2); // Convert mm to m and take the ^2 for going from std to cov
    double v_covariance = pow(latlonmsg->v_accuracy * 1e-3, 2); // Convert mm to m and take the ^2 for going from std to cov
    fix.position_covariance[0] = h_covariance;                  // x = 0, 0
    fix.position_covariance[4] = h_covariance;                  // y = 1, 1
    fix.position_covariance[8] = v_covariance;                  // z = 2, 2
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

    fake_ori.addXY(x, y);
    // fake_ori.printAll();
    //ROS_INFO_STREAM(fake_ori.getOri());

    if (first_run_z_coord)
    {
      z_coord_start = latlonmsg->height;
      first_run_z_coord = false;
    }

    // z_coord_ref_switch can be zero / exact / zero_based / orig
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
    fake_pose_msg.header = pose_msg.header;
    fake_pose_msg.pose.position = pose_msg.pose.position;
    tf2::Quaternion fake_quat;
    fake_quat.setRPY(0.0, 0.0, fake_ori.getOri() + M_PI);
    fake_pose_msg.pose.orientation.w = fake_quat.getW();
    fake_pose_msg.pose.orientation.x = fake_quat.getX();
    fake_pose_msg.pose.orientation.y = fake_quat.getY();
    fake_pose_msg.pose.orientation.z = fake_quat.getZ();
    fake_pub.publish(fake_pose_msg);

    if (orientation_source.compare("gps")==0)
    {
    pose_pub.publish(pose_msg);
    }
    else if (orientation_source.compare("odom")==0)
    {
    pose_pub.publish(fake_pose_msg);
    }


    fake_ori.setStatus(fix_mode);
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
    }

    nav_fix_pub.publish(fix);
  }
  status_stri_pub.publish(stflags);
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
  }
}

void time_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  msg_gps_time_t *time_gps = (msg_gps_time_t *)msg;
  sensor_msgs::TimeReference time_msg;

  time_msg.header.frame_id = "ros_time";
  //time_msg.header.stamp.nsec = ros::Time::now().nsec / 1000;
  //time_msg.header.stamp.sec = ros::Time::now().sec;
  time_msg.header.stamp = ros::Time::now();

  //rounded msec + residual nsec -> truncated sec + remainder nsec
  long long int towtemp = time_gps->tow % 1000;
  long long int ttemp = (towtemp * 1000000 + time_gps->ns_residual) % 1000000000;
  time_msg.time_ref.nsec = ttemp;
  time_msg.time_ref.sec = time_gps->tow / 1000 + time_gps->wn * 604800 + 315964782;
  time_msg.source = "gps_duro";
  std_msgs::Float64 diff_msg;
  ros::Duration diff = time_msg.time_ref - time_msg.header.stamp;
  diff_msg.data = diff.toSec();
  //diff_msg.data = time_gps->tow;
  std_msgs::String gps_str_msg;
  gps_str_msg.data = std::to_string(time_gps->wn) + " " + std::to_string(time_gps->tow) + " " + std::to_string(time_gps->ns_residual);

  time_gps_str_pub.publish(gps_str_msg);
  time_ref_pub.publish(time_msg);
  time_diff_pub.publish(diff_msg);


}

void orientation_euler_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  // enable MSG ID 545 in swift console
  msg_orient_euler_t *orimsg = (msg_orient_euler_t *)msg;
  geometry_msgs::Vector3 eulervect;
  geometry_msgs::Vector3 eulervect_fake;
  eulervect.x = orimsg->roll / 57292374.; // 57292374: raw > microdegrees > rad constant
  eulervect.y = orimsg->pitch / 57292374.;
  eulervect.z = orimsg->yaw / 57292374.;
  eulervect_fake.z = fake_ori.getOri();
  euler_pub.publish(eulervect);
  euler_pub_fake.publish(eulervect_fake);
  if (euler_based_orientation)
  {
    tf2::Quaternion fromeuler;
    fromeuler.setRPY(eulervect.x, eulervect.y, (eulervect.z * -1) + M_PI_2); // left-handerd / right handed orientation
    pose_msg.pose.orientation.w = fromeuler.getW();
    pose_msg.pose.orientation.x = fromeuler.getX();
    pose_msg.pose.orientation.y = fromeuler.getY();
    pose_msg.pose.orientation.z = fromeuler.getZ();
  }
}

const double G_TO_M_S2 = 9.80665;       // constans to convert g to m/s^2
const double GRAD_TO_RAD_ACC = 0.01745; // constans to convert to rad/sec
void imu_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  if (linear_acc_conf > 0)
  {
    msg_imu_raw_t *imumsg = (msg_imu_raw_t *)msg;
    sensor_msgs::Imu imu_ros_msg;
    imu_ros_msg.header.stamp = ros::Time::now();
    imu_ros_msg.header.frame_id = imu_frame_id;
    imu_ros_msg.linear_acceleration.x = double(imumsg->acc_x) / linear_acc_conf * G_TO_M_S2;
    imu_ros_msg.linear_acceleration.y = double(imumsg->acc_y) / linear_acc_conf * G_TO_M_S2;
    imu_ros_msg.linear_acceleration.z = double(imumsg->acc_z) / linear_acc_conf * G_TO_M_S2;

    imu_ros_msg.angular_velocity.x = double(imumsg->gyr_x) / angular_vel_conf * GRAD_TO_RAD_ACC; // Angular rate around IMU frame X axis
    imu_ros_msg.angular_velocity.y = double(imumsg->gyr_y) / angular_vel_conf * GRAD_TO_RAD_ACC;
    imu_ros_msg.angular_velocity.z = double(imumsg->gyr_z) / angular_vel_conf * GRAD_TO_RAD_ACC;

    imu_ros_msg.orientation.w = pose_msg.pose.orientation.w;
    imu_ros_msg.orientation.x = pose_msg.pose.orientation.x;
    imu_ros_msg.orientation.y = pose_msg.pose.orientation.y;
    imu_ros_msg.orientation.z = pose_msg.pose.orientation.z;
    imu_pub.publish(imu_ros_msg);
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
    ROS_INFO("Duro IMU initalized");
    first_run_imu_conf = false;
  }
}

void mag_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
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
  return read(socket_desc, buff, n);
}

int main(int argc, char **argv)
{
  sbp_state_t s;
  ros::init(argc, argv, "duro");
  ros::NodeHandle n;
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);
  pose_pub = n.advertise<geometry_msgs::PoseStamped>("current_pose", 100);
  fake_pub = n.advertise<geometry_msgs::PoseStamped>("current_pose_fake_orientation", 100);
  nav_fix_pub = n.advertise<sensor_msgs::NavSatFix>("fix", 100);
  mag_pub = n.advertise<sensor_msgs::MagneticField>("mag", 100);
  imu_pub = n.advertise<sensor_msgs::Imu>("imu", 100);
  euler_pub = n.advertise<geometry_msgs::Vector3>("rollpitchyaw", 100);
  euler_pub_fake = n.advertise<geometry_msgs::Vector3>("rollpitchyaw_fake", 100);
  status_flag_pub = n.advertise<std_msgs::UInt8>("status_flag", 100);
  status_stri_pub = n.advertise<std_msgs::String>("status_string", 100);
  time_ref_pub = n.advertise<sensor_msgs::TimeReference>("time_ref", 100);
  time_diff_pub = n.advertise<std_msgs::Float64>("time_diff", 100);
  time_gps_str_pub = n.advertise<std_msgs::String>("time_gps_str", 100);

  ros::NodeHandle n_private("~");
  n_private.param<std::string>("ip_address", tcp_ip_addr, "192.168.0.222");
  n_private.param<int>("port", tcp_ip_port, 55555);
  n_private.param<std::string>("gps_receiver_frame_id", gps_receiver_frame_id, "duro_link");
  n_private.param<std::string>("imu_frame_id", imu_frame_id, gps_receiver_frame_id);
  n_private.param<std::string>("utm_frame_id", utm_frame_id, "utm");
  n_private.param<std::string>("z_coord_ref_switch", z_coord_ref_switch, "zero");
  n_private.param<float>("z_coord_exact_height", z_coord_exact_height, 0.1);
  n_private.param<std::string>("orientation_source", orientation_source, "gps");
  n_private.param<bool>("euler_based_orientation", euler_based_orientation, true);
  ROS_INFO("Connecting to duro on %s:%d", tcp_ip_addr.c_str(), tcp_ip_port);

  setup_socket();
  sbp_state_init(&s);
  sbp_register_callback(&s, SBP_MSG_POS_LLH, &pos_ll_callback, NULL, &pos_ll_callback_node);
  sbp_register_callback(&s, SBP_MSG_ORIENT_QUAT, &orientation_callback, NULL, &orientation_callback_node);
  sbp_register_callback(&s, SBP_MSG_ORIENT_EULER, &orientation_euler_callback, NULL, &orientation_euler_callback_node);
  sbp_register_callback(&s, SBP_MSG_IMU_RAW, &imu_callback, NULL, &imu_callback_node);
  sbp_register_callback(&s, SBP_MSG_IMU_AUX, &imu_aux_callback, NULL, &imu_aux_callback_node);
  sbp_register_callback(&s, SBP_MSG_MAG_RAW, &mag_callback, NULL, &mag_callback_node);
  sbp_register_callback(&s, SBP_MSG_GPS_TIME, &time_callback, NULL, &time_callback_node);
  ROS_INFO("Success on %s:%d", tcp_ip_addr.c_str(), tcp_ip_port);

  while (ros::ok())
  {
    sbp_process(&s, &socket_read);
    ros::spinOnce();
  }
  close_socket();
  return 0;
}
