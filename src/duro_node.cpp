// standard c headers
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <cmath>

#include "duro_gps_driver/duro_node.hpp"

// TODO: Constructor

void DuroNode::setup_socket()
{
  struct sockaddr_in server;
  socket_desc = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_desc == -1)
  {
    RCLCPP_ERROR(this->get_logger(), "Could not create socket");
  }

  memset(&server, '0', sizeof(server));
  server.sin_addr.s_addr = inet_addr(tcp_ip_addr.c_str());
  server.sin_family = AF_INET;
  server.sin_port = htons(tcp_ip_port);

  if (connect(socket_desc, (struct sockaddr *)&server, sizeof(server)) < 0)
  {
    RCLCPP_ERROR(this->get_logger(), "Connection error");
  }
}

void DuroNode::close_socket()
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
void DuroNode::pos_ll_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  msg_pos_llh_t *latlonmsg = (msg_pos_llh_t *)msg;
  // navsatfix (latlon) message over ROS
  navsatfix_msg_.header.stamp = this->now();
  navsatfix_msg_.header.frame_id = gps_receiver_frame_id;

  int ins_mode = (latlonmsg->flags & INS_MODE_MASK) >> INS_MODE_POSITION; // INS mode seems to remain 0...
  status_flag_msg_.data = (latlonmsg->flags & FIX_MODE_MASK) >> FIX_MODE_POSITION;

  status_string_msg_.data = "Invalid";

  status_flag_pub_->publish(status_flag_msg_); // 0: Invalid 1: Single Point Position (SPP) 2: Differential GNSS (DGNSS) 3:
                                         // Float RTK 4: Fixed RTK 5: Dead Reckoning 6: SBAS Position

  if (status_flag_msg_.data > fix_modes::INVALID)
  {
    navsatfix_msg_.latitude = latlonmsg->lat;
    navsatfix_msg_.longitude = latlonmsg->lon;
    navsatfix_msg_.altitude = latlonmsg->height;
    // covariance matrix
    double h_covariance = pow(latlonmsg->h_accuracy * 1e-3, 2); // Convert mm to m and take the ^2 for going from std to cov
    double v_covariance = pow(latlonmsg->v_accuracy * 1e-3, 2); // Convert mm to m and take the ^2 for going from std to cov
    navsatfix_msg_.position_covariance[0] = h_covariance;                  // x = 0, 0
    navsatfix_msg_.position_covariance[4] = h_covariance;                  // y = 1, 1
    navsatfix_msg_.position_covariance[8] = v_covariance;                  // z = 2, 2
    navsatfix_msg_.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    double x = 0, y = 0;
    coordinate_transition.LatLonToUTMXY(latlonmsg->lat, latlonmsg->lon, x, y);

    odom_msg_.header.stamp = this->now();
    odom_msg_.header.frame_id = utm_frame_id;
    odom_msg_.child_frame_id = gps_receiver_frame_id;
    odom_msg_.pose.pose.position.x = x;
    odom_msg_.pose.pose.position.y = y;
    odom_msg_.pose.pose.position.z = latlonmsg->height;
    odom_pub_->publish(odom_msg_);

    pose_msg_.header.stamp = this->now();
    pose_msg_.header.frame_id = utm_frame_id;
    pose_msg_.pose.position.x = x;
    pose_msg_.pose.position.y = y;

    fake_ori.addXY(x, y);
    // fake_ori.printAll();
    //ROS_INFO_STREAM(fake_ori.getOri());

    if (first_run_z_coord)
    {
      z_coord_start = latlonmsg->height;
      first_run_z_coord = false;
    }

    // z_coord_ref_switch can be zero / zero_based / orig
    if (z_coord_ref_switch.compare("zero") == 0)
    {
      pose_msg_.pose.position.z = 0;
    }
    else if (z_coord_ref_switch.compare("zero_based") == 0)
    {
      pose_msg_.pose.position.z = latlonmsg->height - z_coord_start;
    }
    else if (z_coord_ref_switch.compare("orig") == 0)
    {
      pose_msg_.pose.position.z = latlonmsg->height;
    }
    fake_pose_msg_.header = pose_msg_.header;
    fake_pose_msg_.pose.position = pose_msg_.pose.position;
    tf2::Quaternion fake_quat;
    fake_quat.setRPY(0.0, 0.0, fake_ori.getOri() + M_PI);
    fake_pose_msg_.pose.orientation.w = fake_quat.getW();
    fake_pose_msg_.pose.orientation.x = fake_quat.getX();
    fake_pose_msg_.pose.orientation.y = fake_quat.getY();
    fake_pose_msg_.pose.orientation.z = fake_quat.getZ();
    fake_pub_->publish(fake_pose_msg_);

    if (orientation_source.compare("gps")==0)
    {
    pose_pub_->publish(pose_msg_);
    }
    else if (orientation_source.compare("odom")==0)
    {
    pose_pub_->publish(fake_pose_msg_);
    }


    fake_ori.setStatus(status_flag_msg_.data);
    switch (status_flag_msg_.data)
    {
    case fix_modes::INVALID:
      navsatfix_msg_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
      status_string_msg_.data = "Invalid";
      break;
    case fix_modes::SINGLE_POINT_POSITION:
      navsatfix_msg_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      status_string_msg_.data = "Single Point Position (SPP)";
      break;
    case fix_modes::DIFFERENTIAL_GNSS:
      navsatfix_msg_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
      status_string_msg_.data = "Differential GNSS (DGNSS)";
      break;
    case fix_modes::FLOAT_RTK:
      navsatfix_msg_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
      status_string_msg_.data = "Float RTK";
      break;
    case fix_modes::FIXED_RTK:
      navsatfix_msg_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
      status_string_msg_.data = "Fixed RTK";
      break;
    case fix_modes::DEAD_RECKONING:
      navsatfix_msg_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
      status_string_msg_.data = "Dead Reckoning (DR)";
      break;
    case fix_modes::SBAS_POSITION:
      navsatfix_msg_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
      status_string_msg_.data = "SBAS Position";
      break;
    default:
      navsatfix_msg_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
      RCLCPP_WARN(this->get_logger(), "Acquired a navsatfix with a mode that's not implemented. You are likely"
                      "using an unsupported version of libsbp.");
      status_string_msg_.data = "Not implemented";
    }

    navsatfix_pub_->publish(navsatfix_msg_);
  }
  status_string_pub_->publish(status_string_msg_);
}

void DuroNode::orientation_callback(u16 sender_id, u8 len, u8 msg[], void *context)
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
    pose_msg_.pose.orientation.w = tf_aligned.w() * -1;
    pose_msg_.pose.orientation.x = tf_aligned.y();      // left-handerd / right handed orientation
    pose_msg_.pose.orientation.y = tf_aligned.x() * -1; // left-handerd / right handed orientation
    pose_msg_.pose.orientation.z = tf_aligned.z();      // left-handerd / right handed orientation
  }
}

void DuroNode::time_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  msg_gps_time_t *time_gps = (msg_gps_time_t *)msg;
  time_ref_msg_.header.frame_id = "ros_time_frame";
  time_ref_msg_.header.stamp = this->now();

  //rounded msec + residual nsec -> truncated sec + remainder nsec
  long long int ttemp = (time_gps->tow * 1000000 + time_gps->ns_residual) % 1000000000;
  time_ref_msg_.time_ref.nanosec = ttemp;
  time_ref_msg_.time_ref.sec = time_gps->tow / 1000;
  time_ref_msg_.source = "gps_duro";

  time_ref_pub_->publish(time_ref_msg_);
}

void DuroNode::orientation_euler_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  // enable MSG ID 545 in swift console
  msg_orient_euler_t *orimsg = (msg_orient_euler_t *)msg;
  euler_vector_msg_.x = orimsg->roll / 57292374.; // 57292374: raw > microdegrees > rad constant
  euler_vector_msg_.y = orimsg->pitch / 57292374.;
  euler_vector_msg_.z = orimsg->yaw / 57292374.;
  euler_fake_vector_msg_.z = fake_ori.getOri();
  euler_pub_->publish(euler_vector_msg_);
  euler_pub_fake_->publish(euler_fake_vector_msg_);
  if (euler_based_orientation)
  {
    tf2::Quaternion fromeuler;
    fromeuler.setRPY(euler_vector_msg_.x, euler_vector_msg_.y, (euler_vector_msg_.z * -1) + M_PI_2); // left-handerd / right handed orientation
    pose_msg_.pose.orientation.w = fromeuler.getW();
    pose_msg_.pose.orientation.x = fromeuler.getX();
    pose_msg_.pose.orientation.y = fromeuler.getY();
    pose_msg_.pose.orientation.z = fromeuler.getZ();
  }
}

const double G_TO_M_S2 = 9.80665;       // constans to convert g to m/s^2
const double GRAD_TO_RAD_ACC = 0.01745; // constans to convert to rad/sec
void DuroNode::imu_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  if (linear_acc_conf > 0)
  {
    msg_imu_raw_t *imumsg = (msg_imu_raw_t *)msg;
    imu_msg_.header.stamp = this->now();
    imu_msg_.header.frame_id = imu_frame_id;
    imu_msg_.linear_acceleration.x = double(imumsg->acc_x) / linear_acc_conf * G_TO_M_S2;
    imu_msg_.linear_acceleration.y = double(imumsg->acc_y) / linear_acc_conf * G_TO_M_S2;
    imu_msg_.linear_acceleration.z = double(imumsg->acc_z) / linear_acc_conf * G_TO_M_S2;

    imu_msg_.angular_velocity.x = double(imumsg->gyr_x) / angular_vel_conf * GRAD_TO_RAD_ACC; // Angular rate around IMU frame X axis
    imu_msg_.angular_velocity.y = double(imumsg->gyr_y) / angular_vel_conf * GRAD_TO_RAD_ACC;
    imu_msg_.angular_velocity.z = double(imumsg->gyr_z) / angular_vel_conf * GRAD_TO_RAD_ACC;

    imu_msg_.orientation.w = pose_msg_.pose.orientation.w;
    imu_msg_.orientation.x = pose_msg_.pose.orientation.x;
    imu_msg_.orientation.y = pose_msg_.pose.orientation.y;
    imu_msg_.orientation.z = pose_msg_.pose.orientation.z;
    imu_pub_->publish(imu_msg_);
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

void DuroNode::imu_aux_callback(u16 sender_id, u8 len, u8 msg[], void *context)
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
    RCLCPP_INFO(this->get_logger(), "Duro IMU initalized");
    first_run_imu_conf = false;
  }
}

void DuroNode::mag_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  msg_mag_raw_t *magmsg = (msg_mag_raw_t *)msg;
  mag_msg_.header.stamp = this->now();
  mag_msg_.header.frame_id = imu_frame_id;

  mag_msg_.magnetic_field.x = magmsg->mag_x * 1e-6; // Magnetic field in the body frame X axis [microteslas]
  mag_msg_.magnetic_field.y = magmsg->mag_y * 1e-6;
  mag_msg_.magnetic_field.z = magmsg->mag_z * 1e-6;
  mag_pub_->publish(mag_msg_);
}

s32 DuroNode::socket_read(u8 *buff, u32 n, void *context)
{
  return read(socket_desc, buff, n);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DuroNode>());
  rclcpp::shutdown();
  return 0;
}