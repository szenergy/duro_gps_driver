// ros headers
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/float64.hpp"
//#include "nav_msgs/msg/odometry.hpp" // TODO
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/time_reference.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
//#include "tf2/LinearMath/msg/Quaternion.h" // TODO

// standard c headers
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <cmath>
#include <string>
#include <chrono>
#include <functional>
#include <memory>

// libsbp - Swift Binary Protocol library headers
#include <libsbp/sbp.h>
#include <libsbp/legacy/system.h> //#include <libsbp/system.h>
#include <libsbp/legacy/navigation.h> //#include <libsbp/navigation.h>
#include <libsbp/legacy/orientation.h>
#include <libsbp/legacy/imu.h>
#include <libsbp/legacy/mag.h>
// include folder headers
//#include "UTM.h" // TODO
//#include "fake_orientation.hpp" // TODO


using namespace std::chrono_literals;

static sbp_msg_callbacks_node_t mag_callback_node;

class DuroDriver : public rclcpp::Node
{
  public:
    DuroDriver(): Node("duro_node"), count_(0)
    {
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("current_pose", 100);
        fake_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("current_pose_fake_orientation", 100);
        nav_fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("fix", 100);
        mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("mag", 100);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 100);
        euler_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("rollpitchyaw", 100);
        euler_pub_fake_ = this->create_publisher<geometry_msgs::msg::Vector3>("rollpitchyaw_fake", 100);
        status_flag_pub_ = this->create_publisher<std_msgs::msg::UInt8>("status_flag", 100);
        status_stri_pub_ = this->create_publisher<std_msgs::msg::String>("status_string", 100);
        time_ref_pub_ = this->create_publisher<sensor_msgs::msg::TimeReference>("time_ref", 100);
        timer_ = this->create_wall_timer(500ms, std::bind(&DuroDriver::timer_callback, this));

    }
    int socket_desc = -1;
    std::string tcp_ip_addr;
    int tcp_ip_port;

    void setup_socket()
    {
        this->declare_parameter("ip_address", "192.168.1.222");
        this->declare_parameter("port", 55555);        
        this->declare_parameter("gps_receiver_frame_id", "duro");
        this->declare_parameter("imu_frame_id", "duro");
        this->declare_parameter("utm_frame_id", "map");
        this->declare_parameter("orientation_source", "gps");
        this->declare_parameter("z_coord_ref_switch", "zero");
        this->declare_parameter("z_coord_exact_height", 0.1);
        tcp_ip_addr = this->get_parameter("ip_address").get_parameter_value().get<std::string>();
        tcp_ip_port = this->get_parameter("port").get_parameter_value().get<int>();
        std::string gps_receiver_frame_id = this->get_parameter("gps_receiver_frame_id").get_parameter_value().get<std::string>();
        std::string imu_frame_id = this->get_parameter("imu_frame_id").get_parameter_value().get<std::string>();
        std::string utm_frame_id = this->get_parameter("utm_frame_id").get_parameter_value().get<std::string>();
        std::string orientation_source = this->get_parameter("orientation_source").get_parameter_value().get<std::string>();
        std::string z_coord_ref_switch = this->get_parameter("z_coord_ref_switch").get_parameter_value().get<std::string>();
        float z_coord_exact_height = this->get_parameter("z_coord_exact_height").get_parameter_value().get<float>();
        RCLCPP_INFO_STREAM(this->get_logger(), "ip_address: " << tcp_ip_addr);
        RCLCPP_INFO_STREAM(this->get_logger(), "port: " << tcp_ip_port);
        RCLCPP_INFO_STREAM(this->get_logger(), "gps_receiver_frame_id: " << gps_receiver_frame_id);
        RCLCPP_INFO_STREAM(this->get_logger(), "imu_frame_id: " << imu_frame_id);
        RCLCPP_INFO_STREAM(this->get_logger(), "utm_frame_id: " << utm_frame_id);
        RCLCPP_INFO_STREAM(this->get_logger(), "orientation_source: " << orientation_source);
        RCLCPP_INFO_STREAM(this->get_logger(), "z_coord_ref_switch: " << z_coord_ref_switch);
        RCLCPP_INFO_STREAM(this->get_logger(), "z_coord_exact_height: " << z_coord_exact_height);
        RCLCPP_INFO(this->get_logger(), "Setup socket");
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

    void close_socket()
    {
        close(socket_desc);
    }

  private:
    void timer_callback()
    {
      auto stflags = std_msgs::msg::String();
      stflags.data = "TODO " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", stflags.data.c_str());
      status_stri_pub_->publish(stflags);
    }
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr fake_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_fix_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr euler_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr euler_pub_fake_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr status_flag_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_stri_pub_;
    rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr time_ref_pub_;
    rclcpp::TimerBase::SharedPtr timer_;


    size_t count_;
};

void mag_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  msg_mag_raw_t *magmsg = (msg_mag_raw_t *)msg;
  sensor_msgs::msg::MagneticField mag_ros_msg;
  /*
  mag_ros_msg.header.stamp = //this->get_clock()->now();
  mag_ros_msg.header.frame_id = imu_frame_id;

  mag_ros_msg.magnetic_field.x = magmsg->mag_x * 1e-6; // Magnetic field in the body frame X axis [microteslas]
  mag_ros_msg.magnetic_field.y = magmsg->mag_y * 1e-6;
  mag_ros_msg.magnetic_field.z = magmsg->mag_z * 1e-6;
  mag_pub.publish(mag_ros_msg);
  */
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto duro_node = std::make_shared<DuroDriver>();
    duro_node->setup_socket();
    sbp_state_t s;
    sbp_state_init(&s);
    sbp_register_callback(&s, SBP_MSG_MAG_RAW, &mag_callback, NULL, &mag_callback_node);
    //rclcpp::spin(std::make_shared<DuroDriver>());
    while (rclcpp::ok())
    {
        //sbp_process(&s, &socket_read);
        rclcpp::spin(duro_node);
    }
    rclcpp::shutdown();
    duro_node->close_socket();
    return 0;
}