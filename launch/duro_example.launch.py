from launch import LaunchDescription
from launch_ros.actions import Node
"""
orientation_source can be gps / odom  
- gps: orientation provided from the default gps modules 
- odom: orientation counted from previous positions        
z_coord_ref_switch can be zero / exact / zero_based / orig 
- zero: the Z coordinate is always 0
- exact: the Z coorindinate is always z_coord_exact_height param (must be set in this launch)
- zero_based: Z coordinate starts from 0 and relative
- orig: the original Z provided by Duro / Piksi
zero_based_pose:
- false (default): UTM coordinates as pose
- true: 0,0 starting point as pose (similar to an odometry)
euler_based_orientation:
- true: euler based, not enabled by default, please enable SPB message SBP_MSG_ORIENT_EULER 0x0221 decimal 545
- false: quaternion based, not enabled by default, please enable SPB message SBP_MSG_ORIENT_QUAT 0x0220 decimal 544
publish_fake_orientation:
- true: publishes fake orientation based on movement (default: true)
- false: publishes only real orientation (imu calculated, if available)
publish_imu_messages:
- true: publishes imu messages (default: true)
- false: does not publish imu messages
publish_odom_messages:
- true: publishes odometry messages (default: true)
- false: does not publish odometry messages
"""
def generate_launch_description():
    ld = LaunchDescription()
    duro_node = Node(
        package="duro_gps_driver",
        executable="duro_node",
        parameters=[
            {"ip_address": "192.168.1.10"},
            {"port": 55555},
            {"gps_receiver_frame_id": "duro"},
            {"imu_frame_id": "duro"},
            {"utm_frame_id": "map"},
            {"orientation_source": "gps"},
            {"z_coord_ref_switch": "zero"},
            {"z_coord_exact_height": 0.2},
            {"zero_based_pose": False},
            {"euler_based_orientation": True}           
            {"publish_fake_orientation": True}, # default: true
            {"publish_imu_messages": True}, # default: true
            {"publish_odom_messages": True}, # default: true
            {"publish_tf": True}, # default: true
        ]
    )
    ld.add_action(duro_node)
    return ld