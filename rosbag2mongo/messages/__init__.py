msg_utility_packages = {
    'geometry_msgs/msg/Twist': ('rosbag2mongo.messages.twist', 'TwistUtils'),
    'geometry_msgs/msg/WrenchStamped': ('rosbag2mongo.messages.wrench_stamped', 'WrenchStampedUtils'),
    'sensor_msgs/msg/Image': ('rosbag2mongo.messages.image', 'ImageUtils'),
    'sensor_msgs/msg/Imu': ('rosbag2mongo.messages.imu', 'ImuUtils'),
    'sensor_msgs/msg/JointState': ('rosbag2mongo.messages.joint_state', 'JointStateUtils'),
    'sensor_msgs/msg/Float64': ('rosbag2mongo.messages.float', 'FloatUtils'),
    'sensor_msgs/msg/Int32': ('rosbag2mongo.messages.int', 'IntUtils'),
    'tf2_msgs/msg/TFMessage': ('rosbag2mongo.messages.tf_message', 'TFMessageUtils'),
    'xarm_msgs/msg/RobotMsg': ('rosbag2mongo.messages.xarm_robot', 'XArmRobotMsgUtils'),
    'robot_swiss_knife_msgs/msg/AdmittanceControlParams': ('rosbag2mongo.messages.admittance_control_params', 'AdmittanceControlParamsMsgUtils')
}