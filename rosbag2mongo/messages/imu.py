from typing import List, Dict
import numpy as np
from sensor_msgs.msg import Imu

from rosbag2mongo.messages.message_utilities_base import MessageUtilitiesBase

class ImuUtils(MessageUtilitiesBase):
    """Utility interface for messages of type sensor_msgs/msg/Imu.
    """
    def __init__(self, msg: Imu):
        super(ImuUtils, self).__init__(msg)

    def get_data(self) -> Dict[str, List[float]]:
        """Extracts IMU data and returns it in the form of a dictionary in which
        the keys are data item names --- 'quat_orientation', 'angular_velocity',
        and 'linear_acceleration' --- and the values are stored as numpy arrays.
        """
        imu_data = {
            'quat_orientation': [self.msg.orientation.x, self.msg.orientation.y, self.msg.orientation.z, self.msg.orientation.w],
            'angular_velocity': [self.msg.angular_velocity.x, self.msg.angular_velocity.y, self.msg.angular_velocity.z],
            'linear_acceleration': [self.msg.linear_acceleration.x, self.msg.linear_acceleration.y, self.msg.linear_acceleration.z]
        }
        return imu_data