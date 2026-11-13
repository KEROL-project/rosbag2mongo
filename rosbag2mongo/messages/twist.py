from typing import List, Dict
import numpy as np
from geometry_msgs.msg import Twist

from rosbag2mongo.messages.message_utilities_base import MessageUtilitiesBase

class TwistUtils(MessageUtilitiesBase):
    """Utility interface for messages of type geometry_msgs/msg/Twist.
    """
    def __init__(self, msg: Imu):
        super(TwistUtils, self).__init__(msg)

    def get_data(self) -> Dict[str, List[float]]:
        """Extracts wrench data and returns it in the form of a dictionary in which
        the keys are data item names ('linear' and 'angular'), and the values are stored as lists.
        """
        twist_data = {
            'linear': [self.msg.linear.x, self.msg.linear.y, self.msg.linear.z],
            'angular': [self.msg.angular.x, self.msg.angular.y, self.msg.angular.z]
        }
        return twist_data