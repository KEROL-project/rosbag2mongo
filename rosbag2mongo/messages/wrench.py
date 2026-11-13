from typing import List, Dict
import numpy as np
from geometry_msgs.msg import Wrench

from rosbag2mongo.messages.message_utilities_base import MessageUtilitiesBase

class WrenchUtils(MessageUtilitiesBase):
    """Utility interface for messages of type geometry_msgs/msg/Wrench.
    """
    def __init__(self, msg: Imu):
        super(WrenchUtils, self).__init__(msg)

    def get_data(self) -> Dict[str, List[float]]:
        """Extracts wrench data and returns it in the form of a dictionary in which
        the keys are data item names ('force' and 'torque'), and the values are stored as lists.
        """
        wrench_data = {
            'force': [self.msg.force.x, self.msg.force.y, self.msg.force.z],
            'torque': [self.msg.torque.x, self.msg.torque.y, self.msg.torque.z]
        }
        return wrench_data