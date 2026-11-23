from typing import List, Dict
import numpy as np
from geometry_msgs.msg import WrenchStamped

from rosbag2mongo.messages.message_utilities_base import MessageUtilitiesBase

class WrenchStampedUtils(MessageUtilitiesBase):
    """Utility interface for messages of type geometry_msgs/msg/Wrench.
    """
    def __init__(self, msg: WrenchStamped):
        super(WrenchStampedUtils, self).__init__(msg)

    def get_data(self) -> Dict[str, List[float]]:
        """Extracts wrench data and returns it in the form of a dictionary in which
        the keys are data item names ('force' and 'torque'), and the values are stored as lists.
        """
        wrench_data = {
            'force': [self.msg.wrench.force.x, self.msg.wrench.force.y, self.msg.wrench.force.z],
            'torque': [self.msg.wrench.torque.x, self.msg.wrench.torque.y, self.msg.wrench.torque.z]
        }
        return wrench_data