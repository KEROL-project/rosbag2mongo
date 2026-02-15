from typing import List, Dict
import numpy as np
from xarm_msgs.msg import RobotMsg

from rosbag2mongo.messages.message_utilities_base import MessageUtilitiesBase

class XArmRobotMsgUtils(MessageUtilitiesBase):
    """Utility interface for messages of type xarm_msgs/msg/RobotMsg.
    """
    def __init__(self, msg: RobotMsg):
        super(XArmRobotMsgUtils, self).__init__(msg)

    def get_data(self) -> Dict[str, List]:
        """Extracts the xarm data and returns it in the form of a dictionary in which
        the keys are data item names --- 'joint_angles', 'ee_pose' --- and the values
        are stored as lists.
        """
        joint_state_data = {
            'joint_angles': list(map(float, self.msg.angle)),
            'ee_pose': [
                float(self.msg.pose[0]) / 1000., # the x position is given in mm, so we convert it to m
                float(self.msg.pose[1]) / 1000., # the y position is given in mm, so we convert it to m
                float(self.msg.pose[2]) / 1000., # the z position is given in mm, so we convert it to m
                float(self.msg.pose[3]),
                float(self.msg.pose[4]),
                float(self.msg.pose[5])
            ]
        }
        return joint_state_data