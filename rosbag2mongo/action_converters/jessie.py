from typing import List

from xarm_msgs.msg import RobotMsg
from rosbag2mongo.action_converters.action_extractor_base import ActionExtractorBase

class ActionConverter(ActionExtractorBase):
    """Utility interface for extracting actions from the Jessie robot.
    """
    def get_actions(self, prev_msg: RobotMsg, last_msg: RobotMsg) -> List[float]:
        """Extracts data from xarm_msgs/msg/RobotMsg messages.
        """
        actions = [
            (last_msg.pose[0] - prev_msg.pose[0]) / 100., # the x position is given in cm, so we convert it to m
            (last_msg.pose[1] - prev_msg.pose[1]) / 100., # the x position is given in cm, so we convert it to m
            (last_msg.pose[2] - prev_msg.pose[2]) / 100., # the x position is given in cm, so we convert it to m
            last_msg.pose[3] - prev_msg.pose[3],
            last_msg.pose[4] - prev_msg.pose[4],
            last_msg.pose[5] - prev_msg.pose[5]
        ]
        return list(map(float, actions))