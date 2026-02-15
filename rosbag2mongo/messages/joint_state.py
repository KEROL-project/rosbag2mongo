from typing import List, Dict
from sensor_msgs.msg import JointState

from rosbag2mongo.messages.message_utilities_base import MessageUtilitiesBase

class JointStateUtils(MessageUtilitiesBase):
    """Utility interface for messages of type sensor_msgs/msg/JointState.
    """
    def __init__(self, msg: JointState):
        super(JointStateUtils, self).__init__(msg)

    def get_data(self) -> Dict[str, List]:
        """Extracts joint state data and returns it in the form of a dictionary in which
        the keys are data item names --- 'names', 'positions', 'velocities', and 'efforts'
        --- and the values are stored as lists.
        """
        joint_state_data = {
            'names': list(self.msg.name),
            'positions': list(self.msg.position),
            'velocities': list(self.msg.velocity),
            'efforts': list(self.msg.effort)
        }
        return joint_state_data