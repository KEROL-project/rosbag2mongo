from typing import Any

from rosbag2mongo.messages.message_utilities_base import MessageUtilitiesBase

class FloatUtils(MessageUtilitiesBase):
    """Utility interface for messages of type std_msgs/msg/Float{XX}
    """
    def __init__(self, msg: Any):
        super(FloatUtils, self).__init__(msg)

    def get_data(self) -> float:
        """Extracts data from a float message.
        """
        return self.msg.data