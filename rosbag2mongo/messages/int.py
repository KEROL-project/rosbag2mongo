from typing import Any

from rosbag2mongo.messages.message_utilities_base import MessageUtilitiesBase

class IntUtils(MessageUtilitiesBase):
    """Utility interface for messages of type std_msgs/msg/Int{XX}
    """
    def __init__(self, msg: Any):
        super(IntUtils, self).__init__(msg)

    def get_data(self) -> int:
        """Extracts data from an int message.
        """
        return self.msg.data