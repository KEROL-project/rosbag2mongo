from robot_swiss_knife_msgs.msg import Event

from rosbag2mongo.messages.message_utilities_base import MessageUtilitiesBase

class EventUtils(MessageUtilitiesBase):
    """Utility interface for messages of type robot_swiss_knife_msgs/msg/Event
    """
    def __init__(self, msg: Event):
        super(EventUtils, self).__init__(msg)

    def get_data(self) -> int:
        """Extracts data from an event message.
        """
        return self.msg.description