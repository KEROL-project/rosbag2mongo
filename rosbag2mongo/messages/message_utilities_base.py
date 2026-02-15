from abc import ABC, abstractmethod

class MessageUtilitiesBase(ABC):
    """Abstract base class for ROS message utilities.
    """
    def __init__(self, msg):
        self.msg = msg

    @abstractmethod
    def get_data(self):
        """Abstract method for extracting message data.
        """
        raise NotImplementedError()