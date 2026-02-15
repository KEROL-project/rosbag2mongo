from typing import Sequence
import numpy as np
import matplotlib.pyplot as plt
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from rosbag2mongo.messages.message_utilities_base import MessageUtilitiesBase

class ImageUtils(MessageUtilitiesBase):
    """Utility interface for messages of type sensor_msgs/msg/Image.
    """
    def __init__(self, msg: Image):
        super(ImageUtils, self).__init__(msg)
        self.cv_bridge = CvBridge()

    def get_data(self) -> Sequence:
        """Extracts image data and returns it in the form of a numpy array.
        """
        image_array = self.cv_bridge.imgmsg_to_cv2(self.msg, desired_encoding='passthrough')
        image_array = cv2.resize(image_array, (266, 200), interpolation=cv2.INTER_AREA)
        image_array = np.array(image_array, dtype=np.uint8)
        return image_array.tolist()