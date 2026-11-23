from typing import List, Dict
import numpy as np
from tf2_msgs.msg import TFMessage

from rosbag2mongo.messages.message_utilities_base import MessageUtilitiesBase

class TFMessageUtils(MessageUtilitiesBase):
    """Utility interface for messages of type tf2_msgs/msg/TFMessage.
    """
    def __init__(self, msg: TFMessage):
        super(TFMessageUtils, self).__init__(msg)

    def get_data(self) -> List[Dict]:
        """Extracts TF data and returns it as a list of dictionaries,
        each of which represents an individual transform.
        """
        tf_data = []
        for tf in self.msg.transforms:
            current_tf_data = {}
            current_tf_data['parent_frame'] = tf.header.frame_id
            current_tf_data['child_frame'] = tf.child_frame_id
            current_tf_data['transform'] = {}
            current_tf_data['transform']['translation'] = [tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z]
            current_tf_data['transform']['rotation'] = [tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w]
            tf_data.append(current_tf_data)
        return tf_data