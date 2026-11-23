from typing import List, Dict
import numpy as np
from robot_swiss_knife_msgs.msg import AdmittanceControlParams

from rosbag2mongo.messages.message_utilities_base import MessageUtilitiesBase

class AdmittanceControlParamsMsgUtils(MessageUtilitiesBase):
    """Utility interface for messages of type robot_swiss_knife_msgs/msg/AdmittanceControlParams.
    """
    def __init__(self, msg: AdmittanceControlParams):
        super(AdmittanceControlParamsMsgUtils, self).__init__(msg)

    def get_data(self) -> Dict[str, List[float]]:
        """Extracts admittance control parameter data and returns it in the form of a dictionary in which
        the keys are data item names ('selected_axes', 'masses', 'stiffnesses', and 'damping_ratios'),
        and the values are stored as lists.
        """
        admittance_control_params_data = {
            'selected_axes': self.msg.selected_axes.tolist() if type(self.msg.selected_axes) == np.ndarray else self.msg.selected_axes,
            'masses': self.msg.masses.tolist() if type(self.msg.masses) == np.ndarray else self.msg.masses,
            'stiffnesses': self.msg.stiffnesses.tolist() if type(self.msg.stiffnesses) == np.ndarray else self.msg.stiffnesses,
            'damping_ratios': self.msg.damping_ratios.tolist() if type(self.msg.damping_ratios) == np.ndarray else self.msg.damping_ratios
        }
        return admittance_control_params_data