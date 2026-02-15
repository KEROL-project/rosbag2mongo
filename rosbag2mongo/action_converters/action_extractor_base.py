from abc import ABC, abstractmethod

class ActionExtractorBase(ABC):
    """Abstract base class for extracting robot actions from raw data.
    """
    @abstractmethod
    def get_actions(self):
        """Abstract method for extracting robot actions.
        """
        raise NotImplementedError()