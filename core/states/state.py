import logging


class RoverState:
    """
    A state class abstraction which serves as part of the state machine
    """

    def __init__(self):
        self.logger = logging.getLogger(__name__)

        # Call start(), use to schedule tasks
        self.start()

    async def run(self):
        pass

    def on_event(self, event):
        pass

    def start(self):
        pass

    def exit(self):
        pass

    def __str__(self):
        return self.__class__.__name__

    def __repr__(self):
        return self.__class__.__name__

    def __eq__(self, other):
        return str(self) == str(other)

    def __hash__(self):
        return hash(self.__class__.__name__)

    def __ne__(self, other):
        return str(self) != str(other)
