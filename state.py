class State(object):

    def __init__(self):
        print("Processing current state: ", str(self))

    def handle_event(self, event):
        pass

    def __str__(self):
        return self.__class__.__name__

    def __repr__(self):
        return self.__str__()

