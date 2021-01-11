import time
import logging


def clamp(n, min_n, max_n):
    return max(min(max_n, n), min_n)


class PIDcontroller:
    """Simple PID controller.
    Does not need to be run at a fixed frequency; can adjust itself using system clock.

    Parameters
    ----------
        Kp : Real Number
            Proportional Gain.
        Ki : Real Number
            Integral Gain.
        Kd : Real number
            Derivative Gain
        accumulated_error_clamp : Positive number
            Maximum allowed accumulated error for integral term
        wraparound : None or tuple
            None in most situations. Maximum possible value when values wrap around after a certain point
            Example: When working with degrees, set wraparound to 360
                     Situation: Position = 350, setpoint = 10
                     wraparound=None: go counterclockwise 340 degrees
                     wraparound=360: go clockwise 20 degrees"""

    def __init__(self, Kp=1, Ki=0, Kd=0, accumulated_error_clamp=100, wraparound=None):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.err_clamp = accumulated_error_clamp
        self.wraparound = wraparound
        self.prevError = 0
        self.accumulatedError = 0
        self.prevTime = time.time()
        self.logger = logging.getLogger(__name__)

    def update(self, setpoint, real_position):
        """
        updates the PID controller with the setpoint and current position,
        returns output we can use to scale smoothly towards our setpoint
        """
        Ts = time.time() - self.prevTime
        error = setpoint - real_position

        if self.wraparound:
            if error > (self.wraparound / 2.0):
                error = error - self.wraparound
            elif error < -(self.wraparound / 2.0):
                error = error + self.wraparound
        self.logger.debug("PID error: ", error, "\tAccumulated Error: ", self.accumulatedError)
        self.accumulatedError += error * Ts
        self.accumulatedError = clamp(self.accumulatedError, -self.err_clamp, +self.err_clamp)
        if abs(error) < 2:
            self.accumulatedError = 0  # PID theory says we should reset the accumulated error to 0 when we're at our target position, allowing 2 degrees to either side of the rover allows that to be reset properly. We might need to further tune this constant at a later time to ensure proper heading holds.
        output = self.Kp * error + self.Ki * self.accumulatedError + self.Kd * ((error - self.prevError) / Ts)
        self.prevError = error
        self.prevTime = time.time()
        return output
