#
# Mars Rover Design Team
# pid_controller.py
#
# Created on Jul 07, 2020
# Updated on Aug 21, 2022
#

import time
import logging

from algorithms.helper_funcs import clamp


class PIDcontroller:
    """
    Simple PID controller
    Does not need to be run at a fixed frequency; can adjust itself using system clock.
    """

    def __init__(self, Kp=1, Ki=0, Kd=0, accumulated_error_clamp=100, wraparound=None):
        """
        :param Kp: (Real Number) Proportional Gain
        :param Ki: (Real Number) Integral Gain
        :param Kd: (Real number) Derivative Gain
        :param accumulated_error_clamp: Positive number Maximum allowed accumulated error for integral term
        :param wraparound: (None or tuple) None in most situations. Maximum possible value when values wrap around
            after a certain point
        """

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.err_clamp = accumulated_error_clamp
        self.wraparound = wraparound
        self.prevError = 0
        self.accumulatedError = 0
        self.prevTime = time.time()
        self.logger = logging.getLogger(__name__)

    def update(self, set_point, real_position):
        """
        updates the PID controller with the set point and current position,
        returns output we can use to scale smoothly towards our set point

        :param set_point:
        :param real_position:
        """
        Ts = time.time() - self.prevTime
        error = set_point - real_position

        if self.wraparound:
            if error > (self.wraparound / 2.0):
                error = error - self.wraparound
            elif error < -(self.wraparound / 2.0):
                error = error + self.wraparound
        self.logger.debug("PID error: ", error, "\tAccumulated Error: ", self.accumulatedError)
        self.accumulatedError += error * Ts
        self.accumulatedError = clamp(self.accumulatedError, -self.err_clamp, +self.err_clamp)
        if abs(error) < 2:
            # PID theory says we should reset the accumulated error to 0 when we're at our
            # target position, allowing 2 degrees to either side of the rover allows that to be reset properly. We
            # might need to further tune this constant at a later time to ensure proper heading holds.
            self.accumulatedError = 0
        output = self.Kp * error + self.Ki * self.accumulatedError + self.Kd * ((error - self.prevError) / Ts)
        self.prevError = error
        self.prevTime = time.time()
        return output
