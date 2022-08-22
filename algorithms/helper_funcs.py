#
# Mars Rover Design Team
# helper_funcs.py
#
# Created on Aug 21, 2022
# Updated on Aug 21, 2022
#

def clamp(x, minimum, maximum):
    """
    Clamps the x value between the min and max values

    :param x: value to clamp
    :param minimum: min value
    :param maximum: max value
    :return: val - the clamped value
    """
    return max(minimum, min(x, maximum))