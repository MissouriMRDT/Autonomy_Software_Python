import core
from core import RoveCommPacket
import logging


def telemetry_handler(event, value, log_msg):
    """
    Sends some numerical data over the socket

    Returns:
        success (int): An integer, either 0 or 1 depending on whether or not
            an exception occured during writing
    """
    logger = logging.getLogger(__name__)

    # Matches log 'event' to event from predefined dict
    if event in core.rovecomm_event_list:
        event_data = core.rovecomm_event_list[event]
        data_id = event_data["data_id"]
        data_type = event_data["data_type"]
        # If the event uses preset values, match the correct one
        if "values" in event_data:
            if value in event_data["values"]:
                data_value = tuple(event_data["values"][value])
            else:
                logger.warning(f"{value} is not a valid value for {event}")
                data_value = ()
        # Otherwise take the value literally
        else:
            data_value = value

        # Pack up and send the data
        packet = RoveCommPacket(data_id, data_type, data_value, "", 0)
        logger.info(f"{event}: {value} - {log_msg}")
        return core.rovecomm_node.write(packet, True)
    else:
        logger.warning(f"{event} is not a valid event")
