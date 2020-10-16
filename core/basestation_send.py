import core
from core.rovecomm import RoveCommPacket
import logging


def basestation_send(event, value, log_msg):
    """
    Sends some data over the socket based on log message
    """
    logger = logging.getLogger(__name__)

    # Matches log 'event' to event from predefined dict
    if event in core.constants.rovecomm_event_list:
        event_data = core.constants.rovecomm_event_list[event]
        data_id = event_data['data_id']
        data_type = event_data['data_type']
        # If the event uses preset values, match the correct one
        if 'values' in event_data:
            if value in event_data['values']:
                data_value = tuple(event_data['values'][value])
            else:
                logger.warning(
                    f'{value} is not a valid value for {event}')
                value = ()
        # Otherwise take the value literally
        else:
            data_value = value

        # Pack up and send the data
        packet = RoveCommPacket(
            data_id,
            data_type,
            data_value,
            "",
            11111
        )
        packet.SetIp("127.0.0.1")
        packet.print()
        logger.info(f"{event} - {value} - {log_msg}")
        core.rovecomm.write(packet, True)
    else:
        logger.warning(f'{event} is not a valid event')
