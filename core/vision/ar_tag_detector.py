import asyncio
import core
import algorithms
import logging
import traceback
import copy
from core.constants import ARUCO_FRAMES_DETECTED

# List to hold the tag info
leg_valid_tags = []
distances = []
clear_tags_toggle = False


async def async_ar_tag_detector():
    """
    Async function to find obstacles.
    """
    # Setup logger for function.
    logger = logging.getLogger(__name__)
    # Declare detection objects.
    TagDetector = algorithms.ar_tag.ArucoARTagDetector()
    # Declare toggle for reseting tags.
    global clear_tags_toggle
    global leg_valid_tags
    global distances

    while True:
        # Maybe this is bad practice but it's helpful.
        try:
            # Get normal image from camera.
            reg_img = core.vision.camera_handler.grab_regular()

            # Detect tags.
            TagDetector.detect_ar_tag(reg_img)
            # Filter tags.
            TagDetector.filter_ar_tags(angle_range=180, distance_range=25, valid_id_range=[0, 1, 2, 3, 4, 5])
            # Get and store tags.
            ar_tags = TagDetector.get_tags()

            # Draw tags.
            reg_img = TagDetector.track_ar_tags(reg_img)
            core.vision.feed_handler.handle_frame("artag", reg_img)

            # Print info about tags if 1 or more detected.
            if len(ar_tags) >= 1:
                # Build output string.
                output = f"{len(ar_tags)} Tags Detected:\n"
                # Add individual tag data to output.
                for tag in ar_tags:
                    output += (
                        f"[ID:{tag.id} DIST:{int(tag.distance)} ANGLE:{int(tag.angle)} DETS:{tag.times_detected}]\n"
                    )
                # Print log output.
                logger.info(output)

            # Check if the waypoint handler has a waypoint.
            if core.waypoint_handler.gps_data:
                # Check if the waypoint is a marker or gate and that we have detected a valid amount of tags.
                if (core.waypoint_handler.gps_data.leg_type == "MARKER" and len(ar_tags) > 0) or (
                    core.waypoint_handler.gps_data.leg_type == "GATE" and len(ar_tags) > 1
                ):
                    # Clear valid_ids list.
                    leg_valid_tags.clear()

                    # Loop through each tag and check if the times_detected for each one is over the threshold.
                    for tag in ar_tags:
                        if tag.times_detected >= ARUCO_FRAMES_DETECTED:
                            leg_valid_tags.append(tag)
                            distances.append(tag.distance)

            if clear_tags_toggle:
                # Clear tags in detector object.
                TagDetector.clear_tags()
                # Reset toggle.
                clear_tags_toggle = False

        except Exception:
            # Because we are using async functions, they don't print out helpful tracebacks. We must do this instead.
            logger.critical(traceback.format_exc())

        await asyncio.sleep(core.EVENT_LOOP_DELAY)


def clear_tags():
    """
    Clears the tag list.

    :returns: None
    """
    # Declare global var.
    global clear_tags_toggle
    global leg_valid_tags
    # Clear detection object by setting toggle.
    clear_tags_toggle = True
    # Clear local lists.
    leg_valid_tags.clear()


def is_marker():
    """
    Returns whether there is a visible marker.

    :return: detect (bool) - whether or not something was detected
    """
    # Declare global var.
    global leg_valid_tags

    return True if len(leg_valid_tags) > 0 and leg_valid_tags[0].id in [0, 1, 2, 3, 4, 5] else False


def is_gate():
    """
    Returns whether there are multiple visible AR tags. We don't look for
    2 specfically because we don't want a false positive to cause this bool
    to fail and abort immediately.

    :return: detect (bool) - whether or not something was detected
    """
    # Declare global var.
    global leg_valid_tags

    return (
        True
        if len(leg_valid_tags) >= 2 and leg_valid_tags[0].id in [4, 5] and leg_valid_tags[1].id in [4, 5]
        else False
    )


def get_distances():
    """
    Returns just the distances of each tag.
    """
    # Declare global var.
    global distances

    return distances


def get_valid_tags():
    """
    Returns a filtered list of valid tags depending on waypoint leg type. (MARKER or GATE)
    """
    # Declare global var.
    global leg_valid_tags

    return leg_valid_tags
