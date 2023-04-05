#
# Mars Rover Design Team
# ar_tag_test.py
#
# Created on Mar 7, 2023
# Updated on Mar 7, 2023
#

import time
import cv2
from core import constants
from algorithms import ar_tag
from interfaces import nav_board
from unittest.mock import MagicMock, patch
import numpy as np


"""
UNIT TEST
FILE: ar_tag.py

This file provides unit tests for ar_tag.py
"""

# Rolla GPS coordinates
rolla_coord = constants.Coordinate(37.951424, -91.768959)


def setup_module(module):
    # Set up the test module by mocking the nav_board location to always be the rolla coordinates
    # This way we can rely on our location to always be the rolla coordinates for testing purposes
    nav_board.location = MagicMock(return_value=rolla_coord)

    # Sleep for .1 second to let the PID controller accurate compute deltas
    time.sleep(0.1)  # unsure if needed, gps_navigate_test had it


def test_tag_init():
    tag = ar_tag.Tag(1, (rolla_coord[0], rolla_coord[1]), (40, 39))

    assert tag.id == 1
    assert tag.lat == rolla_coord[0]
    assert tag.long == rolla_coord[1]
    assert tag.cX == 40
    assert tag.cY == 39
    assert tag.detected == 1
    assert tag.empty == 0
    assert np.isnan(tag.distance)
    assert np.isnan(tag.angle)


def test_check_tag_true():
    tag = ar_tag.Tag(1, (10, 20), (40, 40))
    tag2 = ar_tag.Tag(1, (50, 70), (40, 40))

    assert tag.check_tag(tag2.id, (tag2.lat, tag2.long))


def test_check_tag_false():
    tag = ar_tag.Tag(1, (10, 20), (40, 40))
    tag2 = ar_tag.Tag(2, (50, 70), (40, 40))

    assert not tag.check_tag(tag2.id, (tag2.lat, tag2.long))


def test_tag_spotted_detected_less_than_Frames_detected():
    tag = ar_tag.Tag(1, (10, 20), (40, 40))
    tag.tag_spotted((100, 110))

    assert tag.detected == 2
    assert tag.lat == 100
    assert tag.long == 110


def test_tag_spotted_detected_more_than_Frames_detected():
    with patch('algorithms.ar_tag.track_ar_tag') as mock_track_ar_tag:
        mock_track_ar_tag.return_value = (1, 2)
        tag = ar_tag.Tag(1, (10, 20), (40, 40))
        tag.detected = 4

        tag.tag_spotted((100, 110))

        assert tag.lat == 100
        assert tag.long == 110
        assert tag.distance == 1
        assert tag.angle == 2


def test_reset_spotted():
    tag = ar_tag.Tag(1, (10, 20), (40, 40))
    tag.reset_spotted()

    assert tag.detected == 0


def test_print(capsys):
    tag = ar_tag.Tag(1, (10, 20), (40, 40))
    tag.print()
    captured = capsys.readouterr()

    assert captured.out == "Ids: 01   |   Detected: 01\n"


def test_get_gps():
    lat, lon = ar_tag.get_gps()

    assert lat == rolla_coord[0]
    assert lon == rolla_coord[1]


def test_add_tag():
    with patch('algorithms.ar_tag.get_gps') as mock_get_gps:
        mock_get_gps.return_value = (37.951424, -91.768959)

        ar_tag.add_tag(0, [[[[850., 250.], [916., 268.], [912., 365.], [845., 355.]]]])

        assert ar_tag.detected_tags[0].id == 0
        assert ar_tag.detected_tags[0].lat == rolla_coord[0]
        assert ar_tag.detected_tags[0].long == rolla_coord[1]
        assert ar_tag.detected_tags[0].cX == 883
        assert ar_tag.detected_tags[0].cY == 302.5


def test_detect_ar_tag_none():
    img_path = "./resources/tests/expected/expected_search_pattern.png"
    reg_img_none = cv2.imread(img_path)
    ar_tag.detected_tags = []  # reset detected tags

    tags, reg_img_none = ar_tag.detect_ar_tag(reg_img_none)

    assert len(tags) == 0


def test_detect_ar_tag_one_then_none():
    img_path_none = "./resources/tests/expected/expected_search_pattern.png"
    reg_img_none = cv2.imread(img_path_none)
    ar_tag.detected_tags = []  # reset detected tags

    tag = ar_tag.Tag(1, (rolla_coord[0], rolla_coord[1]), (40, 39))
    ar_tag.detected_tags.append(tag)
    tags, reg_img_none = ar_tag.detect_ar_tag(reg_img_none)

    assert tag.empty == 1


def test_detect_ar_tag_one():
    img_path = "./resources/tests/input/tag00.png"
    reg_img_one = cv2.imread(img_path)
    ar_tag.detected_tags = []  # reset detected tags

    tags, reg_img_one = ar_tag.detect_ar_tag(reg_img_one)

    assert tags[0].id == 0
    assert tags[0].lat == rolla_coord[0]
    assert tags[0].long == rolla_coord[1]
    assert len(tags) == 1


def test_detect_ar_tag_id_not_in_frametags():
    img_path = "./resources/tests/input/tag00.png"
    img_path2 = "./resources/tests/input/tag01.png"
    reg_img_one = cv2.imread(img_path)
    reg_img_two = cv2.imread(img_path2)
    ar_tag.detected_tags = []  # reset detected tags

    tags, reg_img_one = ar_tag.detect_ar_tag(reg_img_one)
    tags, reg_img_two = ar_tag.detect_ar_tag(reg_img_two)

    assert tags[0].empty == 1


def test_detect_ar_tag_multiple():
    img_path = "./resources/tests/input/tag00.png"
    img_path2 = "./resources/tests/input/Aruco_Test.png"
    reg_img_one = cv2.imread(img_path)
    reg_img_multiple = cv2.imread(img_path2)

    tags, reg_img_one = ar_tag.detect_ar_tag(reg_img_one)
    tags, reg_img_multiple = ar_tag.detect_ar_tag(reg_img_multiple)

    assert tags[4].lat == rolla_coord[0]
    assert tags[5].long == rolla_coord[1]
    assert len(tags) == 6

    ar_tag.detected_tags = []  # reset detected tags


@patch('core.vision.camera_handler.grab_depth_data')
@patch('core.vision.camera_handler.get_depth_res')
@patch('core.vision.camera_handler.get_hfov')
def test_track_ar_tag(mock_get_hfov, mock_get_depth_res, mock_grab_depth_data):
    mock_grab_depth_data.return_value = np.array([[1, 1],
                                                 [1, 1]])
    mock_get_depth_res.return_value = (640, 360)
    mock_get_hfov.return_value = 85

    distance, angle = ar_tag.track_ar_tag((0, 0))

    assert distance == 0.001
    assert angle == -42.5
