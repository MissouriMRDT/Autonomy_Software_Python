#
# Mars Rover Design Team
# map_test.py
#
# Created on Jul 06, 2020
# Updated on Aug 21, 2022
#

import interfaces
import core
import algorithms
import math
import gmplot


def main():
    print("***")
    # Create the map plotter:
    apikey = ""  # (your API key here)
    gmap = gmplot.GoogleMapPlotter(37.951755, -91.778007, 14, apikey=apikey)

    # Mark a hidden gem:
    gmap.marker(interfaces.nav_board.location()[0], interfaces.nav_board.location()[1], color="cornflowerblue")

    # Highlight some attractions:
    tag_lats, tag_lngs = zip(*[(37.951753, -91.778077), (37.951753, -91.778053)])
    gmap.scatter(tag_lats, tag_lngs, color="#3B0B39", size=2, marker=False)

    # Calculate bearing and distance for the midpoint between the two tags

    angle1, dist1 = algorithms.geomath.haversine(
        interfaces.nav_board.location()[0], interfaces.nav_board.location()[1], tag_lats[0], tag_lngs[0]
    )
    angle2, dist2 = algorithms.geomath.haversine(
        interfaces.nav_board.location()[0], interfaces.nav_board.location()[1], tag_lats[1], tag_lngs[1]
    )
    angle1 = angle1 - interfaces.nav_board.heading()
    angle2 = angle2 - interfaces.nav_board.heading()

    if angle1 >= 180:
        angle1 -= 360
    if angle2 >= 180:
        angle2 -= 360

    dist1 *= 1000
    dist2 *= 1000
    print(angle1, angle2, dist1, dist2)
    combinedAngle = 0
    # Calculate bearing and distance for the midpoint between the two tags
    # use law of cosines to get the distance between the tags, midpoint will be halfway between them
    if angle1 < 0 and angle2 < 0:
        print("BOTH NEGATIVE")
        larger = min(angle1, angle2)
        smaller = max(angle1, angle2)
        combinedAngle = abs(larger) - abs(smaller)
    elif angle1 >= 0 and angle2 >= 0:
        print("BOTH POSITIVE")
        larger = max(angle1, angle2)
        smaller = min(angle1, angle2)
        combinedAngle = larger - smaller
    else:
        print("IN BETWEEN")
        combinedAngle = abs(angle1) + abs(angle2)
    print("combined angle:", combinedAngle)

    # for calculations, our starting side , "D1", should be the shorter distance
    D1 = min(dist1, dist2)

    gateWidth = math.sqrt((dist1 ** 2) + (dist2 ** 2) - 2 * dist1 * dist2 * math.cos(math.radians(combinedAngle)))
    print("gate width:", gateWidth)
    # use law of sines to get the angle across from tag[0]'s distance and deduce the last angle
    print((math.sin(math.radians(combinedAngle / 2)) * D1) / (gateWidth / 2))
    sinVal = (math.sin(math.radians(combinedAngle / 2)) * D1) / (gateWidth * 0.5)
    if sinVal > 1:
        sinDiff = sinVal - 1
        sinVal = 1 - sinDiff
    elif sinVal < -1:
        sinDiff = sinVal + 1
        sinVal = -1 - sinDiff
    angleAcrossD1 = math.asin(sinVal)
    print("angle across from D1:", math.degrees(angleAcrossD1))
    angleAcrossDm = math.pi - angleAcrossD1 - math.radians(combinedAngle / 2)
    print("angle across from Dm:", math.degrees(angleAcrossDm))
    distToMidpoint = ((gateWidth / 2) * math.sin(angleAcrossDm)) / math.sin(math.radians(combinedAngle / 2))
    print("distance to midpoint", distToMidpoint)

    trueAngle, trueDist = algorithms.geomath.haversine(
        interfaces.nav_board.location()[0], interfaces.nav_board.location()[1], 37.951753, -91.778065
    )

    print("dist should be:", trueDist * 1000)

    if angle1 < 0 and angle2 < 0:
        angleToMidpoint = (interfaces.nav_board.heading() - (abs(larger) - (combinedAngle / 2))) % 360
    elif angle1 >= 0 and angle2 >= 0:
        angleToMidpoint = (interfaces.nav_board.heading() + (abs(larger) - (combinedAngle / 2))) % 360
    else:
        angleToMidpoint = (interfaces.nav_board.heading() - (combinedAngle / 2)) % 360

    start = core.Coordinate(interfaces.nav_board.location()[0], interfaces.nav_board.location()[1])
    print("angle to midpoint: ", angleToMidpoint)
    # Get a GPS coordinate using our distance and bearing
    target = algorithms.obstacle_avoider.coords_obstacle(distToMidpoint, start[0], start[1], angleToMidpoint)

    # Also calculate a second and third point (to run through the gate)
    targetPastGateHeading = ((angleAcrossD1 - (math.pi / 2)) + interfaces.nav_board.heading()) % 360
    print("target gate heading:", targetPastGateHeading)
    targetBeforeGate = algorithms.obstacle_avoider.coords_obstacle(-3, target[0], target[1], targetPastGateHeading)
    targetPastGate = algorithms.obstacle_avoider.coords_obstacle(2, target[0], target[1], targetPastGateHeading)
    print(targetPastGate)
    path_lats, path_lngs = zip(
        *[(target[0], target[1]), (targetPastGate[0], targetPastGate[1]), (targetBeforeGate[0], targetBeforeGate[1])]
    )
    print(path_lats[0], path_lngs[0])
    gmap.scatter(path_lats, path_lngs, color="#FF00FF", size=0.5, marker=False)

    # Draw the map to an HTML file:
    print("***")
    gmap.draw("map.html")
    return 0
