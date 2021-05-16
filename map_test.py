import interfaces
import core
import algorithms
import math
import gmplot


def main():
    print("***")
    # Create the map plotter:
    apikey = "AIzaSyBcdzPM-dICBYxG9D5eqEHnXYDRiqoGHvc"  # (your API key here)
    gmap = gmplot.GoogleMapPlotter(37.951755, -91.778007, 14, apikey=apikey)

    # Mark a hidden gem:
    gmap.marker(interfaces.nav_board.location()[0], interfaces.nav_board.location()[1], color="cornflowerblue")

    # Highlight some attractions:
    tag_lats, tag_lngs = zip(
        *[
            (37.951753, -91.778077),
            (37.951729, -91.778077),
        ]
    )
    gmap.scatter(tag_lats, tag_lngs, color="#3B0B39", size=2, marker=False)

    # Calculate bearing and distance for the midpoint between the two tags

    angle1, dist1 = algorithms.geomath.haversine(
        interfaces.nav_board.location()[0], interfaces.nav_board.location()[1], 37.951753, -91.778077
    )
    angle2, dist2 = algorithms.geomath.haversine(
        interfaces.nav_board.location()[0], interfaces.nav_board.location()[1], 37.951729, -91.778077
    )
    distance = int((dist1 + dist2) / 2 * 1000)
    print(distance)
    angle = int((angle1 + angle2) / 2)

    start = core.Coordinate(interfaces.nav_board.location()[0], interfaces.nav_board.location()[1])

    # Get a GPS coordinate using our distance and bearing
    target = algorithms.obstacle_avoider.coords_obstacle(distance, start[0], start[1], angle)
    print(start)
    print(target)

    # Also calculate second point (to run through the gate)
    halfGateDist = math.sqrt((5.05 ** 2) + (distance ** 2) - 2 * 5.05 * distance * math.cos(angle))
    rightTriSide = math.sin(angle) * 5.05
    complementAngle = math.asin(rightTriSide / halfGateDist)
    targetPastGateHeading = -90 + complementAngle
    targetPastGate = algorithms.obstacle_avoider.coords_obstacle(2, target[0], target[1], targetPastGateHeading)
    print(targetPastGate)
    path_lats, path_lngs = zip(*[(target[0], target[1]), (targetPastGate[0], targetPastGate[1])])
    print(path_lats[0], path_lngs[0])
    # print(path_lats[1], path_lngs[1])
    gmap.scatter(path_lats, path_lngs, color="#FF00FF", size=0.5, marker=False)

    # Draw the map to an HTML file:
    print("***")
    gmap.draw("map.html")
    return 0