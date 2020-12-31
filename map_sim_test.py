from osmviz.animation import TrackingViz, Simulation

# Define begin/end points, duration, and icon for our train

start_lat, start_lon = (45.77, -68.65)  # Northeast
end_lat, end_lon = (30.05, -118.25)  # Southwest
begin_time, end_time = 0, 60  # In 60 seconds!

image_f = "images/train.png"

# Define bounds for the train and zoom level, how much map do we show?

bound_ne_lat, bound_ne_lon = (46, -68.5)
bound_sw_lat, bound_sw_lon = (30, -119)
zoom = 8  # OSM zoom level

# Define an interpolater to create animation points


def locAtTime(t):
    if t < 0:
        return start_lat, start_lon
    if t > 60:
        return end_lat, end_lon
    frac = t / 60.0
    interp_lat = start_lat + frac * (end_lat - start_lat)
    interp_lon = start_lon + frac * (end_lon - start_lon)
    return interp_lat, interp_lon


# Create a TrackingViz

viz = TrackingViz(
    "Continental Espresso",
    image_f,
    locAtTime,
    (begin_time, end_time),
    (bound_sw_lat, bound_ne_lat, bound_sw_lon, bound_ne_lon),
    1,
)  # Drawing order

# Add our TrackingViz to a Simulation and then run the simulation

sim = Simulation(
    [
        viz,
    ],
    [],
    0,
)  # ([actor vizs], [scene vizs], initial time)
sim.run(speed=1, refresh_rate=0.1, osmzoom=zoom)
