import time

import numpy as np
from customPid import PidController


def landing_script(ot):
    dt = 0.1
    body = ot.vessel.orbit.body
    frame = body.reference_frame
    flight = ot.vessel.flight(frame)
    vert_accel_gains = (1.0, 0.0, 0.5)
    accel_bound = 1.5 * body.surface_gravity
    vert_accel_bounds = (-accel_bound, accel_bound)
    vertical_acccel_pid = PidController(gains=vert_accel_gains)

    start_time = time.time()
    while time.time() < start_time + 1.0:
        thrust = ot.vessel.available_thrust
        mu = body.gravitational_parameter
        altitude = flight.mean_altitude
        r = body.equatorial_radius + altitude
        g = -mu / r ** 2

        desired_vertical_speed = -np.sqrt(altitude)
        vertical_speed = flight.vertical_speed
        desired_vertical_accel = vertical_acccel_pid.get_effort(desired_vertical_speed, vertical_speed, dt)
        desired_vertical_thrust = desired_vertical_accel + g

        theta = np.arcsin(desired_vertical_thrust / thrust)


