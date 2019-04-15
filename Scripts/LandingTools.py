import time

import numpy as np
from customPid import PidController

frame_x = np.array([1.0, 0.0, 0.0])
frame_y = np.array([0.0, 1.0, 0.0])
frame_z = np.array([0.0, 0.0, 1.0])


def norm(v):
    return np.linalg.norm(v)


def angle_between_vectors(u, v):
    """ Compute the angle between vector u and v
        Directly from krpc docs.
        https://krpc.github.io/krpc/tutorials/pitch-heading-roll.html
    """
    dp = np.dot(u, v)
    if dp == 0:
        return 0
    u_norm = norm(u)
    v_norm = norm(v)
    return np.math.acos(dp / (u_norm * v_norm)) * (180. / np.pi)


def get_waypoint_here(ot, surface_flight):
    lat = surface_flight.latitude
    long = surface_flight.longitude
    elevation = ot.vessel.orbit.body.surface_height(lat, long)
    print((elevation, lat, long))
    exit()


def landing_script(ot):
    dt = 0.1
    body = ot.vessel.orbit.body
    frame = body.reference_frame
    flight = ot.vessel.flight(frame)

    surface_frame = ot.vessel.surface_reference_frame
    surface_flight = ot.vessel.flight(surface_frame)

    # get_waypoint_here(ot, surface_flight)

    target_lat, target_long = (-0.10672501314899303, -74.56648099627677)
    target_alt = ot.vessel.orbit.body.surface_height(target_lat, target_long)
    pos = body.surface_position(target_lat, target_long, surface_frame)
    target_pos = np.array(pos)
    print("Target Position: {}".format(target_pos))

    alt_pid_gains = (5.0, 0.0, 0.8)
    alt_pid = PidController(gains=alt_pid_gains)

    vert_accel_gains = (3.0, 0.0, 0.01)
    vert_accel_bounds = (-body.surface_gravity, body.surface_gravity)
    vertical_accel_pid = PidController(gains=vert_accel_gains, effort_bounds=vert_accel_bounds)

    lateral_vel_gains = (1.0, 0.0, 1.1)
    lateral_accel_gains = (0.5, 0.0, 0.3)
    y_vel_pid = PidController(gains=lateral_vel_gains)
    y_accel_pid = PidController(gains=lateral_accel_gains)

    z_vel_pid = PidController(gains=lateral_vel_gains)
    z_accel_pid = PidController(gains=lateral_accel_gains)

    desired_alt = surface_flight.mean_altitude
    engage_alt = 2000.0
    ot.auto_on()

    while True:

        mass = ot.vessel.mass

        # Helpers #
        target_alt = ot.vessel.orbit.body.surface_height(target_lat, target_long)
        pos = body.surface_position(target_lat, target_long, surface_frame)
        target_pos = np.array(pos)

        lat = surface_flight.latitude
        long = surface_flight.longitude
        altitude = surface_flight.mean_altitude
        cur_pos = np.array([altitude, lat, long])

        body_flight = ot.vessel.flight(body.reference_frame)
        vel = body_flight.velocity
        velocity = ot.ksc.transform_direction(vel, body.reference_frame, ot.vessel.surface_reference_frame)
        x_vel, y_vel, z_vel = velocity[0], velocity[1], velocity[2]

        vertical_err = altitude - target_alt
        max_lat_speed = 0.1 + 0.0003*(vertical_err)**2
        max_lat_accel = 1.0 + 0.5*np.log(vertical_err+1)
        max_descent_speed = -(3.0 + 0.2 * vertical_err)
        max_vert_speed = 10.0

        alt_pid_bounds = (max_descent_speed, max_vert_speed)
        alt_pid.set_bounds(alt_pid_bounds)
        vertical_accel_pid.set_bounds((0.0, 3.0*body.surface_gravity))

        y_vel_pid.set_bounds((-max_lat_speed, max_lat_speed))
        z_vel_pid.set_bounds((-max_lat_speed, max_lat_speed))
        y_accel_pid.set_bounds((-max_lat_accel, max_lat_accel))
        z_accel_pid.set_bounds((-max_lat_accel, max_lat_accel))

        desired_alt += max_descent_speed * dt
        desired_alt = min(desired_alt, target_alt - 1.0)
        if ot.vessel.situation.name == 'landed':
            print("Landing Completed")
            break

        # Vertical Thrust #
        mu = body.gravitational_parameter
        r = body.equatorial_radius + altitude
        g = mu / r ** 2
        desired_vertical_speed = alt_pid.get_effort(desired_alt, altitude, dt)
        desired_vertical_accel = vertical_accel_pid.get_effort(desired_vertical_speed, x_vel, dt)
        vert_accel = desired_vertical_accel + g

        # Y Thrust #
        desired_y_speed = y_vel_pid.get_effort(target_pos[1], cur_pos[1], dt)
        y_accel = y_accel_pid.get_effort(desired_y_speed, y_vel, dt)

        # Z Thrust #
        desired_z_speed = z_vel_pid.get_effort(target_pos[2], cur_pos[2], dt)
        z_accel = z_accel_pid.get_effort(desired_z_speed, z_vel, dt)

        accel = np.array([vert_accel, -y_accel, -z_accel])

        thrust = mass * accel

        thrust_x, thrust_y, thrust_z = thrust[0], thrust[1], thrust[2]
        thrust_yz = np.sqrt(thrust_y**2 + thrust_z**2)
        thrust_x = max(thrust_yz, thrust_x)

        angle_from_up = (angle_between_vectors(frame_x, np.array(surface_flight.direction)))/(180. / np.pi)
        command_thrust = thrust_x/np.cos(angle_from_up)
        throttle_command = command_thrust / ot.vessel.max_thrust

        if surface_flight.mean_altitude > engage_alt:
            to_target = -target_pos + cur_pos
            target_dir = to_target[0]/5.0, to_target[1], to_target[2]
            ot.vessel.auto_pilot.target_direction = target_dir
            ot.vessel.control.throttle = 0.0
        else:
            ot.vessel.control.throttle = throttle_command
            ot.vessel.auto_pilot.target_direction = (thrust_x, -thrust_y, -thrust_z)

        time.sleep(dt)

    ot.auto_off()
