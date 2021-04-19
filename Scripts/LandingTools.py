import time
from itertools import product

import numpy as np
from Scripts.customPid import PidController
import Scripts.DebugTools as db
from Scripts.Misc import norm, angle_between_vectors, load_config, save_config

frame_x = np.array([1.0, 0.0, 0.0])
frame_y = np.array([0.0, 1.0, 0.0])
frame_z = np.array([0.0, 0.0, 1.0])


# TODO: Update to actually use waypoint?
def get_waypoint_here(ot, surface_flight):
    lat = surface_flight.latitude
    long = surface_flight.longitude
    # y_offset = -0.000050925434539549
    # z_offset = -0.00000601586282
    y_offset = 0.0005
    z_offset = 0.005
    print((lat + y_offset, long + z_offset))
    return lat + y_offset, long + z_offset


def get_waypoint_by_name(ot, name):
    for waypoint in ot.ksc.waypoint_manager.waypoints:
        if waypoint.name == name:
            return waypoint
    return None


def landing_script(ot):
    dt = 0.1
    waypoint_name = "Alwell's Dome"
    waypoint = get_waypoint_by_name(ot, waypoint_name)

    target_lat = waypoint.latitude
    target_long = waypoint.longitude

    config = load_config("branch1")

    lat_speed_coeff = config["lat_speed_coeff"]
    lat_accel_coeff = config["lat_accel_coeff"]
    descent_speed_coeff = config["descent_speed_coeff"]
    base_descent_speed = config["base_descent_speed"]

    alt_pid_p = config["alt_pid_p"]
    alt_pid_d = config["alt_pid_d"]
    vert_accel_p = config["vert_accel_p"]
    vert_accel_d = config["vert_accel_d"]
    lateral_vel_p = config["lateral_vel_p"]
    lateral_vel_d = config["lateral_vel_d"]
    lateral_accel_p = config["lateral_accel_p"]
    lateral_accel_d = config["lateral_accel_d"]
    max_vert_speed = config["max_vert_speed"]

    no_lateral_alt = config["no_lateral_alt"]

    body = ot.vessel.orbit.body
    frame = body.reference_frame
    flight = ot.vessel.flight(frame)

    surface_frame = ot.vessel.surface_reference_frame
    surface_flight = ot.vessel.flight(surface_frame)

    alt_pid_gains = (alt_pid_p, 0.0, alt_pid_d)
    alt_pid = PidController(gains=alt_pid_gains)

    vert_accel_gains = (vert_accel_p, 0.0, vert_accel_d)
    vert_accel_bounds = (-body.surface_gravity, 10.0*body.surface_gravity)
    vertical_accel_pid = PidController(gains=vert_accel_gains, effort_bounds=vert_accel_bounds)

    lateral_vel_gains = (lateral_vel_p, 0.0, lateral_vel_d)
    lateral_accel_gains = (lateral_accel_p, 0.0, lateral_accel_d)
    y_vel_pid = PidController(gains=lateral_vel_gains)
    y_accel_pid = PidController(gains=lateral_accel_gains)

    z_vel_pid = PidController(gains=lateral_vel_gains)
    z_accel_pid = PidController(gains=lateral_accel_gains)


    desired_alt = surface_flight.mean_altitude
    ot.auto_on()
    while True:
        mass = ot.vessel.mass

        # Helpers #
        target_alt = ot.vessel.orbit.body.surface_height(target_lat, target_long)
        target_pos = body.surface_position(target_lat, target_long, surface_frame)
        target_pos = np.array(target_pos)

        lat = surface_flight.latitude
        long = surface_flight.longitude
        altitude = surface_flight.mean_altitude
        cur_pos = np.array([altitude, lat, long])

        body_flight = ot.vessel.flight(body.reference_frame)
        vel = body_flight.velocity
        velocity = ot.ksc.transform_direction(vel, body.reference_frame, ot.vessel.surface_reference_frame)
        x_vel, y_vel, z_vel = velocity[0], velocity[1], velocity[2]

        impact_alt = ot.vessel.orbit.body.surface_height(lat, long)
        print("impact_alt: {}".format(impact_alt))

        vertical_err = altitude - impact_alt
        if vertical_err < 0:
            print("Vertical_err issue")
            break

        vertical_vel_sign = -1 if x_vel < 0 else 1
        max_lat_speed = lat_speed_coeff * np.log(vertical_err + 1)
        max_lat_accel = lat_accel_coeff * np.log(vertical_err + 1)
        # print(round(max_lat_speed, 3), round(max_lat_accel, 3))
        max_descent_speed = vertical_vel_sign*(base_descent_speed + descent_speed_coeff * vertical_err)

        print("vertical_err: {}".format(vertical_err))
        print("descent_speed_coeff: {}".format(descent_speed_coeff))
        print("base_descent_speed: {}".format(base_descent_speed))

        alt_pid_bounds = (max_descent_speed, max_vert_speed)
        alt_pid.set_bounds(alt_pid_bounds)
        vertical_accel_pid.set_bounds((0.0, 12.0 * body.surface_gravity))

        y_vel_pid.set_bounds((-max_lat_speed, max_lat_speed))
        z_vel_pid.set_bounds((-max_lat_speed, max_lat_speed))
        y_accel_pid.set_bounds((-max_lat_accel, max_lat_accel))
        z_accel_pid.set_bounds((-max_lat_accel, max_lat_accel))

        desired_alt += max_descent_speed * dt

        print("max_descent_speed: {}".format(max_descent_speed))
        print("desired_alt: {}".format(desired_alt))
        desired_alt = min(desired_alt, target_alt - 2.0)
        if ot.vessel.situation.name == 'landed':
            touchdown_speed = norm(np.array(velocity))
            print("Landing Completed! Touchdown speed {:0.2f}".format(touchdown_speed))
            break

        # Vertical Thrust #
        mu = body.gravitational_parameter
        r = body.equatorial_radius + altitude
        g = mu / r ** 2

        desired_vertical_speed = alt_pid.get_effort(desired_alt, altitude, dt)
        desired_vertical_accel = vertical_accel_pid.get_effort(desired_vertical_speed, x_vel, dt)
        vert_accel = desired_vertical_accel + g

        print("desired_vertical_speed: {}".format(desired_vertical_speed))
        print("desired_vertical_accel: {}".format(desired_vertical_accel))
        print("vert_accel: {}".format(vert_accel))

        # Y Thrust #
        # desired_y_speed = y_vel_pid.get_effort(target_pos[1], cur_pos[1], dt)
        if vertical_err < no_lateral_alt:
            desired_y_speed = 0.0
            y_accel = y_accel_pid.get_effort(desired_y_speed, y_vel, dt)
        else:
            delta_y = target_pos[1] - cur_pos[1]
            t_f_y = delta_y/y_vel
            y_accel = -y_vel/t_f_y

        # Z Thrust #
        # desired_z_speed = z_vel_pid.get_effort(target_pos[2], cur_pos[2], dt)
        if vertical_err < no_lateral_alt:
            desired_z_speed = 0.0
            z_accel = z_accel_pid.get_effort(desired_z_speed, z_vel, dt)
        else:
            delta_z = target_pos[2] - cur_pos[2]
            t_f_z = delta_z/z_vel
            z_accel = -z_vel/t_f_z

        # desired_horizontal_speed = np.sqrt(desired_y_speed**2 + desired_z_speed**2)
        # print("desired_horizontal_speed: {}".format(desired_horizontal_speed))

        accel = np.array([vert_accel, -y_accel, -z_accel])

        thrust = mass * accel

        thrust_x, thrust_y, thrust_z = thrust[0], thrust[1], thrust[2]
        # thrust_yz = np.sqrt(thrust_y ** 2 + thrust_z ** 2)
        #         #         # thrust_x = min(thrust_yz, thrust_x)

        angle_from_up = (angle_between_vectors(frame_x, np.array(surface_flight.direction))) / (180. / np.pi)
        command_thrust = thrust_x / np.cos(angle_from_up)
        if ot.vessel.max_thrust == 0.0:
            print("Thrust value 0.")
            break

        throttle_command = command_thrust / ot.vessel.max_thrust

        x_ddot = body.surface_gravity + ot.vessel.max_thrust / mass
        stopping_time = get_stopping_time(x_vel, x_ddot)
        stopping_dist = get_height_at_time(0.5*x_ddot, x_vel, vertical_err, stopping_time)

        debug_dict = {
            "x_vel": x_vel,
            "x_ddot": x_ddot,
            "stopping_time": stopping_time,
            "stopping_dist": stopping_dist,
            "alt": vertical_err
        }

        # db.debug_printing(debug_dict)

        if False:
            ot.vessel.auto_pilot.target_direction = (thrust_x, -thrust_y, -thrust_z)
            ot.vessel.control.throttle = 0.0
            print("Too High")
        else:
            print("Enganging")
            ot.vessel.control.gear = True
            ot.vessel.control.throttle = throttle_command
            ot.vessel.auto_pilot.target_direction = (thrust_x, -thrust_y, -thrust_z)

        time.sleep(dt)
    ot.auto_off()


def get_stopping_time(x_dot, x_ddot):
    return -x_dot / x_ddot


def get_height_at_time(a, b, c, t):
    return a * t * t + b * t + c
