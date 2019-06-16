import datetime
import time
from collections import deque

import numpy as np
import pandas as pd

from Scripts.OrbitTools import OrbitTools
from Scripts.customPid import PidController
from Scripts.Misc import angle_between_vectors, norm, DEGREES

TORQUE_FIELD_STR = 'Torque Limit(%)'

VAB_POS = (-0.09681, -74.6174)
LAUNCH_POS = (-0.09718, -74.5577)
WATER_TOWER_POS = (-0.09210, -74.55251)
TRACKING_STATION_POS = (-0.12723, -74.60533)


def main():
    ot = OrbitTools("Copter")
    copter = Copter(ot)
    copter.run_rotors_calibration_routine(land_after=False)
    location_proximity = 0.1

    copter.update_setpoint(
        (120,) + WATER_TOWER_POS,
        (0.0, 0.0, 0.0),
        heading=0.0
    )
    error = np.inf
    while error > location_proximity:
        copter.track_set_point()
        error = np.linalg.norm(copter.tracking_error)

    print("Arrived at water tower: {}".format(len(copter.history)))
    # TODO: Have option in update_setpoint to interpolate to the new setpoint
    # Or make new function that tracks between two setpoints.

    copter.update_setpoint(
        (120,) + LAUNCH_POS,
        (0.0, 0.0, 0.0),
        heading=0.0
    )

    error = np.inf
    while error > location_proximity:
        copter.track_set_point()
        error = np.linalg.norm(copter.tracking_error)

    print("Arrived at VAB: {}".format(len(copter.history)))

    copter.landing_sequence()
    copter.write_history()


def sigmoid(x):
    return 1. / (1. + np.exp(-x))


def get_part_name_tag(part):
    for module in part.modules:
        if module.name == "KOSNameTag":
            return module.get_field('name tag')


def is_stable(history):
    for record in history:
        if abs(record["vs"]) > 1e-1:
            return False
    return True


class DebugDisplay:
    def __init__(self, ot, format_str):
        # UI
        self.canvas = ot.conn.ui.stock_canvas
        self.display_panel = self.canvas.add_panel()
        ss = self.canvas.rect_transform.size
        # TODO: modularize size/position info
        self.display_panel.rect_transform.size = (280, 50)
        display_pos = (100.0 - (ss[0] / 2.0), (ss[1] / 2.0) - 60.0)
        self.display_panel.rect_transform.position = display_pos
        self.display_format_str = format_str
        self.display_text = self.display_panel.add_text("")

    def __del__(self):
        self.display_panel.remove()

    def update(self, data):
        display_str = self.display_format_str.format(*data)
        self.display_text.content = display_str


class Rotor:
    def __init__(self, part):
        self.part = part
        self.rotor_module = None

        # Set rotor module
        for module in part.modules:
            if 'rotor' in module.name.lower():
                self.rotor_module = module
                break
        assert (self.rotor_module is not None)

    def set_torque(self, torque):
        self.rotor_module.set_field_float(TORQUE_FIELD_STR, torque)

    def get_counter_torque_direction(self):
        # Return 1 if cw, -1 if ccw
        return 1 if self.rotor_module.fields["Rotation Direction"] == "True" else -1


class Copter:

    def __init__(self, ot, freq=30.0):
        self.ot = ot
        self.vessel = ot.vessel

        self.debug_display = DebugDisplay(ot, "({:0.2f}, {:0.2f}, {:0.2f})")

        # Telemetry and Control history
        self.history = []
        self.tick_history_frame()

        # Coordinate frame setup
        self.surface_frame = None
        self.surface_flight = None
        self.body = None
        self.body_flight = None
        self._init_frames()

        # State variables
        self.altitude = None
        self.radar_alt = None
        self.position_g = None
        self.velocity_g = None
        self.gravity = None
        self.roll = None
        self.pitch = None
        self.yaw = None
        self.J = None
        self.situation = None
        self.update_state()

        # Target variables (Global Frame)
        self.pos_setpoint = (0.0, 0.0, 0.0)
        self.vel_setpoint = (0.0, 0.0, 0.0)
        self.heading_setpoint = 0.0
        self.tracking_error = None

        # Find and initialize rotors
        self.rotors = []
        self.num_rotors = None
        self._assign_rotors()

        # Used to generate the moments in the body frame (m_b = M @ T)
        self.moment_conversion_matrix = None

        # Controller variables
        self.control_freq = self.dt = None
        self.set_update_freq(freq)  # Hz
        # self.max_tip_angle = 0.05
        self.max_tip_angle = 0.1

        self.vert_vel_pid = None
        self.vert_accel_pid = None
        self.y_vel_pid = None
        self.z_vel_pid = None
        self.y_accel_pid = None
        self.z_accel_pid = None
        self.roll_rate_pid = None
        self.pitch_rate_pid = None
        self.yaw_rate_pid = None
        self.vert_cal_pid = None
        self.direct_torque_pid = None
        self.landing_descent_speed = -0.5
        self.max_landing_descent_speed = -4.0
        self.init_controllers()

        # Calibration Data
        self.is_calibrated = False
        self.rotor_k_m = None
        self.thrust_per_torque = None
        self.altitude_offset = None

    # TODO: Extract telemetry tracking as a class
    def write_history(self):
        df = pd.DataFrame(self.history)
        date_str = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        file_path = "../Telemetry/{}.csv".format(date_str)
        df.to_csv(open(file_path, mode="w+"))

    def record_item(self, name, value):
        self.history[-1][name] = value

    def tick_history_frame(self):
        self.history.append({})

    def init_controllers(self):
        # Vertical Speed
        vs_gains = (0.8, 0.1, 0.2)
        vs_bounds = (-5.0, 5.0)
        self.vert_vel_pid = PidController(
            gains=vs_gains,
            effort_bounds=vs_bounds,
            integral_threshold=3.0
        )

        # Vertical Acceleration
        vert_accel_gains = (1.8, 0.0, 0.0001)
        vert_accel_bounds = (-2.0, 2.0)
        self.vert_accel_pid = PidController(
            gains=vert_accel_gains,
            effort_bounds=vert_accel_bounds
        )

        # Lateral Velocity
        lateral_vel_gains = (0.8, 0.0, 0.4)
        lateral_vel_bounds = (-10, 10)
        self.y_vel_pid = PidController(
            gains=lateral_vel_gains,
            effort_bounds=lateral_vel_bounds
        )
        self.z_vel_pid = PidController(
            gains=lateral_vel_gains,
            effort_bounds=lateral_vel_bounds
        )

        # Lateral Acceleration
        lateral_accel_gains = (0.8, 0.0, 0.001)
        lateral_accel_bounds = (-10, 10)
        self.y_accel_pid = PidController(
            gains=lateral_accel_gains,
            effort_bounds=lateral_accel_bounds
        )
        self.z_accel_pid = PidController(
            gains=lateral_accel_gains,
            effort_bounds=lateral_accel_bounds
        )

        # Roll and Pitch Rates
        roll_pitch_rate_gains = (1.8, 0.0, 1.0)
        roll_pitch_rate_bounds = (-10.0, 10.0)
        self.roll_rate_pid = PidController(
            gains=roll_pitch_rate_gains,
            effort_bounds=roll_pitch_rate_bounds
        )
        self.pitch_rate_pid = PidController(
            gains=roll_pitch_rate_gains,
            effort_bounds=roll_pitch_rate_bounds
        )

        yaw_gains = (1.0, 0.0, 0.2)
        yaw_bounds = (-1.0, 1.0)
        self.yaw_rate_pid = PidController(
            gains=yaw_gains,
            effort_bounds=yaw_bounds
        )

        direct_torque_gains = (3.0, 0.01, 0.5)
        self.direct_torque_pid = PidController(
            gains=direct_torque_gains,
            effort_bounds=(0, 100),
            integral_threshold=10.0
        )
        vert_cal_gains = (2.0, 0.5, 0.2)
        self.vert_cal_pid = PidController(
            gains=vert_cal_gains,
            effort_bounds=(-30, 30),
            integral_threshold=5.0
        )

    def update_display(self):
        # self.debug_display.update(omega_b)
        pass

    def _assign_rotors(self):
        assert (self.rotors == [])
        for part in self.vessel.parts.all:
            if "rotor" in part.name.lower():
                self.rotors.append(Rotor(part))
        self.num_rotors = len(self.rotors)

    def _compute_moment_conversion_matrix(self):
        # Body Frame
        com_frame = self.vessel.reference_frame

        # Pitch and Roll Moment Values
        x_positions = np.zeros(self.num_rotors)
        z_positions = np.zeros(self.num_rotors)
        for index, rotor in enumerate(self.rotors):
            x, _, z = np.array(rotor.part.position(com_frame))
            x_positions[index] = x
            z_positions[index] = z

        # Yaw Moment Values
        yaw_signs = [rotor.get_counter_torque_direction() for rotor in self.rotors]
        k_m_signed = np.array(yaw_signs) * self.rotor_k_m

        self.moment_conversion_matrix = np.array([
            z_positions,
            k_m_signed,
            -x_positions
        ])

    def _init_frames(self):
        self.surface_frame = self.vessel.surface_reference_frame
        self.surface_flight = self.vessel.flight(self.surface_frame)

        # NOTE: This is not the vehicle body frame this is the celestial body
        self.body = self.vessel.orbit.body
        self.body_flight = self.vessel.flight(self.body.reference_frame)

    def control_update(self):
        # Derived from slide 22 of https://bit.ly/2HPIFHl

        self.record_item("x_vel", self.velocity_g[0])
        self.record_item("y_vel", self.velocity_g[1])
        self.record_item("z_vel", self.velocity_g[2])
        self.record_item("altitude", self.altitude)
        self.record_item("target_alt", self.pos_setpoint[0])

        total_thrust = self._vertical_control_helper()

        # Compute desired body Moments
        M_b = self._horizontal_control_helper(total_thrust)

        # Compute rotor values need for desired moment
        A = np.vstack([
            self.moment_conversion_matrix,
            np.ones((1, self.num_rotors))
        ])
        b = np.vstack([M_b, total_thrust])

        rotor_thrusts = np.linalg.lstsq(A, b)[0]

        # Conversion is computed and stored in self.thrust_per_torque, doesn't work well
        rotor_torques = rotor_thrusts/self.thrust_per_torque
        self.set_torques(rotor_torques)
        self._update_error_tracking()
        self.tick_history_frame()

    def _vertical_control_helper(self):
        # Compute and return vertical thrust (Global Frame)
        target_alt = self.pos_setpoint[0]
        vs_cmd = self.vert_vel_pid.get_effort(target_alt, self.altitude, self.dt)

        vs_desired = vs_cmd + self.vel_setpoint[0]
        vs = self.velocity_g[0]
        vert_accel_cmd = self.vert_accel_pid.get_effort(vs_desired, vs, self.dt)
        self.record_item("vert_accel_cmd", vert_accel_cmd)
        self.record_item("vs_desired", vs_desired)

        desired_vert_accel = vert_accel_cmd + self.gravity

        frame_x = np.array([1.0, 0.0, 0.0])
        angle_from_up = angle_between_vectors(
            frame_x,
            np.array(self.surface_flight.direction),
            convert2deg=False
        )

        vertical_thrust = self.vessel.mass * desired_vert_accel
        desired_thrust = vertical_thrust / np.cos(angle_from_up)
        self.record_item("desired_thrust", desired_thrust)
        self.record_item("vertical_thrust", vertical_thrust)
        return desired_thrust

    def _horizontal_control_helper(self, total_thrust):
        _, lat_target, lon_target = self.pos_setpoint
        _, y_vel_target, z_vel_target = self.vel_setpoint
        _, lat, lon = self.position_g

        # Convert to meters
        y_target, z_target = self.convert_lat_lon_to_meters(lat_target, lon_target)
        y_pos, z_pos = self.convert_lat_lon_to_meters(lat, lon)

        self.record_item("y_pos", y_pos)
        self.record_item("z_pos", z_pos)
        self.record_item("y_pos_target", y_target)
        self.record_item("z_pos_target", z_target)

        # Compute desired velocity
        y_vel_cmd = self.y_vel_pid.get_effort(y_target, y_pos, self.dt)
        z_vel_cmd = self.z_vel_pid.get_effort(z_target, z_pos, self.dt)
        y_vel_desired = y_vel_cmd + y_vel_target
        z_vel_desired = z_vel_cmd + z_vel_target

        self.record_item("y_vel_desired", y_vel_desired)
        self.record_item("z_vel_desired", z_vel_desired)

        # Compute desired acceleration
        x_vel, y_vel, z_vel = self.velocity_g
        y_accel_desired = self.y_accel_pid.get_effort(y_vel_desired, y_vel, self.dt)
        z_accel_desired = self.z_accel_pid.get_effort(z_vel_desired, z_vel, self.dt)

        # Compute desired Thrust vector
        T_y_g = self.vessel.mass * y_accel_desired
        T_z_g = self.vessel.mass * z_accel_desired
        T_yz_squared = T_y_g**2 + T_z_g**2
        T_x_g = np.sqrt(abs(total_thrust**2 - T_yz_squared))

        # Ensure the copter doesn't tip too far
        T_yz = np.sqrt(T_yz_squared)
        tip_angle = np.arctan(T_yz/T_x_g)

        if tip_angle > self.max_tip_angle:
            scale_factor = np.tan(self.max_tip_angle)*T_x_g / T_yz
            T_y_g *= scale_factor
            T_z_g *= scale_factor
        T_g = np.array([T_x_g, T_y_g, T_z_g])
        self.record_item("T_x_g", T_x_g)
        self.record_item("T_y_g", T_y_g)
        self.record_item("T_z_g", T_z_g)

        # Compute velocity in body frame
        c_roll, s_roll = np.cos(self.roll), np.sin(self.roll)
        c_pitch, s_pitch = np.cos(self.pitch), np.sin(self.pitch)
        c_yaw, s_yaw = np.cos(self.yaw), np.sin(self.yaw)

        # Compute yaw desired
        yaw_desired = self.heading_setpoint

        # Compute pitch desired
        rotation_g2yaw = np.array([
            [1.0,    0.0,   0.0],
            [0.0,  c_yaw, s_yaw],
            [0.0, -s_yaw, c_yaw]
        ])
        T_yaw = rotation_g2yaw @ T_g
        T_yaw_x, T_yaw_y = T_yaw[0], T_yaw[1]
        xy = np.sqrt(T_yaw_x**2 + T_yaw_y**2)
        pitch_desired = -np.arcsin(T_yaw_y/xy)

        # Compute Roll desired
        c_pitch_desired = np.cos(pitch_desired)
        s_pitch_desired = np.sin(pitch_desired)
        rotation_yaw2pitch = np.array([
            [c_pitch_desired,  s_pitch_desired, 0.0],
            [-s_pitch_desired, c_pitch_desired, 0.0],
            [0.0,              0.0,             1.0]
        ])
        T_pitch = rotation_yaw2pitch @ T_yaw
        T_pitch_x, T_pitch_z = T_pitch[0], T_pitch[2]
        xz = np.sqrt(T_pitch_x**2 + T_pitch_z**2)
        roll_desired = np.arcsin(T_pitch_z/xz)

        # Wrap yaw angle to closest direction
        if yaw_desired - self.yaw > np.pi:
            yaw_desired -= 2.0*np.pi
        elif yaw_desired - self.yaw < -np.pi:
            yaw_desired += 2.0*np.pi

        self.record_item("roll", self.roll)
        self.record_item("pitch", self.pitch)
        self.record_item("yaw", self.yaw)

        self.record_item("roll_desired", roll_desired)
        self.record_item("pitch_desired", pitch_desired)
        self.record_item("yaw_desired", yaw_desired)

        # Compute desired pitch/roll rates to meet desired pitch/roll
        roll_rate = -self.roll_rate_pid.get_effort(roll_desired, self.roll, self.dt)
        pitch_rate = -self.pitch_rate_pid.get_effort(pitch_desired, self.pitch, self.dt)
        yaw_rate = self.yaw_rate_pid.get_effort(yaw_desired, self.yaw, self.dt)
        rpy_dot = np.array([pitch_rate, yaw_rate, roll_rate])

        # Compute desired rotation rates in body frame
        tan_pitch = np.tan(self.pitch)
        sec_pitch = 1.0 / np.cos(self.pitch)
        rpy2pqr_transform = np.array([
            [1.0, s_roll*tan_pitch, c_roll*tan_pitch],
            [0.0,           c_roll,          -s_roll],
            [0.0, s_roll*sec_pitch, c_roll*sec_pitch]
        ])
        p_dot, q_dot, r_dot = rpy2pqr_transform.T @ rpy_dot

        # Compute rotation rates in body frame
        omega_g = self.vessel.angular_velocity(self.body.reference_frame)
        omega_b = self.ot.ksc.transform_direction(
            omega_g,
            self.body.reference_frame,
            self.vessel.reference_frame
        ) * np.array([-1, 1, -1])
        p, q, r = omega_b

        # Compute desired moment in body frame
        J_x, J_y, J_z = self.J
        M_x = -(p_dot - (J_y - J_z)*(q*r)/J_x)*J_x
        M_y = -(q_dot - (J_z - J_x)*(p*r)/J_y)*J_y
        M_z = -(r_dot - (J_x - J_y)*(p*q)/J_z)*J_z
        desired_moment_body = np.array([M_x, M_y, M_z]).reshape(-1, 1)

        self.record_item("M_x", M_x)
        self.record_item("M_y", M_y)
        self.record_item("M_z", M_z)

        self.debug_display.update(desired_moment_body.reshape(-1))

        return desired_moment_body

    def update_state(self):
        self._update_location()
        self._update_velocity()
        self._update_orientation()

        # Misc updates
        mu = self.body.gravitational_parameter
        r = self.body.equatorial_radius + self.altitude
        self.gravity = mu / r ** 2

        self.J = self.vessel.moment_of_inertia

        self.situation = self.vessel.situation

        # lat, lon, alt = self.position_g
        # self.debug_display.update((lat, lon, alt))

        # self.update_display()

    def _update_location(self):
        self.altitude = self.surface_flight.mean_altitude
        self.radar_alt = self.surface_flight.surface_altitude
        latitude = self.surface_flight.latitude
        longitude = self.surface_flight.longitude
        self.position_g = self.altitude, latitude, longitude

    def _update_velocity(self):
        vel = self.body_flight.velocity
        self.velocity_g = self.ot.ksc.transform_direction(
            vel,
            self.body.reference_frame,
            self.vessel.surface_reference_frame
        )

    def _update_orientation(self):
        x, y, z = np.eye(3)

        # Compute Yaw
        body_z_g = np.array(
            self.ot.ksc.transform_direction(
                z,
                self.vessel.reference_frame,
                self.vessel.surface_reference_frame
            )
        )
        lateral_projection = body_z_g.copy()
        lateral_projection[0] = 0  # Remove x term to project on yz plane
        self.yaw = np.arctan2(body_z_g[2], body_z_g[1])

        # Compute Pitch
        lateral_projection_norm = norm(lateral_projection)
        self.pitch = np.arctan2(body_z_g[0], lateral_projection_norm)

        # Compute Roll
        body_x_g = np.array(
            self.ot.ksc.transform_direction(
                x,
                self.vessel.reference_frame,
                self.vessel.surface_reference_frame
            )
        )

        c_yaw = np.cos(self.yaw)
        s_yaw = np.sin(self.yaw)
        rotation_yaw = np.array([
            [1.0, 0.0, 0.0],
            [0.0, c_yaw, s_yaw],
            [0.0, -s_yaw, c_yaw]
        ])
        pre_roll_body_x_g = rotation_yaw.T @ x
        self.roll = angle_between_vectors(
            body_x_g,
            pre_roll_body_x_g,
            convert2deg=False
        ) - np.pi/2.0

    def set_torques(self, torques):
        largest_value = np.max(torques)
        if largest_value > 100.0:
            overshoot = largest_value - 100.0
            torques -= overshoot
        for index in range(self.num_rotors):
            self.rotors[index].set_torque(float(torques[index]))

    def simple_hover(self, target_alt):
        # Hovers at the target altitude
        start_time = current_time = time.time()
        hover_time = current_time - start_time
        while hover_time < 10000.0:
            self.update_state()
            vs = self.velocity_g[0]
            vs_cmd = self.vert_vel_pid.get_effort(target_alt, self.altitude, self.dt)
            torque_cmd = self.direct_torque_pid.get_effort(vs_cmd, vs, self.dt)

            self.set_torques(np.array([torque_cmd] * self.num_rotors, np.float))
            time.sleep(self.dt)
            current_time = time.time()
            hover_time = current_time - start_time

    def run_rotors_calibration_routine(self, land_after=True):
        # Spin up motors to hover above current alt
        calibration_alt = 5.0
        stable_time = 2.0
        history_size = int(stable_time / self.dt)
        hover_alt = self.altitude + calibration_alt
        hover_calibration_complete = False
        hover_prop_torque = 0.0
        history = deque(maxlen=history_size)
        # TODO: Implement logging system
        print("Calibration Starting")
        while not hover_calibration_complete:
            self.update_state()
            vs = self.velocity_g[0]
            vs_cmd = self.vert_cal_pid.get_effort(hover_alt, self.altitude, self.dt)
            torque_cmd = self.direct_torque_pid.get_effort(vs_cmd, vs, self.dt)
            self.set_torques(np.array([torque_cmd] * self.num_rotors, np.float))
            history.append({"vs": vs, "rotor_torque": torque_cmd})

            if self.situation in [self.ot.ksc.VesselSituation.landed, self.ot.ksc.VesselSituation.pre_launch]:
                # Empty history so we don't calibrate before we leave the ground
                history = deque(maxlen=history_size)
            elif len(history) == history_size and is_stable(history):
                hover_prop_torque = np.mean([record['rotor_torque'] for record in history])
                self.altitude_offset = hover_alt - self.altitude
                hover_calibration_complete = True

            time.sleep(self.dt)

        print("Hover achieved!!!")

        # Use hover torque values compute torque -> thrust
        thrust_to_hover = self.gravity*self.vessel.mass
        thrust_per_rotor = thrust_to_hover / self.num_rotors
        self.thrust_per_torque = thrust_per_rotor / hover_prop_torque

        # TODO:
        # Spin up/down cw/ccw rotors measure yaw rate compute k_m
        self.rotor_k_m = -1.0

        if land_after:
            self.landing_sequence()

        self._compute_moment_conversion_matrix()

        self.is_calibrated = True

    def landing_sequence(self, ease_time=10.0):
        print("Landing sequence started.")
        landing_start = time.time()
        while self.situation != self.ot.ksc.VesselSituation.landed:
            self.update_state()
            time_elapsed = time.time()-landing_start
            ease_in_factor = sigmoid(0.5*(time_elapsed-ease_time))
            vs_cmd = self.landing_descent_speed - self.radar_alt*0.2
            # Values are negative so max instead of min
            vs_cmd = max(vs_cmd*ease_in_factor, self.max_landing_descent_speed)
            alt_diff = vs_cmd * self.dt
            new_pos = (self.pos_setpoint[0] + alt_diff,) + self.pos_setpoint[1:]
            self.update_setpoint(
                new_pos,
                (vs_cmd, 0.0, 0.0),
                self.heading_setpoint
            )

            self.control_update()
            time.sleep(self.dt)
        self.set_torques(np.array([0.0]*self.num_rotors))
        print("Landing Complete!!!")

    def set_update_freq(self, freq):
        self.control_freq = freq
        self.dt = 1.0 / freq

    def update_setpoint(self, pos, vel, heading=None):
        self.pos_setpoint = pos
        self.vel_setpoint = vel
        if heading is None:
            heading = self.yaw
        self.heading_setpoint = heading

    def track_set_point(self):
        self.update_state()
        self.control_update()
        time.sleep(self.dt)

    def _update_error_tracking(self):
        altitude_error = self.vert_vel_pid.e_prev
        y_pos_error = self.y_vel_pid.e_prev
        z_pos_error = self.z_vel_pid.e_prev

        # Convert to meters
        self.tracking_error = np.array([
            altitude_error, y_pos_error, z_pos_error
        ])

    # Naive Implementation, should work near KSC/equator
    def convert_lat_lon_to_meters(self, lat, lon):
        r = self.body.equatorial_radius
        latitude_r = r * np.cos(lat*DEGREES)
        y = lat*DEGREES*r
        z = lon*DEGREES*latitude_r
        return y, z


if __name__ == '__main__':
    main()
