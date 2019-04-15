import time
import krpc
from customPid import PidController
from RendezvousTools import *
from math import e
import numpy as np


class OrbitTools:

    def __init__(self, script_name):

        # Connecting to the game and getting the vessel info
        conn = krpc.connect(name=script_name)
        self.conn = conn
        self.ksc = conn.space_center
        self.vessel = self.ksc.active_vessel

        # Setting up value streams
        self.altitude = conn.add_stream(getattr, self.vessel.flight(), 'mean_altitude')
        self.apoapsis = conn.add_stream(getattr, self.vessel.orbit, 'apoapsis_altitude')
        self.thrust = conn.add_stream(getattr, self.vessel, 'thrust')
        self.mass = conn.add_stream(getattr, self.vessel, 'mass')
        self.vertical_speed = conn.add_stream(
            getattr,
            self.vessel.flight(self.vessel.orbit.body.reference_frame),
            'vertical_speed'
        )

        # Control Parameters
        self.ascent_angle_flatten_alt = 33000
        self.parking_orbit_alt = 85000
        self.close_in_factor = 0.95
        self.inclination = 90
        self.slow_burn_throttle = 0.2

        self.twr_pid = PidController(
            gains=(0.05, 0.001, 0.0001),
            effort_bounds=(-1.0, 1.0),

        )

    def set_launch_params(self):
        body_name = self.vessel.orbit.body.name
        if body_name == "Kerbin":
            self.parking_orbit_alt = 75000
            self.ascent_angle_flatten_alt = 33000
        elif body_name == "Ike":
            self.parking_orbit_alt = 54000
            self.ascent_angle_flatten_alt = 8000
        elif body_name == "Mun":
            self.parking_orbit_alt = 35000
            self.ascent_angle_flatten_alt = 6000
        else:
            self.parking_orbit_alt = 54000
            self.ascent_angle_flatten_alt = 8000

    def get_target(self):
        target = self.ksc.target_vessel
        if target:
            return target
        target = self.ksc.target_body
        if target:
            return target

        return None

    def get_target_orbit(self):
        target = self.get_target()
        if target:
            return target.orbit
        return None

    def create_circularization_node(self, dp, threshold):
        nd = self.vessel.control.add_node(self.ksc.ut + self.vessel.orbit.time_to_apoapsis, 0.0)
        offset = 0.0
        best = 999999999
        while best > threshold:
            nd.prograde = offset - dp
            e1 = nd.orbit.eccentricity
            nd.prograde = offset + dp
            e2 = nd.orbit.eccentricity
            if e1 < e2:
                offset -= dp
                best = e1
            else:
                offset += dp
                best = e2
            dp *= 0.8
        nd.prograde = offset

    def create_period_modification_node(self, dp, threshold, new_period, ut):
        nd = self.vessel.control.add_node(ut, 0.0)
        offset = 0.0
        best = 999999999
        while best > threshold:
            nd.prograde = offset - dp
            e1 = abs(nd.orbit.period - new_period)
            nd.prograde = offset + dp
            e2 = abs(nd.orbit.period - new_period)
            if e1 < e2:
                offset -= dp
                best = e1
            else:
                offset += dp
                best = e2
            dp *= 0.8
        nd.prograde = offset

    # Creates a node that matches your inclination with the target.
    def create_inclination_matching_node(self, dn, threshold):
        target = self.ksc.target_vessel

        # Decide which is closer ascending or descending node.
        anomaly_dn = self.vessel.orbit.true_anomaly_at_dn(target.orbit)
        anomaly_an = self.vessel.orbit.true_anomaly_at_an(target.orbit)
        ut_at_dn = self.vessel.orbit.ut_at_true_anomaly(anomaly_dn)
        ut_at_an = self.vessel.orbit.ut_at_true_anomaly(anomaly_an)
        ut_at_node = min(ut_at_an, ut_at_dn)

        # Create Node
        nd = self.vessel.control.add_node(ut_at_node, 0.0)
        offset = 0.0
        best = 999999999
        while best > threshold:
            nd.normal = offset - dn
            diff1 = nd.orbit.relative_inclination(target.orbit)
            nd.normal = offset + dn
            diff2 = nd.orbit.relative_inclination(target.orbit)
            if diff1 < diff2:
                offset -= dn
                best = diff1
            else:
                offset += dn
                best = diff2
            dn *= 0.8
        nd.normal = offset

    # TODO have this account for partial stages.
    def get_burn_time(self, dv):
        body = self.vessel.orbit.body
        g = body.gravitational_parameter / self.vessel.orbit.radius

        isp = self.vessel.specific_impulse
        m_init = self.vessel.mass
        m_final = m_init * e ** (-dv / (isp * g))
        m_prop = m_init - m_final
        m_dot = self.vessel.max_thrust / (isp * g)
        burn_time = m_prop / m_dot
        return burn_time

    # TODO have this account for partial stages.
    # Executes the next maneuver node.
    #  Variables:
    #  remove_after: Bool indicating if the node should be removed after execution.
    #  buffer_time: Amount of time before node execution starts to stop time warp.
    #  slow_burn_time: How much of the burn is reserved for slow burn.
    # Assumptions:
    #  Relative Inclination near zero
    def execute_next_node(self, buffer_time=20.0, slow_burn_time=0.5):
        nd = self.vessel.control.nodes[0]
        reference_frame = self.vessel.surface_reference_frame
        burn_direction = nd.direction(reference_frame)
        self.vessel.auto_pilot.target_direction = burn_direction

        dt = 1/20.0

        dv = nd.delta_v
        burn_time = self.get_burn_time(dv)
        burn_throttle = 1.0

        small_burn_ratio = 5.0
        if burn_time < 2.0:
            burn_throttle /= small_burn_ratio
            burn_time *= small_burn_ratio

        # Orient ship
        burn_direction = nd.direction(reference_frame)
        ship_direction = self.vessel.direction(reference_frame)
        self.auto_on()
        while not np.dot(burn_direction, ship_direction) >= .98:
            self.vessel.auto_pilot.target_direction = burn_direction
            time.sleep(dt)
            reference_frame = self.vessel.surface_reference_frame
            burn_direction = nd.direction(reference_frame)
            ship_direction = self.vessel.direction(reference_frame)

        self.auto_off()

        print("Expecting {}s burn.".format(burn_time))
        warp_time = nd.ut - (burn_time / 2.0) - buffer_time
        self.ksc.warp_to(warp_time)
        print("Warping")
        print(self.ksc, warp_time)

        self.auto_on()
        reference_frame = self.vessel.surface_reference_frame
        burn_direction = nd.remaining_burn_vector(reference_frame)
        self.vessel.auto_pilot.target_direction = burn_direction

        while self.ksc.ut < nd.ut - (burn_time/2.0):
            time.sleep(dt)

        self.vessel.control.throttle = burn_throttle
        # Perform most of the burn, stopping when half a second is remaining.
        while self.get_burn_time(nd.remaining_delta_v) > slow_burn_time:
            reference_frame = self.vessel.surface_reference_frame
            burn_direction = nd.remaining_burn_vector(reference_frame)
            self.vessel.auto_pilot.target_direction = burn_direction
            if self.vessel.thrust < 1e-6:
                self.stage_if_needed()

        # Start slow burn
        self.vessel.control.throttle = self.slow_burn_throttle
        # Home in on remainder of burn.
        while nd.remaining_delta_v > 0.5:
            burn_direction = nd.remaining_burn_vector(reference_frame)
            self.vessel.auto_pilot.target_direction = burn_direction
        # Throttle Off
        self.vessel.control.throttle = 0.0
        self.auto_off()
        nd.remove()

    def execute_node_with_feedback(self, feedback_function, threshold=0.1,
                                   buffer_time=10.0, slow_burn_time=0.5):
        nd = self.vessel.control.nodes[0]
        burn_direction = nd.direction(self.vessel.surface_reference_frame)
        self.vessel.auto_pilot.target_direction = burn_direction
        self.vessel.auto_pilot.engage()

        dv = nd.delta_v
        burn_time = self.get_burn_time(dv)
        print("Expecting {}s burn.".format(burn_time))
        print("Warping")
        self.ksc.warp_to(nd.ut - (burn_time / 2.0) - buffer_time)
        burn_direction = nd.direction(self.vessel.surface_reference_frame)
        self.vessel.auto_pilot.target_direction = burn_direction
        time.sleep(buffer_time)

        self.vessel.control.throttle = 1.0
        # Perform most of the burn, stopping when half a second is remaining.
        while self.get_burn_time(nd.remaining_delta_v) > slow_burn_time:
            burn_direction = nd.remaining_burn_vector(self.vessel.surface_reference_frame)
            self.vessel.auto_pilot.target_direction = burn_direction

        # Start slow burn
        self.vessel.control.throttle = self.slow_burn_throttle
        # Home in on remainder of burn.
        feedback = feedback_function(self.vessel)
        while feedback > threshold:
            print(feedback)
            time.sleep(0.001)
            feedback = feedback_function(self.vessel)
        # Throttle Off
        self.vessel.control.throttle = 0.0
        nd.remove()

    def stage_if_needed(self, ):
        resources_in_stage = self.vessel.resources_in_decouple_stage(self.vessel.control.current_stage - 1, True)
        fuel_list = filter(lambda item: item.name in ["LiquidFuel", "SolidFuel"], resources_in_stage.all)
        fuel_in_stage = sum(item.amount for item in fuel_list)
        if fuel_in_stage < 0.05:
            print("Staging!")
            self.vessel.control.throttle = 0.0
            self.vessel.control.activate_next_stage()
            time.sleep(0.1)
            self.vessel.control.throttle = 1.0

        time.sleep(0.1)

    def wait_until_drag_is_negligible(self, ):
        while get_magnitude(self.vessel.flight().drag) > 2.5:
            time.sleep(1)

    def get_ascent_angle(self, ):
        altitude_ratio = self.altitude() / self.ascent_angle_flatten_alt
        return max(5, 90 - (90 * altitude_ratio))

    def set_twr(self, twr_target, dt):
        weight = (self.mass() * self.vessel.orbit.body.surface_gravity)
        twr = self.thrust() / weight
        thrust_cmd = twr * weight
        offset_cmd = self.twr_pid.get_effort(twr_target, twr, dt)

        throttle = thrust_cmd / self.vessel.max_thrust

        throttle += offset_cmd
        self.vessel.control.throttle = throttle

    def get_ascent_twr(self):
        initial_twr = 2.0
        initial_speed_trigger = 150.0
        secondary_speed_trigger = 280.0
        low_atmos_twr = 1.6
        upper_atmos_twr = 3.0

        # Note each condition conjuncts with the inverse of the questions before.
        # TODO make this based on air drag
        alt = self.altitude()
        vertical_speed = self.vertical_speed()
        if vertical_speed < initial_speed_trigger:
            return initial_twr
        if vertical_speed < secondary_speed_trigger:
            return initial_twr - (initial_twr - low_atmos_twr) * (
                        vertical_speed - initial_speed_trigger) / secondary_speed_trigger
        if alt < 15000:
            return low_atmos_twr
        if alt > 15000:
            return upper_atmos_twr

    def rendezvous_with_target(self, max_number_of_orbits=10):
        target = self.ksc.target_vessel

        inclination_threshold = 0.05

        # Match inclinations
        if self.vessel.orbit.relative_inclination(target.orbit) > inclination_threshold:
            print("Inclination matching.")
            self.create_inclination_matching_node(1000, 0.0001)
            self.execute_next_node()
        else:
            print("Skipping Inclination matching.")

        assert (self.vessel.orbit.relative_inclination(target.orbit) <= inclination_threshold)

        # Adjust period for rendezvous
        t = self.vessel.orbit.period
        target_t = target.orbit.period
        target_lag = calculate_target_lag(self.vessel, target)

        mu = self.vessel.orbit.body.gravitational_parameter
        new_period, num_orbits = find_optimal_rendevous_period(t, target_t, target_lag, mu,
                                                               range(1, max_number_of_orbits))
        print("{} orbits to intersect.".format(num_orbits))
        print(new_period, " second orbit requested.")
        intersect_ut = get_next_approach(self.vessel, target)

        self.create_period_modification_node(10.0, 0.001, new_period, intersect_ut)
        print("test")
        self.execute_next_node()

        # Warp until a half orbit before the final approach.
        self.wait_until_ut(intersect_ut + num_orbits*new_period - 20.0)
        #
        # self.zero_relative_velocity(target)
        self.auto_off()
        print("Done")

    def wait_until_ut(self, ut):
        self.auto_off()
        self.ksc.warp_to(ut)
        self.auto_on()

    def zero_relative_velocity(self, target, buffer_time=5.0):
        frame = self.vessel.surface_reference_frame

        initial_threshold = 10.0
        medium_threshold = 2.0
        final_threshold = 0.1
        relative_v = target.velocity(frame)

        self.vessel.auto_pilot.target_direction = relative_v
        self.auto_on()
        time.sleep(buffer_time)
        self.vessel.control.throttle = 1.0
        while get_magnitude(relative_v) > initial_threshold:
            relative_v = target.velocity(frame)
            self.vessel.auto_pilot.target_direction = relative_v

        # Start slow burn
        self.vessel.control.throttle = self.slow_burn_throttle
        while get_magnitude(relative_v) > medium_threshold:
            relative_v = target.velocity(frame)
            self.vessel.auto_pilot.target_direction = relative_v

        # Throttle Off
        self.vessel.control.throttle = 0.0

    def burn_towards_target(self, target, vel):
        frame = self.vessel.surface_reference_frame

        relative_pos = target.position(frame)

        self.vessel.auto_pilot.target_direction = relative_pos
        self.vessel.auto_pilot.engage()
        self.vessel.auto_pilot.wait()

        burn_time = self.get_burn_time(vel)
        self.vessel.control.throttle = 1.0
        time.sleep(burn_time)
        self.vessel.control.throttle = 0.0

    def auto_off(self):
        self.vessel.auto_pilot.disengage()
        self.vessel.control.sas = True
        self.vessel.control.throttle = 0.0

    def auto_on(self):
        self.vessel.control.sas = False
        self.vessel.auto_pilot.engage()
