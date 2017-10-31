import time
import krpc
from RendezvousTools import *
from math import e


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

        # Control Parameters
        self.ascent_angle_flatten_alt = 41000
        self.parking_orbit_alt = 80000
        self.close_in_factor = 0.95
        self.inclination = 90

    def reset_control_parameters_to_default(self):
        # Constants
        self.ascent_angle_flatten_alt = 41000
        self.parking_orbit_alt = 80000
        self.close_in_factor = 0.95
        self.inclination = 90

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

    def create_period_modification_node(self, dp, threshold, new_period, anomaly):
        ut = self.vessel.orbit.ut_at_true_anomaly(anomaly)
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
        anomaly_dn = self.vessel.orbit.true_anomaly_at_dn(target)
        anomaly_an = self.vessel.orbit.true_anomaly_at_an(target)
        ut_at_dn = self.vessel.orbit.ut_at_true_anomaly(anomaly_dn)
        ut_at_an = self.vessel.orbit.ut_at_true_anomaly(anomaly_an)
        ut_at_node = min(ut_at_an, ut_at_dn)

        # Create Node
        nd = self.vessel.control.add_node(ut_at_node, 0.0)
        offset = 0.0
        best = 999999999
        while best > threshold:
            nd.normal = offset - dn
            diff1 = nd.orbit.relative_inclination(target)
            nd.normal = offset + dn
            diff2 = nd.orbit.relative_inclination(target)
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
    def execute_next_node(self, remove_after=True, buffer_time=10.0, slow_burn_time=0.5):
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
        self.vessel.control.throttle = 0.1
        # Home in on remainder of burn.
        while nd.remaining_delta_v > 0.5:
            burn_direction = nd.remaining_burn_vector(self.vessel.surface_reference_frame)
            self.vessel.auto_pilot.target_direction = burn_direction
        # Throttle Off
        self.vessel.control.throttle = 0.0

        if remove_after:
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
        while get_magnitude(self.vessel.flight().drag) > 0.005:
            time.sleep(1)

    def get_ascent_angle(self, ):
        altitude_ratio = self.altitude() / self.ascent_angle_flatten_alt
        return max(5, 90 - (90 * altitude_ratio))

    def rendezvous_with_target(self, max_number_of_orbits=10):
        target = self.ksc.target_vessel

        # Match inclinations
        self.create_inclination_matching_node(1000, 0.0001)
        self.execute_next_node()

        # Adjust period for rendezvous
        t = self.vessel.orbit.period
        target_t = target.orbit.period
        target_lag = calculate_target_lag(self.vessel, target)

        mu = self.vessel.orbit.body.gravitational_parameter
        new_period = find_optimal_rendevous_period(t, target_t, target_lag, mu, range(1, max_number_of_orbits))
        intersect_ut = get_next_approach(self.vessel, target)
        intersect_anomaly = self.vessel.orbit.true_anomaly_at_ut(intersect_ut)

        self.create_period_modification_node(1000, 0.001, new_period, intersect_anomaly)
        self.execute_next_node()

    def zero_relative_velocity(self, target, buffer_time=5.0):
        frame = self.vessel.surface_reference_frame

        initial_threshold = 10.0
        medium_threshold = 2.0
        final_threshold = 0.1
        relative_v = target.velocity(frame)

        self.vessel.auto_pilot.target_direction = relative_v
        self.vessel.auto_pilot.engage()
        time.sleep(buffer_time)
        self.vessel.control.throttle = 1.0
        while get_magnitude(relative_v) > initial_threshold:
            relative_v = target.velocity(frame)
            self.vessel.auto_pilot.target_direction = relative_v

        # Start medium burn
        self.vessel.control.throttle = 0.1
        while get_magnitude(relative_v) > medium_threshold:
            relative_v = target.velocity(frame)
            self.vessel.auto_pilot.target_direction = relative_v

        # Start slow burn
        self.vessel.control.throttle = 0.01
        while get_magnitude(relative_v) > final_threshold:
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


