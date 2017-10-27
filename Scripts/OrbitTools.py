import time
import math
import krpc

e = math.e


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

    def execute_next_node(self, remove_after=True):
        nd = self.vessel.control.nodes[0]
        burn_direction = nd.direction(self.vessel.surface_reference_frame)
        self.vessel.auto_pilot.target_direction = burn_direction
        self.vessel.auto_pilot.engage()
        # vessel.auto_pilot.wait()
        dv = nd.delta_v
        burn_time = self.get_burn_time(dv)
        print("Expecting {}s burn.".format(burn_time))
        print("Warping")
        self.ksc.warp_to(nd.ut - burn_time / 2.0)
        burn_direction = nd.direction(self.vessel.surface_reference_frame)
        self.vessel.auto_pilot.target_direction = burn_direction
        self.vessel.control.throttle = 1.0

        # Perform most of the burn
        while self.get_burn_time(nd.remaining_delta_v) > 1.0:
            time.sleep(0.2)
            burn_direction = nd.remaining_burn_vector(self.vessel.surface_reference_frame)
            self.vessel.auto_pilot.target_direction = burn_direction

        # Throttle down
        self.vessel.control.throttle = 0.1
        # Home in on remainder of burn.
        while nd.remaining_delta_v > 0.5:
            burn_direction = nd.remaining_burn_vector(self.vessel.surface_reference_frame)
            self.vessel.auto_pilot.target_direction = burn_direction
            time.sleep(0.01)
        # Throttle Off
        self.vessel.control.throttle = 0.0

        if remove_after:
            nd.remove()

    def stage_if_needed(self, ):
        resources_in_stage = self.vessel.resources_in_decouple_stage(self.vessel.control.current_stage-1, True)
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
        while get_magnitude(self.vessel.flight().drag) > 0.05:
            time.sleep(1)

    def get_ascent_angle(self, ):
        altitude_ratio = self.altitude() / self.ascent_angle_flatten_alt
        return max(5, 90 - (90 * altitude_ratio))


def get_magnitude(x):
    return math.sqrt(sum([val * val for val in x]))
