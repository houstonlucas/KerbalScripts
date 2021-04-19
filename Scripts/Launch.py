import math
import sys
import time

from Scripts.LandingTools import landing_script
from Scripts.OrbitTools import *
from Scripts.DockingTools import docking_assist
from Scripts.MiningTools import mining_operations
from Scripts.ManeuverTools import plan_maneuver
import Scripts.CustomErrors as errors
from Scripts.PlaneAutoPilot import plane_autopilot

e = math.e


def main():
    program_cmd = sys.argv[1]
    print(program_cmd)

    ot = OrbitTools(program_cmd)
    _ = str(input("Press Enter to Continue"))

    try:
        if program_cmd == "launch":
            launch(ot)
        elif program_cmd == "execute_node":
            execute_node(ot)
        elif program_cmd == "circularize":
            circularize(ot)
        elif program_cmd == "rendezvous":
            rendezvous(ot)
        elif program_cmd == "docking_assist":
            docking_assist(ot)
        elif program_cmd == "mining_operations":
            mining_operations(ot)
        elif program_cmd == "landing_script":
            landing_script(ot)
        elif program_cmd == "planner_test":
            plan_maneuver(ot)
        elif program_cmd == "level_flight":
            plane_autopilot(ot)
        elif program_cmd == "test":
            test(ot)
        else:
            print("invalid command")

    except errors.AssumptionViolation as error:
        print("Assumption was violated: {}".format(error.msg))


def test(ot):
    zero = (0, 0, 0)
    one = (1, 0, 0)
    frame = ot.vessel.reference_frame
    ot.conn.drawing.add_line(zero, one, frame)
    time.sleep(3)


def rendezvous(ot):
    ot.rendezvous_with_target(max_number_of_orbits=8)


def circularize(ot):
    ot.create_circularization_node(100, 0.01)
    ot.execute_next_node()


def execute_node(ot):
    ot.execute_next_node()


def launch(ot):
    # Pre-launch operations
    ot.vessel.control.sas = False
    ot.vessel.control.rcs = False

    ot.set_launch_params()

    countdown()

    # Launch!
    ot.vessel.control.throttle = 1.0
    ot.vessel.control.activate_next_stage()
    ot.vessel.auto_pilot.engage()
    ot.vessel.auto_pilot.target_pitch_and_heading(90, ot.inclination)
    Hz = 30.0
    dt = 1 / Hz

    # Ascent
    while ot.apoapsis() < ot.parking_orbit_alt:
        ot.vessel.auto_pilot.target_pitch_and_heading(ot.get_ascent_angle(), ot.inclination)
        ot.stage_if_needed()

        if ot.apoapsis() > ot.close_in_factor*ot.parking_orbit_alt:
            ot.vessel.control.throttle = ot.slow_burn_throttle
        # desired_twr = ot.get_ascent_twr()
        # ot.set_twr(desired_twr, dt)

        time.sleep(dt)

    ot.auto_off()

    # Coast through atmosphere
    print("Coasting")
    ot.vessel.control.throttle = 0.0
    if ot.has_atmos:
        ot.wait_until_drag_is_negligible()

    ot.auto_on()
    circularize(ot)
    ot.auto_off()


def countdown():
    print('3...')
    time.sleep(1)
    print('2...')
    time.sleep(1)
    print('1...')
    time.sleep(1)
    print('Launch!')


if __name__ == '__main__':
    main()
