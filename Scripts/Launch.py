import math
import sys
import time

from Scripts.LandingTools import landing_script
from OrbitTools import *
from DockingTools import docking_assist
from Misc import mining_operations
from ManeuverTools import plan_maneuver
import CustomErrors as errors

e = math.e


def main():
    ot = OrbitTools("Launch Script")

    print(sys.argv[1])
    _ = str(input("Press Enter to Continue"))

    try:
        if sys.argv[1] == "launch":
            launch(ot)
        elif sys.argv[1] == "execute_node":
            execute_node(ot)
        elif sys.argv[1] == "circularize":
            circularize(ot)
        elif sys.argv[1] == "rendezvous":
            rendezvous(ot)
        elif sys.argv[1] == "docking_assist":
            docking_assist(ot)
        elif sys.argv[1] == "mining_operations":
            mining_operations(ot)
        elif sys.argv[1] == "landing_script":
            landing_script(ot)
        elif sys.argv[1] == "planner_test":
            plan_maneuver(ot)
        else:
            print("invalid command")

    except errors.AssumptionViolation as error:
        print("Assumption was violated: {}".format(error.msg))


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
    ot.vessel.control.throttle = 1.0

    ot.set_launch_params()

    countdown()

    # Launch!
    ot.vessel.control.activate_next_stage()
    ot.vessel.auto_pilot.engage()
    ot.vessel.auto_pilot.target_pitch_and_heading(90, ot.inclination)

    Hz = 30.0
    dt = 1 / Hz

    # Ascent
    while ot.apoapsis() < ot.parking_orbit_alt:
        ot.vessel.auto_pilot.target_pitch_and_heading(ot.get_ascent_angle(), ot.inclination)
        ot.stage_if_needed()

        desired_twr = ot.get_ascent_twr()
        ot.set_twr(desired_twr, dt)
        time.sleep(dt)

    ot.auto_off()

    # Coast through atmosphere
    print("Coasting")
    ot.vessel.control.throttle = 0.0
    ot.wait_until_drag_is_negligible()

    ot.auto_on()

    ot.create_circularization_node(100, 0.001)
    ot.execute_next_node()

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
