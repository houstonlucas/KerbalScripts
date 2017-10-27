from OrbitTools import *

e = math.e

# TODO use this for better burn time calculation


def main():
    ot = OrbitTools("Launch Script")

    # Pre-launch operations
    ot.vessel.control.sas = False
    ot.vessel.control.rcs = False
    ot.vessel.control.throttle = 1.0

    countdown()

    # Launch!
    ot.vessel.control.activate_next_stage()
    ot.vessel.auto_pilot.engage()
    ot.vessel.auto_pilot.target_pitch_and_heading(90, ot.inclination)

    # Ascent
    while ot.apoapsis() < ot.parking_orbit_alt:
        ot.vessel.auto_pilot.target_pitch_and_heading(ot.get_ascent_angle(), ot.inclination)
        ot.stage_if_needed()

    # Coast through atmosphere
    print("Coasting")
    ot.vessel.control.throttle = 0.0
    ot.wait_until_drag_is_negligible()

    ot.create_circularization_node(1000, 0.001)
    ot.execute_next_node()


def countdown():
    print('3...')
    time.sleep(1)
    print('2...')
    time.sleep(1)
    print('1...')
    time.sleep(1)
    print('Launch!')


if __name__ == '__main__':
    _ = raw_input("Press Enter to Continue")
    main()
