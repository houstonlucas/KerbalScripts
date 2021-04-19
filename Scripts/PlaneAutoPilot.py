import krpc
import time
from customPid import PidController
import DebugTools as db


def plane_autopilot(ot):
    altitude = ot.conn.add_stream(getattr, ot.vessel.flight(), 'mean_altitude')
    vertical_speed = ot.conn.add_stream(
        getattr,
        ot.vessel.flight(ot.vessel.orbit.body.reference_frame),
        'vertical_speed'
    )
    roll = ot.conn.add_stream(getattr, ot.vessel.flight(), 'roll')

    desired_altitude = altitude()

    control = ot.vessel.control

    roll_control_limit = 0.05
    alt_pid = PidController(
        gains=(0.05, 0.025, 0.30),
        effort_bounds=(-10.0, 10.0),
        integral_threshold=100.0
    )
    vertical_speed_pid = PidController(
        gains=(0.038, 0.0, 0.015),
        effort_bounds=(-0.95, 0.95),
        integral_threshold=0.2
    )
    roll_pid = PidController(
        gains=(0.005, 0.0, 0.001),
        effort_bounds=(-roll_control_limit, roll_control_limit)
    )

    vs_cmd_dot = 0.8
    roll_cmd_dot = 0.5
    pitch_cmd_dot = 0.002
    yaw_cmd_dot = 0.8

    control.sas = False
    vs_cmd_prev = 0.0
    roll_cmd_prev = 0.0

    dt = 1 / 30.0
    while True:
        vs = vertical_speed()
        roll_value = roll()
        alt = altitude()
        yaw = control.yaw

        vs_command = alt_pid.get_effort(desired_altitude, alt, dt)
        vs_command = control_update(vs_command, vs_cmd_prev, vs_cmd_dot)

        pitch_cmd = vertical_speed_pid.get_effort(vs_command, vs, dt)
        control.pitch = control_update(pitch_cmd, control.pitch, pitch_cmd_dot)

        roll_cmd = roll_pid.get_effort(0.0, roll_value, dt)
        control.roll = control_update(roll_cmd, roll_cmd_prev, roll_cmd_dot)

        yaw_cmd = 0.0
        control.yaw = control_update(yaw_cmd, yaw, yaw_cmd_dot)

        vs_cmd_prev = vs_command
        roll_cmd_prev = control.roll


        debug_dict = {
            "vs_cmd": vs_command,
            "Altitude": alt,
            "Desired": desired_altitude
        }
        db.debug_printing(debug_dict)

        if control.sas:
            print("Returning control")
            break

        time.sleep(dt)


def control_update(new_cmd, old_cmd, max_change):
    delta = new_cmd - old_cmd
    if delta == 0:
        return new_cmd
    sign = delta / abs(delta)
    return new_cmd if abs(delta) < max_change else old_cmd + sign * max_change
