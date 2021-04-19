import datetime
import threading
import time
from collections import deque

import numpy as np
import pandas as pdb

from inputs import get_gamepad

from Scripts.OrbitTools import OrbitTools
from Scripts.customPid import PidController
from Scripts.Misc import angle_between_vectors, norm, DEGREES, sigmoid
from Scripts.DebugTools import DebugDisplay

PITCH_SCALING = 45.0
YAW_SCALING = 25.0
ROLL_SCALING = 45.0
BRAKE_SCALING = 45.0
dt = 1 / 60.0


def main():
    usb = UsbController()
    ot = OrbitTools("BellyFlopper")
    # debug_display = DebugDisplay(ot, "({:0.2f}, {:0.2f}, {:0.2f})")
    bf = BellyFlopper(ot, ot.vessel)

    home_angles = np.array([45.0, 45.0, 45.0, 45.0])

    pitch_vector = np.array([1.0, 1.0, -1.0, -1.0])
    yaw_vector = np.array([-1.0, 1.0, -1.0, 1.0])
    roll_vector = np.array([-1.0, 1.0, 1.0, -1.0])
    brake_vector = np.array([-1.0, -1.0, -1.0, -1.0])

    pitch_component = np.zeros(4)
    yaw_component = np.zeros(4)
    roll_component = np.zeros(4)
    brake_component = np.zeros(4)

    usb.start()
    while usb.running:
        bf.adjust_targets()
        # pitch_scale = 10.0 + usb.RJS_Y * PITCH_SCALING
        pitch_scale = bf.compute_pitch_effort(dt) * PITCH_SCALING
        pitch_component = pitch_scale * pitch_vector
        # print(pitch_scale)

        # yaw_scale = usb.RJS_X * YAW_SCALING
        yaw_scale = bf.compute_yaw_effort(dt) * YAW_SCALING
        yaw_component = -yaw_scale * yaw_vector

        # roll_scale = usb.LJS_X * ROLL_SCALING
        roll_scale = bf.compute_roll_effort(dt) * ROLL_SCALING
        roll_component = roll_scale * roll_vector

        brake_scale = usb.LJS_Y * BRAKE_SCALING
        brake_component = brake_scale * brake_vector

        angles = home_angles + pitch_component + yaw_component + roll_component + brake_component
        bf.set_target_angles(angles)
        bf.actuate()
        time.sleep(dt)

    landing_angles = np.array([0.0, 0.0, 0.0, 0.0])
    bf.set_target_angles(landing_angles)
    bf.actuate()
    usb.stop()

    bf.vessel.control.sas = True
    bf.vessel.control.rcs = True
    time.sleep(0.2)
    # bf.vessel.auto_pilot.roll_pids_gains = (0.2, 0.0, 0.1)
    bf.vessel.auto_pilot.sas_mode = ot.ksc.SASMode.retrograde


class UsbController:
    MAX_ANALOG_INPUT = 32767

    def __init__(self):
        self.thread = threading.Thread(target=self.run)
        self.running = False

        self.LJS_X = 0.0
        self.LJS_Y = 0.0
        self.RJS_X = 0.0
        self.RJS_Y = 0.0

    def start(self):
        self.running = True
        self.thread.start()

    def stop(self):
        if self.running:
            self.running = False
            self.thread.join()

    def run(self):
        while self.running:
            events = get_gamepad()
            for event in events:
                if event.code == "BTN_SELECT":
                    self.running = False
                    break
                elif event.code == "ABS_RX":
                    self.RJS_X = event.state / UsbController.MAX_ANALOG_INPUT
                elif event.code == "ABS_RY":
                    self.RJS_Y = event.state / UsbController.MAX_ANALOG_INPUT
                elif event.code == "ABS_X":
                    self.LJS_X = event.state / UsbController.MAX_ANALOG_INPUT
                elif event.code == "ABS_Y":
                    self.LJS_Y = event.state / UsbController.MAX_ANALOG_INPUT
                # else:
                #     print(self.event.code)


class KrpcServo:
    TARGET_ANGLE_STR = "Target Angle"

    def __init__(self, part):
        self.part = part
        self.module = None

        # Set rotor module
        for module in part.modules:
            if 'servo' in module.name.lower():
                self.module = module
                break
        assert (self.module is not None)

    def set_angle(self, angle):
        self.module.set_field_float(KrpcServo.TARGET_ANGLE_STR, angle)


class BellyFlopper:

    def __init__(self, ot, vessel):
        self.vessel = vessel
        self.ot = ot

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 270.0

        self.target_pitch = 15.0
        self.target_roll = 0.0
        self.target_yaw = 0.0
        self.flight = self.vessel.flight(self.vessel.orbit.body.reference_frame)

        self.pitch_stream = ot.conn.add_stream(getattr, self.vessel.flight(), 'pitch')
        self.roll_stream = ot.conn.add_stream(getattr, self.vessel.flight(), 'roll')
        self.yaw_stream = ot.conn.add_stream(getattr, self.vessel.flight(), "heading")

        self.pitch_pid = PidController((0.05, 0.03, 0.01), dt=dt)
        self.roll_pid = PidController((0.01, 0.0002, 0.001), dt=dt)
        self.yaw_pid = PidController((10.9, 0.01, 0.5), dt=dt)

        self.servos = []
        self.target_angles = [0.0, 0.0, 0.0, 0.0]
        for part in self.ot.vessel.parts.all:
            if "servo" in part.name.lower():
                servo = KrpcServo(part)
                self.servos.append(servo)

    def set_target_angles(self, angles):
        for i in range(4):
            self.target_angles[i] = max(min(angles[i], 90.0), 0.0)

    def actuate(self):
        for i, servo in enumerate(self.servos):
            servo.set_angle(self.target_angles[i])

    def compute_pitch_effort(self, dt):
        self.pitch = self.pitch_stream()
        # print("Pitch: {}".format(self.pitch))
        return -self.pitch_pid.get_effort(self.target_pitch, self.pitch, dt)

    def compute_roll_effort(self, dt):
        self.roll = self.roll_stream()
        # print("Roll: {}".format(self.roll))
        return self.roll_pid.get_effort(self.target_roll, self.roll, dt)

    def compute_yaw_effort(self, dt):
        self.yaw = self.yaw_stream()
        # print("Yaw: {:3.3}".format(self.yaw))
        return -self.yaw_pid.get_effort(self.target_yaw, self.yaw, dt)/10.0

        print("Target Yaw: {:0.2f}".format(self.target_yaw))

    def adjust_targets(self):
        self.target_pitch = np.rad2deg(np.arctan2(self.flight.vertical_speed, self.flight.horizontal_speed))
        self.target_pitch += 80
        # print(self.target_pitch)
        self.target_pitch = np.clip(self.target_pitch, 0.0, 60.0)
        print("Target Pitch: {:0.2f}".format(self.target_pitch))
        # print("#"*20)


if __name__ == '__main__':
    main()
