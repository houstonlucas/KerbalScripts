import time
import numpy as np

from customPid import PidController


class DockingInstance:
    def __init__(self, ot):

        self.is_running = True

        lat_vel_pid_gains = (0.7, 0.0, 0.2)
        long_vel_pid_gains = (0.5, 0.0, 0.2)
        vel_bounds = (-0.75, 0.75)

        self.z_vel_pid = PidController(gains=lat_vel_pid_gains, effort_bounds=vel_bounds)
        self.x_vel_pid = PidController(gains=lat_vel_pid_gains, effort_bounds=vel_bounds)
        self.y_vel_pid = PidController(gains=long_vel_pid_gains, effort_bounds=vel_bounds)

        lat_pid_gains = (1.4, 0.0, 0.4)
        long_pid_gains = (1.4, 0.0, 1.2)
        rcs_effort_bounds = (-1.0, 1.0)

        self.z_pid = PidController(gains=lat_pid_gains, effort_bounds=rcs_effort_bounds)
        self.x_pid = PidController(gains=lat_pid_gains, effort_bounds=rcs_effort_bounds)
        self.y_pid = PidController(gains=long_pid_gains, effort_bounds=rcs_effort_bounds)

        self.target_distance = 100.0
        self.auto_pilot_error_threshold = 0.6
        self.docking_approach_speed = 0.2  # m/s

        self.ot = ot
        self.control = self.ot.vessel.control
        self.auto_pilot = self.ot.vessel.auto_pilot
        self.orbital_frame = self.ot.vessel.orbit.body.orbital_reference_frame
        self.auto_pilot.reference_frame = self.orbital_frame

        self.relative_pos = None
        self.relative_vel = None
        self.z_vel_cmd = None
        self.x_vel_cmd = None
        self.y_vel_cmd = None
        self.up_cmd = None
        self.right_cmd = None
        self.forward_cmd = None

        self.target_port = None
        self.target_part = None
        self.my_docking_port = None
        self.my_frame = None
        self.target_frame = None

    def acquire_target_info(self):
        self.target_port_selection()
        self.my_docking_port = self.ot.vessel.parts.docking_ports[0]
        self.my_frame = self.my_docking_port.reference_frame
        self.target_frame = self.target_port.reference_frame

        dist_2_targ = np.linalg.norm(self.target_part.position(self.my_frame))
        self.target_distance = dist_2_targ

    def target_port_selection(self):
        if self.ot.ksc.target_docking_port is None:
            if self.ot.ksc.target_vessel is None:
                print("No target selected.")
                self.shutdown()
            else:
                docking_ports = self.ot.ksc.target_vessel.parts.docking_ports
                num_docking_ports = len(docking_ports)
                if num_docking_ports == 0:
                    print("Target has no docking ports")
                    self.shutdown()
                elif num_docking_ports > 1:
                    self.target_port = self.select_nearest_docking_port(docking_ports)
                else:
                    self.target_port = docking_ports[0]
                    if self.target_port.state != self.ot.ksc.DockingPortState.ready:
                        print("Target docking port not in ready state. (%s) " % self.target_port.state.name)
                        self.shutdown()
        else:
            self.target_port = self.ot.ksc.target_docking_port

        self.target_part = self.target_port.part

    def shutdown(self):
        print("Shutting Down")
        self.is_running = False
        self.ot.auto_off()
        self.control.rcs = False

    def orient_to_target(self, wait=True):
        target_direction = self.target_port.direction(self.orbital_frame)
        self.auto_pilot.target_direction = tuple(-di for di in target_direction)
        self.ot.auto_on()
        if wait:
            while self.auto_pilot.error > self.auto_pilot_error_threshold:
                time.sleep(0.1)

    def compute_controls(self, dt):
        relative_pos = self.target_part.position(self.my_frame)
        relative_vel = self.target_part.velocity(self.my_frame)
        x, y, z = self.relative_pos = relative_pos
        x_vel, y_vel, z_vel = self.relative_vel = relative_vel

        dist_2_targ = np.linalg.norm(self.target_part.position(self.my_frame))
        approach_speed = (self.docking_approach_speed + 0.4 * y)
        self.target_distance = dist_2_targ - approach_speed * dt

        self.z_vel_cmd = self.z_pid.get_effort(0.0, z, dt)
        self.x_vel_cmd = self.x_pid.get_effort(0.0, x, dt)
        self.y_vel_cmd = self.y_pid.get_effort(self.target_distance, y, dt)
        self.up_cmd = self.z_vel_pid.get_effort(self.z_vel_cmd, z_vel, dt)
        self.right_cmd = -self.x_vel_pid.get_effort(self.x_vel_cmd, x_vel, dt)
        self.forward_cmd = -self.y_vel_pid.get_effort(self.y_vel_cmd, y_vel, dt)

    def apply_controls(self):
        self.control.up = self.up_cmd
        self.control.right = self.right_cmd
        self.control.forward = self.forward_cmd

    def print_stats(self):
        msg = ""
        msg += "*" * 50 + "\n"
        msg += "Target Distance:\n\t\t%0.3f\n" % self.target_distance
        msg += "Measured Position:\n\tx:%0.2f, y:%0.2f, z:%0.2f\n" % self.relative_pos
        msg += "Measured Velocity:\n\tx:%0.2f, y:%0.2f, z:%0.2f\n" % self.relative_vel
        msg += "Command Velocity:\n\tx:%0.2f, y:%0.2f, z:%0.2f\n" % (self.x_vel_cmd, self.y_vel_cmd, self.z_vel_cmd)
        msg += "Command RCS values:\n\tx:%0.2f, y:%0.2f, z:%0.2f\n" % (self.right_cmd, self.forward_cmd, self.up_cmd)
        print(msg)

    def select_nearest_docking_port(self, ports):
        # TODO: select open port closest to ship.
        for docking_port in self.ot.ksc.target_vessel.parts.docking_ports:
            pass
        return ports[0]

    def cycle_rcs(self):
        self.control.rcs = not self.control.rcs

    def check_end_condition(self):
        end_condition_states = [
            self.ot.ksc.DockingPortState.docked,
            self.ot.ksc.DockingPortState.docking
        ]
        if self.my_docking_port.state in end_condition_states:
            print("Succesfully docked")
            self.is_running = False


def docking_assist(ot):
    dt = 0.1

    docking_controller = DockingInstance(ot)
    docking_controller.acquire_target_info()
    docking_controller.orient_to_target()

    iterations = 0
    while docking_controller.is_running:
        iterations += 1
        docking_controller.compute_controls(dt)
        docking_controller.apply_controls()
        if iterations % 10 == 0:
            docking_controller.cycle_rcs()
            docking_controller.print_stats()

        time.sleep(dt)
        docking_controller.check_end_condition()
    docking_controller.shutdown()
