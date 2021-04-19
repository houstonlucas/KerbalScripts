import time
from copy import deepcopy

import numpy as np
import CustomErrors as errors


def plan_maneuver(ot):
    planner = ManeuverPlanner(ot, closest_target_distance)
    np_node = planner.node.to_numpy()
    print(np_node)
    planner.find_maneuver()


def fuel_efficient_circularization(ot, nd):
    ap = nd.node.orbit.apoapsis
    pe = nd.node.orbit.periapsis
    # if nd.node.delta_v > 100.0:
    #     return np.inf
    gamma = 1e-1
    return (ap - pe) + gamma * nd.node.delta_v


def closest_target_distance(ot, nd):
    target = ot.get_target()
    targets_orbit = ot.get_target_orbit()
    orbit = nd.node.orbit
    temp_orbit = orbit
    use_periapsis = False
    while temp_orbit.next_orbit:
        if orbit.next_orbit.body.name == targets_orbit.body.name:
            orbit = orbit.next_orbit
        elif orbit.next_orbit.body == target:
            orbit = orbit.next_orbit
            use_periapsis = True
            break
        temp_orbit = temp_orbit.next_orbit

    if targets_orbit is None:
        error = errors.AssumptionViolation()
        error.msg = "Error no target selected"
        raise error

    offset = 0.0
    if use_periapsis:
        closest = orbit.periapsis
        offset = 1000.0
    else:
        closest = orbit.distance_at_closest_approach(targets_orbit)
    target_dist = 150000
    target_error = np.abs(target_dist - closest)/1e4
    return target_error + 10.0 * nd.node.delta_v  - offset


class ManeuverNode:
    def __init__(self, node):
        self.node = node

    def to_numpy(self):
        return np.array([
            self.node.prograde,
            self.node.radial,
            self.node.normal,
            self.node.ut
        ])

    def from_numpy(self, array):
        self.node.prograde = array[0]
        self.node.radial = array[1]
        self.node.normal = array[2]
        self.node.ut = array[3]


class ManeuverPlanner:
    def __init__(self, ot, objective, node_ut=None, use_existing=True, eps=1e-3, minimize=True):
        self.ot = ot
        self.objective_function = objective
        self.eps = eps
        self.is_min_problem = minimize

        if use_existing and self.ot.vessel.control.nodes:
            self.node = ManeuverNode(self.ot.vessel.control.nodes[0])
        else:
            if node_ut is None:
                node_ut = self.ot.ksc.ut + 1000000.0  # self.ot.vessel.orbit.time_to_periapsis
            nd = self.ot.vessel.control.add_node(node_ut, 0.0)
            self.node = ManeuverNode(nd)

        self.grad = None

    def find_maneuver(self):
        potential_step_sizes = np.array([pow(1.5, i) for i in np.linspace(-8, 4, 20)])
        if self.is_min_problem:
            potential_step_sizes *= -1.0

        step_taken = np.full(4, np.inf)
        delta_norm = np.linalg.norm(step_taken)
        np_node = self.node.to_numpy()

        while delta_norm > self.eps:
            np_node = self.node.to_numpy()
            grad = self.compute_grad()
            grad_norm = np.linalg.norm(grad)
            grad = grad / grad_norm

            steps = np.outer(potential_step_sizes, grad)
            best_index = self.get_best_step_index(steps)
            best_step = steps[best_index]
            step_taken = best_step
            delta_norm = np.linalg.norm(step_taken)
            np_node += step_taken
            self.node.from_numpy(np_node)
            objective_value = self.objective_function(self.ot, self.node)
            print("Value: {}".format(objective_value))
            print("Delta:", step_taken)
            print("Delta norm:", delta_norm)
            # print("########################################")

    def get_best_step_index(self, steps):
        np_node_initial = self.node.to_numpy()
        objective_values = []
        for step in steps:
            self.node.from_numpy(np_node_initial + step)
            value = self.objective_function(self.ot, self.node)
            objective_values.append(value)

        if self.is_min_problem:
            return np.argmin(objective_values)
        else:
            return np.argmax(objective_values)

    def compute_grad(self, h=0.001):
        np_node_initial = self.node.to_numpy()

        grad = np.zeros(4)
        for i in range(4):
            grad[i] = self.compute_grad_component(np_node_initial, i, h)

        self.node.from_numpy(np_node_initial)
        return grad

    def compute_grad_component(self, np_node_initial, component_index, h=0.001):
        np_node_temp = np_node_initial.copy()

        # Compute f(x - h)
        np_node_temp[component_index] = np_node_initial[component_index] - h
        temp = deepcopy(self.node)
        temp.from_numpy(np_node_temp)
        left_obj = self.objective_function(self.ot, temp)

        # Compute f(x + h)
        np_node_temp[component_index] = np_node_initial[component_index] + h
        temp = deepcopy(self.node)
        temp.from_numpy(np_node_temp)
        right_obj = self.objective_function(self.ot, temp)

        return (right_obj - left_obj) / (2.0 * h)
