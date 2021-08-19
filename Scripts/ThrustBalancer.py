import time

import casadi

from Scripts.OrbitTools import OrbitTools
from cvxopt import matrix, solvers
from casadi import qpsol, vertcat, MX, nlpsol
import numpy as np


def main():
    solvers.options['show_progress'] = False
    ot = OrbitTools("ThrustBalancer")
    tb = ThrustBalancer(ot, ot.vessel)

    running = True
    while running:
        try:
            tb.rebalance()
        except Exception as e:
            print(e)
            print("Ooops!")
            running = False


class ThrustBalancer:
    def __init__(self, ot, vessel):
        self.ot = ot
        self.vessel = vessel
        self.torques = []
        self.engines = []
        self.active_engines = []
        self.num_active_engines = 0

        self.thrust_limits = None

    def compute_torques_(self):
        # Reset the torques and active engines lists
        self.engines = self.vessel.parts.engines
        self.active_engines = []
        self.torques = []

        vessel_frame = self.vessel.reference_frame

        # For each engine compute the torque it produces
        for engine in self.engines:
            if engine.active:
                thrust_value = engine.max_thrust
                thrust = thrust_value * np.array(engine.part.direction(vessel_frame))
                r = np.array(engine.part.position(vessel_frame))
                torque = np.cross(thrust, r)
                self.torques.append(torque)
                self.active_engines.append(engine)
        self.num_active_engines = len(self.torques)

    def create_P_matrix(self):
        n = self.num_active_engines
        P = np.zeros((n, n))
        for i in range(n):
            for j in range(i, n):
                TiTj = np.dot(self.torques[i], self.torques[j])
                P[i][j] = TiTj
                P[j][i] = TiTj
        return P

    def casadi_solve_qp(self):
        n = self.num_active_engines

        x = []
        P = self.create_P_matrix()
        # This linear term is not correct, but tends to work reasonably well
        q = [-np.linalg.norm(t) for t in self.torques]

        # Create the optimization variables
        for i in range(n):
            x_i = MX.sym(f'x_{i}')
            x.append(x_i)
        loss = 0

        for i in range(n):
            for j in range(n):
                loss += P[i][j] * x[i] * x[j]
            loss += q[i] * x[i]

        qp = {'x': vertcat(*x), 'f': loss}
        opts = {"printLevel": "none", "error_on_fail": False}
        solver = qpsol('solver', 'qpoases', qp, opts)

        lower_bound_x = [0.0] * n
        upper_bound_x = [1.0] * n
        solution = solver(lbx=lower_bound_x, ubx=upper_bound_x)

        # Update thrust limits to midpoints between current solution and previous solution
        # This helps mitigate wild changes in the solution to be more gradual. Lazy man's low pass filter
        if self.thrust_limits is not None:
            self.thrust_limits = [(self.thrust_limits[i] + float(solution['x'][i])) / 2.0 for i in range(n)]
        else:
            self.thrust_limits = [float(solution['x'][i]) for i in range(n)]

        # Re-scale solution for maximum thrust
        max_thrust = max(self.thrust_limits)
        self.thrust_limits = [self.thrust_limits[i] / max_thrust for i in range(n)]

    # Solution I came up with after making most of the QP video.
    # Uses non linear constraints to avoid having to guess a linear term
    def casadi_solve2_qp(self):
        n = self.num_active_engines

        # Create Opti object
        opti = casadi.Opti()
        sq_mag = 0.0

        x = []
        # Create the optimization variables
        for i in range(n):
            x.append(opti.variable())
            sq_mag += x[i] ** 2
            opti.subject_to(opti.bounded(0, x[i], 1))

        # Create loss function
        loss = 0
        P = self.create_P_matrix()
        for i in range(n):
            for j in range(n):
                loss += P[i][j] * x[i] * x[j]
            loss += 1e-3 * x[i]

        opti.minimize(loss)
        opti.subject_to(sq_mag == 2.0)

        for i in range(n):
            if self.thrust_limits is None:
                opti.set_initial(x[i], casadi.sqrt(1 / n))
            else:
                opti.set_initial(x[i], self.thrust_limits[i])
        p_opts = {"print_time": 0}
        s_opts = {"print_level": 0}
        opti.solver('ipopt', p_opts, s_opts)

        solution = opti.solve()

        # print(solution)
        if self.thrust_limits is not None:
            self.thrust_limits = [(self.thrust_limits[i] + float(solution['x'][i])) / 2.0 for i in range(n)]
        else:
            self.thrust_limits = [float(solution.value(x_i)) for x_i in x]

        # Re-scale solution for maximum thrust
        max_thrust = max(self.thrust_limits)
        self.thrust_limits = [self.thrust_limits[i] / max_thrust for i in range(n)]

    # Initially I wrote the optimization in cvx_opt, but ended up using casadi for the flexibility
    def cvx_solve_qp(self):
        n = self.num_active_engines
        P = matrix(self.create_P_matrix())
        # Incentivize larger thrust values
        q = matrix(-np.ones(n))

        # Enforce thrust limits between 0 and 1
        G1 = -np.eye(n)
        G2 = np.eye(n)
        G = matrix(np.vstack([G1, G2]))

        h1 = np.zeros(n)
        h2 = np.ones(n)
        h = matrix(np.hstack([h1, h2]))

        if self.thrust_limits is None:
            init = matrix(np.ones(n))
        else:
            init = self.thrust_limits
        solution = solvers.qp(P, q, G, h, initvals=init)
        self.thrust_limits = solution['x']

        # Re-scale solution for maximum thrust
        max_thrust = max(self.thrust_limits)
        self.thrust_limits = [self.thrust_limits[i] / max_thrust for i in range(n)]

    def enact_engine_thrust_limits(self):
        # Set the engine thrust limits
        for i, engine in enumerate(self.active_engines):
            engine.thrust_limit = self.thrust_limits[i]

    def rebalance(self):
        self.compute_torques_()
        self.casadi_solve_qp()
        self.enact_engine_thrust_limits()


# Debugging tool to see value of cost function.
def f(x, P):
    n = len(x)
    ans = 0.0
    for i in range(n):
        for j in range(n):
            ans += P[i][j] * x[i] * x[j]
    return ans


if __name__ == '__main__':
    main()
