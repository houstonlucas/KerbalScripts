import time

from Scripts.OrbitTools import OrbitTools
from cvxopt import matrix, solvers
import numpy as np


def main():
    solvers.options['show_progress'] = False
    ot = OrbitTools("ThrustBalancer")
    tb = ThrustBalancer(ot, ot.vessel)

    running = True
    while running:
        try:
            tb.rebalance()
            time.sleep(0.1)
        except Exception as e:
            print(e)
            print("Ooops!")
            running = False

class ThrustBalancer:
    def __init__(self, ot, vessel):
        self.ot = ot
        self.vessel = vessel
        self.torques = []

    def compute_torques_(self):
        # Reset the torques list
        self.torques = []

        engines = self.vessel.parts.engines
        vessel_frame = self.vessel.reference_frame

        # For each engine compute the torque it produces
        for engine in engines:
            thrust = engine.max_thrust * np.array(engine.part.direction(vessel_frame))
            r = np.array(engine.part.position(vessel_frame))
            torque = np.cross(thrust, r)
            self.torques.append(torque)

    def create_Q_matrix(self):
        n = len(self.torques)
        Q = np.zeros((n, n))
        for i in range(n):
            for j in range(i, n):
                TiTj = np.dot(self.torques[i], self.torques[j])
                Q[i][j] = TiTj
                Q[j][i] = TiTj
        return matrix(Q)

    def solve_qp(self):
        n = len(self.torques)
        Q = self.create_Q_matrix()
        q = matrix(-1e9 * np.ones(n))

        # TODO: Generalize for more engines (Currently hard coded for 2)
        # Enforce thrust limits between 0 and 1
        G = matrix([
            [-1.0, 0.0],
            [0.0, -1.0],
            [1.0, 0.0],
            [0.0, 1.0]
        ]).T
        h = matrix([0.0, 0.0, 1.0, 1.0])

        sol = solvers.qp(Q, q, G, h, initvals=matrix([1.0, 1.0]))
        thrust_limits = sol['x']
        for i, engine in enumerate(self.vessel.parts.engines):
            engine.thrust_limit = thrust_limits[i]

    def rebalance(self):
        self.compute_torques_()
        self.solve_qp()


if __name__ == '__main__':
    main()
