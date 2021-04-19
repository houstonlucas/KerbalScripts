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
        self.engines = []

        self.thrust_limits = None

    def compute_torques_(self):
        # Reset the torques list
        self.torques = []
        self.engines = self.vessel.parts.engines

        vessel_frame = self.vessel.reference_frame

        # For each engine compute the torque it produces
        for engine in self.engines:
            thrust = engine.max_thrust * np.array(engine.part.direction(vessel_frame))
            r = np.array(engine.part.position(vessel_frame))
            torque = np.cross(thrust, r)
            self.torques.append(torque)

    def create_P_matrix(self):
        n = len(self.torques)
        P = np.zeros((n, n))
        for i in range(n):
            for j in range(i, n):
                TiTj = np.dot(self.torques[i], self.torques[j])
                P[i][j] = TiTj
                P[j][i] = TiTj
        return matrix(P)

    def solve_qp(self):
        n = len(self.torques)
        P = self.create_P_matrix()
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
        sol = solvers.qp(P, q, G, h, initvals=init)
        self.thrust_limits = sol['x']

        # Re-scale solution for maximum thrust
        max_thrust = max(self.thrust_limits)
        self.thrust_limits = [self.thrust_limits[i] / max_thrust for i in range(n)]

        # Set the engine thrust limits
        for i, engine in enumerate(self.engines):
            engine.thrust_limit = self.thrust_limits[i]

    def rebalance(self):
        self.compute_torques_()
        self.solve_qp()


if __name__ == '__main__':
    main()
