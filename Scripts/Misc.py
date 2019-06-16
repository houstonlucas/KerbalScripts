import time
import numpy as np

DEGREES = np.pi / 180.0

# TODO: move mining to it's own file.
class MiningOperations:
    def __init__(self, ot):
        self.ot = ot
        self.harvesters = self.ot.vessel.parts.resource_harvesters

    def deploy_equipment(self):
        # Deploy Mining Equipment
        for harvester in self.harvesters:
            if harvester.state == self.ot.ksc.ResourceHarvesterState.retracted:
                harvester.deployed = True

    def activate_equipment(self):
        wait_states = [
            self.ot.ksc.ResourceHarvesterState.deploying,
            self.ot.ksc.ResourceHarvesterState.retracted
        ]
        for harvester in self.harvesters:
            while harvester.state in wait_states:
                time.sleep(0.01)
            harvester.active = True

    def warp_until_full(self):
        time.sleep(1.0)
        ore_amount = self.ot.vessel.resources.amount("Ore")
        while ore_amount < self.ot.vessel.resources.max("Ore"):
            self.ot.ksc.rails_warp_factor = 5
            time.sleep(0.1)
            ore_amount = self.ot.vessel.resources.amount("Ore")
        self.ot.ksc.rails_warp_factor = 0

    def deactivate_equipment(self):
        for harvester in self.harvesters:
            harvester.active = False

    def retract_equipment(self):
        # Retract Mining Equipment
        for harvester in self.harvesters:
            harvester.deployed = False


def mining_operations(ot):
    m_ops = MiningOperations(ot)
    m_ops.deploy_equipment()
    m_ops.activate_equipment()
    m_ops.warp_until_full()
    m_ops.deactivate_equipment()
    m_ops.retract_equipment()


def norm(v):
    return np.linalg.norm(v)


def angle_between_vectors(u, v, convert2deg=True):
    """ Compute the angle between vector u and v
        Directly from krpc docs.
        https://krpc.github.io/krpc/tutorials/pitch-heading-roll.html
    """
    dp = np.dot(u, v)
    if dp == 0:
        return 0
    u_norm = norm(u)
    v_norm = norm(v)
    angle = np.math.acos(dp / (u_norm * v_norm))
    if convert2deg:
        angle *= (180. / np.pi)
    return angle
