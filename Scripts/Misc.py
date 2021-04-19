import json
import os
import time
import numpy as np

DEGREES = np.pi / 180.0


def sigmoid(x):
    return 1. / (1. + np.exp(-x))


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


def get_part_name_tag(part):
    for module in part.modules:
        if module.name == "KOSNameTag":
            return module.get_field('name tag')


def load_config(config_name):
    config_path = os.path.join("Configurations", config_name + ".json")
    config = json.load(open(config_path, "r"))
    return config


def save_config(config, config_name, pretty=False):
    config_path = os.path.join("Configurations", config_name + ".json")
    if pretty:
        json.dump(config, open(config_path, "w+"), indent=2)
    else:
        json.dump(config, open(config_path, "w+"))
