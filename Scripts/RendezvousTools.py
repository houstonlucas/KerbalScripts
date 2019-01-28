from math import pi, sqrt


# Returns the orbital period that will intersect the target in n of the targets orbits.
# Variables:
#  period: *My* orbital period
#  targets_period: Orbital period of the target.
#  target_lag: number seconds until the target next reaches the intersect location
#  n: number of target orbits until intersect.
#  k: number of *my* orbits until intersect.
# Assumptions:
#  Relative Inclination near zero
#
def calculate_new_period(period, targets_period, target_lag, n):
    time_to_meetup = target_lag + targets_period * n
    # Approximate number of *my* orbits needed to intersect target.
    k_hat = time_to_meetup / period
    k = round(k_hat)
    new_t = time_to_meetup / float(k)
    return new_t


# Returns the orbital period that will intersect the target
# Variables:
#  period: *My* orbital period
#  targets_period: Orbital period of the target.
#  target_lag: number seconds until the target next reaches the intersect location
#  mu: Gravitational parameter for orbited body.
#  valid_n: List of acceptable number of targets orbits to wait until intersect.
# Assumptions:
# Relative Inclination near zero
#
def find_optimal_rendevous_period(period, target_period, target_lag, mu, valid_n=range(1, 5)):
    # Iterate over valid_n, choose n that minimizes the dv needed to change orbital period.
    v = velocity_from_semi_major(semi_major_from_period(period, mu), mu)
    potential_period = map(lambda n: calculate_new_period(period, target_period, target_lag, n), valid_n)
    potential_a = map(lambda t_new: semi_major_from_period(t_new, mu), potential_period)
    potential_v = map(lambda a: velocity_from_semi_major(a, mu), potential_a)
    dvs = map(lambda vn: abs(v - vn), potential_v)
    dv = min(dvs)
    index = dvs.index(dv)
    return potential_period[index], valid_n[index]


def calculate_target_lag(vessel, target):
    # The ut that we will use for the rendezvous setup burn
    vessel_approach_ut = get_next_approach(vessel, target)

    # Only works if we make the assumption that the orbits are similar in inclination
    # and that the longitudes of ascending nodes are also similar
    periapsis_offset = vessel.orbit.argument_of_periapsis - target.orbit.argument_of_periapsis
    anomaly_intersect = vessel.orbit.true_anomaly_at_ut(vessel_approach_ut)
    target_intersect_anomaly = anomaly_intersect + periapsis_offset

    target_approach_ut = target.orbit.ut_at_true_anomaly(target_intersect_anomaly)
    lag = (target_approach_ut - vessel_approach_ut) % target.orbit.period
    if lag > target.orbit.period/2.0:
        # If they are lagging by more than half an orbit then calculate it in front instead of behind.
        lag = target.orbit.period - lag
    return lag

def semi_major_from_period(t, mu):
    return ((t * t * mu) / (4.0 * pi * pi)) ** (1.0 / 3.0)


# Currently an approximation
def velocity_from_semi_major(a, mu):
    return (mu / a) ** 0.5


def get_next_approach(vessel, target):
    approaches = vessel.orbit.list_closest_approaches(target.orbit, 2)
    if len(approaches) > 1:
        return approaches[0][0]
    else:
        return approaches[0][0]


def get_magnitude(x):
    return sqrt(sum([val * val for val in x]))


def get_difference(x, y):
    return [x[i] - y[i] for i in range(len(x))]


def scale(x, s):
    return [a * s for a in x]
