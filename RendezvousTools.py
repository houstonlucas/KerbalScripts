from math import pi


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
def find_optimal_rendevous_orbit(period, target_period, target_lag, mu, valid_n=range(1, 10)):
    # Iterate over valid_n, choose n that minimizes the dv needed to change orbital period.
    v = velocity_from_semi_major(semi_major_from_period(period, mu), mu)
    potential__t = map(lambda n: calculate_new_period(period, target_period, target_lag, n), valid_n)
    potential_a = map(lambda t_new: semi_major_from_period(t_new, mu), potential__t)
    potential_v = map(lambda a: velocity_from_semi_major(a, mu), potential_a)
    dvs = map(lambda vn: abs(v - vn), potential_v)
    dv = min(dvs)
    index = dvs.index(dv)
    print("{} orbits to intersect.".format(valid_n[index]))
    return potential__t[index]


def semi_major_from_period(t, mu):
    return ((t * t * mu) / (4.0 * pi * pi)) ** (1.0 / 3.0)


# Currently an approximation
def velocity_from_semi_major(a, mu):
    return (mu / a) ** 0.5
