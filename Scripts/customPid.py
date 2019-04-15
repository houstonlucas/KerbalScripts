class PidController:
    def __init__(self, gains=(0.0, 0.0, 0.0),
                 effort_bounds=(-1.0, 1.0),
                 integral_threshold=0.25):
        self.k_p, self.k_i, self.k_d = gains
        self.e_prev, self.i_prev = None, 0.0
        self.min_effort, self.max_effort = effort_bounds
        self.integration_threshold = integral_threshold

    def get_effort(self, target, measurement, dt):
        if self.e_prev is None:
            self.e_prev = target - measurement

        e = target - measurement
        p = e
        i = self.i_prev + ((e + self.e_prev) * dt / 2.0)
        d = (e - self.e_prev) / dt

        if not (-self.integration_threshold < e < self.integration_threshold):
            i = 0.0

        self.e_prev = e
        self.i_prev = i

        effort = self.k_p * p + self.k_i * i + self.k_d * d
        effort = max(min(effort, self.max_effort), self.min_effort)
        return effort

    def set_bounds(self, effort_bounds):
        self.min_effort, self.max_effort = effort_bounds
