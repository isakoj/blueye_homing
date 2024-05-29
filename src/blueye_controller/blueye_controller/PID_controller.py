import numpy as np

class PIDController:
    def __init__(self, kp, ki, kd, integral_limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.previous_error = 0.0
        self.integral = 0.0

    def update(self, error, velocity, delta_time):
        self.integral += error * delta_time
        self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)

        output = self.kp * error + self.ki * self.integral - self.kd * velocity
        return output