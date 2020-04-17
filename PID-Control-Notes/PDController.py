"""
PD-Control
    α = - τ_p * CTE - τ_d * (d/dt) * CTE

    α -> Steering Angle
    τ -> Controller Constant
    CTE -> Cross Track Error

    About [ - τ_d * (d/dt) * CTE ] :
     -> As time goes on, the error changes and forces the countersteering
     -> Purpose to change the amplitude of oscillation

     (d/dt) * CTE = CTE_t - CTE_(t-1) / ΔT

Implementation:

"""
import random
import numpy as np
import matplotlib.pyplot as plt

class Robot(object):
    def __init__(self, length = 20.0):
        """
        Creates robot and intializes location/orientation to zeros
        """
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0
        self.length = length
        self.steering_noise = 0.0
        self.distance_noise = 0.0
        self.steering_drift = 0.0

    def set(self, x , y, orientation):
        """
        Sets coordinate for robot
        """
        self.x = x
        self.y = y
        self.orientation = orientation % (2.0 * np.pi)

    def set_noise(self, steering_noise, distance_noise):
        """
        Sets the noise parameters
        """
        self.steering_noise = steering_noise
        self.distance_noise = distance_noise

    def set_steering_drift(self, drift):
        """
        Sets the systematical steering drift parameter
        """
        self.steering_drift = drift

    def move(self, steering, distance, tolerance = 0.001, max_steering_angle = np.pi/4.0):
        """
        steering = front wheel steering angle, limited by max_steering_angle
        distance = total distance driven, most be non-negative
        """
        if steering > max_steering_angle:
            steering = max_steering_angle
        if steering < -max_steering_angle:
            steering = -max_steering_angle
        if distance < 0.0:
            distance = 0.0

        #Apply Noise
        steering2 = random.gauss(steering, self.steering_noise)
        distance2  = random.gauss(distance, self.distance_noise)

        steering2 += self.steering_drift # Apply Steering Drift
        turn = np.tan(steering2) * distance2 / self.length

        if abs(turn) < tolerance:
            # approximate by straight line motion
            self.x += distance2 * np.cos(self.orientation)
            self.y += distance2 * np.sin(self.orientation)
            self.orientation = (self.orientation + turn) % (2.0 * np.pi)
        else:
            # approximate bicycle model for motion
            radius = distance2 / turn
            cx = self.x - (np.sin(self.orientation) * radius)
            cy = self.y + (np.cos(self.orientation) * radius)
            self.orientation = (self.orientation + turn) % (2.0 * np.pi)
            self.x = cx + (np.sin(self.orientation) * radius)
            self.y = cy - (np.cos(self.orientation) * radius)

    def __repr__(self):
        return '[x=%.5f y=%.5f orient=%.5f]' % (self.x, self.y, self.orientation)

robot = Robot()
robot.set(0.0, 1.0, 0.0)

def run(robot, tau_p, tau_d, n = 100, speed = 1.0):
    x_trajectory = []
    y_trajectory = []
    cte_prev = robot.y
    for i in range(n):
        cte = robot.y #Cross Track Error
        steering_angle = (-tau_p * cte) - (tau_d * (cte - cte_prev)) #Steeing Angle is Prop. to Tau * CTE
        robot.move(steering_angle, speed)
        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)
        cte_prev = cte
    return x_trajectory, y_trajectory

tau = 0.1 # If we increase tau, steering angle increases. Oscillates faster (decrease period)
x_trajectory, y_trajectory = run(robot, 0.2, 3.0)
n = len(x_trajectory)
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8))
ax1.plot(x_trajectory, y_trajectory, 'g', label='P controller')
ax1.plot(x_trajectory, np.zeros(n), 'r', label='reference')
plt.show()
