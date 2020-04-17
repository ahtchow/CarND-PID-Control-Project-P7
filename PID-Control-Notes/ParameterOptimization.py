"""
Parameter Optimization [TWIDDLE ALGORITHM]

    Our function: run() must return a score that grades the parameters TAU.

    E.g For parameter = [0,0,0] and dp  = [1,1,1]:

    Pseudocode:

        best_error = run(parameter)
        while sum(dp) > 0.0001:
            for i in range(len(parameter)):
                parameter[i] += dp[i]
                error = run(parameter)
                if error < best_error:
                    best_error = error
                    dp[i] *= 1.1
                else:
                    parameter[i] -= 2*dp[i]  #Subtract twice, since added before
                    error = run[parameter]
                    if not error < best_error:
                        parameter[i] += dp[i] #Back to original
                        dp[i] *= 0.9


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

def make_robot():
    robot = Robot()
    robot.set(0.0, 1.0, 0.0)
    robot.set_steering_drift(10 / 180 * np.pi)
    return robot

def run(robot, params,  n = 100, speed = 1.0):
    x_trajectory = []
    y_trajectory = []
    cte_prev = robot.y
    sum_cte, error = 0, 0
    for i in range(2* n):
        cte = robot.y #Cross Track Error
        diff_cte = cte - cte_prev
        sum_cte += cte
        steering_angle = (-params[0] * cte) - (params[1] * diff_cte) - (params[2] * sum_cte)
        robot.move(steering_angle, speed)
        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)
        cte_prev = cte
        if i >= n:
            error += cte ** 2
    return x_trajectory, y_trajectory, (error / n)

def twiddle(tol = 0.2):
    p = [0,0,0]
    dp =[1,1,1]
    robot = make_robot()
    x_trajectory, y_trajectory, best_error = run(robot, p)
    while sum(dp) > tol:
        for i in range(len(p)):
            p[i] += dp[i]
            robot = make_robot()
            x_trajectory, y_trajectory, error = run(robot, p)
            if error < best_error:
                best_error = error
                dp[i] *= 1.1
            else:
                p[i] -= 2*dp[i]  #Subtract twice, since added before
                robot = make_robot()
                x_trajectory, y_trajectory, error = run(robot ,p)

                if error < best_error:
                    best_error = error
                    dp[i] *= 1.1

                else:
                    p[i] += dp[i] #Back to original
                    dp[i] *= 0.9

    return p, best_error

params, err = twiddle()
print("The finals Tau Parameters:" + str(params))
robot = make_robot()
x_trajectory, y_trajectory, err = run(robot, params)
n = len(x_trajectory)
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8))
ax1.plot(x_trajectory, y_trajectory, 'g', label='Twiddle PID controller')
ax1.plot(x_trajectory, np.zeros(n), 'r', label='reference')
plt.show()
