import numpy as np
import math
from utils import *
import pdb
"""
Starter script for lab1.
Author: Chris Correa
"""

# IMPORTANT: the init methods in this file may require extra parameters not
# included in the starter code.

class MotionPath:
    def target_position(self, time):
        raise NotImplementedError

    def target_velocity(self, time):
        raise NotImplementedError

    def target_acceleration(self, time):
        raise NotImplementedError

class LinearPath(MotionPath):
    def __init__(self, start_pos, target_pos, target_time):
        self.start_pos = np.array(start_pos)
        self.target_pos = np.array(target_pos)
        self.target_time = np.array(target_time)

    def target_position(self, time):
        return (self.start_pos*(self.target_time - time) + self.target_pos*time) / self.target_time

    def target_velocity(self, time):
        return (self.target_pos - self.start_pos) / self.target_time

    def target_acceleration(self, time):
        warmup = 0.4
        if time < warmup:
            delta = self.target_pos - self.start_pos
            v = self.target_velocity(time)
            return delta * np.linalg.norm(v)
        elif self.target_time - time < warmup:
            delta = self.target_pos - self.start_pos
            v = self.target_velocity(time)
            return -delta * np.linalg.norm(v)
        return 0

class CircularPath(MotionPath):
    def __init__(self, center, start_pos, target_time):
        self.center = np.array(center)
        self.start_pos = np.array(start_pos)
        self.radius = np.linalg.norm([start_pos[0] - center[0], start_pos[1] - center[1]])
        self.start_angle = np.arctan2(start_pos[1] - center[1], start_pos[0] - center[0])
        self.target_time = target_time

    def target_position(self, time):
        x = self.center[0] + self.radius * np.cos(self.start_angle + 2*np.pi / self.target_time)
        y = self.center[1] + self.radius * np.sin(self.start_angle + 2*np.pi / self.target_time)
        z = self.start_pos[2]
        return np.array([x, y, z])

    def target_velocity(self, time):
        circum = np.pi * 2 * self.radius
        return circum / self.target_time \
            * np.array([np.cos(self.start_angle + 2*np.pi / self.target_time + np.pi/2),
                np.sin(self.start_angle + 2*np.pi / self.target_time + np.pi/2),
                0])

    def target_acceleration(self, time):
        circum = np.pi * 2 * self.radius
        return -(self.target_position(time) - self.center) / self.radius * (circum / self.target_time)**2 / self.radius

# You can implement multiple paths a couple ways.  The way I chose when I took
# the class was to create several different paths and pass those into the
# MultiplePaths object, which would determine when to go onto the next path.

class MultiplePaths(MotionPath):
    def __init__(self, paths):
        pass
    def get_current_path(self, time):
        pass