#!/usr/bin/env python
"""
Starter script for lab1.
Author: Chris Correa
"""
import copy
import rospy
import sys
import argparse, pdb

import baxter_interface
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped

# import IPython
import tf
import time
import numpy as np
from utils import *
from baxter_pykdl import baxter_kinematics
import signal
from controllers import PDWorkspaceVelocityController, PDJointVelocityController, PDJointTorqueController
from paths import LinearPath, CircularPath, MultiplePaths

def lookup_tag(tag_number):
    listener = tf.TransformListener()
    from_frame = 'base'
    to_frame = 'ar_marker_{}'.format(tag_number)
    while not rospy.is_shutdown():
        try: 
            # if not listener.frameExists(from_frame) or not listener.frameExists(to_frame):
            #     print 'Frames not found'
            #     print 'Did you place AR marker {} within view of the baxter left hand camera?'.format(tag_number)
            #     exit(0)     
            t = listener.getLatestCommonTime(from_frame, to_frame)
            tag_pos, _ = listener.lookupTransform(from_frame, to_frame, t)
            break
        except:
            continue
    return tag_pos[0:3] # only return x, y, z

if __name__ == "__main__":
    def sigint_handler(signal, frame):
        sys.exit(0)
    signal.signal(signal.SIGINT, sigint_handler)
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_node')
    time.sleep(1)

    parser = argparse.ArgumentParser()
    parser.add_argument('-ar_marker', '-ar', type=float, default=5)
    parser.add_argument('-controller', '-c', type=str, default='velocity') # workspace, velocity, or torque
    parser.add_argument('-arm', '-a', type=str, default='left') # or left
    args = parser.parse_args()

    limb = baxter_interface.Limb(args.arm)
    kin = baxter_kinematics('left')

    if args.controller == 'workspace':
        # YOUR CODE HERE
        Kp = 0.75
        Kv = 0.75
        controller = PDWorkspaceVelocityController(limb, kin, Kp, Kv)
    if args.controller == 'velocity':
        # YOUR CODE HERE
        Kp = 0.75
        Kv = 0.75
        controller = PDJointVelocityController(limb, kin, Kp, Kv)
    if args.controller == 'torque':
        # YOUR CODE HERE
        Kp = np.array([2, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5])
        Kv = 0
        controller = PDJointTorqueController(limb, kin, Kp, Kv)

    raw_input('Press <Enter> to start')

    execute_multiple_paths = True

    if not execute_multiple_paths:
        tag_pos = lookup_tag(args.ar_marker)

        cur_pos = limb.endpoint_pose()['position']
        target_time = 5
        # path = LinearPath(cur_pos, tag_pos + np.array([0, 0, 0.15], target_time)
        path = CircularPath(cur_pos, tag_pos, target_time)

        def finished(c, p, t):
            epsilon = 0.075
            cur_pos = limb.endpoint_pose()['position']
            if isinstance(p, CircularPath):
                if np.linalg.norm(cur_pos - p.start_pos) < epsilon and t > p.target_time*0.5:
                    return True
                return False
            elif isinstance(p, LinearPath):
                if np.linalg.norm(cur_pos - p.target_pos) < epsilon:
                    return True
                return False
            else:
                print("dont recognize path type")
                return True

        controller.execute_path(path, finished, timeout=target_time*1.2, log=False)
    else:
        tag_numbers = [4, 5, 6, 8]
        z_offset = 0.15
        time_per_path = 5.0
        tag_positions = []
        for tag in tag_numbers:
            tag_positions.append(lookup_tag(tag))
            print("found " + str(tag))
        paths = []
        last_pos = limb.endpoint_pose()['position']
        for tag_pos in tag_positions:
            tag_pos[2] += z_offset
            paths.append(LinearPath(last_pos, tag_pos, time_per_path))
            last_pos = tag_pos

        multiple_path = MultiplePaths(paths, time_per_path)
        controller.execute_path(multiple_path, None, timeout=len(tag_numbers)*time_per_path*1.2, log=False)
