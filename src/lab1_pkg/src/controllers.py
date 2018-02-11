import rospy
import pdb
import numpy as np
from utils import *
from geometry_msgs.msg import PoseStamped
"""
Starter script for lab1.
Author: Chris Correa
"""
class Controller:
    def step_path(self, path, t):
        raise NotImplementedError

    def execute_path(self, path, finished, timeout=None, log=False):
        start_t = rospy.Time.now()
        times = list()
        actual_positions = list()
        actual_velocities = list()
        target_positions = list()
        target_velocities = list()
        r = rospy.Rate(200)
        while True:
            t = (rospy.Time.now() - start_t).to_sec()
            if timeout is not None and t >= timeout:
                return False
            self.step_path(path, t)
            if log:
                times.append(t)
                actual_position = self.limb.endpoint_pose()['position']
                actual_positions.append([actual_position.x, actual_position.y, actual_position.z])
                joint_names = self.limb.joint_names()
                joint_v = self.limb.joint_velocities()
                joint_v = np.array([joint_v[joint_name] for joint_name in joint_names])
                j = self.kin.jacobian()
                end_v = np.matmul(j, joint_v)
                actual_velocities.append(end_v[0, :3].tolist()[0])
                target_positions.append(path.target_position(t))
                target_velocities.append(path.target_velocity(t))
            if finished is not None and finished(self, path, t):
                break
            r.sleep()

        if log:
            import matplotlib.pyplot as plt

            np_actual_positions = np.array(actual_positions)
            np_actual_velocities = np.array(actual_velocities)
            # np_actual_positions = np.zeros((len(times), 3))
            # np_actual_velocities = np.zeros((len(times), 3))
            # for i in range(len(times)):
            #     # print actual_positions[i]
            #     actual_positions_dict = dict((joint, actual_positions[i][j]) for j, joint in enumerate(self.limb.joint_names()))
            #     print "dictionary version", actual_positions_dict
            #     np_actual_positions[i] = self.kin.forward_position_kinematics(joint_values=actual_positions_dict)[:3]
            #     np_actual_velocities[i] = self.kin.jacobian(joint_values=actual_positions_dict)[:3].dot(actual_velocities[i])
            target_positions = np.array(target_positions)
            target_velocities = np.array(target_velocities)
            plt.figure()
            # print len(times), actual_positions.shape()
            plt.subplot(3,2,1)
            plt.plot(times, np_actual_positions[:,0], label='Actual')
            plt.plot(times, target_positions[:,0], label='Desired')
            plt.xlabel("Time (t)")
            plt.ylabel("X Position Error")

            plt.subplot(3,2,2)
            plt.plot(times, np_actual_velocities[:,0], label='Actual')
            plt.plot(times, target_velocities[:,0], label='Desired')
            plt.xlabel("Time (t)")
            plt.ylabel("X Velocity Error")

            plt.subplot(3,2,3)
            plt.plot(times, np_actual_positions[:,1], label='Actual')
            plt.plot(times, target_positions[:,1], label='Desired')
            plt.xlabel("time (t)")
            plt.ylabel("Y Position Error")

            plt.subplot(3,2,4)
            plt.plot(times, np_actual_velocities[:,1], label='Actual')
            plt.plot(times, target_velocities[:,1], label='Desired')
            plt.xlabel("Time (t)")
            plt.ylabel("Y Velocity Error")

            plt.subplot(3,2,5)
            plt.plot(times, np_actual_positions[:,2], label='Actual')
            plt.plot(times, target_positions[:,2], label='Desired')
            plt.xlabel("time (t)")
            plt.ylabel("Z Position Error")

            plt.subplot(3,2,6)
            plt.plot(times, np_actual_velocities[:,2], label='Actual')
            plt.plot(times, target_velocities[:,2], label='Desired')
            plt.xlabel("Time (t)")
            plt.ylabel("Z Velocity Error")

            plt.show()

        return True

class PDWorkspaceVelocityController(Controller):
    def __init__(self, limb, kin, Kp, Kv):
        self.limb = limb
        self.kin = kin
        self.Kp = Kp
        self.Kv = Kv

    def step_path(self, path, t):
        targ_x = path.target_position(t)
        cur_x = self.limb.endpoint_pose()['position']

        targ_v = path.target_velocity(t)
        cur_v = self.limb.endpoint_velocity()['linear']

        delta_x = cur_x - targ_x
        delta_v = cur_v - targ_v

        control_v = -self.Kp*delta_x - self.Kv*delta_v
        control_v = np.pad(control_v, (0, 3), 'constant')
        inv_j = self.kin.jacobian_pseudo_inverse()
        joint_v = np.matmul(inv_j, control_v).tolist()[0]

        joint_names = self.limb.joint_names()
        joint_dict = {joint_name: jv for (joint_name, jv) in zip(joint_names, joint_v)}
        self.limb.set_joint_velocities(joint_dict)

class PDJointVelocityController(Controller):
    def __init__(self, limb, kin, Kp, Kv):
        self.limb = limb
        self.kin = kin
        self.Kp = Kp
        self.Kv = Kv

    def step_path(self, path, t):
        # YOUR CODE HERE
        joint_names = self.limb.joint_names()
        cur_angles = self.limb.joint_angles()
        cur_pos = [cur_angles[joint_name] for joint_name in joint_names] # list of current angles

        targ_x = path.target_position(t)
        orientation = self.limb.endpoint_pose()['orientation']
        targ_pos = self.kin.inverse_kinematics(targ_x, orientation, cur_pos).tolist() # list of target joint angles

        cur_v = self.limb.joint_velocities()
        cur_vel = [cur_v[joint_name] for joint_name in joint_names] # list of current joint velocities

        targ_v = path.target_velocity(t)
        targ_v = np.pad(targ_v, (0, 3), 'constant')
        inv_j = self.kin.jacobian_pseudo_inverse()
        targ_vel = np.matmul(inv_j, targ_v).tolist()[0] # list of target joint velocities

        delta_pos = np.array([cur - targ for (cur, targ) in zip(cur_pos, targ_pos)])
        delta_vel = np.array([cur - targ for (cur, targ) in zip(cur_vel, targ_vel)])

        joint_v = -self.Kp*delta_pos - self.Kv*delta_vel

        self.limb.set_joint_velocities({joint_name: jv for (joint_name, jv) in zip(joint_names, joint_v.tolist())})

class PDJointTorqueController(Controller):
    def __init__(self, limb, kin, Kp, Kv):
        self.limb = limb
        self.kin = kin
        self.Kp = Kp
        self.Kv = Kv

    def step_path(self, path, t):
        joint_names = self.limb.joint_names()
        cur_angles = self.limb.joint_angles()
        cur_pos = [cur_angles[joint_name] for joint_name in joint_names] # list of current joint angles

        targ_x = path.target_position(t)
        orientation = self.limb.endpoint_pose()['orientation']
        targ_pos = self.kin.inverse_kinematics(targ_x, orientation, cur_pos).tolist() # list of target joint angles

        cur_v = self.limb.joint_velocities()
        cur_vel = [cur_v[joint_name] for joint_name in joint_names] # list of current joint velocities

        targ_v = path.target_velocity(t)
        targ_v = np.pad(targ_v, (0, 3), 'constant')
        inv_j = self.kin.jacobian_pseudo_inverse()
        targ_vel = np.matmul(inv_j, targ_v).tolist()[0] # list of target joint velocities

        delta_pos = np.array([cur - targ for (cur, targ) in zip(cur_pos, targ_pos)])
        delta_vel = np.array([cur - targ for (cur, targ) in zip(cur_vel, targ_vel)])

        joint_v = -self.Kp*delta_pos - self.Kv*delta_vel # backwards correction term
        # print('delta_pos', delta_pos)
        # print('joint_names', joint_names)
        # print('joint_v', joint_v)
        # print('forces', np.matmul(np.linalg.pinv(self.kin.jacobian().T), joint_v))
        
        Ms = self.kin.inertia().tolist()

        targ_a = path.target_acceleration(t)
        targ_a = np.pad(targ_a, (0, 3), 'constant')
        targ_acc = np.matmul(inv_j, targ_a).tolist()[0]
        torque_dict = {joint_name: np.dot(M, targ_acc) + jv for (joint_name, jv, M) in zip(joint_names, joint_v.tolist(), Ms)}
        # print(torque_dict)
        self.limb.set_joint_torques(torque_dict)
