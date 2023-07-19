#!/usr/bin/env python3

"""
      @author   Carlos Suarez Zapico carlossuarezzapico@gmail.com
 
      @internal
      Created  8-Apr-2022
      Company  Edinburgh Centre for Robotics
      Description iiwa and iiwa classes to handle the vehicle and arm
      motion, F/T sensor 

"""

# Standard imports
import math
import numpy as np

# ROS MSGS
from sensor_msgs.msg import JointState
from geometry_msgs.msg import  TransformStamped
# ROS imports
import rospy
import rospkg
import tf2_ros

# KDL Kinematics library and Track-ik
import PyKDL as kdl
import kdl_parser_py.urdf as kdl_parser


URDF_location_str = rospkg.RosPack().get_path('kmr_iiwa_carlos') + \
    "/kmr_iiwa_urdf/urdf/kmr_iiwa_carlos_kdl.urdf"

# Auxiliary functions
def VAL_SAT(value, maxValue, minValue):
    if (value >= maxValue):
        return maxValue
    elif (value <= minValue):
        return minValue
    else:
        return value


def kdl_to_mat(data):
    mat = np.mat(np.zeros((data.rows(), data.columns())))
    for i in range(data.rows()):
        for j in range(data.columns()):
            mat[i, j] = data[i, j]
    return mat

def skew(x):
    return np.array([[0, -x[2], x[1]],
                     [x[2], 0, -x[0]],
                     [-x[1], x[0], 0]])

class iiwa():

    def __init__(self):
        (status, tree) = kdl_parser.treeFromFile(URDF_location_str)
        print("\n *** Successfully parsed urdf file and constructed kdl tree \n" if status else "Failed to parse urdf file to kdl tree")
        self.chain_iiwa = tree.getChain('base_iiwa', 'adapter')
        self.num_joints_iiwa = self.chain_iiwa.getNrOfJoints()
        print("\n*** The iiwa has %s joints *** \n" % self.num_joints_iiwa)

        #CHANGE JOINT LIMITS
        transiet_interval = 10*(3.14/180)
        # Joint 1
        self.q_joint1_min = -math.pi
        self.q_joint1_max = math.pi
        self.q_joint1_iInf = self.q_joint1_min + transiet_interval
        self.q_joint1_iSup = self.q_joint1_max - transiet_interval
        # Joint 2
        self.q_joint2_min = -math.pi
        self.q_joint2_max = math.pi
        self.q_joint2_iInf = self.q_joint2_min + transiet_interval
        self.q_joint2_iSup = self.q_joint2_max - transiet_interval
        # Joint 3
        self.q_joint3_min = -math.pi
        self.q_joint3_max = math.pi
        self.q_joint3_iInf = self.q_joint3_min + transiet_interval
        self.q_joint3_iSup = self.q_joint3_max - transiet_interval
        # Joint 4
        self.q_joint4_min = -math.pi
        self.q_joint4_max = math.pi
        self.q_joint4_iInf = self.q_joint4_min + transiet_interval
        self.q_joint4_iSup = self.q_joint4_max - transiet_interval
        # Joint 5
        self.q_joint5_min = -math.pi
        self.q_joint5_max = math.pi
        self.q_joint5_iInf = self.q_joint5_min + transiet_interval
        self.q_joint5_iSup = self.q_joint5_max - transiet_interval
        # Joint 6
        self.q_joint6_min = -math.pi
        self.q_joint6_max = math.pi
        self.q_joint6_iInf = self.q_joint6_min + transiet_interval
        self.q_joint6_iSup = self.q_joint6_max - transiet_interval
        # Joint 7
        self.q_joint7_min = -math.pi
        self.q_joint7_max = math.pi
        self.q_joint7_iInf = self.q_joint7_min + transiet_interval
        self.q_joint7_iSup = self.q_joint7_max - transiet_interval


    def jacobian(self, joint):
        # Velocity FK with Jacobian
        jacobian_solver = kdl.ChainJntToJacSolver(self.chain_iiwa)
        J = kdl.Jacobian(self.num_joints_iiwa)
        qKdl = kdl.JntArray(len(joint))
        qKdl[0] = joint[0]
        qKdl[1] = joint[1]
        qKdl[2] = joint[2]
        qKdl[3] = joint[3]
        qKdl[4] = joint[4]
        qKdl[5] = joint[5]
        qKdl[6] = joint[6]
        jacobian_solver.JntToJac(qKdl, J)
        J = kdl_to_mat(J)
        return J

    def local_jacobian(self, joint):
        J = self.jacobian(joint)
        T06 = self.fw_kdl(joint)
        rot = T06[0:3, 0:3]
        t1 = np.concatenate((rot.transpose(), np.zeros(
            (3, 3))), axis=1)
        t2 = np.concatenate((np.zeros((3, 3)), rot.transpose()), axis=1)
        Tinv = np.concatenate((t1,  t2), axis=0)
        locJacobian = np.matmul(Tinv, J)
        return locJacobian


    def fw_kdl(self, joint):
        # NOT FINISHED
        fk = kdl.ChainFkSolverPos_recursive(self.chain_iiwa)
        eeFrame = kdl.Frame()
        qKdl = kdl.JntArray(len(joint))
        qKdl[0] = joint[0]
        qKdl[1] = joint[1]
        qKdl[2] = joint[2]
        qKdl[3] = joint[3]
        qKdl[4] = joint[4]
        qKdl[5] = joint[5]
        qKdl[6] = joint[6]
        fk.JntToCart(qKdl, eeFrame)
        pos = eeFrame.p
        rot = eeFrame.M
        T_base_ee = np.array([[rot[0, 0], rot[0, 1], rot[0, 2], pos[0]],
                              [rot[1, 0], rot[1, 1], rot[1, 2], pos[1]], [rot[2, 0], rot[2, 1], rot[2, 2], pos[2]], [0, 0, 0, 1]])
        return T_base_ee

    def manipulability(self, joint):
        # Velocity FK with Jacobian
        jacobian_solver = kdl.ChainJntToJacSolver(self.chain_iiwa)
        J = kdl.Jacobian(self.num_joints_iiwa)
        qKdl = kdl.JntArray(self.num_joints_iiwa)
        qKdl[0] = joint[0]
        qKdl[1] = joint[1]
        qKdl[2] = joint[2]
        qKdl[3] = joint[3]
        qKdl[4] = joint[4]
        qKdl[5] = joint[5]
        qKdl[6] = joint[6]
        jacobian_solver.JntToJac(qKdl, J)
        J = kdl_to_mat(J)
        # Conditional Number
        cond = np.linalg.cond(J)
        return cond

    def jacobian_vel_fixed_axis(self, joint, desired_ee_velocity):
        Jacobian = self.jacobian(joint)
        invJacobian = np.linalg.pinv(Jacobian)
        q_output_velocity = np.matmul(invJacobian, (desired_ee_velocity))
        return q_output_velocity

    def jacobian_vel_mobiles_axis(self, joint, desired_ee_velocity):
        Jacobian_local = self.local_jacobian(joint)
        invJacobian_local = np.linalg.pinv(Jacobian_local)
        q_output_velocity = np.matmul(invJacobian_local, (desired_ee_velocity))
        return q_output_velocity

    def jac_vel_robust_mobile_axis(self, joint, desired_ee_velocity):
        epsilon = 0.045
        delta_max = 0.06
        Jacobian_local = self.local_jacobian(joint)
        J_T = Jacobian_local.transpose()
        u, s, vh = np.linalg.svd(Jacobian_local, full_matrices=True)
        lowest_sing_value = s[len(s)-1]
        if (lowest_sing_value >= epsilon):
            delta = 0
        else:
            delta = (1 - (lowest_sing_value/epsilon)**2)*(delta_max)**2
        q_output_velocity = np.matmul(np.matmul(np.linalg.pinv(np.matmul(
            J_T, Jacobian_local) + (delta**2)*np.identity(7)), J_T),  (desired_ee_velocity))
        return q_output_velocity


    def jacobian_vel_singularity_robustness(self, joint, desired_ee_velocity):
        epsilon = 0.045
        delta_max = 0.045
        Jacobian = self.jacobian(joint)
        J_T = Jacobian.transpose()
        u, s, vh = np.linalg.svd(Jacobian, full_matrices=True)
        lowest_sing_value = s[len(s)-1]
        if (lowest_sing_value >= epsilon):
            delta = 0
        else:
            delta = (1 - (lowest_sing_value/epsilon)**2)*(delta_max)**2
        q_output_velocity = np.matmul(np.matmul(np.linalg.pinv(np.matmul(
            J_T, Jacobian) + (delta**2)*np.identity(7)), J_T),  (desired_ee_velocity))
        return q_output_velocity

    def weight_joint_limit(q, qmin, qinf, qsup,  qmax):
        ais = 2
        bis = -3*(qmax + qsup)
        cis = 6*qmax*qsup
        dis = qmax**3 - 3*(qmax**2)*qsup
        aii = 2
        bii = -3*(qmin + qinf)
        cii = 6*qmin*qinf
        dii = qmin**3 - 3*(qmin**2)*qinf

        if((q >= qsup) and (q <= qmax)):
            w = (1/(qmax - qsup)**3)*(ais*q**3 + bis*q**2 + cis*q + dis)
        elif((q >= qmin) and (q <= qinf)):
            w = (1/(qmin - qinf)**3)*(aii*q**3 + bii*q**2 + cii*q + dii)
        elif((q > qinf) and (q < qsup)):
            w = 1
        else:
            w = 0
        return w

    def jacobian_vel_joint_limits(self, joint, desired_ee_velocity):
        Jacobian = self.jacobian(joint)

        w1 = self.weight_joint_limit(
            joint[0], self.q_joint1_min, self.q_joint1_iInf, self.q_joint1_iSup,  self.q_joint1_max)
        w2 = self.weight_joint_limit(
            joint[1], self.q_joint2_min, self.q_joint2_iInf, self.q_joint2_iSup,  self.q_joint2_max)
        w3 = self.weight_joint_limit(
            joint[2], self.q_joint3_min, self.q_joint3_iInf, self.q_joint3_iSup,  self.q_joint3_max)
        w4 = self.weight_joint_limit(
            joint[3], self.q_joint4_min, self.q_joint4_iInf, self.q_joint4_iSup,  self.q_joint4_max)
        w5 = self.weight_joint_limit(
            joint[4], self.q_joint5_min, self.q_joint5_iInf, self.q_joint5_iSup,  self.q_joint5_max)
        w6 = self.weight_joint_limit(
            joint[5], self.q_joint6_min, self.q_joint6_iInf, self.q_joint6_iSup,  self.q_joint6_max)
        w7 = self.weight_joint_limit(
            joint[6], self.q_joint7_min, self.q_joint7_iInf, self.q_joint7_iSup,  self.q_joint7_max)

        W_qLimit = np.zeros((7, 7))
        np.fill_diagonal(W_qLimit, [w1, w2, w3, w4, w5, w6, w7])
        invW_qLimit = np.linalg.pinv(W_qLimit)
        J_T = Jacobian.transpose()
        K = 0
        component = np.matmul(np.matmul(invW_qLimit, J_T), np.linalg.pinv(
            np.matmul(np.matmul(Jacobian, invW_qLimit), J_T) + K*np.identity(6)))
        q_output_velocity = np.matmul(component, (desired_ee_velocity))
        return q_output_velocity


class kmr_iiwa():
    def __init__(self):

        (status, tree) = kdl_parser.treeFromFile(URDF_location_str)
        print("\n *** Successfully parsed iiwa urdf file and constructed kdl tree \n" if status else "Failed to parse urdf file to kdl tree")
        self.chain_iiwa = tree.getChain('base_link_yaw', 'adapter')
        self.num_joints_iiwa = self.chain_iiwa.getNrOfJoints()
        print("\n*** The rotation joints in vehicle has %s joints *** \n" % self.num_joints_iiwa)

        self.robot_arm = iiwa()

        # Velocity vehicle
        self.disable_integration = False
        self.velocity_vehicle_sim = np.zeros(3)  # X, Y, YAW
        self.velocity_arm_sim = np.zeros(6)
        self.desired_vel =    np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.secondary_goal = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.penaltyX = 0.0
        self.penaltyY = 0.0
        self.penaltyYaw = 0.0
        self.saturationX = 0.5
        self.saturationY = 0.5
        self.saturationYaw = 0.9


        # Simulation MODEL URDF
        self.kmr_iiwa_joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        self.kmr_iiwa_joint_names = ["X", "Y", "Yaw", "joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6", "joint_a7"]

    def publish_joint_states(self, q_vehicle, q_arm):
        iiwa_joint_states = JointState()
        iiwa_joint_states.position.append(q_vehicle[0])
        iiwa_joint_states.position.append(q_vehicle[1])
        iiwa_joint_states.position.append(q_vehicle[2])
        iiwa_joint_states.position.append(q_arm[0])
        iiwa_joint_states.position.append(q_arm[1])
        iiwa_joint_states.position.append(q_arm[2])
        iiwa_joint_states.position.append(q_arm[3])
        iiwa_joint_states.position.append(q_arm[4])
        iiwa_joint_states.position.append(q_arm[5])
        iiwa_joint_states.position.append(q_arm[6])
        iiwa_joint_states.header.stamp = rospy.get_rostime()
        iiwa_joint_states.name = self.kmr_iiwa_joint_names
        self.kmr_iiwa_joint_state_pub.publish(iiwa_joint_states)
    
    def publish_joint_states(self, q):
        iiwa_joint_states = JointState()
        iiwa_joint_states.position.append(q[0])
        iiwa_joint_states.position.append(q[1])
        iiwa_joint_states.position.append(q[2])
        iiwa_joint_states.position.append(q[3])
        iiwa_joint_states.position.append(q[4])
        iiwa_joint_states.position.append(q[5])
        iiwa_joint_states.position.append(q[6])
        iiwa_joint_states.position.append(q[7])
        iiwa_joint_states.position.append(q[8])
        iiwa_joint_states.position.append(q[9])
        iiwa_joint_states.header.stamp = rospy.get_rostime()
        iiwa_joint_states.name = self.kmr_iiwa_joint_names
        self.kmr_iiwa_joint_state_pub.publish(iiwa_joint_states)

    def manipulability(self, yaw, joint):
        jacobian_solver = kdl.ChainJntToJacSolver(self.chain_iiwa)
        J = kdl.Jacobian(self.num_joints_iiwa)
        qKdl = kdl.JntArray(self.num_joints_iiwa)
        qKdl[0] = yaw
        qKdl[1] = joint[0]
        qKdl[2] = joint[1]
        qKdl[3] = joint[2]
        qKdl[4] = joint[3]
        qKdl[5] = joint[4]
        qKdl[6] = joint[5]
        qKdl[7] = joint[6]
        jacobian_solver.JntToJac(qKdl, J)
        J = self.kdl_to_mat(J)
        # Conditional Number
        cond = np.linalg.cond(J)
        return cond

    def jacobian(self, yaw, joint):
        # Jacobian only using XYZ dof of the vehicle
        jacobian_solver = kdl.ChainJntToJacSolver(self.chain_iiwa)
        J = kdl.Jacobian(self.num_joints_iiwa)
        qKdl = kdl.JntArray(self.num_joints_iiwa)
        qKdl[0] = yaw
        qKdl[1] = joint[0]
        qKdl[2] = joint[1]
        qKdl[3] = joint[2]
        qKdl[4] = joint[3]
        qKdl[5] = joint[4]
        qKdl[6] = joint[5]
        qKdl[7] = joint[6]
        jacobian_solver.JntToJac(qKdl, J)
        J = kdl_to_mat(J)
        v = np.array([[1, 0], [0, 1], [0, 0], [0, 0], [0, 0], [0, 0]])
        vm = np.asmatrix(v)
        J_iiwa = np.concatenate((vm, J), axis=1)
        return J_iiwa
    
    def local_jacobian(self, x, y, yaw, joint):
        J = self.jacobian(yaw, joint)
        T06 = self.fw_kdl(x, y, yaw, joint)
        rot = T06[0:3, 0:3]
        t1 = np.concatenate((rot.transpose(), np.zeros(
            (3, 3))), axis=1)
        t2 = np.concatenate((np.zeros((3, 3)), rot.transpose()), axis=1)
        Tinv = np.concatenate((t1,  t2), axis=0)
        locJacobian = np.matmul(Tinv, J)
        return locJacobian

    def jacobian_vel_fixed_axis(self, yaw, joint, desired_ee_velocity):
        Jacobian= self.jacobian(yaw, joint)
        invJacobian = np.linalg.pinv(Jacobian)
        q_output_velocity = np.matmul(invJacobian, (desired_ee_velocity))
        return q_output_velocity

    def jacobian_vel_mobiles_axis(self, x, y, yaw, joint, desired_ee_velocity):
        Jacobian_local = self.local_jacobian(x, y, yaw, joint)
        invJacobian_local = np.linalg.pinv(Jacobian_local)
        q_output_velocity = np.matmul(invJacobian_local, (desired_ee_velocity))
        return q_output_velocity

    def jac_vel_robust_mobile_axis(self, x, y, yaw, joint, desired_ee_velocity):
        epsilon = 0.045
        delta_max = 0.06
        Jacobian_local = self.local_jacobian(x, y, yaw, joint)
        J_T = Jacobian_local.transpose()
        u, s, vh = np.linalg.svd(Jacobian_local, full_matrices=True)
        lowest_sing_value = s[len(s)-1]
        if (lowest_sing_value >= epsilon):
            delta = 0
        else:
            delta = (1 - (lowest_sing_value/epsilon)**2)*(delta_max)**2
        q_output_velocity = np.matmul(np.matmul(np.linalg.pinv(np.matmul(
            J_T, Jacobian_local) + (delta**2)*np.identity(10)), J_T),  (desired_ee_velocity))
        return q_output_velocity
    
    def redundancy_robust_mobile_axis(self, x, y, yaw, joint, desired_ee_velocity, secondary_goal_vel):
        epsilon = 0.045
        delta_max = 0.06
        Jacobian_local = self.local_jacobian(x, y, yaw, joint)
        J_T = Jacobian_local.transpose()
        u, s, vh = np.linalg.svd(Jacobian_local, full_matrices=True)
        lowest_sing_value = s[len(s)-1]
        if (lowest_sing_value >= epsilon):
            delta = 0
        else:
            delta = (1 - (lowest_sing_value/epsilon)**2)*(delta_max)**2
        component0 = np.matmul(np.linalg.pinv(np.matmul(J_T, Jacobian_local) + (delta**2)*np.identity(10)), J_T)
        component1 = np.matmul(component0,  (desired_ee_velocity))
        component2 = np.matmul(np.identity(10)- np.matmul(component0, Jacobian_local), secondary_goal_vel)
        q_output_velocity = component1 + component2
        return q_output_velocity


    def jacobian_vel(self, yaw, joint, desired_ee_velocity):
        Jacobian = self.jacobian(yaw, joint)
        invJacobian = np.linalg.pinv(Jacobian)
        q_output_velocity = np.matmul(invJacobian, (desired_ee_velocity))
        return q_output_velocity

    def fw_kdl(self, x, y, yaw, joint):
        # NOT FINISHED
        fk = kdl.ChainFkSolverPos_recursive(self.chain_iiwa)
        eeFrame = kdl.Frame()
        qKdl = kdl.JntArray(len(joint)+1)
        qKdl[0] = yaw
        qKdl[1] = joint[0]
        qKdl[2] = joint[1]
        qKdl[3] = joint[2]
        qKdl[4] = joint[3]
        qKdl[5] = joint[4]
        qKdl[6] = joint[5]
        qKdl[7] = joint[6]
        fk.JntToCart(qKdl, eeFrame)
        pos = eeFrame.p
        rot = eeFrame.M
        T_world_vehicle = np.array(
            [[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, 0], [0, 0, 0, 1]])
        
        T_base_ee = np.array([[rot[0, 0], rot[0, 1], rot[0, 2], pos[0]], [
                             rot[1, 0], rot[1, 1], rot[1, 2], pos[1]], [rot[2, 0], rot[2, 1], rot[2, 2], pos[2]], [0, 0, 0, 1]])
        T_world_ee = np.matmul(T_world_vehicle, T_base_ee)
        return T_world_ee

    def weight_joint_limit(self, q, qmin, qinf, qsup,  qmax):
        ais = 2
        bis = -3*(qmax + qsup)
        cis = 6*qmax*qsup
        dis = qmax**3 - 3*(qmax**2)*qsup
        aii = 2
        bii = -3*(qmin + qinf)
        cii = 6*qmin*qinf
        dii = qmin**3 - 3*(qmin**2)*qinf

        if((q >= qsup) and (q <= qmax)):
            w = (1/(qmax - qsup)**3)*(ais*q**3 + bis*q**2 + cis*q + dis)
        elif((q >= qmin) and (q <= qinf)):
            w = (1/(qmin - qinf)**3)*(aii*q**3 + bii*q**2 + cii*q + dii)
        elif((q > qinf) and (q < qsup)):
            w = 1
        else:
            w = 0
        return w

    def jacobian_vel_joint_limits(self, joint, desired_ee_velocity):
        Jacobian = self.jacobian(joint)

        w1 = self.weight_joint_limit(
            joint[0], self.robot_arm.q_joint1_min, self.robot_arm.q_joint1_iInf, self.robot_arm.q_joint1_iSup,  self.robot_arm.q_joint1_max)
        w2 = self.weight_joint_limit(
            joint[1], self.robot_arm.q_joint2_min, self.robot_arm.q_joint2_iInf, self.robot_arm.q_joint2_iSup,  self.robot_arm.q_joint2_max)
        w3 = self.weight_joint_limit(
            joint[2], self.robot_arm.q_joint3_min, self.robot_arm.q_joint3_iInf, self.robot_arm.q_joint3_iSup,  self.robot_arm.q_joint3_max)
        w4 = self.weight_joint_limit(
            joint[3], self.robot_arm.q_joint4_min, self.robot_arm.q_joint4_iInf, self.robot_arm.q_joint4_iSup,  self.robot_arm.q_joint4_max)
        w5 = self.weight_joint_limit(
            joint[4], self.robot_arm.q_joint5_min, self.robot_arm.q_joint5_iInf, self.robot_arm.q_joint5_iSup,  self.robot_arm.q_joint5_max)
        w6 = self.weight_joint_limit(
            joint[5], self.robot_arm.q_joint6_min, self.robot_arm.q_joint6_iInf, self.robot_arm.q_joint6_iSup,  self.robot_arm.q_joint6_max)
        w7 = self.weight_joint_limit(
            joint[6], self.robot_arm.q_joint7_min, self.robot_arm.q_joint7_iInf, self.robot_arm.q_joint7_iSup,  self.robot_arm.q_joint7_max)
        W_qLimit = np.zeros((10, 10))
        np.fill_diagonal(W_qLimit, [1, 1, 1, w1, w2, w3, w4, w5, w6, w7])
        invW_qLimit = np.linalg.pinv(W_qLimit)
        J_T = Jacobian.transpose()
        K = 0
        component = np.matmul(np.matmul(invW_qLimit, J_T), np.linalg.pinv(
            np.matmul(np.matmul(Jacobian, invW_qLimit), J_T) + K*np.identity(6)))
        q_output_velocity = np.matmul(component, (desired_ee_velocity))
        return q_output_velocity


    
    def weights_jointLimits_robust_mobile_axis(self, x, y, yaw, joint, desired_ee_velocity, secondary_goal_vel, penaltyX, penaltyY, penaltyYaw):
        w1 = self.weight_joint_limit(
            joint[0], self.robot_arm.q_joint1_min, self.robot_arm.q_joint1_iInf, self.robot_arm.q_joint1_iSup,  self.robot_arm.q_joint1_max)
        w2 = self.weight_joint_limit(
            joint[1], self.robot_arm.q_joint2_min, self.robot_arm.q_joint2_iInf, self.robot_arm.q_joint2_iSup,  self.robot_arm.q_joint2_max)
        w3 = self.weight_joint_limit(
            joint[2], self.robot_arm.q_joint3_min, self.robot_arm.q_joint3_iInf, self.robot_arm.q_joint3_iSup,  self.robot_arm.q_joint3_max)
        w4 = self.weight_joint_limit(
            joint[3], self.robot_arm.q_joint4_min, self.robot_arm.q_joint4_iInf, self.robot_arm.q_joint4_iSup,  self.robot_arm.q_joint4_max)
        w5 = self.weight_joint_limit(
            joint[4], self.robot_arm.q_joint5_min, self.robot_arm.q_joint5_iInf, self.robot_arm.q_joint5_iSup,  self.robot_arm.q_joint5_max)
        w6 = self.weight_joint_limit(
            joint[5], self.robot_arm.q_joint6_min, self.robot_arm.q_joint6_iInf, self.robot_arm.q_joint6_iSup,  self.robot_arm.q_joint6_max)
        w7 = self.weight_joint_limit(
            joint[6], self.robot_arm.q_joint7_min, self.robot_arm.q_joint7_iInf, self.robot_arm.q_joint7_iSup,  self.robot_arm.q_joint7_max)

        W_qLimit = np.zeros((10, 10))
        np.fill_diagonal(W_qLimit, [penaltyX, penaltyY, penaltyYaw, w1, w2, w3, w4, w5, w6, w7])
        invW_qLimit = np.linalg.pinv(W_qLimit)

        epsilon = 0.045
        delta_max = 0.06
        Jacobian_local = self.local_jacobian(x, y, yaw, joint)
        J_T = Jacobian_local.transpose()
        u, s, vh = np.linalg.svd(Jacobian_local, full_matrices=True)
        lowest_sing_value = s[len(s)-1]
        if (lowest_sing_value >= epsilon):
            delta = 0
        else:
            delta = (1 - (lowest_sing_value/epsilon)**2)*(delta_max)**2
        
        component0 = np.matmul(np.matmul(invW_qLimit, J_T), np.linalg.pinv(np.matmul(np.matmul(Jacobian_local, invW_qLimit), J_T) + (delta**2)*np.identity(6)))
        component1 = np.matmul(component0,  (desired_ee_velocity))
        component2 = np.matmul(np.identity(10)- np.matmul(component0, Jacobian_local), secondary_goal_vel)
        q_output_velocity = component1 + component2
        return q_output_velocity
    
    def weighted_base_robust_mobile_axis(self, x, y, yaw, joint, desired_ee_velocity, secondary_goal_vel, penaltyX, penaltyY, penaltyYaw):
        W_qLimit = np.zeros((10, 10))
        np.fill_diagonal(W_qLimit, [penaltyX, penaltyY, penaltyYaw, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        invW_qLimit = np.linalg.pinv(W_qLimit)
        
        epsilon = 0.045
        delta_max = 0.06
        Jacobian_local = self.local_jacobian(x, y, yaw, joint)
        J_T = Jacobian_local.transpose()
        u, s, vh = np.linalg.svd(Jacobian_local, full_matrices=True)
        lowest_sing_value = s[len(s)-1]
        if (lowest_sing_value >= epsilon):
            delta = 0
        else:
            delta = (1 - (lowest_sing_value/epsilon)**2)*(delta_max)**2
        
        component0 = np.matmul(np.matmul(invW_qLimit, J_T), np.linalg.pinv(np.matmul(np.matmul(Jacobian_local, invW_qLimit), J_T) + (delta**2)*np.identity(6)))
        component1 = np.matmul(component0,  (desired_ee_velocity))
        component2 = np.matmul(np.identity(10)- np.matmul(component0, Jacobian_local), secondary_goal_vel)
        q_output_velocity = component1 + component2
        return q_output_velocity


    def weights_jointLimits_robust_fixed_axis(self, x, y, yaw, joint, desired_ee_velocity, secondary_goal_vel, penaltyX, penaltyY, penaltyYaw):
        w1 = self.weight_joint_limit(
            joint[0], self.robot_arm.q_joint1_min, self.robot_arm.q_joint1_iInf, self.robot_arm.q_joint1_iSup,  self.robot_arm.q_joint1_max)
        w2 = self.weight_joint_limit(
            joint[1], self.robot_arm.q_joint2_min, self.robot_arm.q_joint2_iInf, self.robot_arm.q_joint2_iSup,  self.robot_arm.q_joint2_max)
        w3 = self.weight_joint_limit(
            joint[2], self.robot_arm.q_joint3_min, self.robot_arm.q_joint3_iInf, self.robot_arm.q_joint3_iSup,  self.robot_arm.q_joint3_max)
        w4 = self.weight_joint_limit(
            joint[3], self.robot_arm.q_joint4_min, self.robot_arm.q_joint4_iInf, self.robot_arm.q_joint4_iSup,  self.robot_arm.q_joint4_max)
        w5 = self.weight_joint_limit(
            joint[4], self.robot_arm.q_joint5_min, self.robot_arm.q_joint5_iInf, self.robot_arm.q_joint5_iSup,  self.robot_arm.q_joint5_max)
        w6 = self.weight_joint_limit(
            joint[5], self.robot_arm.q_joint6_min, self.robot_arm.q_joint6_iInf, self.robot_arm.q_joint6_iSup,  self.robot_arm.q_joint6_max)
        w7 = self.weight_joint_limit(
            joint[6], self.robot_arm.q_joint7_min, self.robot_arm.q_joint7_iInf, self.robot_arm.q_joint7_iSup,  self.robot_arm.q_joint7_max)

        W_qLimit = np.zeros((10, 10))
        np.fill_diagonal(W_qLimit, [penaltyX, penaltyY, penaltyYaw, w1, w2, w3, w4, w5, w6, w7])
        invW_qLimit = np.linalg.pinv(W_qLimit)

        epsilon = 0.045
        delta_max = 0.06
        Jacobian = self.jacobian( yaw, joint)
        J_T = Jacobian.transpose()
        u, s, vh = np.linalg.svd(Jacobian, full_matrices=True)
        lowest_sing_value = s[len(s)-1]
        if (lowest_sing_value >= epsilon):
            delta = 0
        else:
            delta = (1 - (lowest_sing_value/epsilon)**2)*(delta_max)**2
        
        component0 = np.matmul(np.matmul(invW_qLimit, J_T), np.linalg.pinv(np.matmul(np.matmul(Jacobian, invW_qLimit), J_T) + (delta**2)*np.identity(6)))
        component1 = np.matmul(component0,  (desired_ee_velocity))
        component2 = np.matmul(np.identity(10)- np.matmul(component0, Jacobian), secondary_goal_vel)
        q_output_velocity = component1 + component2
        return q_output_velocity

    
    def retrive_transform(child_link, base_link, tfBuffer):
        trans = TransformStamped()
        success_retrieved = False
        try:
            trans = tfBuffer.lookup_transform( child_link, base_link, rospy.Time())
            success_retrieved = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            success_retrieved = False      
        return success_retrieved, trans


    
    