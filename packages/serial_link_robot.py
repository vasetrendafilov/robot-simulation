from packages.utils import *
from packages.dh2urdf import *
import sympy as sp
import numpy as np
import os
import pybullet as p
import time 

class SerialLinkRobot:
    """
    A class to easily create and interact with robotic arm.
    """
    
    def __init__(self, offset = (0,0,0), orientation = (0,0,0,1)):
        self.robot = None
        self.offset = offset
        self.orientation = orientation
        self.reset()
        self.connect()

    
    def reset(self):
        """ Reset the robotic arm data. """
        self.links = []
        self.constraints = []
        self.joint_variables = []
        self.subs_joints = []
        self.subs_additional = []
        if self.robot:
            p.removeBody(self.robot)

    def connect(self):
        p.connect(p.GUI)
        p.configureDebugVisualizer(lightPosition= (0,0,10))
        p.setAdditionalSearchPath(os.getcwd())
        p.resetDebugVisualizerCamera( cameraDistance=15, cameraYaw=15, cameraPitch=-20, cameraTargetPosition=[0,0,0])
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS,0)

    def interact(self):
        self.reset
        subs = self.subs_joints + self.subs_additional
        DH_params = sp.Matrix(self.links).subs(subs).evalf()
        dh2urdf = DH2Urdf(DH_params.tolist(),self.constraints)
        dh2urdf.save_urdf('outfile.urdf')
        self.robot = p.loadURDF("outfile.urdf", self.offset, self.orientation, useFixedBase=True)
        parameters = []
        joints = []
        for i in range(p.getNumJoints(0)):
            joint_info = p.getJointInfo(0,i)
            if joint_info[2] == p.JOINT_PRISMATIC:
                joints.append(i)
                p.resetJointState(0,i,0)
                parameters.append((False,p.addUserDebugParameter("D"+str(i),joint_info[8],joint_info[9],1)))
            if joint_info[2] == p.JOINT_REVOLUTE:
                joints.append(i)
                p.resetJointState(0,i,0)
                parameters.append((True,p.addUserDebugParameter("Theta"+str(i),np.rad2deg(joint_info[8]),np.rad2deg(joint_info[9]),0)))

        timeStep=1./60.
        p.setTimeStep(timeStep)
        p.setGravity(0,-9.8,0)
        
        while (1):
            p.stepSimulation()
            targets = [(np.deg2rad(p.readUserDebugParameter(parameter)) if revolute else p.readUserDebugParameter(parameter)) for revolute,parameter in parameters]
            p.setJointMotorControlArray(0,joints,p.POSITION_CONTROL,targets)
            time.sleep(timeStep)

    def add_revolute_joint(self, theta, d, a, alpha, lower = -np.pi, upper = np.pi,velocity = 2.6, effort = 10):
        """
        Add a revolute joint to the robotic arm according to the DH convention.
    
        :param theta: is the angle of rotation around z-axis.
        :type theta: Symbol
        ...
        :param d: is the displacement along z-axis.
        :type d: number or Symbol
        ...
        :param a: the displacement along x-axis.
        :type a: number or Symbol
        ...
        :param alpha: the angle of rotation around x-axis.
        :type alpha: number or Symbol
        """
        self.links.append((p.JOINT_REVOLUTE, theta, d, a, alpha))
        self.joint_variables.append(theta)
        self.subs_joints.append((theta, 0))
        self.constraints.append((effort, lower, upper, velocity))

    def add_prismatic_joint(self, theta, d, a, alpha, lower = 0.7, upper = 4,velocity = 2.6, effort = 10):
        """
        Add a prismatic joint to the robotic arm according to the DH convention.
    
        :param theta: is the angle of rotation around z-axis.
        :type theta: number or Symbol
        ...
        :param d: is the displacement along z-axis.
        :type d: Symbol
        ...
        :param a: the displacement along x-axis.
        :type a: number or Symbol
        ...
        :param alpha: the angle of rotation around x-axis.
        :type alpha: number or Symbol
        """
        self.links.append((p.JOINT_PRISMATIC, theta, d, a, alpha))
        self.joint_variables.append(d)
        self.subs_joints.append((d, 0))
        self.constraints.append((effort, lower, upper, velocity))
        
    def add_fixed_joint(self, theta, d, a, alpha):
        """
        Add a fixed joint to the robotic arm according to the DH convention.
    
        :param theta: is the angle of rotation around z-axis.
        :type theta: number or Symbol
        ...
        :param d: is the displacement along z-axis.
        :type d: Symbol
        ...
        :param a: the displacement along x-axis.
        :type a: number or Symbol
        ...
        :param alpha: the angle of rotation around x-axis.
        :type alpha: number or Symbol
        """
        self.links.append((p.JOINT_FIXED, theta, d, a, alpha))
        self.constraints.append(None)
    
    def add_subs(self, subs):
        """
        Add the symbol values for plotting purposes.
        
        :param subs: is a list of tuples, each consisted of a symbol and its value.
        :type subs: [(symbol1, value1), (symbol2, value2), ... (symbol3, value3)]
        """
        self.subs_additional = subs
    
    def get_dh_joint_to_joint(self, start_joint, end_joint):
        """
        Get the DH model subsection transformation matrix for the joint id range(start_joint, end_joint).
        
        :param start_joint: is the starting joint id of the desired dh model susbsection.
        :type start_joint: integer
        ...
        :param end_joint: is the final joint id of the desired dh model susbsection.
        :type end_joint: integer
        ...
        :return: DH model subsection transformation matrix for joint id range(start_joint, end_joint).
        """
        pose = hpose3()
        for link in self.links[start_joint:end_joint]:
            joint_type, theta, d, a, alpha = link
            pose = pose * dh_joint_to_joint(theta, d, a, alpha)
        pose.simplify()
        return pose

    def get_dh_matrix(self):
        """ Get the DH model transformation matrix for the whole robotic arm. """
        return self.get_dh_joint_to_joint(start_joint=0, end_joint=len(self.links))
    
    def get_dh_table(self):
        """ Return the DH table intended for visual purposes only. """
        return sp.Matrix(self.links)[:, 1:]
    
    def linear_jacobian(self):
        """ Return the linear jacobian for this robotic arm. """
        linear_jacobian = self.get_dh_matrix()[:3, 3].jacobian(self.joint_variables)
        linear_jacobian.simplify()
        return linear_jacobian
    
    def angular_jacobian(self):
        """ Return the angular jacobian for this robotic arm. """
        pose = hpose3()
        angular_jacobian = sp.Matrix([[], [], []])
        for link in self.links:
            joint_type, theta, d, a, alpha = link
            z_i_m1 = sp.Matrix([0, 0, 0]) if joint_type == 'prismatic' else pose[:3, 2]
            angular_jacobian = sp.Matrix.hstack(angular_jacobian, z_i_m1)
            pose = pose * dh_joint_to_joint(theta, d, a, alpha)
        
        angular_jacobian.simplify()
        return angular_jacobian
    
    def jacobian(self):
        """ Return the jacobian for this robotic arm. """
        return sp.Matrix.vstack(self.linear_jacobian(), self.angular_jacobian())