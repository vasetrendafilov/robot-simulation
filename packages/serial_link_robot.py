from packages.utils import *
from packages.dh2urdf import *
import sympy as sp
import numpy as np
import os
import pybullet as p
import time 

class PybulletSimulation:

    def __init__(self, connection_mode = p.GUI, fps = 60, gravity = (0, 0, -9.8)):
        self.connection_mode = connection_mode
        self.time_step= 1/fps
        self.gravity = gravity

    def configure(self):
        p.setTimeStep(self.time_step)
        p.setGravity(self.gravity[0],self.gravity[1],self.gravity[2])
        p.setAdditionalSearchPath(os.getcwd())
        
        p.configureDebugVisualizer(lightPosition= (0,0,10))
        p.resetDebugVisualizerCamera( cameraDistance=15, cameraYaw=15, cameraPitch=-20, cameraTargetPosition=[0,0,0])
        #p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS,0)

    def connect(self):
        if p.connect(self.connection_mode) != -1: # connected
            self.configure()
            return True
        return False

class SerialLinkRobot:
    """
    A class to easily create and interact with robotic arm.
    """
    
    def __init__(self, offset = (0,0,0), orientation = (0,0,0,1), name = 'my_robot', time_step = None):
        self.name = name
        self.robot = None
        self.offset = offset
        self.orientation = orientation
        #sympy vars
        self.links = []
        self.constraints = []
        self.joint_variables = []
        self.subs_joints = []
        self.subs_additional = []
        #pybullet vars
        self.joint_ids = []
        self.joint_frames = []
        self.use_orientation = False

        if not p.isConnected():
            sim = PybulletSimulation()
            if sim.connect():
                self.time_step = sim.time_step
        else:
            self.time_step = time_step 

    def interact(self, kinematics = 'forward'):
        self.kinematics = kinematics
        subs = self.subs_joints + self.subs_additional
        DH_params = sp.Matrix(self.links).subs(subs).evalf()
        dh2urdf = DH2Urdf(DH_params.tolist(),self.constraints)
        dh2urdf.save_urdf(self.name+'.urdf')
        self.load_robot()
        self.step()
       
    def load_robot(self):
        self.robot = p.loadURDF(self.name+'.urdf', self.offset, self.orientation, useFixedBase=True)
        self.joint_ids = [i for i in range(p.getNumJoints(self.robot)) if (p.getJointInfo(self.robot,i)[2] in [p.JOINT_PRISMATIC, p.JOINT_REVOLUTE])]
        self.add_joint_frame_lines()
        if self.kinematics == 'forward':
            self.add_joint_sliders()
        else:
            self.add_pose_sliders() 

    def add_pose_sliders(self):
        self.pose_sliders = []
        self.pose_sliders.append(p.addUserDebugParameter(f"x",-5,5,1))
        self.pose_sliders.append(p.addUserDebugParameter(f"y",-5,5,1))
        self.pose_sliders.append(p.addUserDebugParameter(f"z",-5,5,2))
        if True not in [item.is_number for sublist in self.get_dh_matrix()[:3,:3].tolist() for item in sublist]:
            self.use_orientation = True
            self.pose_sliders.append(p.addUserDebugParameter(f"R",-180, 180, 0))
            self.pose_sliders.append(p.addUserDebugParameter(f"P",-180, 180, 0))
            self.pose_sliders.append(p.addUserDebugParameter(f"Y",-180, 180, 0))

    def add_joint_sliders(self):
        self.joint_sliders = []
        for i in range(p.getNumJoints(self.robot)):
            joint_info = p.getJointInfo(self.robot,i)
            if joint_info[2] == p.JOINT_PRISMATIC:
                self.joint_sliders.append((False,p.addUserDebugParameter(f"D{i}",joint_info[8],joint_info[9],joint_info[8])))
            if joint_info[2] == p.JOINT_REVOLUTE:
                self.joint_sliders.append((True,p.addUserDebugParameter(f"Theta{i}",np.rad2deg(joint_info[8]),np.rad2deg(joint_info[9]),0)))
    
    def add_joint_frame_lines(self):
        for joint_id in self.joint_ids:
            self.subs_joints[joint_id//2] = self.subs_joints[joint_id//2][0], p.getJointState(self.robot,joint_id)[0]

        for joint in range(0,p.getNumJoints(self.robot),2):
            transform = self.get_dh_joint_to_joint(joint//2,joint//2+1).subs(self.subs_joints+self.subs_additional).evalf()
            line_n, line_o, line_a = frame_lines(transform, 0.8)
            self.joint_frames.append(p.addUserDebugLine([item for sublist in line_n[:,0].tolist() for item in sublist], 
                [item for sublist in line_n[:,1].tolist() for item in sublist],
                parentObjectUniqueId=0,parentLinkIndex=joint,lineColorRGB=(1,0,0), lineWidth = 2))
            self.joint_frames.append(p.addUserDebugLine([item for sublist in line_a[:,0].tolist() for item in sublist],
                [item for sublist in line_a[:,1].tolist() for item in sublist],
                parentObjectUniqueId=0,parentLinkIndex=joint,lineColorRGB=(0,0,1), lineWidth = 2))

    def step(self):
        end_pose_param = p.addUserDebugText("1,1,2", [0.7, 0.7, 0.7],textColorRGB=[1, 0, 0],
            textSize= 1.3,parentObjectUniqueId=self.robot,parentLinkIndex=self.joint_ids[-1]+2)
        while (p.isConnected()):
            p.stepSimulation()
            if self.kinematics == 'forward':
                joint_targets = [(np.deg2rad(p.readUserDebugParameter(parameter)) if revolute else p.readUserDebugParameter(parameter)) for revolute,parameter in self.joint_sliders]
            else:
                position = (p.readUserDebugParameter(self.pose_sliders[0]),p.readUserDebugParameter(self.pose_sliders[1]),p.readUserDebugParameter(self.pose_sliders[2]))
                if self.use_orientation:
                    orientation = p.getQuaternionFromEuler([np.deg2rad(p.readUserDebugParameter(self.pose_sliders[3])),
                        np.deg2rad(p.readUserDebugParameter(self.pose_sliders[4])),
                        np.deg2rad(p.readUserDebugParameter(self.pose_sliders[5]))])
                    joint_targets = p.calculateInverseKinematics(self.robot,self.joint_ids[-1]+2, position, orientation, maxNumIterations=5)
                else:
                    joint_targets = p.calculateInverseKinematics(self.robot,self.joint_ids[-1]+2, position, maxNumIterations=5)

            p.setJointMotorControlArray(self.robot,self.joint_ids,p.POSITION_CONTROL,joint_targets)
            x,y,z = p.getLinkState(0,self.joint_ids[-1]+2)[4]
            p.addUserDebugText(f"{x:.2f},{y:.2f},{z:.2f}", [0.7, 0.7, 0.7],textColorRGB=[1, 0, 0],textSize=1.3,
                parentObjectUniqueId=self.robot,parentLinkIndex=self.joint_ids[-1]+2,replaceItemUniqueId = end_pose_param)
            time.sleep(self.time_step)
    
    def reset(self):
        """ Reset the robotic arm """
        for joint_id in self.joint_ids:
            p.resetJointState(self.robot,joint_id,self.constraints[joint_id//2][1])

    def import_robot(self,file_name = 'my_robot'):
        self.name = file_name
        self.robot = p.loadURDF(self.name+'.urdf', self.offset, self.orientation, useFixedBase=True)
        self.joint_ids = [i for i in range(p.getNumJoints(self.robot)) if (p.getJointInfo(self.robot,i)[2] in [p.JOINT_PRISMATIC, p.JOINT_REVOLUTE])]

        file = open(file_name+'.urdf', 'r')
        lines = file.readlines()
        for i, line in enumerate(lines[1:]):
            if line.strip() == '-->':
                break
            dh_list = line.strip().split(',')
            joint_info = p.getJointInfo(self.robot,self.joint_ids[i])
            if dh_list[0] == 'revolute':
                self.add_revolute_joint(sp.symbols(f'theta{i+1}'),float(dh_list[2]),float(dh_list[3]),float(dh_list[4]),
                    joint_info[8],joint_info[9],joint_info[11],joint_info[10])
            elif dh_list[0] == 'prismatic':
                self.add_prismatic_joint(float(dh_list[1]),sp.symbols(f'd{i+1}'),float(dh_list[3]),float(dh_list[4]),
                    joint_info[8],joint_info[9],joint_info[11],joint_info[10])
            else:
                self.add_fixed_joint(float(dh_list[1]),float(dh_list[2]),float(dh_list[3]),float(dh_list[4]))
        self.add_joint_frame_lines()
        self.step()

    def add_revolute_joint(self, theta, d, a, alpha, lower = -np.pi, upper = np.pi,velocity = 2.6, effort = 10, visual = True):
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
        self.constraints.append((effort, lower, upper, velocity, visual))

    def add_prismatic_joint(self, theta, d, a, alpha, lower = 0.8, upper = 4, velocity = 2.6, effort = 10, visual = True):
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
        self.constraints.append((effort, lower, upper, velocity, visual))
        
    def add_fixed_joint(self, theta, d, a, alpha, visual = True):
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
        self.constraints.append([visual])
    
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
            z_i_m1 = sp.Matrix([0, 0, 0]) if joint_type == p.JOINT_PRISMATIC else pose[:3, 2]
            if joint_type != p.JOINT_FIXED:
                angular_jacobian = sp.Matrix.hstack(angular_jacobian, z_i_m1)
            pose = pose * dh_joint_to_joint(theta, d, a, alpha)
        
        angular_jacobian.simplify()
        return angular_jacobian
    
    def jacobian(self):
        """ Return the jacobian for this robotic arm. """
        return sp.Matrix.vstack(self.linear_jacobian(), self.angular_jacobian())