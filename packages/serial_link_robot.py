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
    
    def __init__(self, offset = (0,0,0), orientation = (0,0,0,1), name = 'my_robot', time_step = None, ik_function = None):
        self.name = name
        self.robot = None
        self.offset = offset
        self.orientation = orientation
        self.ik_function = ik_function
        #sympy vars
        self.links = []
        self.constraints = []
        self.joint_variables = []
        self.subs_joints = []
        self.subs_additional = []
        #pybullet vars
        self.joint_ids = []
        self.joint_frames = []
        self.is_imported = False
        self.use_orientation_ik = False
        self.pos_param, self.orn_param = None,None
        # drawing trajectory
        self.hasPrevPose = False
        self.prevPose1 = self.prevPose2 = [0,0,0]

        if not p.isConnected():
            sim = PybulletSimulation()
            if sim.connect():
                self.time_step = sim.time_step
        else:
            self.time_step = time_step


    def write_text(self,text,ofset):
        for letter in text:
            self.write_letter(letter,ofset)
            ofset[1]-=0.25

    def write_letter(self,letter,pos):
        # frame of the letter i 1x1 
        if letter == 'T':
            #(pocx, pocy,po so da odi i koja nasoka, kolku da otide)
            comands = [(0,1,'1 0',np.arange(0.0,1.0,0.05)),(0.5,0,'0 1',np.arange(0.0,1.0,0.05))]
        elif letter == 'E':
            comands = [(0,1,'1 0',np.arange(0.0,1.0,0.05)),(0,0.5,'1 0',np.arange(0.0,1.0,0.05)),
            (0,0,'1 0',np.arange(0.0,1.0,0.05)),(0,0,'0 1',np.arange(0.0,1.0,0.05))]
        elif letter == 'I':
            comands = [(0.5,0,'0 1',np.arange(0.0,1.0,0.05)),(0.2,0,'1 0',np.arange(0.0,0.6,0.05)),(0.2,1,'1 0',np.arange(0.0,0.6,0.05))]
        elif letter == 'L':
            comands = [(0,0,'0 1',np.arange(0.0,1.0,0.05)),(0,0,'1 0',np.arange(0.0,1.0,0.05))]
        elif letter == 'H':
            comands = [(0,0,'0 1',np.arange(0.0,1.0,0.05)),(1,0,'0 1',np.arange(0.0,1.0,0.05)),(0,0.5,'1 0',np.arange(0.0,1.0,0.05))]
        elif letter == 'F':
            self.move((pos[0],    pos[1],pos[2], 0,0,0),(2+pos[0],pos[1],pos[2], 0,0,0),'linear',30)
            self.move((2+pos[0],pos[1],pos[2], 0,0,0),(2+pos[0],-1.5+pos[1],pos[2], 0,0,0),'linear',15)
            self.move((1+pos[0],pos[1],pos[2], 0,0,0),(1+pos[0],-1.5+pos[1],pos[2], 0,0,0),'linear',15)
            return True
        elif letter == 'N':
            comands = [(0,0,'0 1',np.arange(0.0,1.0,0.05)),(1,0,'0 1',np.arange(0.0,1.0,0.05)),(0,1,'1 -1',np.arange(0.0,1.0,0.05))]
        elif letter == 'M':
            comands = [(0,0,'0 1',np.arange(0.0,1.0,0.05)),(1,0,'0 1',np.arange(0.0,1.0,0.05)),
            (0,1,'1 -1',np.arange(0.0,0.5,0.05)),(0.5,0.5,'1 1',np.arange(0.0,0.5,0.05))]
        elif letter == 'X':
            comands = [(0,1,'1 -1',np.arange(0.0,1.0,0.05)),(0,0,'1 1',np.arange(0.0,1.0,0.05))]
        elif letter == 'V':
            comands = [(0,1,'0.5 -1',np.arange(0.0,1,0.05)),(0.5,0,'0.5 1',np.arange(0.0,1,0.05))]
        elif letter == 'A':
            comands = [(0,0,'0.5 1',np.arange(0.0,1,0.05)),(0.5,1,'0.5 -1',np.arange(0.0,1,0.05)),(0.3,0.5,'1 0',np.arange(0.0,0.5,0.05))]
        elif letter == 'D':
            self.move((pos[0],pos[1],pos[2], 0,0,180),(0.2+pos[0],pos[1],pos[2], 0,0,180),'linear',20,(0.2,0.15,1,1))
            self.move((0.2+pos[0],pos[1],pos[2], 0,0,180),(0.2+pos[0],-0.03+pos[1],pos[2], 0,0,180),'linear',5,(0.2,0.15,1,1))
            self.move((0.2+pos[0],-0.03+pos[1],pos[2], 0,0,180),(pos[0],-0.03+pos[1],pos[2], 0,0,180),'circular',40,(0.1,0.1,0,1))
            self.move((pos[0],-0.03+pos[1],pos[2], 0,0,180),(pos[0],pos[1],pos[2], 0,0,180),'linear',5,(0.2,0.15,1,1))
            return True
               
    def move2point(self,position,orientation=None, converge = 10, dynamically = True):
        """
        Move to a desired position and rotation of robot. 
        Parameters
        ----------
        position: tuple()
            Set (x,y,z) coordinates.
        rotation: tuple()
            Set (R,P,Y) angles.
        """
      
        steps = 0 # temporary not good with dynamically
        while (p.isConnected()):
            p.stepSimulation() 
            steps+=1
        
            if self.ik_function:
                joint_targets = self.ik_function(position, orientation)
            else:  
                joint_targets = p.calculateInverseKinematics(self.robot,self.joint_ids[-1]+2, position, 
                    p.getQuaternionFromEuler(orientation), maxNumIterations=5)
            
            if dynamically:
                p.setJointMotorControlArray(self.robot,self.joint_ids,p.POSITION_CONTROL,joint_targets,
                targetVelocities = [0]*len(joint_targets),forces=[500]*len(joint_targets))#,positionGains = [1]*len(joint_targets),velocityGains = [0.1]*len(joint_targets))
            else:
                for i,joint_id in enumerate(self.joint_ids):
                    p.resetJointState(self.robot,joint_id,joint_targets[i])
            
            pos = p.getLinkState(0,self.joint_ids[-1]+2,computeForwardKinematics = True)[4]
            self.display_pos_and_orn(p.getLinkState(self.robot,self.joint_ids[-1]+2))

            self.draw_trajectory(position,pos)
            time.sleep(self.time_step)

            if dynamically and np.allclose([p.getJointState(self.robot,joint_id)[0] for joint_id in self.joint_ids],joint_targets,0.01*(steps/converge)):
                break
            elif not dynamically and steps > 1:
                break
        return True
        

    def move(self,start,end,interpolation='linear',steps=30,param=None,closed=False):
        """
        Move to a desired position and rotation of robot. 
        Parameters
        ----------
        position: tuple()
            Set (x,y,z) cordinates.
        rotation: tuple()
            Set (R,P,Y) angles.
        interpolation: string
            Oprions: linear / circular.
        steps: int
            Number of points generated.
        param: tuple()
            Set (a,b,res_num,side):
            - a,b are aparameters for the elipse
            - res_num is to chose witch solution you like 0 or 1
            - side is to chose witch path to take 0 or 1
        closed: bool
            To draw the whole elipse
        """
        self.hasPrevPose = False
        x1,y1,z1,R1,P1,Y1 = start
        x2,y2,z2,R2,P2,Y2 = end
        if interpolation == 'linear':
            for x,y,z,R,P,Y in zip(np.linspace(x1,x2,steps),np.linspace(y1,y2,steps),np.linspace(z1,z2,steps),
                                   np.linspace(R1,R2,steps),np.linspace(P1,P2,steps),np.linspace(Y1,Y2,steps)):
                if not self.move2point((x,y,z),(R,P,Y)):
                    return False
        elif interpolation == 'circular' and param != None:
            xs,ys = sp.symbols('xs,ys')
            a,b,res_num,side = param
    
            eq1 = sp.Eq((xs-x1)**2/a**2+(ys-y1)**2/b**2,1)
            eq2 = sp.Eq((xs-x2)**2/a**2+(ys-y2)**2/b**2,1)
            results = sp.solve([eq1,eq2],(xs,ys))
            if not all(x[0].is_real or x[1].is_real for x in results): 
                print("Нема решение на центарот за тие a,b")
                return False

            x_center,y_center = results[res_num]
            res_t = [-np.arccos(float((x1-x_center)/a)) + 2*np.pi, np.arccos(float((x1-x_center)/a))]
            start_angle = [t for t in res_t if np.around(float(y_center + b*np.sin(t)),3) == np.around(y1,3)][0]
            res_t = [-np.arccos(float((x2-x_center)/a)) + 2*np.pi, np.arccos(float((x2-x_center)/a))]
            end_angle = [t for t in res_t if np.around(float(y_center + b*np.sin(t)),3) == np.around(y2,3)][0]
            
            if side:
                t = np.linspace(start_angle, end_angle, steps)
            else:
                if start_angle < end_angle:
                    t = np.hstack((np.linspace(start_angle, 0,int(steps*start_angle/(2*np.pi-end_angle+start_angle)) ),
                    np.linspace(2*np.pi, end_angle, int(steps*(2*np.pi-end_angle)/(2*np.pi-end_angle+start_angle)) )))
                else:
                    t = np.hstack((np.linspace(start_angle, 2*np.pi, int(steps*(2*np.pi-start_angle)/(2*np.pi-start_angle+end_angle)) ),
                    np.linspace(0, end_angle, int(steps*end_angle/(2*np.pi-start_angle+end_angle)) )))
            
            if closed:
                start_angle, end_angle = (0,2*np.pi) if dir else (2*np.pi,0)

            for x,y,z,R,P,Y in zip(x_center + a*np.cos(t),y_center + b*np.sin(t),np.linspace(z1, z2, steps),
                                np.linspace(R1,R2,steps),np.linspace(P1,P2,steps),np.linspace(Y1,Y2,steps)):
                if not self.move2point((x,y,z),(R,P,Y)):
                    return False 

    def interact(self, kinematics = 'forward', use_orientation_ik = False, dynamically = True):
        self.kinematics = kinematics
        self.use_orientation_ik = use_orientation_ik
        self.load_robot()
        if self.kinematics == 'forward':
            self.add_joint_sliders()
        else:
            self.add_pose_sliders() 
        self.step(dynamically)
  
    def load_robot(self):
        if not self.is_imported:
            subs = self.subs_joints + self.subs_additional
            DH_params = sp.Matrix(self.links).subs(subs).evalf()
            dh2urdf = DH2Urdf(DH_params.tolist(),self.constraints)
            dh2urdf.save_urdf(self.name+'.urdf')
        self.robot = p.loadURDF(self.name+'.urdf', self.offset, self.orientation, useFixedBase=True)
        self.joint_ids = [i for i in range(p.getNumJoints(self.robot)) if (p.getJointInfo(self.robot,i)[2] in [p.JOINT_PRISMATIC, p.JOINT_REVOLUTE])]
        self.add_joint_frame_lines()
        self.reset_joints()
    
    def display_pos_and_orn(self,link):
        x,y,z = link[4]
        R,P,Y = np.rad2deg(p.getEulerFromQuaternion(link[5]))
        if not self.pos_param and not self.orn_param:
            self.pos_param = p.addUserDebugText(f"{x:.2f},{y:.2f},{z:.2f}", [0.7, 0.7, 0.7],textColorRGB=[1, 0, 0],
                textSize= 1.3,parentObjectUniqueId=self.robot,parentLinkIndex=self.joint_ids[-1]+2)
            self.orn_param = p.addUserDebugText(f"{R:.1f},{P:.1f},{Y:.1f}", [0.7, 0.7, 1],textColorRGB=[1, 0, 0],
                textSize= 1.3,parentObjectUniqueId=self.robot,parentLinkIndex=self.joint_ids[-1]+2)

        p.addUserDebugText(f"{x:.2f},{y:.2f},{z:.2f}", [0.7, 0.7, 0.7],textColorRGB=[1, 0, 0],textSize= 1.3,
            parentObjectUniqueId=self.robot,parentLinkIndex=self.joint_ids[-1]+2, replaceItemUniqueId = self.pos_param)
        p.addUserDebugText(f"{R:.1f},{P:.1f},{Y:.1f}", [0.7, 0.7, 1],textColorRGB=[1, 0, 0],textSize= 1.3,
            parentObjectUniqueId=self.robot,parentLinkIndex=self.joint_ids[-1]+2, replaceItemUniqueId = self.orn_param)


    def draw_trajectory(self,target,current, lifeTime = 15):
        if (self.hasPrevPose):
            p.addUserDebugLine(self.prevPose1, target, [0, 0, 0.3], 1, lifeTime)
            p.addUserDebugLine(self.prevPose2, current, [1, 0, 0] , 1, lifeTime)
        self.prevPose1 = target
        self.prevPose2 = current
        self.hasPrevPose = True 

    def add_pose_sliders(self):
        self.pose_sliders = []
        self.pose_sliders.append(p.addUserDebugParameter(f"x",-5,5,1))
        self.pose_sliders.append(p.addUserDebugParameter(f"y",-5,5,1))
        self.pose_sliders.append(p.addUserDebugParameter(f"z",-5,5,2))
        if self.use_orientation_ik:
            #if True not in [item.is_number for sublist in self.get_dh_matrix()[:3,:3].tolist() for item in sublist]: ubavo no sporo 
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

        for joint in range(0,p.getNumJoints(self.robot)-2,2):
            transform = self.get_dh_joint_to_joint(joint//2,joint//2+1).subs(self.subs_joints+self.subs_additional).evalf()
            line_n, line_o, line_a = frame_lines(transform, 0.8)
            self.joint_frames.append(p.addUserDebugLine([item for sublist in line_n[:,0].tolist() for item in sublist], 
                [item for sublist in line_n[:,1].tolist() for item in sublist],parentObjectUniqueId=0,parentLinkIndex=joint,
                lineColorRGB=(1,0,0) if p.getNumJoints(self.robot)-3 != joint else (0,1,1), lineWidth = 2))
            self.joint_frames.append(p.addUserDebugLine([item for sublist in line_a[:,0].tolist() for item in sublist],
                [item for sublist in line_a[:,1].tolist() for item in sublist],parentObjectUniqueId=0,parentLinkIndex=joint,
                lineColorRGB=(0,0,1) if p.getNumJoints(self.robot)-3 != joint else (1,1,0), lineWidth = 2))

    def step(self,dynamically = True):
        while (p.isConnected()):
            p.stepSimulation()
            if self.kinematics == 'forward':
                joint_targets = [(np.deg2rad(p.readUserDebugParameter(parameter)) if revolute else p.readUserDebugParameter(parameter)) for revolute,parameter in self.joint_sliders]
            else:
                position = (p.readUserDebugParameter(self.pose_sliders[0]),p.readUserDebugParameter(self.pose_sliders[1]),p.readUserDebugParameter(self.pose_sliders[2]))
                if self.use_orientation_ik:
                    orientation = p.getQuaternionFromEuler([np.deg2rad(p.readUserDebugParameter(self.pose_sliders[3])),
                        np.deg2rad(p.readUserDebugParameter(self.pose_sliders[4])),
                        np.deg2rad(p.readUserDebugParameter(self.pose_sliders[5]))])
                
                if self.ik_function:
                    joint_targets = self.ik_function(position, orientation if self.use_orientation_ik else None)
                else:
                    joint_targets = p.calculateInverseKinematics(self.robot,self.joint_ids[-1]+2, position, 
                        orientation if self.use_orientation_ik else None, maxNumIterations=5)
                
            if dynamically:
                p.setJointMotorControlArray(self.robot,self.joint_ids,p.POSITION_CONTROL,joint_targets)#,forces=[500]*len(joint_targets))
            else:
                for i,joint_id in enumerate(self.joint_ids):
                    p.resetJointState(self.robot,joint_id,joint_targets[i])

            self.display_pos_and_orn(p.getLinkState(self.robot,self.joint_ids[-1]+2))
            time.sleep(self.time_step)
    
    def reset_joints(self):
        """ Reset the robotic arm """
        for joint_id in self.joint_ids:
            p.resetJointState(self.robot,joint_id,
            self.constraints[joint_id//2][1] if self.links[joint_id//2][0] == p.JOINT_PRISMATIC else 0) 

    def import_robot(self,file_name = 'my_robot'):
        self.name = file_name
        self.is_imported = True
        file = open(file_name+'.urdf', 'r')
        lines = file.readlines()
        for i, line in enumerate(lines[2:]):
            if line.strip() == '-->':
                break
            dh_list = line.strip().split()
            if dh_list[0] == 'revolute':
                self.add_revolute_joint(sp.symbols(f'theta{i+1}'),float(dh_list[2]),float(dh_list[3]),float(dh_list[4]),
                    float(dh_list[6]),float(dh_list[7]),float(dh_list[8]),float(dh_list[5]),int(dh_list[9]))
            elif dh_list[0] == 'prismatic':
                self.add_prismatic_joint(float(dh_list[1]),sp.symbols(f'd{i+1}'),float(dh_list[3]),float(dh_list[4]),
                    float(dh_list[6]),float(dh_list[7]),float(dh_list[8]),float(dh_list[5]),int(dh_list[9]))
            else:
                self.add_fixed_joint(float(dh_list[1]),float(dh_list[2]),float(dh_list[3]),float(dh_list[4]),int(dh_list[9]))

    def add_revolute_joint(self, theta, d, a, alpha, lower = -180, upper = 180, velocity = 2.6, effort = 10, visual = True):
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
        self.links.append([p.JOINT_REVOLUTE, theta, d, a, alpha])
        self.joint_variables.append(theta)
        self.subs_joints.append((theta, 0))
        self.constraints.append([effort, np.deg2rad(lower), np.deg2rad(upper), velocity, visual])

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
        self.links.append([p.JOINT_PRISMATIC, theta, d, a, alpha])
        self.joint_variables.append(d)
        self.subs_joints.append((d, 0))
        self.constraints.append([effort, lower, upper, velocity, visual])
        
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
    
        self.links.append([p.JOINT_FIXED, theta, d, a, alpha])
        self.subs_joints.append((sp.symbols('temp'+str(np.random.random(1)[0]*100)), 0))
        self.constraints.append([0,0,0,0,visual])
    
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