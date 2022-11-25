from packages.utils import *
from packages.dh2urdf import *
from packages.pybullet_sim import *
import sympy as sp
import numpy as np
import struct
import glob
import os
import pybullet as p
import time 

class RobotArm:
    """
    A class to easily create and interact with robotic arm.
    """
    def __init__(self, offset = (0,0,0), orientation = (0,0,0,1),use_dynamics = True,
                    name = 'my_robot', time_step = 1/60, scaling = 1, ik_function = None):
        self.name = name
        self.file_head  = 'robot_arms/' + name
        self.robot = None
        self.offset = offset
        self.orientation = orientation
        self.ik_function = ik_function
        self.use_dynamics = use_dynamics
        #sympy vars
        self.links = []
        self.constraints = []
        self.joint_variables = []
        self.subs_joints = []
        self.subs_additional = []
        #pybullet vars
        self.log, self.record_param = None,None
        self.joint_ids = []
        self.joint_frames = []
        self.is_imported = False
        self.use_orientation_ik = False
        self.pose_param = None
        # attachemt vars
        self.attachment = None
        self.attachment_button = None
        self.attachment_joint_ids = []
        # drawing trajectory
        self.hasPrevPose = [False]*5
        self.prevPose1 = [[0,0,0]]*5
        self.prevPose2 = [[0,0,0]]*5
        # buttons
        self.quit_button = None
        self.capture_image_button = None
        self.prev_button_state = 2
        self.scaling = scaling

        if not p.isConnected():
            sim = PybulletSimulation()
            if sim.connect():
                self.time_step = sim.time_step
        else:
            self.time_step = time_step
    
    def add_pose_sliders(self,x,y,z):
        self.pose_sliders = []
        self.pose_sliders.append(p.addUserDebugParameter(f"x",x[0],x[1],x[2]))
        self.pose_sliders.append(p.addUserDebugParameter(f"y",y[0],y[1],y[2]))
        self.pose_sliders.append(p.addUserDebugParameter(f"z",z[0],z[1],z[2]))
        if self.use_orientation_ik:
            self.pose_sliders.append(p.addUserDebugParameter(f"R",-180, 180, 0))
            self.pose_sliders.append(p.addUserDebugParameter(f"P",-180, 180, 0))
            self.pose_sliders.append(p.addUserDebugParameter(f"Y",-180, 180, 0))

    def add_joint_sliders(self):
        self.joint_sliders = []
        for i in self.joint_ids:
            joint_info = p.getJointInfo(self.robot,i)
            if joint_info[2] == p.JOINT_PRISMATIC:
                self.joint_sliders.append((False,p.addUserDebugParameter(f"D{i}",joint_info[8],joint_info[9],joint_info[8])))
            if joint_info[2] == p.JOINT_REVOLUTE:
                self.joint_sliders.append((True,p.addUserDebugParameter(f"Theta{i}",np.rad2deg(joint_info[8]),np.rad2deg(joint_info[9]),0)))
    
    def add_joint_frame_lines(self):
        if not self.links: # if is foreign
            rot = np.array(p.getMatrixFromQuaternion(
                    p.getLinkState(self.robot,self.joint_ids[-1],computeForwardKinematics = True)[5])).reshape(3,3)
            line_n, line_a = self.project_point_to_plane(rot,(0.8,0,0)),self.project_point_to_plane(rot,(0,0,0.8))
            self.joint_frames.append(p.addUserDebugLine((0,0,0),line_n,parentObjectUniqueId=self.robot,
                                        parentLinkIndex=self.joint_ids[-1],lineColorRGB=(0,1,1), lineWidth = 2))
            self.joint_frames.append(p.addUserDebugLine((0,0,0),line_a,parentObjectUniqueId=self.robot,
                                        parentLinkIndex=self.joint_ids[-1],lineColorRGB=(1,1,0), lineWidth = 2))
        else:
            for joint_id in self.joint_ids:
                self.subs_joints[joint_id//2] = self.subs_joints[joint_id//2][0], p.getJointState(self.robot,joint_id)[0]
            for joint in range(0,self.last_joint_id,2):
                transform = self.get_dh_joint_to_joint(joint//2,joint//2+1).subs(self.subs_joints+self.subs_additional).evalf()
                line_n, _, line_a = frame_lines(transform, 0.8)
                self.joint_frames.append(p.addUserDebugLine([item for sublist in line_n[:,0].tolist() for item in sublist], 
                    [item for sublist in line_n[:,1].tolist() for item in sublist],parentObjectUniqueId=self.robot,parentLinkIndex=joint,
                    lineColorRGB=(1,0,0) if self.last_joint_id-2 != joint else (0,1,1), lineWidth = 2))
                self.joint_frames.append(p.addUserDebugLine([item for sublist in line_a[:,0].tolist() for item in sublist],
                    [item for sublist in line_a[:,1].tolist() for item in sublist],parentObjectUniqueId=self.robot,parentLinkIndex=joint,
                    lineColorRGB=(0,0,1) if self.last_joint_id-2 != joint else (1,1,0), lineWidth = 2))
    
    def add_joint_ids(self):
        add_attachment_joints = False
        for i in range(p.getNumJoints(self.robot)): 
            if p.getJointInfo(self.robot,i)[1] == b'attachment_joint' or add_attachment_joints:
                if add_attachment_joints == False:
                    self.last_joint_id = i-1
                add_attachment_joints = True
                if (p.getJointInfo(self.robot,i)[2] in [p.JOINT_PRISMATIC, p.JOINT_REVOLUTE]):
                    self.attachment_joint_ids.append(i)   
            else:
                if (p.getJointInfo(self.robot,i)[2] in [p.JOINT_PRISMATIC, p.JOINT_REVOLUTE]):
                    self.joint_ids.append(i)   
        if add_attachment_joints is False:
            self.last_joint_id = i

    def reset_joints(self):
        """ Reset the robotic arm """
        for joint_id in self.joint_ids:
            joint_info = p.getJointInfo(self.robot,joint_id)
            p.resetJointState(self.robot,joint_id, joint_info[8] if joint_info[2] == p.JOINT_PRISMATIC else 0)
    
    def load_robot(self):
        if not self.is_imported:
            subs = self.subs_joints + self.subs_additional
            DH_params = sp.Matrix(self.links).subs(subs).evalf()
            dh2urdf = DH2Urdf(DH_params.tolist(),self.constraints,self.attachment)
            if not os.path.exists(f'{self.file_head}/'):
                os.mkdir(f'{self.file_head}/')
            dh2urdf.save_urdf(f'{self.file_head}/{self.name}.urdf')

        self.robot = p.loadURDF(f'{self.file_head}/{self.name}.urdf', self.offset, self.orientation,
                                    useFixedBase=True, globalScaling = self.scaling)
        self.add_joint_ids()
        self.add_joint_frame_lines()
        self.reset_joints()
    
    def interact(self, kinematics = 'forward', use_orientation_ik = False, x_range = (-5,5,1), y_range = (-5,5,1), z_range = (-5,5,1)):
        self.kinematics = kinematics
        self.use_orientation_ik = use_orientation_ik
        self.load_robot()
        if self.kinematics == 'forward':
            self.add_joint_sliders()
        else:
            self.add_pose_sliders(x_range, y_range, z_range)           
        self.step()

    def step(self):
        while (p.isConnected()):
            p.stepSimulation()
            if self.kinematics == 'forward':
                position = None
                joint_targets = [(np.deg2rad(p.readUserDebugParameter(parameter)) if revolute else p.readUserDebugParameter(parameter)) 
                                    for revolute,parameter in self.joint_sliders]
            elif self.kinematics == 'inverse':
                position = (p.readUserDebugParameter(self.pose_sliders[0]),p.readUserDebugParameter(self.pose_sliders[1]),p.readUserDebugParameter(self.pose_sliders[2]))
                if self.use_orientation_ik:
                    orientation = p.getQuaternionFromEuler([np.deg2rad(p.readUserDebugParameter(self.pose_sliders[3])),
                        np.deg2rad(p.readUserDebugParameter(self.pose_sliders[4])),
                        np.deg2rad(p.readUserDebugParameter(self.pose_sliders[5]))])
                
                if self.ik_function:
                    joint_targets = self.ik_function(position, orientation if self.use_orientation_ik else None)
                else:
                    joint_targets = p.calculateInverseKinematics(self.robot,self.last_joint_id, position, 
                        orientation if self.use_orientation_ik else None, maxNumIterations=5)[:len(self.joint_ids)]

            if self.use_dynamics:
                p.setJointMotorControlArray(self.robot,self.joint_ids,p.POSITION_CONTROL,joint_targets)#,forces=[500]*len(joint_targets))
            else:
                for i,joint_id in enumerate(self.joint_ids):
                    p.resetJointState(self.robot,joint_id,joint_targets[i])
            
            if self.attachment_joint_ids:
                self.actuate_attachment(self.attachment_joint_ids)

            link_state = p.getLinkState(self.robot,self.last_joint_id,computeForwardKinematics = True)
            
            self.capture_image(capture_now=False)
            #self.display_pos_and_orn(link_state[4],link_state[5],self.last_joint_id)
            self.draw_trajectory(position,link_state[4],5)
            self.state_logging(self.kinematics)
            if self.quit_simulation():
                break
            time.sleep(self.time_step)
    
    def display_pos_and_orn(self, position, orientation, link_id):
        x,y,z = position
        R,P,Y = np.rad2deg(p.getEulerFromQuaternion(orientation))
        if not self.pose_param:
            self.pose_param =  p.addUserDebugText(f"(x:{x:.2f} y:{y:.2f} z:{z:.2f}) (R:{R:.1f} P:{P:.1f} Y:{Y:.1f})", [0.5, 0, 0.5],textColorRGB=[0, 0, 0],
                textSize= 1,parentObjectUniqueId=self.robot,parentLinkIndex=link_id)       
        p.addUserDebugText(f"(x:{x:.2f} y:{y:.2f} z:{z:.2f}) (R:{R:.1f} P:{P:.1f} Y:{Y:.1f})", [0.5, 0, 0.5],textColorRGB=[0, 0, 0],textSize= 1,
            parentObjectUniqueId=self.robot,parentLinkIndex=link_id, replaceItemUniqueId = self.pose_param)
     
    def draw_trajectory(self, target, current, lifeTime = 15, line_index = 0):
        if (self.hasPrevPose[line_index] is True):
            if target:
                p.addUserDebugLine(self.prevPose1[line_index], target, [0, 0, 0.3], 1, lifeTime)
            p.addUserDebugLine(self.prevPose2[line_index], current, [1, 0, 0] , 1, lifeTime)
        self.prevPose1[line_index] = target
        self.prevPose2[line_index] = current
        self.hasPrevPose[line_index] = True 
    
    def quit_simulation(self):
        if self.quit_button is None:
            self.quit_button = p.addUserDebugParameter("Quit Simulation",1,0,1)
        else:
            if p.readUserDebugParameter(self.quit_button) %2 == 0: # on click
                p.disconnect()
                return True
            else:
                return False
    
    def actuate_attachment(self, joint_ids, joint_targets = None):
        if joint_targets is None:
            joint_targets = [0,0]
            if self.attachment_button is None:
                self.attachment_button = p.addUserDebugParameter("Open/Close Gripper",1,0,1)
            else:
                if p.readUserDebugParameter(self.attachment_button) %2 == 0: # close 
                    joint_targets = [0.3,-0.3]
                else:
                    joint_targets = [0,0]
        p.setJointMotorControlArray(self.robot,joint_ids,p.POSITION_CONTROL,joint_targets, forces=[20]*len(joint_ids))
    
    def capture_image(self, link_state = None, offset_camera = (0,0,0.5), near = 0.1, far = 5, size = (320,320), fov = 40, capture_now = False):
        if link_state is None:
            link_state = p.getLinkState(self.robot,self.last_joint_id, computeForwardKinematics=True)
        if capture_now is False:
            if self.capture_image_button is None:
                self.capture_image_button = p.addUserDebugParameter("Capture Image",1,0,1)
            else:
                if p.readUserDebugParameter(self.capture_image_button) == self.prev_button_state: # on click
                    self.prev_button_state +=1
                    capture_now = True    
        if capture_now:
            rot = np.array(p.getMatrixFromQuaternion(link_state[5])).reshape(3,3)
            offset = self.project_point_to_plane(rot, offset_camera)
            target = (link_state[4][0]+offset[0],link_state[4][1]+offset[1],link_state[4][2]+offset[2])
            orn = p.getEulerFromQuaternion(link_state[5])
            camera = Camera(target,0.1,(0,np.rad2deg(orn[0])+90,np.rad2deg(orn[2])), near, far, size, fov)
            camera.shot()
            return camera     

    def state_logging(self, log_name, object_ids = None, start_stop = None):
        if object_ids is None:
            object_ids = [self.robot]

        if start_stop is None:
            if self.record_param is None:
                self.record_param = p.addUserDebugParameter("Start/Stop Logging",1,0,1)
            else:
                if p.readUserDebugParameter(self.record_param) %2 == 0: # start
                    if self.log is None: # once
                        if not os.path.exists(f'{self.file_head}/logs/'):
                            os.mkdir(f'{self.file_head}/logs/')
                        self.log = p.startStateLogging(p.STATE_LOGGING_GENERIC_ROBOT,
                            f"{self.file_head}/logs/{log_name}_{int(p.readUserDebugParameter(self.record_param)//2)}.txt", object_ids)
                elif self.log is not None:
                    p.stopStateLogging(self.log)
                    self.log = None
        else:
            if start_stop is True: # start
                if not os.path.exists(f'{self.file_head}/logs/'):
                    os.mkdir(f'{self.file_head}/logs/')
                self.log = p.startStateLogging(p.STATE_LOGGING_GENERIC_ROBOT,f"{self.file_head}/logs/move_{log_name}.txt", object_ids)
            else:
                p.stopStateLogging(self.log)
    
    def convert_logs_to_prg(self, log_name):
        log = []
        for log_name_index in sorted(glob.glob(f'{self.file_head}/logs/'+f"{log_name}*.txt"), key=os.path.getmtime):
            log += self.readLogFile(log_name_index)
        if not log:
            return "No logs with that name"
        if not p.isConnected():
            return "Load the robot"
        recordNum = len(log)
        objectNum = 1
        number_steps = int(recordNum / objectNum)
        step_index = -1
        pgr_string =  f"1 Servo on  \n2 Wait M_Svo=1 \n3 Base 0 \n4 Ovrd 10 \n5 Dim jStep({number_steps})"
        pgr_string += f"6 For m1 = 1 To {number_steps} \n7 Mov jStep(m1) \n8 Next m1 \n9 end \n"
        while (p.isConnected()):
            p.stepSimulation()
            step_index+=1
            for objectId in range(objectNum):
                record = log[step_index * objectNum + objectId]
                Id = record[2]
                numJoints = p.getNumJoints(Id)
                pgr_string +=  f"jStep({step_index+1})=("
                for i in range(numJoints):
                    qIndex = p.getJointInfo(Id, i)[3]
                    if qIndex > -1:
                        pgr_string += f"{np.rad2deg(record[qIndex - 7 + 17]):.3f}"
                        pgr_string +=  ',' if i != numJoints-1 else ')\n'
            if  step_index == recordNum / objectNum - 1:
                break  
        file = open(f"{self.file_head}/{self.name}.prg", "w")
        file.write(pgr_string)
        file.close() 

    def replay_logs(self, log_name, skim_trough = True, object_ids = None):
        if object_ids is None:
            object_ids = [self.robot]
        log = []
        for log_name_index in sorted(glob.glob(f'{self.file_head}/logs/'+f"{log_name}*.txt"), key=os.path.getmtime):
            log += self.readLogFile(log_name_index)
        if not log:
            return "No logs with that name"
        recordNum, objectNum = len(log), len(object_ids)
        if skim_trough:
            step_index_param = p.addUserDebugParameter("Replay Step", 0, recordNum / objectNum - 1, 0)
        else:
            step_index = -1
        while (p.isConnected()):
            p.stepSimulation()
            if skim_trough:
                step_index = int(p.readUserDebugParameter(step_index_param))
            else:
                step_index+=1
            for objectId in range(objectNum):
                record = log[step_index * objectNum + objectId]
                Id = record[2]
                p.resetBasePositionAndOrientation(Id, [record[3], record[4], record[5]], 
                                                [record[6], record[7], record[8], record[9]])
                for i in range(p.getNumJoints(Id)):
                    qIndex = p.getJointInfo(Id, i)[3]
                    if qIndex > -1:
                        if self.use_dynamics:
                            p.setJointMotorControl2(Id, i, p.POSITION_CONTROL, record[qIndex - 7 + 17])
                        else:
                            p.resetJointState(Id, i, record[qIndex - 7 + 17])
            if step_index == recordNum / objectNum - 1 and skim_trough is False:
                break
            if self.quit_simulation():
                break
            time.sleep(self.time_step)

    def readLogFile(self, filename, verbose=False):
        f = open(filename, 'rb')
        keys = f.readline().decode('utf8').rstrip('\n').split(',')
        fmt = f.readline().decode('utf8').rstrip('\n')
        # The byte number of one record
        sz = struct.calcsize(fmt)
        # The type number of one record
        ncols = len(fmt)

        if verbose:
            print(f'Keys{keys}')
            print(f"Format: {fmt}")
            print(f'Size: {sz}')
            print(f'Columns: {ncols}')

        # Read data
        wholeFile = f.read()
        # split by alignment word
        chunks = wholeFile.split(b'\xaa\xbb')
        log = []
        for chunk in chunks:
            if len(chunk) == sz:
                values = struct.unpack(fmt, chunk)
                record = []
                for i in range(ncols):
                    record.append(values[i])
                log.append(record)
        return log

    def import_foreign_robot(self, file_path, scaling=5):
        self.is_imported = True
        self.scaling = scaling
        file_head, tail = os.path.split(file_path)
        self.file_head = file_head
        self.name = tail[:-5]

    def import_robot(self,file_path = 'robot_arms/my_robot/my_robot.urdf'):
        self.is_imported = True
        file_head, tail = os.path.split(file_path)
        self.file_head = file_head
        self.name = tail[:-5]
        file = open(file_path, 'r')
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
    
    def write_text(self,text,offset):
        for letter in text:
            self.write_letter(letter,offset)
            offset[1]-=0.25

    def write_letter(self,letter,pos,orn,plane):
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
            self.move((pos[0],    pos[1],pos[2], orn[0],orn[1],orn[2]),(2+pos[0],pos[1],pos[2], orn[0],orn[1],orn[2]),plane,'linear',30)
            self.move((2+pos[0],pos[1],pos[2], orn[0],orn[1],orn[2]),(2+pos[0],-1.5+pos[1],pos[2], orn[0],orn[1],orn[2]),plane,'linear',15)
            self.move((1+pos[0],pos[1],pos[2], orn[0],orn[1],orn[2]),(1+pos[0],-1.5+pos[1],pos[2], orn[0],orn[1],orn[2]),plane,'linear',15)
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
        
    def move(self, start, end, plane = (0,0,0), interpolation='linear',steps=30,param=None,closed=False,log_move = False):
        """
        Move to a desired position and rotation of robot. 
        Parameters
        ----------
        position: tuple()
            Set (x,y,z) coordinates.
        rotation: tuple()
            Set (R,P,Y) angles.
        interpolation: string
            Options: linear / circular.
        steps: int
            Number of points generated.
        param: tuple()
            Set (a,b,res_num,side):
            - a,b are parameters for the ellipse
            - res_num is to chose witch solution you like 0 or 1
            - side is to chose witch path to take 0 or 1
        closed: bool
            To draw the whole elipse
        """
        if log_move:    self.state_logging(f"{interpolation}_{start}_{end}",start_stop=True)
        # self.hasPrevPose = False treba da se nagradi
        x1,y1,z1,R1,P1,Y1 = start
        x2,y2,z2,R2,P2,Y2 = end
        rot_mat = rotation_matrix_from_euler_angles(plane,'xyz')
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
            
            if closed:
                start_angle, end_angle = (0,2*np.pi) if side else (2*np.pi,0)

            if side:
                t = np.linspace(start_angle, end_angle, steps)
            else:
                if start_angle < end_angle:
                    t = np.hstack((np.linspace(start_angle, 0,int(steps*start_angle/(2*np.pi-end_angle+start_angle)) ),
                    np.linspace(2*np.pi, end_angle, int(steps*(2*np.pi-end_angle)/(2*np.pi-end_angle+start_angle)) )))
                else:
                    t = np.hstack((np.linspace(start_angle, 2*np.pi, int(steps*(2*np.pi-start_angle)/(2*np.pi-start_angle+end_angle)) ),
                    np.linspace(0, end_angle, int(steps*end_angle/(2*np.pi-start_angle+end_angle)) )))
            
            

            for x,y,z,R,P,Y in zip(x_center + a*np.cos(t),y_center + b*np.sin(t),np.linspace(z1, z2, steps),
                                np.linspace(R1,R2,steps),np.linspace(P1,P2,steps),np.linspace(Y1,Y2,steps)):
                if not self.move2point(self.project_point_to_plane(rot_mat,(x,y,z)),(R,P,Y)):
                    return False
        if log_move:   self.state_logging("",start_stop=False) 

    def move2point(self,position,orientation=None, converge = 10):
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
                joint_targets = p.calculateInverseKinematics(self.robot,self.last_joint_id, position, 
                    p.getQuaternionFromEuler(orientation), maxNumIterations=5)[:len(self.joint_ids)]
            
            if self.use_dynamics:
                p.setJointMotorControlArray(self.robot,self.joint_ids,p.POSITION_CONTROL,joint_targets)
                #,targetVelocities = [0]*len(joint_targets),forces=[500]*len(joint_targets))#,positionGains = [1]*len(joint_targets),velocityGains = [0.1]*len(joint_targets))
            else:
                for i,joint_id in enumerate(self.joint_ids):
                    p.resetJointState(self.robot,joint_id,joint_targets[i])
            
            link_state = p.getLinkState(self.robot,self.last_joint_id,computeForwardKinematics = True)
            self.display_pos_and_orn(link_state[4],link_state[5],self.last_joint_id)
            self.draw_trajectory(position,link_state[4])
            
            if self.quit_simulation():
                break
            time.sleep(self.time_step)

            if self.use_dynamics and np.allclose([p.getJointState(self.robot,joint_id)[0] for joint_id in self.joint_ids],joint_targets,0.1*(steps/converge)):
                break
            elif not self.use_dynamics and steps > 1:
                break
        return True
    
    def project_point_to_plane(self, rot_mat, point):
        return point[0] * rot_mat[:3, 0] + point[1] * rot_mat[:3, 1] + point[2] * rot_mat[:3, 2]

    def add_attachment(self, name = 'gripper', orientation = (-np.pi/2,0,0), position = (0,0,0.3)):
        self.attachment = [name,orientation,position]

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

    def add_prismatic_joint(self, theta, d, a, alpha, lower = 0.8, upper = 3, velocity = 2.6, effort = 10, visual = True):
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