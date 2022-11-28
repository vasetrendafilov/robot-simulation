from packages.utils import *
from packages.dh2urdf import DH2Urdf
from packages.pybullet_sim import PybulletSimulation, Camera
import pybullet as p
import sympy as sp
import numpy as np
import struct
import time 
import glob
import os

class RobotArm:
    """
    A class to easily create and interact with robotic arm.
    """
    def __init__(self, offset = (0,0,0), orientation = (0,0,0,1), use_dynamics = True, name = 'my_robot',
                    fps = 60, joint_force = 1200, scaling = 1, ik_function = None):
        # sympy vars
        self.links, self.joint_variables, self.constraints= [],[],[]
        self.subs_joints, self.subs_additional = [],[]
        # load robot vars
        self.robot = None
        self.file_head, self.name =  'robot_arms/' + name, name
        self.is_imported, self.is_foreign = False, False
        self.offset, self.orientation = offset, orientation
        self.scaling = scaling
        # joint motor control vars
        self.use_dynamics = use_dynamics
        self.frames_to_complete = 20
        self.joint_force = joint_force
        self.axis_limits = []
        # ik vars
        self.ik_joint_error = 0.02
        self.use_orientation_ik = True
        self.ik_function = ik_function
        # attachment vars
        self.attachment_joint_ids = []
        self.attachment, self.attachment_button = None, None
        self.attachment_open_targets, self.attachment_close_targets = (0,0), (0.3,-0.3)
        # draw trajectory vars
        self.trajectory_lifetime = 5
        self.hasPrevPose = [False]*5
        self.prevPose1, self.prevPose2 = [[0,0,0]]*5, [[0,0,0]]*5
        # capture image vars
        self.capture_image_button = None
        self.prev_button_state = 2
        self.camera_offset = (0,0,0.5)
        # buttons
        self.quit_button = None
        self.pose_text = None
        # state logging
        self.log, self.record_button = None,None
        self.rec_cnt = 1
        # connect to simulation
        if not p.isConnected():
            sim = PybulletSimulation(fps=fps)
            if sim.connect():
                self.time_step = sim.time_step
        else:
            self.time_step = 1/fps
    
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
        self.joint_frames = []
        if self.is_foreign is True: # if is foreign
            rot = np.array(p.getMatrixFromQuaternion(
                    p.getLinkState(self.robot,self.joint_ids[-1],computeForwardKinematics = True)[5])).reshape(3,3)
            line_n, line_a = self.rotate_point(rot,(0.8,0,0)),self.rotate_point(rot,(0,0,0.8))
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
        self.joint_ids = []
        add_attachment_joints = False
        add_constraints_if_foreign = False
        for i in range(p.getNumJoints(self.robot)): 
            joint_info = p.getJointInfo(self.robot,i)
            if joint_info[1] == b'attachment_joint' or add_attachment_joints:
                if add_attachment_joints == False:
                    self.last_joint_id = i-1
                add_attachment_joints = True
                if (joint_info[2] in [p.JOINT_PRISMATIC, p.JOINT_REVOLUTE]):
                    self.attachment_joint_ids.append(i)   
            else:
                if (joint_info[2] in [p.JOINT_PRISMATIC, p.JOINT_REVOLUTE]):
                    self.joint_ids.append(i)
                    if self.is_foreign and (len(self.constraints) == 0 or add_constraints_if_foreign):
                        add_constraints_if_foreign = True # if constraints is empty add them
                        self.constraints.append([joint_info[10], joint_info[8], joint_info[9], joint_info[11], True])   
        if add_attachment_joints is False:
            self.last_joint_id = i

    def get_joint_states(self):
        joint_states = []
        for joint_id in self.joint_ids:
            joint_states.append(p.getJointState(self.robot,joint_id)[0])
        return joint_states

    def reset_joints(self,joint_targets = None):
        """ Reset the robotic arm """
        for i,joint_id in enumerate(self.joint_ids):
            joint_info = p.getJointInfo(self.robot,joint_id)
            if joint_targets:
                p.resetJointState(self.robot,joint_id, joint_targets[i])
            else:
                p.resetJointState(self.robot,joint_id, joint_info[8] if joint_info[2] == p.JOINT_PRISMATIC else 0)
    
    def load_robot(self):
        if self.is_imported is False:
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
    
    def interact(self, kinematics = 'forward', use_orientation_ik = True, use_draw_trajectory = True, use_display_pos_and_orn = False,
                    x_range = (-5,5,1), y_range = (-5,5,1), z_range = (-5,5,1)):
        self.kinematics = kinematics
        self.use_orientation_ik = use_orientation_ik
        self.load_robot()
        if self.kinematics == 'forward':
            self.add_joint_sliders()
        else:
            self.add_pose_sliders(x_range, y_range, z_range)           
        self.step(use_draw_trajectory, use_display_pos_and_orn)

    def step(self, use_draw_trajectory = True, use_display_pos_and_orn = False):
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
                    joint_targets = self.ik_function(self.limit_pos_targets(position), orientation if self.use_orientation_ik else None)
                else:
                    joint_targets = p.calculateInverseKinematics(self.robot,self.last_joint_id,self.limit_pos_targets(position), 
                        orientation if self.use_orientation_ik else None, maxNumIterations=5)[:len(self.joint_ids)]
                    joint_targets = self.limit_joint_targets(joint_targets)        

            if self.use_dynamics:
                p.setJointMotorControlArray(self.robot,self.joint_ids,p.POSITION_CONTROL,joint_targets,forces=[self.joint_force]*len(joint_targets))
            else:
                for i,joint_id in enumerate(self.joint_ids):
                    p.resetJointState(self.robot,joint_id,joint_targets[i])
          
            if self.attachment_joint_ids:
                self.actuate_attachment()

            link_state = p.getLinkState(self.robot,self.last_joint_id,computeForwardKinematics = True)  
            self.capture_image(camera_offset = self.camera_offset, capture_now=False)
            if use_draw_trajectory:
                self.draw_trajectory(link_state[4],position,self.trajectory_lifetime)
            if use_display_pos_and_orn:
                self.display_pos_and_orn(link_state[4],link_state[5],self.last_joint_id)
            self.state_logging(self.kinematics)
            if self.quit_simulation():
                break
            time.sleep(self.time_step)
    
    def display_pos_and_orn(self, position, orientation, link_id):
        x,y,z = position
        R,P,Y = np.rad2deg(p.getEulerFromQuaternion(orientation))
        if self.pose_text is None:
            self.pose_text =  p.addUserDebugText(f"(x:{x:.2f} y:{y:.2f} z:{z:.2f}) (R:{R:.1f} P:{P:.1f} Y:{Y:.1f})", [0.5, 0, 0.5],textColorRGB=[0, 0, 0],
                textSize= 1,parentObjectUniqueId=self.robot,parentLinkIndex=link_id)       
        p.addUserDebugText(f"(x:{x:.2f} y:{y:.2f} z:{z:.2f}) (R:{R:.1f} P:{P:.1f} Y:{Y:.1f})", [0.5, 0, 0.5],textColorRGB=[0, 0, 0],textSize= 1,
            parentObjectUniqueId=self.robot,parentLinkIndex=link_id, replaceItemUniqueId = self.pose_text)
     
    def draw_trajectory(self, current, target = None, lifeTime = 15, line_index = 0):
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
    
    def actuate_attachment(self, joint_ids = None, joint_targets = None):
        if joint_ids is None:
            joint_ids = self.attachment_joint_ids
        if joint_targets is None:
            joint_targets = self.attachment_open_targets
            if self.attachment_button is None:
                self.attachment_button = p.addUserDebugParameter("Close/Open Gripper",1,0,1)
            else:
                if p.readUserDebugParameter(self.attachment_button) %2 == 0: # close the gripper
                    joint_targets = self.attachment_close_targets
                else:
                    joint_targets = self.attachment_open_targets
        p.setJointMotorControlArray(self.robot,joint_ids,p.POSITION_CONTROL,joint_targets, forces=[20]*len(joint_ids))
    
    def capture_image(self, link_state = None, camera_offset = (0,0,0.5), near = 0.1, far = 5, size = (320,320), fov = 40, capture_now = False):
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
            offset = self.rotate_point(rot, camera_offset)
            target = (link_state[4][0]+offset[0],link_state[4][1]+offset[1],link_state[4][2]+offset[2])
            orn = p.getEulerFromQuaternion(link_state[5])
            camera = Camera(target,0.01,(np.rad2deg(orn[2]),np.rad2deg(-orn[0]),np.rad2deg(orn[1])), near, far, size, fov)
            camera.shot()
            return camera     

    def state_logging(self, log_name='', start_stop = None, object_ids = None):
        if object_ids is None:
            object_ids = [self.robot]

        if start_stop is None:
            if self.record_button is None:
                self.record_button = p.addUserDebugParameter("Start/Stop Logging",1,0,1)
            else:
                if p.readUserDebugParameter(self.record_button) %2 == 0: # start
                    if self.log is None: # once
                        if not os.path.exists(f'{self.file_head}/logs/'):
                            os.mkdir(f'{self.file_head}/logs/')
                        self.log = p.startStateLogging(p.STATE_LOGGING_GENERIC_ROBOT,
                            f"{self.file_head}/logs/{log_name}_{int(p.readUserDebugParameter(self.record_button)//2)}.txt", object_ids)
                elif self.log is not None:
                    p.stopStateLogging(self.log)
                    self.log = None
        else:
            if start_stop is True: # start
                if not os.path.exists(f'{self.file_head}/logs/'):
                    os.mkdir(f'{self.file_head}/logs/')
                self.log = p.startStateLogging(p.STATE_LOGGING_GENERIC_ROBOT,
                                f"{self.file_head}/logs/move_{log_name}_{self.rec_cnt}.txt", object_ids)
                self.rec_cnt +=1
            else:
                p.stopStateLogging(self.log)
    
    def autocomplete_logs(self,log_names):
        def sortKeyFunc(s):
            return int(os.path.basename(s).split('_')[-1][:-4])
        log = []
        if type(log_names) != list:
            log_names = [log_names]
        for log_name in log_names:
            for log_name_index in sorted(glob.glob(f'{self.file_head}/logs/'+f"{log_name}*.txt"), key=sortKeyFunc):
                log += self.readLogFile(log_name_index)
        return log

    def convert_logs_to_prg(self, log_names):
        logs = self.autocomplete_logs(log_names)
        if not logs:
            return "No logs found"
        num_steps = len(logs)
        pgr_string =  f"1 Servo on  \n2 Wait M_Svo=1 \n3 Base 0 \n4 Ovrd 10 \n5 Dim jStep({num_steps})\n"
        pgr_string += f"6 For m1 = 1 To {num_steps} \n7 Mov jStep(m1) \n8 Next m1 \n9 end \n"
        for step in range(num_steps):
            pgr_string +=  f"jStep({step+1})=("
            for joint in self.joint_ids:
                pgr_string += f"{np.rad2deg(logs[step][joint + 17]):.3f}"
                pgr_string +=  ',' if joint != self.joint_ids[-1] else ')\n'
        file = open(f"{self.file_head}/{self.name}.prg", "w")
        file.write(pgr_string)
        file.close() 

    def replay_logs(self, log_names, skim_trough = True, object_ids = None):
        if object_ids is None:
            object_ids = [self.robot]
        logs = self.autocomplete_logs(log_names)
        if not logs:
            return "No logs found"
        recordNum, objectNum = len(logs), len(object_ids)
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
                record = logs[step_index * objectNum + objectId]
                Id = record[2]
                p.resetBasePositionAndOrientation(Id, [record[3], record[4], record[5]], 
                                                [record[6], record[7], record[8], record[9]])
                for i in range(p.getNumJoints(Id)):
                    qIndex = p.getJointInfo(Id, i)[3]
                    if qIndex > -1:
                        if self.use_dynamics:
                            p.setJointMotorControl2(Id, i, p.POSITION_CONTROL, record[qIndex - 7 + 17],force=self.joint_force)
                        else:
                            p.resetJointState(Id, i, record[qIndex - 7 + 17])
    
            self.draw_trajectory(p.getLinkState(self.robot,self.last_joint_id,computeForwardKinematics = True)[4],
                                    lifeTime= self.trajectory_lifetime)
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
        self.is_imported = self.is_foreign = True
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
    
    def write_text(self,text,position,orientation,offset=1,plane=(0,0,0),scale = 0.5):
        for letter in text:
            self.write_letter(letter,position,orientation,plane,scale)
            position[1] += offset

    def write_letter(self,letter,position,orientation,plane=(0,0,0),s=0.5):
        x,y,z = position
        R,P,Y = orientation
        # frame of the letter 2x1.5
        if letter == 'T':
            self.move((x,y,z,R,P,Y),(x,y+1.5*s,z, R,P,Y),plane,'linear',30)
            self.move((x,y+0.75*s,z, R,P,Y),(x+2*s,y+0.75*s,z, R,P,Y),plane,'linear',30)
        elif letter == 'E':
            self.move((x+2*s,y,z,R,P,Y),(x,y,z, R,P,Y),plane,'linear',30)
            self.move((x,y,z, R,P,Y),(x,y+1.5*s,z, R,P,Y),plane,'linear',15)
            self.move((x+1*s,y,z, R,P,Y),(x+1*s,y+1.5*s,z, R,P,Y),plane,'linear',15)
            self.move((x+2*s,y,z, R,P,Y),(x+2*s,y+1.5*s,z, R,P,Y),plane,'linear',15)
        elif letter == 'I':
            self.move((x,y+0.4*s,z,R,P,Y),(x,y+1.1*s,z, R,P,Y),plane,'linear',15)
            self.move((x,y+0.75*s,z, R,P,Y),(x+2*s,y+0.75*s,z, R,P,Y),plane,'linear',30)
            self.move((x+2*s,y+0.4*s,z, R,P,Y),(x+2*s,y+1.1*s,z, R,P,Y),plane,'linear',15)
        elif letter == 'L':
            self.move((x,y,z,R,P,Y),(x+2*s,y,z, R,P,Y),plane,'linear',30)
            self.move((x+2*s,y,z, R,P,Y),(x+2*s,y+1.5*s,z, R,P,Y),plane,'linear',15)
        elif letter == 'H':
            self.move((x+2*s,y,z,R,P,Y),(x,y,z, R,P,Y),plane,'linear',30)
            self.move((x+1*s,y,z, R,P,Y),(x+1*s,y+1.5*s,z, R,P,Y),plane,'linear',15)
            self.move((x+2*s,y+1.5*s,z, R,P,Y),(x,y+1.5*s,z, R,P,Y),plane,'linear',30)
        elif letter == 'F':
            self.move((x+2*s,y,z,R,P,Y),(x,y,z, R,P,Y),plane,'linear',30)
            self.move((x,y,z, R,P,Y),(x,y+1.5*s,z, R,P,Y),plane,'linear',15)
            self.move((x+1*s,y,z, R,P,Y),(x+1*s,y+1.5*s,z, R,P,Y),plane,'linear',15)
        elif letter == 'N':
            self.move((x+2*s,y,z,R,P,Y),(x,y,z, R,P,Y),plane,'linear',30)
            self.move((x,y,z, R,P,Y),(x+2*s,y+1.5*s,z, R,P,Y),plane,'linear',30)
            self.move((x+2*s,y+1.5*s,z, R,P,Y),(x,y+1.5*s,z, R,P,Y),plane,'linear',30)
        elif letter == 'V':
            self.move((x,y,z, R,P,Y),(x+2*s,y+0.75*s,z, R,P,Y),plane,'linear',30)
            self.move((x+2*s,y+0.75*s,z, R,P,Y),(x,y+1.5*s,z, R,P,Y),plane,'linear',30)
        elif letter == 'A':
            self.move((x+2*s,y,z, R,P,Y),(x,y+0.75*s,z, R,P,Y),plane,'linear',30)
            self.move((x,y+0.75*s,z, R,P,Y),(x+2*s,y+1.5*s,z, R,P,Y),plane,'linear',30)
            self.move((x+0.75*s,y+0.45*s,z, R,P,Y),(x+0.75*s,y+1.05*s,z, R,P,Y),plane,'linear',30)
        elif letter == 'D':
            self.move((x,y,z,R,P,Y),(x+2*s,y,z,R,P,Y),plane,'linear',30)
            self.move((x+2*s,y,z, R,P,Y),(x,y,z, R,P,Y),plane,'circular',30,(0.5,0.5,0,0))

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
        self.hasPrevPose = [False]*5 # restart drawing
        x1,y1,z1,R1,P1,Y1 = start
        x2,y2,z2,R2,P2,Y2 = end
        if log_move:
            self.move2point((x1,y1,z1),(R1,P1,Y1)) # for cleaner start to log 
            self.hasPrevPose = [False]*5 # restart drawing  
            self.state_logging(f"{interpolation}_{start}_{end}"+ ('' if param is None else f"_{param}"),True)
        rot_mat = rotation_matrix_from_euler_angles(np.deg2rad(plane),'xyz')
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
                if not self.move2point(self.rotate_point(rot_mat,(x,y,z)),(R,P,Y)):
                    return False
        if log_move:   
            self.state_logging(start_stop=False) 

    def move2point(self,position,orientation=None):
        """
        Move to a desired position and rotation of robot. 
        Parameters
        ----------
        position: tuple()
            Set (x,y,z) coordinates.
        rotation: tuple()
            Set (R,P,Y) angles.
        """
        steps = 0 
        position = self.limit_pos_targets(position)
        while (p.isConnected()):
            if self.ik_function:
                joint_targets = self.ik_function(position, orientation)
            else:
                joint_targets = p.calculateInverseKinematics(self.robot,self.last_joint_id, position, 
                    p.getQuaternionFromEuler(np.deg2rad(orientation)), maxNumIterations=5)[:len(self.joint_ids)]
                joint_targets = self.limit_joint_targets(joint_targets)

            if self.use_dynamics:
                p.setJointMotorControlArray(self.robot,self.joint_ids,p.POSITION_CONTROL,joint_targets,forces=[self.joint_force]*len(joint_targets))
            else:
                for i,joint_id in enumerate(self.joint_ids):
                    p.resetJointState(self.robot,joint_id,joint_targets[i])
            
            link_state = p.getLinkState(self.robot,self.last_joint_id,computeForwardKinematics = True)
            #self.display_pos_and_orn(link_state[4],link_state[5],self.last_joint_id)
            self.draw_trajectory(link_state[4],position,self.trajectory_lifetime)

            if self.quit_simulation():
                return False
            if self.use_dynamics and (np.allclose([p.getJointState(self.robot,joint_id)[0] for joint_id in self.joint_ids],
                                        joint_targets,self.ik_joint_error) or steps > self.frames_to_complete):
                break
            elif not self.use_dynamics and steps > 1:
                break   

            steps+=1
            p.stepSimulation() 
            time.sleep(self.time_step)
        return True
    
    def set_position_limits(self, x_min, x_max, y_min, y_max, z_min, z_max):
        self.axis_limits = [(x_min,x_max), (y_min,y_max), (z_min,z_max)]

    def limit_pos_targets(self,position):
        new_position = []
        for i,axis in enumerate(position):
            if self.axis_limits:
                new_position.append(np.clip(axis, self.axis_limits[i][0],  self.axis_limits[i][1]))
            else:
                new_position.append(axis)
        return new_position

    def limit_joint_targets(self, joint_targets):
        new_joint_targets = []
        for i,joint in enumerate(joint_targets):
            new_joint_targets.append(np.clip(joint, self.constraints[i][1], self.constraints[i][2]))
        return new_joint_targets

    def rotate_point(self, rot_mat, point):
        return point[0] * rot_mat[:3, 0] + point[1] * rot_mat[:3, 1] + point[2] * rot_mat[:3, 2]

    def add_attachment(self, name = 'prismatic_gripper', orientation = (-90,0,0), position = (0,0,0.3)):
        self.attachment = [name,np.deg2rad(orientation),position]

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