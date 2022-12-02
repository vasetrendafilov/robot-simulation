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
    """ Class to easily create, import and interact with robotic arm."""

    def __init__(self, position=(0, 0, 0), orientation=(0, 0, 0, 1), name='my_robot', ik_function=None,fps=60, 
                    joint_forces=1000, scaling=1, use_dynamics=True, use_draw_trajectory=True, use_display_pos_and_orn=False):
        """ Initialise all the robot variables and connect to the pybullet simulation.

            Parameters
            ----------
            position : tuple
                Base position of the robot arm
            orientation : tuple
                Base orientation of the robot arm
            name: string
                Name of the robot arm
            ik_function: fuc(pos, orn)
                Add function to calculate inverse kinematics
            fps: int
                Speed of the simulation
            joint_forces: float
                Maximal force applied to the joints
            scaling: float
                Set the scaling of the robot when loading it
            use_dynamics: bool
                Choose how to step trough the simulation
            use_draw_trajectory: bool
                Choose to draw the trajectory from the end effector position
            use_display_pos_and_orn: bool
                Choose to display the current position and orientation   
        """
        # sympy vars
        self.links, self.joint_variables, self.constraints= [],[],[]
        self.subs_joints, self.subs_additional = [],[]
        # load robot vars
        self.robot = None
        self.file_head, self.name =  'robot_arms/' + name, name
        self.is_imported, self.is_foreign = False, False
        self.position, self.orientation = position, orientation
        self.scaling = scaling
        # joint motor control vars
        self.use_dynamics = use_dynamics
        self.frames_to_complete = 20
        self.joint_forces = joint_forces
        self.axis_limits = []
        # ik vars
        self.ik_joint_error = 0.01
        self.use_orientation_ik = True
        self.ik_function = ik_function
        # attachment vars
        self.attachment_joint_ids = []
        self.change_attachment_force = False
        self.attachment, self.attachment_button = None, None
        self.attachment_open_targets, self.attachment_close_targets = (0,0), (0.3,-0.3)
        # draw trajectory vars
        self.use_draw_trajectory = use_draw_trajectory
        self.trajectory_lifetime = 5
        self.hasPrevPose = [False]*10
        self.prevPose1, self.prevPose2 = [[0,0,0]]*10, [[0,0,0]]*10
        # capture image vars
        self.capture_image_button = None
        self.prev_button_state = 2
        self.camera_offset = (0,0,0.5)
        # buttons
        self.quit_button = None
        self.pose_text = None
        # state logging
        self.use_display_pos_and_orn = use_display_pos_and_orn
        self.log, self.record_button = None,None
        self.rec_cnt = 1
        # connect to simulation
        if not p.isConnected():
            sim = PybulletSimulation(fps=fps)
            if sim.connect():
                self.time_step = sim.time_step
        else:
            self.time_step = 1/fps
    
    def add_pose_sliders(self, x_range=(-5, 5, 1), y_range=(-5, 5, 1), z_range=(-5, 5, 1)):
        """ Adds position and orientation sliders to pybullet for selecting desired position and orientation of the end effector.

            Parameters
            ----------
            x_range, y_range, z_range : tuple
                Minimal, maximal and default position of the slider for each axis
        """
        self.pose_sliders = []
        self.pose_sliders.append(p.addUserDebugParameter(f"x", x_range[0], x_range[1], x_range[2]))
        self.pose_sliders.append(p.addUserDebugParameter(f"y", y_range[0], y_range[1], y_range[2]))
        self.pose_sliders.append(p.addUserDebugParameter(f"z", z_range[0], z_range[1], z_range[2]))
        if self.use_orientation_ik:
            self.pose_sliders.append(p.addUserDebugParameter(f"R", -180, 180, 0))
            self.pose_sliders.append(p.addUserDebugParameter(f"P", -180, 180, 0))
            self.pose_sliders.append(p.addUserDebugParameter(f"Y", -180, 180, 0))

    def add_joint_sliders(self):
        """ Adds joint sliders to pybullet for controlling each joint where the range of the sliders is taken from the joint constrains. """
        self.joint_sliders = []
        for i in self.joint_ids:
            joint_info = p.getJointInfo(self.robot, i)
            if joint_info[2] == p.JOINT_PRISMATIC:
                self.joint_sliders.append((False, p.addUserDebugParameter(f"D{i}", joint_info[8], joint_info[9], joint_info[8])))
            # convert slider range to degrees for rotational joins
            if joint_info[2] == p.JOINT_REVOLUTE:
                self.joint_sliders.append((True, p.addUserDebugParameter(f"Theta{i}", np.rad2deg(joint_info[8]), np.rad2deg(joint_info[9]), 0)))

    def add_joint_frame_lines(self):
        """ Adds frame lines to x and z axis to each joint."""
        self.joint_frames = []
        if self.is_foreign is True:  # Adds frame lines to just the last joint if the robot is foreign
            rot = np.array(p.getMatrixFromQuaternion(p.getLinkState(self.robot, self.last_joint_id)[5])).reshape(3, 3)
            line_n, line_a = self.rotate_point(rot, (0.8, 0, 0)), self.rotate_point(rot, (0, 0, 0.8))
            self.joint_frames.append(p.addUserDebugLine((0, 0, 0), line_n, parentObjectUniqueId=self.robot,
                                        parentLinkIndex=self.last_joint_id, lineColorRGB=(0, 1, 1), lineWidth=2))
            self.joint_frames.append(p.addUserDebugLine((0, 0, 0), line_a, parentObjectUniqueId=self.robot,
                                        parentLinkIndex=self.last_joint_id, lineColorRGB=(1, 1, 0), lineWidth=2))
        else:
            for joint_id in self.joint_ids:
                self.subs_joints[joint_id//2] = self.subs_joints[joint_id//2][0], p.getJointState(self.robot, joint_id)[0]
            for joint in range(0, self.last_joint_id, 2):
                transform = self.get_dh_joint_to_joint(joint//2, joint//2+1).subs(self.subs_joints+self.subs_additional).evalf()
                line_n, _, line_a = frame_lines(transform, 0.8)
                self.joint_frames.append(p.addUserDebugLine(line_n.T[0,:], line_n.T[1,:],
                                            (1, 0, 0) if self.last_joint_id-2 != joint else (0, 1, 1), 2, 0, self.robot, joint))
                self.joint_frames.append(p.addUserDebugLine(line_a.T[0,:],line_a.T[1,:],
                                            (0, 0, 1) if self.last_joint_id-2 != joint else (1, 1, 0), 2, 0, self.robot, joint))

    def find_joint_ids(self):
        """ Goes trough each joint of the robot arm and saves the body and attachment joint if it is prismatic of revolute. """
        self.joint_ids = []
        add_attachment_joints = False
        add_constraints_if_foreign = False
        for i in range(p.getNumJoints(self.robot)):
            joint_info = p.getJointInfo(self.robot, i)
            # start adding attachment joints if joint name is attachment_joint
            if joint_info[1] == b'attachment_joint' or add_attachment_joints:
                if add_attachment_joints == False:
                    self.last_joint_id = i-1
                add_attachment_joints = True
                if (joint_info[2] in [p.JOINT_PRISMATIC, p.JOINT_REVOLUTE]):
                    self.attachment_joint_ids.append(i)
            else:
                if (joint_info[2] in [p.JOINT_PRISMATIC, p.JOINT_REVOLUTE]):
                    self.joint_ids.append(i)
                    # special case where if robot is foreign or there are not any constraints read them from the urdf
                    if self.is_foreign and (len(self.constraints) == 0 or add_constraints_if_foreign):
                        add_constraints_if_foreign = True
                        self.constraints.append(
                            [joint_info[10], joint_info[8], joint_info[9], joint_info[11], True])
        if add_attachment_joints is False:
            self.last_joint_id = i

    def reset_joints(self, joint_states=None):
        """ Reset each joint to default value or the provided array.
        
            Parameters
            ----------
            joint_states : tuple or array
                List of joint states to reset the arm to
        """
        for i, joint_id in enumerate(self.joint_ids):
            joint_info = p.getJointInfo(self.robot, joint_id)
            if joint_states:
                p.resetJointState(self.robot, joint_id, joint_states[i])
            else: # set to lower limit of joint if prismatic or zero if revolute
                p.resetJointState(self.robot, joint_id, joint_info[8] if joint_info[2] == p.JOINT_PRISMATIC else 0)
    
    def load_robot(self):
        """ Load the robot arm in pybullet, find the joints, add joint frame lines and reset the joints. """
        if self.is_imported is False:
            subs = self.subs_joints + self.subs_additional
            DH_params = sp.Matrix(self.links).subs(subs).evalf()
            dh2urdf = DH2Urdf(DH_params.tolist(),self.constraints, self.attachment)
            # create the directory if it does not exist and save the robot if it is not imported
            if not os.path.exists(f'{self.file_head}/'):
                os.mkdir(f'{self.file_head}/')
            dh2urdf.save_urdf(f'{self.file_head}/{self.name}.urdf')

        self.robot = p.loadURDF(f'{self.file_head}/{self.name}.urdf', self.position, self.orientation, 
                                    useFixedBase=True, globalScaling=self.scaling)
        self.find_joint_ids()
        self.add_joint_frame_lines()
        self.reset_joints()
        if self.attachment_joint_ids: # open the gripper so it works with move2point
            self.actuate_attachment(joint_targets = self.attachment_open_targets)
    
    def interact(self, kinematics='forward', use_orientation_ik=True, x_range=(-5, 5, 1), y_range=(-5, 5, 1), z_range=(-5, 5, 1)):
        """ Interact with the robot arm where the robot arm is loaded in pybullet, depending on the kinematics 
            choose what sliders to add and start the simulation.
        
            Parameters
            ----------
            kinematics : string
                You can choose between forward or inverse kinematics for robot arm    
            use_orientation_ik: bool
                If you are using inverse kinematics choose whatever to use orientation too
            x_range, y_range, z_range : tuple
                Minimal, maximal and default position of the slider for each axis
        """
        self.kinematics = kinematics
        self.use_orientation_ik = use_orientation_ik
        self.load_robot()
        if self.kinematics == 'forward':
            self.add_joint_sliders()
        elif self.kinematics == 'inverse':
            self.add_pose_sliders(x_range, y_range, z_range)
        self.step()

    def step(self):
        """ Start the simulation in pybullet, read the desired sliders for forward or inverse kinematics, 
            control the joints dynamically or just reset them to the new position and step trough all the modules of the simulation.
        """
        while (p.isConnected()):
            p.stepSimulation()
            if self.kinematics == 'forward':
                position = None # read the sliders for joint desired joints position
                joint_targets = [(np.deg2rad(p.readUserDebugParameter(parameter)) if revolute else p.readUserDebugParameter(parameter))
                                    for revolute, parameter in self.joint_sliders]
            elif self.kinematics == 'inverse':
                # read the sliders for the desired position and orientation
                position = (p.readUserDebugParameter(self.pose_sliders[0]), 
                            p.readUserDebugParameter(self.pose_sliders[1]), 
                            p.readUserDebugParameter(self.pose_sliders[2]))
                if self.use_orientation_ik:
                    orientation = p.getQuaternionFromEuler([np.deg2rad(p.readUserDebugParameter(self.pose_sliders[3])),
                                                            np.deg2rad(p.readUserDebugParameter(self.pose_sliders[4])),
                                                            np.deg2rad(p.readUserDebugParameter(self.pose_sliders[5]))])
                if self.ik_function: # use inverse kinematics function if it is provided 
                    joint_targets = self.ik_function(self.limit_pos_target(position), orientation if self.use_orientation_ik else None)
                else:  # use the build in numerical ik and limit the joint and position targets
                    joint_targets = p.calculateInverseKinematics(self.robot, self.last_joint_id, self.limit_pos_target(position),
                        orientation if self.use_orientation_ik else None, maxNumIterations=5)[:len(self.joint_ids)]
                    joint_targets = self.limit_joint_targets(joint_targets)

            if self.use_dynamics: 
                p.setJointMotorControlArray(self.robot, self.joint_ids, p.POSITION_CONTROL, joint_targets, 
                    forces= self.joint_forces if type(self.joint_forces) in (list,tuple) else [self.joint_forces]*len(joint_targets))
            else:
                for i, joint_id in enumerate(self.joint_ids):
                    p.resetJointState(self.robot, joint_id, joint_targets[i])
           
            # modules for the simulation
            if self.attachment_joint_ids:
                self.actuate_attachment()

            self.state_logging(self.kinematics)
            link_state = p.getLinkState(self.robot, self.last_joint_id)
            self.capture_image(link_state,self.camera_offset)

            if self.use_draw_trajectory:
                self.draw_trajectory(link_state[4], position, self.trajectory_lifetime)
            if self.use_display_pos_and_orn:
                self.display_pos_and_orn(link_state[4], link_state[5], self.last_joint_id)
            if self.quit_simulation():
                break
            time.sleep(self.time_step)
    
    def display_pos_and_orn(self, position, orientation, link_id):
        """ Display the position and orientation near the desired link.
        
            Parameters
            ----------
            position: tuple
                Coordinates of the link position
            orientation: tuple
                Orientation of the link in a quaternion
            link_id:
                Id of the parent link index
        """
        x, y, z = position
        R, P, Y = np.rad2deg(p.getEulerFromQuaternion(orientation))
        if self.pose_text is None:
            self.pose_text = p.addUserDebugText(f"(x:{x:.2f} y:{y:.2f} z:{z:.2f}) (R:{R:.1f} P:{P:.1f} Y:{Y:.1f})", [0.5, 0, 0.5],
                textColorRGB=[0, 0, 0], textSize=1, parentObjectUniqueId=self.robot, parentLinkIndex=link_id)
        p.addUserDebugText(f"(x:{x:.2f} y:{y:.2f} z:{z:.2f}) (R:{R:.1f} P:{P:.1f} Y:{Y:.1f})", [0.5, 0, 0.5], textColorRGB=[0, 0, 0], 
            textSize=1,parentObjectUniqueId=self.robot, parentLinkIndex=link_id, replaceItemUniqueId=self.pose_text)
     
    def draw_trajectory(self, current, target=None, life_time=15, line_index=0):
        """ Draw the current and target trajectory where you can choose the life time of the lines and index to differentiate the lines.
        
            Parameters
            ----------
            current, target: tuple
                Coordinates of the current and target link
            life_time: int
                Life time of the lines in seconds
            line_index: int
                Index of the lines
        """
        if (self.hasPrevPose[line_index] is True): # don't draw the first time 
            if target:
                p.addUserDebugLine(self.prevPose1[line_index], target, [0, 0, 0.3], 1, life_time)
            p.addUserDebugLine(self.prevPose2[line_index], current, [1, 0, 0], 1, life_time)
        self.prevPose1[line_index] = target
        self.prevPose2[line_index] = current
        self.hasPrevPose[line_index] = True
    
    def quit_simulation(self, quit_now = False):
        """ Adds button to pybullet to quit the simulation and returns conformation.
        
            Parameters
            ----------
            quit_now: bool
                Optional argument to quit the simulation trough code
        """
        if not quit_now and self.quit_button is None:
            self.quit_button = p.addUserDebugParameter("Quit Simulation",1,0,1)
        else:
            if quit_now or p.readUserDebugParameter(self.quit_button) %2 == 0:
                p.disconnect()
                return True
            else:
                return False
    
    def actuate_attachment(self, joint_ids=None, joint_targets=None):
        """ Actuate the attachment of the robot arm, the current implementation is for opening and closing a gripper. 
            If the parameters are not used, the function will default to all attachment joint ids and targets.  
        
            Parameters
            ----------
            joint_ids: list
                Chose which joints to actuate 
            joint_targets: list
                Provide the joints target for use when controlling with code
        """
        if joint_ids is None:
            joint_ids = self.attachment_joint_ids
        if joint_targets is None:
            joint_targets = self.attachment_open_targets
            if self.attachment_button is None: # add button to open and close the gripper
                self.attachment_button = p.addUserDebugParameter("Close/Open Gripper",1,0,1)
            else:
                if p.readUserDebugParameter(self.attachment_button) %2 == 0: # close the gripper
                    joint_targets = self.attachment_close_targets
                else:
                    joint_targets = self.attachment_open_targets
                    self.change_attachment_force = False 
            # dynamically control the joints
            max_force, joint_stopped = 500, True
            for joint_state in p.getJointStates(self.robot, self.attachment_joint_ids):
                if abs(joint_state[3]) < 100:
                    joint_stopped = False
            if (joint_stopped and joint_targets == self.attachment_close_targets) or self.change_attachment_force: 
                self.change_attachment_force = True 
                max_force = 50
            p.setJointMotorControlArray(self.robot, joint_ids, p.POSITION_CONTROL, joint_targets, forces=[max_force]*len(joint_ids))
        else: # make at least 30 steps to close or open the attachment
            for step in range(30):
                p.stepSimulation()
                max_force, joint_stopped = 500, True
                for joint_state in p.getJointStates(self.robot, self.attachment_joint_ids):
                    if abs(joint_state[3]) < 100:
                        joint_stopped = False
                if (joint_stopped and joint_targets == self.attachment_close_targets) or self.change_attachment_force: 
                    self.change_attachment_force = True 
                    max_force = 50 
                p.setJointMotorControlArray(self.robot, joint_ids, p.POSITION_CONTROL, joint_targets, forces=[max_force]*len(joint_ids))
                time.sleep(self.time_step)
    
    def capture_image(self, link_state = None, camera_offset=(0, 0, 0.5), near=0.1, far=5, size=(320, 320), fov=40, capture_now=False):
        """ Capture image from the position and orientation of the link, in this case the image is taken in line of the z axis. 
            You can set all the parameters from here and there is an option to capture the image imminently and return the image.  
        
            Parameters
            ----------
            link_state: list
                List that comes from getLinkState and used to get the position and orientation of the link
            camera_offset: tuple
                Offset from the link to set the target of the camera
            near, far: float
                Set the near and far threshold the camera can take
            size: tuple
                Size in pixels on the camera image
            fov: int 
                Set the field of view of the camera
            capture_now: bool
                Optional argument to take the shot trough code
        """
        if link_state is None:
            link_state = p.getLinkState(self.robot, self.last_joint_id)
        if capture_now is False:
            if self.capture_image_button is None: # add button to capture image
                self.capture_image_button = p.addUserDebugParameter("Capture Image", 1, 0, 1)
            else:
                if p.readUserDebugParameter(self.capture_image_button) == self.prev_button_state:
                    self.prev_button_state += 1
                    capture_now = True
        if capture_now:
            rot = np.array(p.getMatrixFromQuaternion(link_state[5])).reshape(3, 3)
            offset = self.rotate_point(rot, camera_offset)
            target = (link_state[4][0]+offset[0], link_state[4][1]+offset[1], link_state[4][2]+offset[2])
            orn = p.getEulerFromQuaternion(link_state[5])
            camera = Camera(target, 0.01, (np.rad2deg(orn[2]), np.rad2deg(-orn[0]), np.rad2deg(orn[1])), near, far, size, fov)
            camera.shot()
            return camera

    def state_logging(self, log_name='', start_stop=None, object_ids=None):
        """ Start and stop logging any object in the simulation. You can set the log name and object ids, also you can call the
            function trough code to not depend on the button to start. Every log ends with recording step to sort the files.  
        
            Parameters
            ----------
            log_name: string
                Name of the log 
            start_stop: string
                Optional parameter to choose to start or stop logging
            object_ids: list
                Enter the ids of the objects you like to log
        """
        if object_ids is None: # log just the robot arm
            object_ids = [self.robot]

        if start_stop is None:
            if self.record_button is None: # add button to start and stop logging
                self.record_button = p.addUserDebugParameter("Start/Stop Logging", 1, 0, 1)
            else:
                if p.readUserDebugParameter(self.record_button) % 2 == 0:  # start logging
                    if self.log is None:  # start only once
                        if not os.path.exists(f'{self.file_head}/logs/'):
                            os.mkdir(f'{self.file_head}/logs/') # make directory if it does not exist
                        self.log = p.startStateLogging(p.STATE_LOGGING_GENERIC_ROBOT,
                            f"{self.file_head}/logs/{log_name}_{int(p.readUserDebugParameter(self.record_button)//2)}.txt", object_ids)
                elif self.log is not None:
                    p.stopStateLogging(self.log)
                    self.log = None
        else:
            if start_stop == 'start':  # start the log trough code
                if not os.path.exists(f'{self.file_head}/logs/'):
                    os.mkdir(f'{self.file_head}/logs/')
                self.log = p.startStateLogging(p.STATE_LOGGING_GENERIC_ROBOT,
                                f"{self.file_head}/logs/{log_name}_{self.rec_cnt}.txt", object_ids)
                self.rec_cnt += 1
            else:
                p.stopStateLogging(self.log)
    
    def readLogFile(self, file_path):
        """ Read the file, decode the message and return the logs in a list. """
        f = open(file_path, 'rb')
        keys = f.readline().decode('utf8').rstrip('\n').split(',')
        fmt = f.readline().decode('utf8').rstrip('\n')
        sz = struct.calcsize(fmt)
        # read the file
        whole_file = f.read()
        chunks = whole_file.split(b'\xaa\xbb')
        log = []
        for chunk in chunks:
            if len(chunk) == sz:
                values = struct.unpack(fmt, chunk)
                record = []
                for i in range(len(fmt)):
                    record.append(values[i])
                log.append(record)
        return log

    def autocomplete_logs(self, log_names):
        """ Find similar files to the `log_names` parameter which can be one name or a list of names and return the combined logs. """
        def sortKeyFunc(s): # sort using the last index in the file name
            return int(os.path.basename(s).split('_')[-1][:-4])
        log = []
        if type(log_names) != list:
            log_names = [log_names]
        for log_name in log_names:
            for log_name_index in sorted(glob.glob(f'{self.file_head}/logs/'+f"{log_name}*.txt"), key=sortKeyFunc):
                log += self.readLogFile(log_name_index)
        return log

    def replay_logs(self, log_names, skim_trough=True, object_ids=None):
        """ Replay the logs found in `log_names` parameter and choose to skim_trough them.   
        
            Parameters
            ----------
            log_names: string or list of strings
                Name of the log file you want to replay 
            skim_trough: bool
                Choose to skim trough the logs with a slider or step trough them
            object_ids: list
                Enter the ids of the objects in the log file 
        """
        if object_ids is None: # by default it is just the robot arm 
            object_ids = [self.robot]

        logs = self.autocomplete_logs(log_names)
        if not logs:
            return "No logs found"

        recordNum, objectNum = len(logs), len(object_ids)
        if skim_trough: # add slider to replay steps
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
                Id = record[2] # set object position and orientation 
                p.resetBasePositionAndOrientation(Id, [record[3], record[4], record[5]], 
                                                      [record[6], record[7], record[8], record[9]])
                for i in range(p.getNumJoints(Id)):
                    qIndex = p.getJointInfo(Id, i)[3]
                    if qIndex > -1:
                        if self.use_dynamics:
                            p.setJointMotorControl2(Id, i, p.POSITION_CONTROL, record[qIndex - 7 + 17],
                            force= self.joint_forces[i] if type(self.joint_forces) in (list,tuple) else self.joint_forces)
                        else:
                            p.resetJointState(Id, i, record[qIndex - 7 + 17])

            link_state = p.getLinkState(self.robot,self.last_joint_id)
            if self.use_draw_trajectory:
                self.draw_trajectory(link_state[4], life_time = self.trajectory_lifetime)
            if self.use_display_pos_and_orn:
                self.display_pos_and_orn(link_state[4], link_state[5], self.last_joint_id)

            if step_index == recordNum / objectNum - 1 and skim_trough is False:
                break # break condition for the last log
            if self.quit_simulation():
                break
            time.sleep(self.time_step)

    def convert_logs_to_prg(self, log_names, file_name):
        """ Find similar files to the `log_names` parameter and convert the logs to RT Toolbox 2 program file.
        
            Parameters
            ----------
            log_names: string or list of strings
                Name of the log file you want to convert 
            file_name: string
                Name of the file
        """
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
        file = open(f"{self.file_head}/{file_name}.prg", "w")
        file.write(pgr_string)
        file.close() 

    def import_robot(self, file_path='robot_arms/my_robot/my_robot.urdf'):
        """ Import robot generated from this script. Save the name and file head form the file path and read the dh parameters 
            and constraints from the file."""
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
                    np.rad2deg(float(dh_list[6])),np.rad2deg(float(dh_list[7])),float(dh_list[8]),float(dh_list[5]),int(dh_list[9]))
            elif dh_list[0] == 'prismatic':
                self.add_prismatic_joint(float(dh_list[1]),sp.symbols(f'd{i+1}'),float(dh_list[3]),float(dh_list[4]),
                    float(dh_list[6]),float(dh_list[7]),float(dh_list[8]),float(dh_list[5]),int(dh_list[9]))
            else:
                self.add_fixed_joint(float(dh_list[1]),float(dh_list[2]),float(dh_list[3]),float(dh_list[4]),int(dh_list[9]))
    
    def import_foreign_robot(self, file_path, scaling=5):
        """ Import a custom urdf robot, set the scaling and save the name and file head form the file path."""
        self.is_imported = self.is_foreign = True
        self.scaling = scaling
        file_head, tail = os.path.split(file_path)
        self.file_head = file_head
        self.name = tail[:-5]

    def write_text(self, text, position, orientation, spacing=1, plane=(0, 0, 0), scale=0.5, log_text=False):
        """ Write text at a desired position and orientation. Set the spacing and scale of the letters.   
        
            Parameters
            ----------
            text: string
                Input the text in uppercase and use the available letters
            position: list
                Choose the top left corner of the text 
            orientation: list in degrees
                Orientation of the end effector of the robot arm
            spacing: float
                Spacing between the letters
            plane: list in degrees
                Rotate the text around the 3 axes
            scale: float
                Set the scale of the letters
            log_text: bool
                Choose to log the text
        """
        if log_text:
            self.state_logging(text,'start')
        for letter in text:
            self.write_letter(letter, position, orientation, plane, scale)
            position[1] += spacing
        if log_text:
            self.state_logging(start_stop='stop')

    def write_letter(self, letter, position, orientation, plane=(0, 0, 0), s=0.5):
        """ Write letter at a desired position and orientation. Set the plane and scale of the letter.   
        
            Parameters
            ----------
            letter: string
                Input the letter in uppercase and use the available letters
            position: list
                Choose the top left corner of the letter 
            orientation: list in degrees
                Orientation of the end effector of the robot arm
            plane: list in degrees
                Rotate the letter around the 3 axes
            scale: float
                Set the scale of the letter
        """
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

    def move(self, start, end, plane=(0, 0, 0), interpolation='linear', steps=30, param=None, closed=False, log_move=False):
        """ Interpolate a trajectory between two points. Choose the interpolation type for now there is linear and circular, 
            set the number of steps and additional parameters for the circular interpolation. You can also log the moves.
        
            Parameters
            ----------
            start, end: tuple of points and rotations in degrees
                Desired position and orientation of the end effector
            plane: list in degrees
                Rotate the point around the 3 axes 
            interpolation: string
                Choose between linear or circular
            step: int
                Number of points between the start and end
            param: list with a, b, result_num and side
                Set a and b parameters for the ellipse size and choose the between the two results and side to interpolate
            closed: bool
                Interpolate the whole ellipse
            log_move: bool
                Optional parameter to log the moves
        """
        self.hasPrevPose = [False]*5 # restart drawing trajectory
        x1, y1, z1, R1, P1, Y1 = start
        x2, y2, z2, R2, P2, Y2 = end
        if log_move: # first move to point then start logging for cleaner log
            self.move2point((x1,y1,z1),(R1,P1,Y1)) 
            self.hasPrevPose = [False]*5  
            self.state_logging(f"move_{interpolation}_{start}_{end}"+ ('' if param is None else f"_{param}"),'start')
        rot_mat = rotation_matrix_from_euler_angles(np.deg2rad(plane),'xyz')
        if interpolation == 'linear':
            for x,y,z,R,P,Y in zip(np.linspace(x1,x2,steps),np.linspace(y1,y2,steps),np.linspace(z1,z2,steps),
                                   np.linspace(R1,R2,steps),np.linspace(P1,P2,steps),np.linspace(Y1,Y2,steps)):
                if not self.move2point((x,y,z),(R,P,Y)):
                    return False
        elif interpolation == 'circular' and param != None:
            xs,ys = sp.symbols('xs,ys')
            a,b,res_num,side = param
            # solve the equation to find the points
            eq1 = sp.Eq((xs-x1)**2/a**2+(ys-y1)**2/b**2,1)
            eq2 = sp.Eq((xs-x2)**2/a**2+(ys-y2)**2/b**2,1)
            results = sp.solve([eq1,eq2],(xs,ys))
            if not all(x[0].is_real or x[1].is_real for x in results): 
                print("No result for the parameters a and b")
                return False
            # calculate the starting and ending angle
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
            self.state_logging(start_stop='stop') 

    def move2point(self, position, orientation):
        """ Start the simulation and move to the point and orientation.
        
            Parameters
            ----------
            position: tuple (x, y, z)
                Target coordinates
            orientation: tuple (R, P, Y) in degrees
                Target orientation of the end effector
        """
        steps = 0 
        position = self.limit_pos_target(position)
        while (p.isConnected()):
            if self.ik_function:
                joint_targets = self.ik_function(position, orientation)
            else:
                joint_targets = p.calculateInverseKinematics(self.robot,self.last_joint_id, 
                                    position, p.getQuaternionFromEuler(np.deg2rad(orientation)))[:len(self.joint_ids)]
                joint_targets = self.limit_joint_targets(joint_targets)

            if self.use_dynamics:
                p.setJointMotorControlArray(self.robot,self.joint_ids,p.POSITION_CONTROL,joint_targets,
                forces= self.joint_forces if type(self.joint_forces) in (list,tuple) else [self.joint_forces]*len(joint_targets))
            else:
                for i,joint_id in enumerate(self.joint_ids):
                    p.resetJointState(self.robot,joint_id,joint_targets[i])
            # modules for the simulation
            link_state = p.getLinkState(self.robot,self.last_joint_id)
            if self.use_draw_trajectory:
                self.draw_trajectory(link_state[4], position, self.trajectory_lifetime)
            if self.use_display_pos_and_orn:
                self.display_pos_and_orn(link_state[4], link_state[5], self.last_joint_id)

            if self.quit_simulation():
                return False
            # end condition if the target and current joints are close or enough steps are completed
            if self.use_dynamics and (np.allclose([p.getJointState(self.robot,joint_id)[0] for joint_id in self.joint_ids],
                                        joint_targets,self.ik_joint_error) or steps > self.frames_to_complete):
                break
            elif not self.use_dynamics and steps > 1: # without dynamics just wait one step
                break   
            steps+=1
            p.stepSimulation() 
            time.sleep(self.time_step)
        return True
    
    def set_dynamic_conditions(self, frames_to_complete=20, ik_joint_error = 0.01):
        """ Set frames to complete and inverse kinematics joint error flags for stopping the simulation. """
        self.frames_to_complete = frames_to_complete
        self.ik_joint_error = ik_joint_error
    
    def get_joint_states(self):
        """ Returns joints position, velocity and applied torque. """
        return [(row[0],row[1],row[3]) for row in p.getJointStates(self.robot, self.joint_ids)]

    def set_position_limits(self, x_min, x_max, y_min, y_max, z_min, z_max):
        """ Set lower and upper limits for each axis. """
        self.axis_limits = [(x_min,x_max), (y_min,y_max), (z_min,z_max)]

    def limit_pos_target(self, position):
        """ Limit the position target if the limits are set. """
        new_position = []
        for i,axis in enumerate(position):
            if self.axis_limits:
                new_position.append(np.clip(axis, self.axis_limits[i][0],  self.axis_limits[i][1]))
            else:
                new_position.append(axis)
        return new_position

    def limit_joint_targets(self, joint_targets):
        """ Limit the joint targets to the constraints of the robot. """
        new_joint_targets = []
        for i,joint in enumerate(joint_targets):
            new_joint_targets.append(np.clip(joint, self.constraints[i][1], self.constraints[i][2]))
        return new_joint_targets

    def rotate_point(self, rot_mat, point):
        """ Project the point on the rotation matrix. """
        return point[0] * rot_mat[:3, 0] + point[1] * rot_mat[:3, 1] + point[2] * rot_mat[:3, 2]

    def add_attachment(self, name='prismatic_gripper', offset=(0, 0, 0.3), orientation=(-90, 0, 0)):
        """ Add attachment to the robot arm. All the attachments are in the `attachment` folder
            and you can choose or build a new one.  
        
            Parameters
            ----------
            name: string
                Name of the attachment
            offset: tuple
                Offset from the end effector of the robot arm 
            orientation: tuple in degrees
                Orientation of the attachment
        """
        self.attachment = [name,np.deg2rad(orientation),offset]

    def add_revolute_joint(self, theta, d, a, alpha, lower=-180, upper=180, velocity=2.6, effort=10, visual=True):
        """Add a revolute joint to the robotic arm according to the DH convention and set the joint constraints.
    
            Parameters
            ----------
            theta: symbol
                Angle of rotation around z-axis
            d: symbol or number
                Displacement along z-axis
            a: symbol or number
                Displacement along x-axis
            alpha: symbol or number
                Angle of rotation around x-axis
            lower: number in degrees
                Lower limit of the joint
            upper: number in degrees
                Upper limit of the joint
            velocity: number
                Maximal velocity
            effort: number
                Maximal effort
            visual: bool
                Choose to display the joint
        """
        self.links.append([p.JOINT_REVOLUTE, theta, d, a, alpha])
        self.joint_variables.append(theta)
        self.subs_joints.append((theta, 0))
        self.constraints.append([effort, np.deg2rad(lower), np.deg2rad(upper), velocity, visual])

    def add_prismatic_joint(self, theta, d, a, alpha, lower = 0.7, upper = 3, velocity = 2.6, effort = 10, visual = True):
        """Add a prismatic joint to the robotic arm according to the DH convention and set the joint constraints.
    
            Parameters
            ----------
            theta: symbol or number
                Angle of rotation around z-axis
            d: symbol
                Displacement along z-axis
            a: symbol or number
                Displacement along x-axis
            alpha: symbol or number
                Angle of rotation around x-axis
            lower: number
                Lower limit of the joint
            upper: number
                Upper limit of the joint
            velocity: number
                Maximal velocity
            effort: number
                Maximal effort
            visual: bool
                Choose to display the joint
        """
        self.links.append([p.JOINT_PRISMATIC, theta, d, a, alpha])
        self.joint_variables.append(d)
        self.subs_joints.append((d, 0))
        self.constraints.append([effort, lower, upper, velocity, visual])
        
    def add_fixed_joint(self, theta, d, a, alpha, visual=True):
        """Add a fixed joint to the robotic arm according to the DH convention and set the joint constraints.
    
            Parameters
            ----------
            theta: symbol or number
                Angle of rotation around z-axis
            d: symbol or number
                Displacement along z-axis
            a: symbol or number
                Displacement along x-axis
            alpha: symbol or number
                Angle of rotation around x-axis
            visual: bool
                Choose to display the joint
        """ 
    
        self.links.append([p.JOINT_FIXED, theta, d, a, alpha])
        self.subs_joints.append((sp.symbols('temp'+str(np.random.random(1)[0]*100)), 0))
        self.constraints.append([0,0,0,0,visual])
    
    def add_subs(self, subs):
        """ Add the symbol values for plotting purposes.

        Parameters
        ----------
        subs:  [(symbol1, value1), (symbol2, value2), ... (symbol3, value3)]
            List of tuples, each consisted of a symbol and its value
        """
        self.subs_additional = subs
    
    def get_dh_joint_to_joint(self, start_joint, end_joint):
        """ Get the DH model subsection transformation matrix for the joint id range(start_joint, end_joint).
        
        Parameters
        ----------
        start_joint:  int
            Starting joint id of the desired dh model subsection.
        end_joint:  int
            Final joint id of the desired dh model subsection.
        
        Returns
        -------
            pose: symbol matrix
                DH model subsection transformation matrix for joint id range(start_joint, end_joint)
        """
        pose = hpose3()
        for link in self.links[start_joint:end_joint]:
            _, theta, d, a, alpha = link
            pose = pose * dh_joint_to_joint(theta, d, a, alpha)
        pose.simplify()
        return pose

    def get_dh_matrix(self):
        """ Get the DH model transformation matrix for the whole robotic arm. """
        return self.get_dh_joint_to_joint(start_joint=0, end_joint=len(self.links))
    
    def get_dh_table(self):
        """ Return the DH table. """
        return sp.Matrix(self.links)[:, 1:]
    
    def linear_jacobian(self):
        """ Return the linear jacobian for the robotic arm. """
        linear_jacobian = self.get_dh_matrix()[:3, 3].jacobian(self.joint_variables)
        linear_jacobian.simplify()
        return linear_jacobian
    
    def angular_jacobian(self):
        """ Return the angular jacobian for the robotic arm. """
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
        """ Return the jacobian for the robotic arm. """
        return sp.Matrix.vstack(self.linear_jacobian(), self.angular_jacobian())