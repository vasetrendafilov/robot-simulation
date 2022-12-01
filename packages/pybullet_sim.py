import pybullet as p
import pybullet_data as pd
import numpy as np
import os

class PybulletSimulation:
    """ Class to create the world in the simulation. """

    def __init__(self, connection_mode=p.GUI, fps=60, gravity=(0, 0, -9.8), cam_param=(9, 15, -20), cam_target=(0, 0, 0)):
        """ Initialise all the simulation variables.

            Parameters
            ----------
            connection_mode : int
                Choose between gui and direct
            fps : int
                Speed of the simulation
            gravity: tuple
                Set the gravity for the simulation
            cam_param: tuple
                Debugs camera distance, yaw and pitch 
            cam_target: tuple
                Set the debug camera target 
        """
        self.connection_mode = connection_mode
        self.time_step= 1/fps
        self.gravity = gravity
        self.cam_param = cam_param
        self.cam_target = cam_target
        self.flags = p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES

    def configure(self):
        """ Configure the simulation and set the search paths for importing objects. """
        p.setTimeStep(self.time_step)
        p.setGravity(self.gravity[0],self.gravity[1],self.gravity[2])
        p.setAdditionalSearchPath(os.getcwd())
        p.setAdditionalSearchPath(pd.getDataPath())
        p.configureDebugVisualizer(lightPosition= (0,0,5))
        p.resetDebugVisualizerCamera(self.cam_param[0], self.cam_param[1], self.cam_param[2], self.cam_target)
    
    def load_table(self, position = (0,0,-2.9), orientation=(0,0,0,1), scaling = 5):
        """ Load a working table for the robot. """
        p.loadURDF("table/table.urdf", position, orientation, flags=self.flags,globalScaling=scaling)
    
    def load_tray(self,position = (0,0,0.25), orientation=(0,0,0,1), scaling = 5):
        """ Load a tray for holding objects. """
        p.loadURDF("tray/traybox.urdf", position, orientation, flags=self.flags,globalScaling= scaling )

    def load_random_objects(self, count = 5, position = (0,0,1.2), orientation=(0,0,0,1), scaling = 5):
        """ Load random objects found in pybullet data. """
        for num in np.random.randint(1000, size=count):
            rand_position = np.array(position) + np.random.uniform(-1,1,3)
            p.loadURDF(f"random_urdfs/{num:03}/{num:03}.urdf", rand_position, orientation, flags=self.flags,globalScaling=scaling)
    
    def load_common_objects(self, position = (0,0,1.2), orientation=(0,0,0,1), scaling = 7):
        """ Load common objects found in pybullet data. """
        p.loadURDF("lego/lego.urdf",    np.array(position) + np.random.uniform(-1,1,3), orientation, flags=self.flags,globalScaling=scaling)
        p.loadURDF("sphere_small.urdf", np.array(position) + np.random.uniform(-1,1,3), orientation, flags=self.flags,globalScaling=scaling)
        p.loadURDF("domino/domino.urdf",np.array(position) + np.random.uniform(-1,1,3), orientation, flags=self.flags,globalScaling=scaling)
        p.loadURDF("objects/mug.urdf",  np.array(position) + np.random.uniform(-1,1,3), orientation, flags=self.flags,globalScaling=scaling)
        p.loadURDF("block.urdf",        np.array(position) + np.random.uniform(-1,1,3), orientation, flags=self.flags,globalScaling=scaling)
        p.loadURDF("duck_vhacd.urdf",   np.array(position) + np.random.uniform(-1,1,3), orientation, flags=self.flags,globalScaling=scaling)
        p.loadURDF("teddy_vhacd.urdf",  np.array(position) + np.random.uniform(-1,1,3), orientation, flags=self.flags,globalScaling=scaling)

    def connect(self):
        """ Connect and configure the pybullet simulation. """
        if p.connect(self.connection_mode) != -1: # connected
            self.configure()
            return True
        return False

class Camera:
    """ Class to take pictures from the simulation"""

    def __init__(self,cam_target, distance, rpy, near, far, size, fov):
        """ Initialise all the camera variables and compute the view and projection matrix.

            Parameters
            ----------
            cam_target : tuple
                Coordinates of the camera target
            distance: float
                Distance from the target and the camera
            rpy: tuple in degrees
                Orientation of the camera
            near, far: float
                Set the near and far threshold the camera can take
            size: tuple
                Size in pixels on the camera image
            fov: int 
                Set the field of view of the camera
        """
        self.width, self.height = size
        self.near, self.far = near, far
        row,pitch,yaw = rpy
        self.fov = fov

        aspect = self.width / self.height
        self.view_matrix = p.computeViewMatrixFromYawPitchRoll(cam_target,distance,yaw,pitch,row,1)
        self.projection_matrix = p.computeProjectionMatrixFOV(self.fov, aspect, self.near, self.far)

        _view_matrix = np.array(self.view_matrix).reshape((4, 4), order='F')
        _projection_matrix = np.array(self.projection_matrix).reshape((4, 4), order='F')
        self.tran_pix_world = np.linalg.inv(_projection_matrix @ _view_matrix)

    def shot(self):
        """ Capture and save the camera rgb, dept and segmentation values."""
        _, _, rgb, depth, seg = p.getCameraImage(self.width, self.height,self.view_matrix, self.projection_matrix)
        self.rgb, self.depth, self.seg = rgb, depth, seg

    def rgbd_2_world(self, w, h, d):
        """ Convert rdgd to world coordinates.

            Parameters
            ----------
            w : number
                Position of the pixel on the width axis
            h: number
                Position of the pixel on the hight axis
            d: number
                The depth of the pixel

            Returns
            ----------
            position: tuple
                Coordinates in the simulation world
        """
        x = (2 * w - self.width) / self.width
        y = -(2 * h - self.height) / self.height
        z = 2 * d - 1
        pix_pos = np.array((x, y, z, 1))
        position = self.tran_pix_world @ pix_pos
        position /= position[3]

        return position[:3]

    def rgbd_2_world_batch(self, depth):
        """ Convert rdgd to world coordinates in batch.

            Parameters
            ----------
            depth: list
                List of depth values

            Returns
            ----------
            position: tuple
                Coordinates in the simulation world
        """
        # reference: https://stackoverflow.com/a/62247245
        x = (2 * np.arange(0, self.width) - self.width) / self.width
        x = np.repeat(x[None, :], self.height, axis=0)
        y = -(2 * np.arange(0, self.height) - self.height) / self.height
        y = np.repeat(y[:, None], self.width, axis=1)
        z = 2 * depth - 1

        pix_pos = np.array([x.flatten(), y.flatten(), z.flatten(), np.ones_like(z.flatten())]).T
        position = self.tran_pix_world @ pix_pos.T
        position = position.T
        # print(position)

        position[:, :] /= position[:, 3:4]

        return position[:, :3].reshape(*x.shape, -1)
