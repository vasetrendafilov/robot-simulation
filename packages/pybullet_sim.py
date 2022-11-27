import pybullet as p
import pybullet_data as pd
import numpy as np
import os

class PybulletSimulation:

    def __init__(self, connection_mode = p.GUI, fps = 60, gravity = (0, 0, -9.8)):
        self.connection_mode = connection_mode
        self.time_step= 1/fps
        self.gravity = gravity

    def configure(self):
        p.setTimeStep(self.time_step)
        p.setGravity(self.gravity[0],self.gravity[1],self.gravity[2])
        p.setAdditionalSearchPath(os.getcwd())
        p.setAdditionalSearchPath(pd.getDataPath())
        
        p.configureDebugVisualizer(lightPosition= (0,0,5))
        p.resetDebugVisualizerCamera( cameraDistance=10, cameraYaw=15, cameraPitch=-20, cameraTargetPosition=[0,0,0])
        #p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS,0)
    
    def load_playground1(self):
        legos=[]
        flags = p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
        p.loadURDF("tray/traybox.urdf", [0, 0, 0], [0,0,0,1], flags=flags,globalScaling=7 )
        legos.append(p.loadURDF("lego/lego.urdf",np.array([0.1, 0.3, 0.5]), flags=flags,globalScaling=10 ))
        legos.append(p.loadURDF("lego/lego.urdf",np.array([-0.1, 0.3, 0.5]), flags=flags,globalScaling=10 ))
        legos.append(p.loadURDF("lego/lego.urdf",np.array([0.1, 0.3, 0.7]), flags=flags,globalScaling=10 ))
        p.loadURDF("sphere_small.urdf",np.array( [0, 0.3, 0.6]), flags=flags,globalScaling=7 )
        p.loadURDF("sphere_small.urdf",np.array( [0, 0.3, 0.5]), flags=flags,globalScaling=7 )
        p.loadURDF("sphere_small.urdf",np.array( [0, 0.3, 0.7]), flags=flags,globalScaling=7 )

    def connect(self):
        if p.connect(self.connection_mode) != -1: # connected
            self.configure()
            return True
        return False

class Camera:
    def __init__(self,cam_tar, distance, rpy, near, far, size, fov):
        self.width, self.height = size
        self.near, self.far = near, far
        row,pitch,yaw = rpy
        self.fov = fov

        aspect = self.width / self.height
        self.view_matrix = p.computeViewMatrixFromYawPitchRoll(cam_tar,distance,yaw,pitch,row,2)
        self.projection_matrix = p.computeProjectionMatrixFOV(self.fov, aspect, self.near, self.far)

        _view_matrix = np.array(self.view_matrix).reshape((4, 4), order='F')
        _projection_matrix = np.array(self.projection_matrix).reshape((4, 4), order='F')
        self.tran_pix_world = np.linalg.inv(_projection_matrix @ _view_matrix)

    def shot(self):
        # Get depth values using the OpenGL renderer
        _, _, rgb, depth, seg = p.getCameraImage(self.width, self.height,self.view_matrix, self.projection_matrix)
        self.rgb, self.depth, self.seg = rgb, depth, seg

    def rgbd_2_world(self, w, h, d):
        x = (2 * w - self.width) / self.width
        y = -(2 * h - self.height) / self.height
        z = 2 * d - 1
        pix_pos = np.array((x, y, z, 1))
        position = self.tran_pix_world @ pix_pos
        position /= position[3]

        return position[:3]

    def rgbd_2_world_batch(self, depth):
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