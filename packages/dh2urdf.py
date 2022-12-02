import numpy as np

class DH2Urdf(object):
    """ Class to convert dh parameters to urdf and save the robotic arm."""

    def __init__(self, dh_params, constraints, attachment = None):
        """ Initialise all the converter variables.

            Parameters
            ----------
            dh_params : matrix of theta, d, a, alpha
                Load the dh parameters
            constraints : matrix of effort, lower, upper, velocity, visual
                Load the constraints
            attachment: list
                Load the attachment
        """
        self.joint_names = {0: 'revolute', 1: 'prismatic', 4: 'fixed'}
        self.dh_params = dh_params        
        self.constraints = constraints
        self.attachment = attachment
        self.xml = ''    

    def save_urdf(self, file_path):
        """ Convert the dh parameters and save the robot to urdf file.

            Parameters
            ----------
            file_path: string
                Enter the path of the file
        """
        # add a comment with the dh parameters and constraints
        self.xml = "<!-- DH Parameters and constraints\n"
        names_list = ['Name','theta','d','a','alpha','effort','lower','upper','velocity','visual']
        row_format = "{!s:>10s}" * (len(names_list))
        self.xml += row_format.format(*names_list)+str('\n')
        for DH_param, constraint in zip(self.dh_params, self.constraints):
            a = [self.joint_names[int(DH_param[0])]] + [round(float(i),4) for i in DH_param[1:]] + [round(i,4) for i in constraint]
            self.xml += row_format.format(*a)+str('\n')
        self.xml += "-->\n"

        #add fixed joint to help with inverse kinematics   
        self.dh_params.append((4,0,0,0,0)) 
        self.constraints.append([True]) 
        
        self.xml += "<robot name='robot'>\n" # robot colors
        self.xml += "\t<material name='grey'>\n\t\t<color rgba='0.6 0.6 0.6 1'/>\n\t</material>\n"
        self.xml += "\t<material name='white'>\n\t\t<color rgba='1 1 1 1'/>\n\t</material>\n"

        for i in range(len(self.dh_params)):
            # draw the joint motor
            y,z,x,r = (0,0,0,0) if i==0 else self.dh_params[i-1][1:] # transforms [i]
            visual_shapes = [[(r,0,y), (x,0,z), 0.7, 0.43, 'grey']]
            self.write_link(f'a{i}', visual_shapes, self.constraints[i][-1])
            if (i != 0):
                self.write_joint(f'fix_a{i}_to_l{i-1}', 4, f'l{i-1}', f'a{i}', (0,0,0), (0,0,0))
            # draw the link between the joints
            origins_vector = np.array([self.dh_params[i][3], 0 ,self.dh_params[i][2]],dtype = float) # transform [i+1]
            origins_vector_norm = np.linalg.norm(origins_vector)
            cyor = origins_vector/2
            rpy = (0, 0, 0)
            if (origins_vector_norm != 0.0):
                origins_vector_unit = origins_vector/origins_vector_norm
                axis = np.cross(origins_vector, np.array([0, 0, -1]))
                axis_norm = np.linalg.norm(axis)
                if (axis_norm != 0.0):
                    axis = axis/np.linalg.norm(axis)
                angle = np.arccos(origins_vector_unit @ np.array([0, 0, 1]))
                #rpy = R.from_rotvec(angle * axis).as_euler('XYZ')
                rpy = angle*axis
            visual_shapes = [[(rpy[0], rpy[1], rpy[2]),(cyor[0],cyor[1],cyor[2]),origins_vector_norm,0.3,'white']]
            if (self.dh_params[i][0] == 1): 
                visual_shapes.append([(0,0,0), (0,0,-(self.constraints[i][2]/2-0.275)), self.constraints[i][2], 0.3, 'white'])

            self.write_link(f'l{i}',visual_shapes, self.constraints[i][-1])
            self.write_joint(f'move_l{i}_from_a{i}',self.dh_params[i][0],f'a{i}',f'l{i}', (r,0,y),(x,0,z),self.constraints[i])
        
        if self.attachment: # copy and join the attachment to the robot arm
            name, orn, pos = self.attachment
            self.write_joint(f'attachment_joint',4,f'l{i}',f'base', orn, pos)
            file = open(f"attachments/{name}/{name}.urdf",'r')
            for line in file.readlines()[1:-1]:
                self.xml+= line
            file.close()    

        self.xml += "</robot>\n"
        # write the final string to the file
        file = open(file_path, "w")
        file.write(self.xml)
        file.close()

    def write_link(self, name, visual_shapes, save_visual = True):
        """ Add the link.
            Parameters
            ----------
            name: string
                Name of the link
            visual_shapes: list
                Go trough all the visual shapes and add them to the link
            save_visual: bool
                Choose to display the link
        """
        self.xml += "\t<link name='{}'>\n".format(name)
        #self.write_inertial(self.xml)
        if save_visual:
            for v in visual_shapes:
                self.write_visual_shape(v[0], v[1], v[2], v[3], v[4])
        self.xml += "\t</link>\n"

    def write_visual_shape(self, rpy, xyz, length, radius, material_name):
        """ Add visual shape to the link.
            Parameters
            ----------
            rpy: tuple
                Orientation of visual shape
            xyz: tuple
                Position of visual shape
            length,radius: float
                Length and radius of the cylinder
            material_name: string
                Choose what color is the link
        """
        self.xml += "\t\t<visual>\n"
        self.xml += "\t\t\t<origin rpy='{} {} {}' xyz='{} {} {}'/>\n".format(
            rpy[0], rpy[1], rpy[2], xyz[0], xyz[1], xyz[2], prec=5)
        self.xml += "\t\t\t<geometry>\n"
        self.xml += "\t\t\t\t<cylinder length='{}' radius='{}'/>\n".format(length, radius) 
        self.xml += "\t\t\t</geometry>\n"
        self.xml += "\t\t\t<material name='{}'/>\n".format(material_name)
        self.xml += "\t\t</visual>\n"
    
    def write_joint(self, name, joint_type, parent, child, rpy, xyz, constraints = None):
        """ Add the joint.
            Parameters
            ----------
            name: string
                Name of the joint
            joint_type: int
                Id for joint provided by pybullet
            parent: string
                Parent link connected to joint
            child: string
                Child link connected to joint
            rpy: tuple
                Orientation of visual shape
            xyz: tuple
                Position of visual shape
            constraints: list
                Constraints for the joint
        """
        self.xml += "\t<joint name='{}' type='{}'>\n".format(name, self.joint_names[int(joint_type)])
        self.xml += "\t\t<parent link='{}'/>\n".format(parent)
        self.xml += "\t\t<child link='{}'/>\n".format(child)
        self.xml += "\t\t<axis xyz='0 0 1'/>\n"
        self.xml += "\t\t<origin rpy='{} {} {}' xyz='{} {} {}'/>\n".format(
            rpy[0], rpy[1], rpy[2], xyz[0], xyz[1], xyz[2], prec=5)
        if joint_type != 4: #fixed
            self.xml += "\t\t<limit effort='{}' lower='{}' upper='{}' velocity='{}'/>\n".format(
                constraints[0], constraints[1], constraints[2], constraints[3], prec=5)
        self.xml += "\t</joint>\n" 

    def write_inertial(self, rpy, xyz, mass, inertia_xxyyzz):
        """ Add the inertial parameters to the link.
            Parameters
            ----------
            rpy: tuple
                Orientation of visual shape
            xyz: tuple
                Position of visual shape
            mass: float
                Mass of the link
            inertia_xxyyzz: tuple
                Position of the inertia
        """
        self.xml += "\t\t</inertial>\n"
        self.xml += "\t\t\t<origin rpy='{} {} {}' xyz='{} {} {}'/>\n".format(
            rpy[0], rpy[1], rpy[2], xyz[0], xyz[1], xyz[2], prec=5)
        self.xml += "\t\t<mass value='{}'/>\n".format(mass)
        self.xml += "\t\t<inertia ixx='{}' ixy='0' ixz='0' iyy='{}' iyz='0' izz='{}' />\n".format(
            inertia_xxyyzz[0], inertia_xxyyzz[1], inertia_xxyyzz[2], prec=5)
        self.xml += "\t\t</inertial>\n" 