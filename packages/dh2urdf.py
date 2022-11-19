import numpy as np

class DH2Urdf(object):

    def __init__(self, DH_Params, constraints, attachment = None):
        self.joint_names = {0: 'revolute', 1: 'prismatic', 4: 'fixed'}
        self.DH_Params = DH_Params        
        self.constraints = constraints
        self.attachment = attachment
        self.xml = ''    

    def save_urdf(self, file_name):
        self.xml = "<!-- DH Parameters and constraints\n"
        names_list = ['Name','theta','d','a','alpha','effort','lower','upper','velocity','visual']
        row_format = "{!s:>10s}" * (len(names_list))
        self.xml += row_format.format(*names_list)+str('\n')
        for DH_param, constraint in zip(self.DH_Params, self.constraints):
            a = [self.joint_names[int(DH_param[0])]] + [round(float(i),4)for i in DH_param[1:]] + [round(i,4)for i in constraint]
            self.xml += row_format.format(*a)+str('\n')
        self.xml += "-->\n"

        self.DH_Params.append((4,0,0,0,0)) #add fixed joint for inverse kinematics   
        self.constraints.append([True]) 
        
        self.xml += "<robot name='robot'>\n"
        self.xml += "\t<material name='grey'>\n\t\t<color rgba='0.6 0.6 0.6 1'/>\n\t</material>\n"
        self.xml += "\t<material name='white'>\n\t\t<color rgba='1 1 1 1'/>\n\t</material>\n"

        for i in range(len(self.DH_Params)):
            y,z,x,r = (0,0,0,0) if i==0 else self.DH_Params[i-1][1:] # transforms [i]
            visual_shapes = [[(r,0,y), (x,0,z), 0.7, 0.43, 'grey']]
            self.write_link(f'a{i}', visual_shapes, self.constraints[i][-1])
            if (i != 0):
                self.write_joint(f'fix_a{i}_to_l{i-1}', 4, f'l{i-1}', f'a{i}', (0,0,0), (0,0,0))
            
            origins_vector = np.array([self.DH_Params[i][3], 0 ,self.DH_Params[i][2]],dtype = float) # transform [i+1]
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
            if (self.DH_Params[i][0] == 1): 
                visual_shapes.append([(0,0,0), (0,0,-(self.constraints[i][2]/2-0.275)), self.constraints[i][2], 0.3, 'white'])

            self.write_link(f'l{i}',visual_shapes, self.constraints[i][-1])
            self.write_joint(f'move_l{i}_from_a{i}',self.DH_Params[i][0],f'a{i}',f'l{i}', (r,0,y),(x,0,z),self.constraints[i])
        
        if self.attachment:
            name, orn, pos = self.attachment
            self.write_joint(f'attachment_joint',4,f'l{i}',f'base', orn, pos)
            file = open(f"attachments/{name}/{name}.urdf",'r')
            for line in file.readlines()[1:-1]:
                self.xml+= line
            file.close()    

        self.xml += "</robot>\n"
        
        file = open(file_name, "w")
        file.write(self.xml)
        file.close()

    def write_link(self, name, visual_shapes, save_visual = True):
        self.xml += "\t<link name='{}'>\n".format(name)
        #self.write_inertial(self.xml)
        if save_visual:
            for v in visual_shapes:
                self.write_visual_shape(v[0], v[1], v[2], v[3], v[4])
        self.xml += "\t</link>\n"

    def write_visual_shape(self, rpy, xyz, length, radius, material_name, precision=5):
        self.xml += "\t\t<visual>\n"
        self.xml += "\t\t\t<origin rpy='{} {} {}' xyz='{} {} {}'/>\n".format(
            rpy[0], rpy[1], rpy[2], xyz[0], xyz[1], xyz[2], prec=precision)
        self.xml += "\t\t\t<geometry>\n"
        self.xml += "\t\t\t\t<cylinder length='{}' radius='{}'/>\n".format(length, radius) 
        self.xml += "\t\t\t</geometry>\n"
        self.xml += "\t\t\t<material name='{}'/>\n".format(material_name)
        self.xml += "\t\t</visual>\n"
    
    def write_joint(self, name, joint_type, parent, child, rpy, xyz, constraints = None, precision=5):

        self.xml += "\t<joint name='{}' type='{}'>\n".format(name, self.joint_names[int(joint_type)])
        self.xml += "\t\t<parent link='{}'/>\n".format(parent)
        self.xml += "\t\t<child link='{}'/>\n".format(child)
        self.xml += "\t\t<axis xyz='0 0 1'/>\n"
        self.xml += "\t\t<origin rpy='{} {} {}' xyz='{} {} {}'/>\n".format(
            rpy[0], rpy[1], rpy[2], xyz[0], xyz[1], xyz[2], prec=precision)
        if joint_type != 4: #fixed
            self.xml += "\t\t<limit effort='{}' lower='{}' upper='{}' velocity='{}'/>\n".format(
                constraints[0], constraints[1], constraints[2], constraints[3], prec=precision)
        self.xml += "\t</joint>\n" 

    def write_inertial(self, rpy, xyz, mass, inertia_xxyyzz, precision=5):
        self.xml += "\t\t</inertial>\n"
        self.xml += "\t\t\t<origin rpy='{} {} {}' xyz='{} {} {}'/>\n".format(
            rpy[0], rpy[1], rpy[2], xyz[0], xyz[1], xyz[2], prec=precision)
        self.xml += "\t\t<mass value='{}'/>\n".format(mass)
        self.xml += "\t\t<inertia ixx='{}' ixy='0' ixz='0' iyy='{}' iyz='0' izz='{}' />\n".format(
            inertia_xxyyzz[0], inertia_xxyyzz[1], inertia_xxyyzz[2], prec=precision)
        self.xml += "\t\t</inertial>\n" 