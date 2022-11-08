import pybullet as p
cin = p.connect(p.SHARED_MEMORY)
if (cin < 0):
    cin = p.connect(p.GUI)
objects = [p.loadURDF("plane.urdf", 0.000000,0.000000,-0.300000,0.000000,0.000000,0.000000,1.000000)]
objects = [p.loadURDF("husky/husky.urdf", 0.383151,0.041592,-0.301899,-0.000122,-0.000647,0.160519,0.987033)]
ob = objects[0]
jointPositions=[ 0.000000, 0.000000, 0.190810, -0.586010, 0.394773, -0.573414, 0.000000, 0.000000, 0.000000, 0.000000 ]
for jointIndex in range (p.getNumJoints(ob)):
	p.resetJointState(ob,jointIndex,jointPositions[jointIndex])

objects = [p.loadURDF("kuka_iiwa/model_free_base.urdf", 0.476757,0.073366,0.128272,-0.000074,-0.000637,0.160518,0.987033)]
ob = objects[0]
jointPositions=[ -0.630901, -0.546666, -2.478074, -1.972415, 2.959886, -1.971016, -0.148776 ]
for jointIndex in range (p.getNumJoints(ob)):
	p.resetJointState(ob,jointIndex,jointPositions[jointIndex])

cid0 = p.createConstraint(1,-1,2,-1,p.JOINT_FIXED,[0.000000,0.000000,0.000000],[0.000000,0.000000,0.000000],[0.000000,0.000000,-0.500000],[0.000000,0.000000,0.000000,1.000000],[0.000000,0.000000,0.000000,1.000000])
p.changeConstraint(cid0,maxForce=500.000000)
p.setGravity(0.000000,0.000000,-10.000000)
p.stepSimulation()
p.disconnect()
