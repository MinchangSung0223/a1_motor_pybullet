import pybullet as p
import time
import pybullet_data
import numpy as np
np.set_printoptions(precision=6, suppress=True)
p.connect(p.GUI)
p.setGravity(0, 0, -9.8)
p.setTimeStep(1./500)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
a1_motor = p.loadURDF("a1_motor/a1_motor.urdf", [0, 0, 0.0],[0, 0, 0, 1])

for j in range (1,3):
	p.setJointMotorControl2(a1_motor, j, p.VELOCITY_CONTROL, 0, force=0)

while(1):
	jointStates = np.array(p.getJointStates(a1_motor,[1,2]))
	q = np.array(jointStates[:,0])
	q_dot = np.array(jointStates[:,1])	
	pb_ID= np.array(p.calculateInverseDynamics(a1_motor,[q[0],q[1] ],[q_dot[0],q_dot[1] ] ,[0,0] ))	
	print(pb_ID)
	for i in range(1,3):		
		p.setJointMotorControl2(a1_motor, i, p.TORQUE_CONTROL,force=pb_ID[i-1])	
	p.stepSimulation()
	time.sleep(1./500.)


