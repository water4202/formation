#!/usr/bin/python

import rospy
from math import sin,cos,sqrt,atan2,acos,pi
import numpy as np
import gurobipy as gp
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from px4_mavros import Px4Controller
from gurobipy import GRB

P1,P2,P3,PO,A,b = None,None,None,None,None,None
cmd_vel = Twist()
d_safe = 1.0
m,x = None,None

def odom(msg):
	global P1,P2,P3,PO,A,b
	
	UAV1_index = msg.name.index('iris_1')
	UAV2_index = msg.name.index('iris_2')
	UAV3_index = msg.name.index('iris_3')
	obs_index = msg.name.index('obstacle')

	P1 = np.array([msg.pose[UAV1_index].position.x, msg.pose[UAV1_index].position.y, msg.pose[UAV1_index].position.z])
	P2 = np.array([msg.pose[UAV2_index].position.x, msg.pose[UAV2_index].position.y, msg.pose[UAV2_index].position.z])
	P3 = np.array([msg.pose[UAV3_index].position.x, msg.pose[UAV3_index].position.y, msg.pose[UAV3_index].position.z])
	PO = np.array([msg.pose[obs_index].position.x, msg.pose[obs_index].position.y, msg.pose[obs_index].position.z])

	A = np.array([ \
				  (-2*(P1-PO)[:2]).tolist() \
				  ])

	b = np.array([ \
				  np.linalg.norm((P1-PO)[:2])**2 - d_safe**2 \
				  ])

def qp_ini():
	global m,x
	
	m = gp.Model("qp")
	m.setParam("NonConvex", 2.0)
	m.setParam("LogToConsole",0)
	x = m.addVars(2,ub=0.5, lb=-0.5, name="x")

def addCons(i):
	global m

	m.addConstr(A[i,0]*x[0] + A[i,1]*x[1] <= b[i], "c"+str(i))

def	controller():
	global cmd_vel
	
	u_des = np.array([1*((P2[0] - P1[0]) - 2.0 + (P3[0] - P1[0]) - 1.0),1*((P2[1] - P1[1]) + 0.0 + (P3[1] - P1[1]) - sqrt(3)),0.5 - P1[2]])

	obj = (x[0] - u_des[0])**2 + (x[1] - u_des[1])**2
	m.setObjective(obj)

	m.remove(m.getConstrs())

	for i in range (b.size):
		addCons(i)

	m.optimize()
	u_opt = m.getVars()

	cmd_vel.linear.x = u_opt[0].X
	cmd_vel.linear.y = u_opt[1].X
	cmd_vel.linear.z = u_des[2]

	px4_1.vel_control(cmd_vel)

if __name__ == '__main__':
	try:
		rospy.init_node('controller_1')
		px4_1 = Px4Controller("iris_1")
		rospy.Subscriber('/gazebo/model_states', ModelStates, odom, queue_size=10)
		rate = rospy.Rate(100)
		while b is None:
			rate.sleep()

		qp_ini()
		while not rospy.is_shutdown():
			controller()
			rate.sleep()
	except rospy.ROSInterruptException:
		pass
