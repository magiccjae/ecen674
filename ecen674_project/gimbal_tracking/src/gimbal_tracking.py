#!/usr/bin/env python
import rospy
import numpy as np
import math as m
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Vector3
import time

M = 800
v = 0.698132
f = M/2*m.tan(v/2)

desired_az = 0
desired_el = 0
global ell_b
ell_b = np.matrix([[0],[0],[0]])

def callback(data):
	camera_x = data.pose[2].position.x
	camera_y = data.pose[2].position.y
	camera_z = -data.pose[2].position.z
	object_x = data.pose[3].position.x
	object_y = data.pose[3].position.y
	object_z = -data.pose[3].position.y
	global ell_b
	ell_b = np.matrix([[object_x],[object_y],[object_z]]) - np.matrix([[camera_x],[camera_y],[camera_z]])
	ell_b = 1/np.linalg.norm(ell_b)*ell_b

rospy.init_node('gimbal_tracking', anonymous=True)
rospy.Subscriber("/gazebo/model_states", ModelStates, callback)
pub = rospy.Publisher('gimbal_angle_command', Vector3, queue_size=1)
rate = rospy.Rate(10)


def calculate_commanded_angles(ell_b):
	az_d = m.atan2(ell_b[1,0],ell_b[0,0])
	if ell_b[2,0] > 1:
		ell_b[2,0] = 1
	elif ell_b[2,0] < -1:
		ell_b[2,0] = -1
	el_d = -m.asin(ell_b[2,0])
	return (az_d, el_d)

while True:
	(az_d, el_d) = calculate_commanded_angles(ell_b)
	msg = Vector3()
	msg.x = az_d
	msg.y = el_d
	msg.z = 0
	pub.publish(msg)
	time.sleep(0.2)

rospy.spin()