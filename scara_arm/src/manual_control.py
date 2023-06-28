#!/usr/bin/env python
import math
import rospy
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray

inc = 0.00523599		#increament in rotation angle 	= 5 degrees = 0.0872665 radian
incm = 0.002			#increament in motion			= 1 cm 		= 0.01 m

joint1_len = 0.20
joint2_len = 0.22
joint4_pos = 0.0
gripper_pose = 0.0

joint_positions = []

goal_loc = [0 , 0]
count = 0
joint_angles = []

def distance_calc(x_coo, y_coo):
	return math.sqrt(x_coo*x_coo + y_coo * y_coo)

def lawOfCosine(a, b, c, angle_c):
	return math.acos((a*a + b*b - c*c)/(2 * a * b))

def ik_compute(goal_loc):

	x = goal_loc[0]
	y = goal_loc[1]
	
	distnace = distance_calc(goal_loc[0], goal_loc[1])
	
	
	
	joint_angles = [0.0 , 0.0]
	joint_angles[0]= theta + alpha
	joint_angles[1] = math.pi - alpha - beta
	
	print("Temp : ", temp1, temp2)
	print("joint Angles : ", joint_angles)

	return joint_angles

def joint_recaller(data):
	global count
	if count < 10:
		global joint_positions, goal_loc
		global joint1_len
		global joint2_len

		joint_positions = data.position
		J1_angle = joint_positions[1]
		J2_angle = joint_positions[2]

		goal_loc[0] = (joint2_len * math.sin(J1_angle + J2_angle)) + (joint1_len * math.sin(J1_angle))
		goal_loc[1] = (joint2_len * math.cos(J1_angle + J2_angle)) + (joint1_len * math.cos(J1_angle))

		# print("joints : x , y : ", goal_loc)
		count += 1


def joy_callback(data):
	global inc, incm, goal_loc, joint4_pos, gripper_pose, count, pointxy_pub, joint_angles
	point = Float32MultiArray()

	if data.buttons[0]:
		# switch_functions()
		pass

	if data.axes[0] > 0.1:
		if goal_loc[0] < 0.42:
			goal_loc = [ goal_loc[0] + incm , goal_loc[1]]
			joint_angles = ik_compute(goal_loc)
	if data.axes[0] < -0.1:
		if goal_loc[0] > -0.42:
			goal_loc = [ goal_loc[0] - incm , goal_loc[1]]
			joint_angles = ik_compute(goal_loc)
	
	if data.axes[1] > 0.1:
		if goal_loc[1] < 0.42:
			goal_loc = [ goal_loc[0] , goal_loc[1] + incm ]
			joint_angles = ik_compute(goal_loc)
	if data.axes[1] < -0.1:
		if goal_loc[1] > -0.42:
			goal_loc = [ goal_loc[0] , goal_loc[1] - incm]
			joint_angles = ik_compute(goal_loc)

	if data.axes[3] < -0.1:
		joint4_pos -= inc	
	if data.axes[3] > 0.1:
		joint4_pos +=  inc

	if data.buttons[2] > 0:
		gripper_pose = 0.0
	if data.buttons[1] > 0.0:
		gripper_pose = math.pi
	if data.buttons[3] > 0.0:
		count = 0

	z_axis = 100 * - data.axes[4]

	print("goal: "+str(goal_loc))
	
	# print("joint_1 : ",joint_angles[0])
	# print("joint_2 : ",joint_angles[1])
	# print("joint_3 : ",z_axis)
	# print("joint_4 : ",joint4_pos)
	# print("joint_5 : ",gripper_pose)

	joint1_pub.publish(joint_angles[0])
	joint2_pub.publish(joint_angles[1])
	joint3_pub.publish(z_axis)
	joint4_pub.publish(joint4_pos)
	joint5_pub.publish(gripper_pose)
	point.data = goal_loc
	pointxy_pub.publish(point)




def joy_listener():
	global joint1_pub, joint2_pub, joint3_pub, joint4_pub, joint5_pub, pointxy_pub

	rospy.init_node('scara_arm_manual_controller', anonymous=True)

	joint1_pub = rospy.Publisher('/scara_arm_J1_JP_controller/command', Float64, queue_size = 10)
	joint2_pub = rospy.Publisher('/scara_arm_J2_JP_controller/command', Float64, queue_size = 10)
	joint3_pub = rospy.Publisher('/scara_arm_Z_JE_controller/command', Float64, queue_size = 10)
	joint4_pub = rospy.Publisher('/scara_arm_J4_JP_controller/command', Float64, queue_size = 10)
	joint5_pub = rospy.Publisher('/scara_arm_EEF_JP_controller/command', Float64, queue_size = 10)
	pointxy_pub = rospy.Publisher('/scara_arm/xy', Float32MultiArray, queue_size = 10)
	
	


	rospy.Subscriber("/joint_states", JointState, joint_recaller)
	rospy.Subscriber("/joy", Joy, joy_callback)
	rospy.spin()



if __name__ == '__main__':
	try:
		joy_listener()

	except rospy.ROSInterruptException:
		pass