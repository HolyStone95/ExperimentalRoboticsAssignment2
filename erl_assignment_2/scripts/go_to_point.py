#! /usr/bin/env python


"""
.. module:: go_to_point
	:platform: Unix
	:synopsis: Python module for piloting the robot to the target

.. moduleauthor:: Iacopo Pietrasanta iacopo.pietrasanta@gmail.com

ROS node for driving a robot to a specific point within a simulated
environment, given a certain orientation.

Subscribes to:
	/odom topic where the simulator publishes the robot position

Publishes to:
	/cmd_vel the desired robot position

Service :
	/go_to_point to start the robot motion.

"""

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math

active_ = False
"""Bool: Service activity flag
"""

# robot state variables
position_ = Point()
"""Point: current robot position
"""

yaw_ = 0
"""Pose: current robot orientation
"""
# machine state
state_ = 0
"""Int: current state of the server
"""

desired_position_ = Point()
"""Point: ctarget robot position
"""

desired_position_.x = rospy.get_param('des_pos_x')
desired_position_.y = rospy.get_param('des_pos_y')
desired_position_.z = 0

# parameters
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
"""Float: yaw acc +/- 20 deg allowed
"""

yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
"""Float: tight yaw acc +/- 2
"""

dist_precision_ = 0.3
"""Float: linear distance allowed
"""

kp_a = 3.0  # In ROS Noetic, it may be necessary to change the sign of this proportional controller
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6

# publishers
pub = None

# service callbacks


def go_to_point_switch(req):
	''' SERVICE IMPLEMENTATION OF /go_to_point_switch
	
	the service sets the value of the activity flag.
	
	Args:
		req (std_srvs/SetBoolRequest):
			the activity value will be equal to the req.data passed to the
			service
	
	Returns:
		(std_srvs/SetBoolResponse) success is always true
	
	'''
	
	global active_
	active_ = req.data
	res = SetBoolResponse()
	res.success = True
	res.message = 'Done!'
	return res

# callbacks


def clbk_odom(msg):
	'''Description of the callback:
	
	This function retrieves the current robot position for saving
	it within the *position_* global variable and is responsible for
	transforming the orientation from quaternion angles to Euler ones
	
	Args:
		msg(Twist): data retrieved by */cmd_vel* topic
	
	Returns:
		None

	'''	
	global position_
	global yaw_

	# position
	position_ = msg.pose.pose.position

	# yaw
	quaternion = (
		msg.pose.pose.orientation.x,
		msg.pose.pose.orientation.y,
		msg.pose.pose.orientation.z,
		msg.pose.pose.orientation.w)
	euler = transformations.euler_from_quaternion(quaternion)
	yaw_ = euler[2]


def change_state(state):
	''' Description of the change_state function:
	
	This value retrieve and assigns the current state to the
	global one (*state_*)
	
	Args:
		state(int): the state of the robot
	
	Returns:
		None
	'''
	global state_
	state_ = state
	print ('State changed to [%s]' % state_)


def normalize_angle(angle):
	''' Function for normalizing the angle between -pi and pi.
	
	Args:
		angle(Float): the input angle
	
	Returns:
		angle(Float): the normalized angle.
	'''

	if(math.fabs(angle) > math.pi):
		angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
	return angle


def fix_yaw(des_pos):
	''' Description of the fix_yaw function:
	
	This function computes the robot orientation among x and y 
	coordinates and sets the angular velocity needed for achieving
	the desired robot position. 
		
	Args:
		des_pos(Point):  the expected x and y coordinates
	Returns:
		None
	'''	
	global yaw_, pub, yaw_precision_2_, state_
	desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
	err_yaw = normalize_angle(desired_yaw - yaw_)

	rospy.loginfo(err_yaw)

	twist_msg = Twist()
	if math.fabs(err_yaw) > yaw_precision_2_:
		twist_msg.angular.z = kp_a*err_yaw
		if twist_msg.angular.z > ub_a:
			twist_msg.angular.z = ub_a
		elif twist_msg.angular.z < lb_a:
			twist_msg.angular.z = lb_a

	pub.publish(twist_msg)

	# state change conditions
	if math.fabs(err_yaw) <= yaw_precision_2_:
		print ('Yaw error: [%s]' % err_yaw)
		change_state(1)


def go_straight_ahead(des_pos):
	''' Description of the go_straight_ahead function:

	This function computes the robot orientation among x and y 
	coordinates necessary to reach the x,y target point. Once the
	linear velocities have been set, an angular velocity is defined
	by means of an error. It is proportional to this latter and it 
	allows a correction of the trajectory, by checking a treshold
	over a distance
		
		
	Args:
		des_pos(Point): the expected x and y coordinates
	Returns:
		None

	'''

	global yaw_, pub, yaw_precision_, state_
	desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
	err_yaw = desired_yaw - yaw_
	err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
						pow(des_pos.x - position_.x, 2))

	if err_pos > dist_precision_:
		twist_msg = Twist()
		twist_msg.linear.x = kp_d*(err_pos)
		if twist_msg.linear.x > ub_d:
			twist_msg.linear.x = ub_d

		twist_msg.angular.z = kp_a*err_yaw
		pub.publish(twist_msg)
	else:
		print ('Position error: [%s]' % err_pos)
		change_state(2)

	# state change conditions
	if math.fabs(err_yaw) > yaw_precision_:
		print ('Yaw error: [%s]' % err_yaw)
		change_state(0)


def done(des_pos):
	""" Description of done function:
		
	This function marks the goal target as succeeded, once all the
	linear and angular velocities are set to zero  

	Args :
		None

	Returns :
		None
		
	"""
	twist_msg = Twist()
	twist_msg.linear.x = 0
	twist_msg.angular.z = 0
	pub.publish(twist_msg)
	err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
						pow(des_pos.x - position_.x, 2))
	if(err_pos > 0.35):
		change_state(0)


def go_to_point(req):
	""" SERVICE IMPLEMENTATION OF /go_to_point
		
	This function retrieves the ROS params from the ROS
	parameter server. Secondly, it truggers a different 
	robot behaviour, depending on the state's value 

	Args :
		None

	Returns :
		None
		
	"""	
	global desired_position_, state_
	
	rate = rospy.Rate(20)
	state_ = 0
	
	while not rospy.is_shutdown():
		
		desired_position_.x = rospy.get_param('des_pos_x')
		desired_position_.y = rospy.get_param('des_pos_y')
		
		if state_ == 0:
			rospy.loginfo( f"fix_yaw({desired_position_})" )
			fix_yaw(desired_position_)
		elif state_ == 1:
			rospy.loginfo( f"go_straight_ahead({desired_position_})" )
			go_straight_ahead(desired_position_)
		elif state_ == 2:
			rospy.loginfo( "done!" )
			done(desired_position_)
			return SetBoolResponse( )
		else:
			rospy.logerr('Unknown state!')
		
		rate.sleep( )


def main():
	global pub, active_, desired_position_
	
	rospy.init_node('go_to_point')
	
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	
	sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
	
	# testing server 
	# srv = rospy.Service('go_to_point_switch', SetBool, go_to_point_switch)
	
	srv = rospy.Service('go_to_point', SetBool, go_to_point)
	
	rospy.spin( )
	


if __name__ == '__main__':
	main()
