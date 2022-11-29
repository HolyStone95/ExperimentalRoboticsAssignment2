#! /usr/bin/env python

import rospy
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

def reach_pos( cl_nav, x, y ):
	'''reach a position with the node go_to_point
	
	Parameters:
		serv_nav (ros service):
			the navigation client
		x (float) : x coordinate
		y (float) : y coordinate
	'''
	
	rospy.loginfo( f"position ({x}, {y})" )
	r = rospy.Rate(1)
	
	rospy.set_param( "des_pos_x", x )
	rospy.set_param( "des_pos_y", y )
	
	cmd = SetBoolRequest()
	cmd.data = True
	cl_nav(cmd)
	
	rospy.loginfo( "waiting" )
	r.sleep()
	rospy.loginfo( "done." )

if __name__ == '__main__':
	rospy.init_node( "test_nav" )
	
	cl = rospy.ServiceProxy( "/go_to_point", SetBool )
	
	r = rospy.Rate(0.25)
	reach_pos(cl, 0, 3);
	reach_pos(cl, 0, 0);
	
	reach_pos(cl, 0, 0);
	reach_pos(cl, 0, 0);
	
	reach_pos(cl, 3, 0);
	reach_pos(cl, 0, 0);
	
	reach_pos(cl, 0, -3);
	reach_pos( cl, 0, 0);
	
	reach_pos(cl, -3, 0);
	reach_pos(cl, 0, 0);
