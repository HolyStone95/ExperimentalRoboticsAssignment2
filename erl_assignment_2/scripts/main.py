#!/bin/bash

"""
.. module:: main
	:platform: Unix
	:synopsis: Core python module, interfacing with Rosplan

.. moduleauthor:: Iacopo Pietrasanta iacopo.pietrasanta@gmail.com

This node represents the upper layer of execution of the entire simulation:
It initializes the ROS_plan plan, it executes and based on the outcome either
replan or concludes the game.

Subscribes to:
	/clock [rosgraph_msgs/Clock]

Publishes to:
	/rosout [rosgraph_msgs/Log]

Service :
	None

"""


import rospy
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from rosplan_dispatch_msgs.srv import DispatchService, DispatchServiceRequest, DispatchServiceResponse
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService, KnowledgeUpdateServiceRequest, KnowledgeUpdateServiceResponse
from diagnostic_msgs.msg import KeyValue

cl_update_kb = None
"""None: Variable needed for updating the Knowledge base
"""

ADD_KNOWLEDGE = 0
DEL_KNOWLEDGE = 2

KB_KTYPE_FLUENT = 2
KB_KTYPE_PREDICATE = 1

def replan_setup_kb( ):
	''' Description of the replan_setup_kb function

	This function allows to "reset" the ontology predicates' values,
	by setting all the wayoints as "  yet to be visited ", all the
	hints as " yet to be gathered " and the consistent hypothesis
	as "yet to be collected ".


	'''
	set_pred( "has_been_at", [["wp", "wp1"]], False )
	set_pred( "has_been_at", [["wp", "wp2"]], False )
	set_pred( "has_been_at", [["wp", "wp3"]], False )
	set_pred( "has_been_at", [["wp", "wp4"]], False )

	set_pred( "not_has_been_at", [["wp", "wp1"]], True )
	set_pred( "not_has_been_at", [["wp", "wp2"]], True )
	set_pred( "not_has_been_at", [["wp", "wp3"]], True )
	set_pred( "not_has_been_at", [["wp", "wp4"]], True )

	set_pred( "gathered_hint", [["wp", "wp1"]], False )
	set_pred( "gathered_hint", [["wp", "wp2"]], False )
	set_pred( "gathered_hint", [["wp", "wp3"]], False )
	set_pred( "gathered_hint", [["wp", "wp4"]], False )

	set_pred( "not_gathered_hint", [["wp", "wp1"]], True )
	set_pred( "not_gathered_hint", [["wp", "wp2"]], True )
	set_pred( "not_gathered_hint", [["wp", "wp3"]], True )
	set_pred( "not_gathered_hint", [["wp", "wp4"]], True )

	set_pred( "consistent_hypo", [], False )


def set_pred( predicate, params, value ):
	'''Description of the set_pred function

	This function allows to set the predicates and
	populate a request accordingly

	Note:

		params: [ ... , [key, value], ... ]
	'''

	req = KnowledgeUpdateServiceRequest( )

	req.knowledge.attribute_name = predicate
	req.knowledge.knowledge_type = KB_KTYPE_PREDICATE

	if value:
		req.update_type = ADD_KNOWLEDGE
	else:
		req.update_type = DEL_KNOWLEDGE

	for ls in params:
		kv = KeyValue( )
		kv.key = ls[0]
		kv.value = ls[1]
		req.knowledge.values.append(kv)

	cl_update_kb(req)


if __name__ == "__main__":
	rospy.init_node( "main" )

	em = Empty( )
	dis = DispatchServiceResponse( )

	# kb updater
	cl_update_kb = rospy.ServiceProxy( "/rosplan_knowledge_base/update", KnowledgeUpdateService )

	# planning triggers
	cl_problem_interface = rospy.ServiceProxy( "/rosplan_problem_interface/problem_generation_server", Empty )
	cl_planner_interface = rospy.ServiceProxy( "/rosplan_planner_interface/planning_server", Empty )
	cl_parsing_interface = rospy.ServiceProxy( "/rosplan_parsing_interface/parse_plan", Empty )
	cl_plan_dispatcher = rospy.ServiceProxy( "/rosplan_plan_dispatcher/dispatch_plan", DispatchService )

	solved = False
	"""Bool: Variable used as flag to notice whenever the mistery gets solved
	"""

	while not solved:
		rospy.loginfo( "replanning..." )
		cl_problem_interface( )
		cl_planner_interface( )
		cl_parsing_interface( )

		rospy.loginfo( "dispatching plan..." )
		dis = cl_plan_dispatcher( )

		solved = (dis.success and dis.goal_achieved)
		if solved:
			rospy.loginfo( "mystery solved." )
		else:
			rospy.loginfo( "still not solved" )
			replan_setup_kb( )
