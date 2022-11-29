#! /usr/bin/env python

"""
.. module:: main
	:platform: Unix
	:synopsis: Python module aimed at implementong the Reasoning & AI side

.. moduleauthor:: Iacopo Pietrasanta iacopo.pietrasanta@gmail.com

This node implements a knowledge base for the assignment, due to problems in
using the ARMOR ontology given the particular way in which the ID are generated.

Subscribes to:
	/oracle_hint [erl2/ErlOracle]

Publishes to:
	/rosout [rosgraph_msgs/Log]

Service :
	/get_id
	/mark_wrong_id

"""

import rospy
from erl2.msg import ErlOracle
from erl_assignment_2_msgs.srv import GetId, GetIdRequest, GetIdResponse
from erl_assignment_2_msgs.srv import MarkWrongId, MarkWrongIdRequest, MarkWrongIdResponse

record_where = 0
record_what = 1
record_who = 2
is_active = 3
is_complete = 4

kb = None
''' a list of 6 tuples, corresponding to the 6 possible solution IDs.

list( ..., tuple( <where>, <what>, <who>, <is_active>, <is_complete> ), ... )

Note:
	a cell is empty when its content is a empty string "" .
'''

kb_consistent = None
''' indexex of the remaining active IDs, not necessailry complete
'''

srv_get_id = None
''' service handle for /get_id
'''



def print_kb_content():
	''' print the content inside the ontology on screen
	'''

	global kb, kb_consistent
	global record_who, record_what, record_where

	# print consistent indexes
	if len(kb_consistent) > 0:
		rospy.loginfo(f"(kb) remaining active hints: {len(kb_consistent)}")
		rospy.loginfo(f"(kb) values:")
		for idx in kb_consistent:
			rospy.loginfo(f"(kb) ID={idx}")

	rospy.loginfo(f"(kb) kb status: ({len(kb)} possible solutions)")
	for i in range(0,len(kb)):
		is_active_str = "false"
		if kb[i][3]:
			is_active_str = "true"

		is_complete_str = "false"
		if kb[i][4]:
			is_complete_str = "true"

		rospy.loginfo(f"(kb) ID={i} WHO={kb[i][record_who]} WHERE={kb[i][record_where]} WHAT={kb[i][record_what]} -- is_active={is_active_str} is_complete={is_complete_str}")



def add_hint( hint ):
	''' receive and store (if possible) the hint

	Parameters:
		hint (erl2/ErlOracle):
			the hint received directly from the Oracle
	'''

	if is_valid_hint( hint ):
		rospy.loginfo(f"evaluating hint with data (key={hint.key} , value={hint.value})")
		add_hint_to_list( hint )
	else:
		rospy.loginfo(f"received a unvalid hint with data (key={hint.key} , value={hint.value})")

	# print_kb_content()



def add_hint_to_list( hint ):
	''' add an hint to the list, if possible

	the function tries to add a hint, checking if it is still consistent;
	if the required field is already occupied (i.e. the string in that cell
	is not empty), the ID is marked as inconsistent and deactivated, and
	its index is removed from the indexes list.

	Parameters:
		hint (erl2/ErlOracle):
			the hint to store in the KB

	'''

	global kb, kb_consistent
	global record_who, record_what, record_where
	global is_active, is_complete

	delete_that = False;

	if hint.key == "where":
		if kb[hint.ID][record_where] == "" :
			rospy.loginfo( f"adding hint ID={hint.ID} WHERE={hint.value}" )
			kb[hint.ID][record_where] = hint.value;

		elif kb[hint.ID][record_where] == hint.value :
			rospy.loginfo( f"skipping hint ID={hint.ID} WHERE={hint.value}" )

		else:
			# ID not consistent
			rospy.loginfo( f"removing hint ID={hint.ID} WHERE={hint.value}" )
			delete_that = True

	elif hint.key == "what":
		if kb[hint.ID][record_what] == "" :
			rospy.loginfo( f"adding hint ID={hint.ID} WHAT={hint.value}" )
			kb[hint.ID][record_what] = hint.value;

		elif kb[hint.ID][record_what] == hint.value :
			rospy.loginfo( f"skipping hint ID={hint.ID} WHAT={hint.value}" )

		else:
			# ID not consistent
			rospy.loginfo( f"deleting hint ID={hint.ID} WHAT={hint.value}" )
			delete_that = True

	elif hint.key == "who":
		if kb[hint.ID][record_who] == "" :
			rospy.loginfo( f"adding hint ID={hint.ID} WHO={hint.value}" )
			kb[hint.ID][record_who] = hint.value;

		elif kb[hint.ID][record_who] == hint.value :
			rospy.loginfo( f"skipping hint ID={hint.ID} WHO={hint.value}" )

		else:
			# ID not consistent
			rospy.loginfo( f"deleting hint ID={hint.ID} WHO={hint.value}" )
			delete_that = True

	else:
		rospy.logwarn( f"(cluedo_kb -> add_hint_to_list) received a unknown hint.key : {hint.key}" )

	# --- --- --- --- --- --- --- ---  Testing --- --- --- --- --- --- ---
	# rospy.loginfo(f"empty WHO? {kb[hint.ID][record_who] == ''}")
	# rospy.loginfo(f"empty WHERE? {kb[hint.ID][record_where] == ''}")
	# rospy.loginfo(f"empty WHAT? {kb[hint.ID][record_what] == ''}")

	if (kb[hint.ID][record_where] != "") and (kb[hint.ID][record_what] != "") and (kb[hint.ID][record_who] != ""):
		# rospy.loginfo(f"ID{hint.ID} is complete")
		kb[hint.ID][is_complete] = True
	else:
		# rospy.loginfo(f"ID{hint.ID} is not complete")
		pass

	if delete_that and len(kb_consistent) > 0:
		rospy.loginfo( f"discard hypothesis with ID={hint.ID}" )

		# ID not consistent
		kb[hint.ID][is_active] = False
		kb[hint.ID][is_complete] = False

		# delete that from the index list
		if hint.ID in kb_consistent:
			kb_consistent.remove( hint.ID );
	elif len(kb_consistent) == 0:
		rospy.loginfo( f"nothing to discard (received a unconsistent ID={hint.ID})" )


def is_valid_hint( hint ):
	''' check if the hint is vald or not

	the Oracle sometimes could send a wrong hint, i.e. some field is
	an empty string and/or some filed has value "-1". the function
	detects the quality of the hint, and returns if it is admissible
	or not. Also the ID could be negative or zero.

	Parameters:
		hint (erl2/ErlOracle):
			the hint to store in the KB

	Returns:
		(bool) if the hint is admissible or not.
	'''

	if hint.ID < 0 or hint.ID > 5:
		return False
	if hint.key == "" or hint.key == "-1":
		return False;
	if hint.value == "" or hint.value == "-1":
		return False

	return True;



def get_id( req ):
	''' implementation of the service /get_id

	the service tries to return the first available index, looking in the
	list of remaining indexes. if no consistent index is available, the
	service returns response.consistent_found=False .

	Parameters:
		req (erl_assignment_2/GetIdRequest):
			the service request

	Returns:
		(erl_assignment_2/GetIdResponse) the id and if there are
		still available IDs.

	Note:
		the service could return also (true, -1) in a situation in which
		there are some active ID, but none of them is complete.

	'''

	global kb, kb_consistent
	global record_who, record_what, record_where
	global is_active, is_complete

	res = GetIdResponse( )
	res.consistent_found = ( len( kb_consistent ) > 0 )
	res.consistent_id = -1

	rospy.logwarn("called get_id() -- kb content:")
	print_kb_content()

	if res.consistent_found:
		for id in kb_consistent:
			if kb[id][is_complete]:
				res.consistent_id = id
				rospy.logwarn(f"called get_id() -- found consistent hypothesis with ID={res.consistent_id}")
				break
	else:
		rospy.logwarn("called get_id() -- no consistent hypotheses!")

	rospy.logwarn(f"called get_id() -- returning res with res.consistent_found={res.consistent_found} res.consistent_id={res.consistent_id}")
	return res



def mark_wrong_id( req ):
	'''discard a ID from the system.

	Parameters:
		req (erl_assignment_2/MarkWrongIdRequest):
			the service request

	Returns:
		req (erl_assignment_2/MarkWrongIdResponse) empty
	'''

	global kb, kb_consistent
	global record_who, record_what, record_where
	global is_active, is_complete

	# delete the ID from the index list
	if len( kb_consistent ) > 0 :
		kb[req.ID][is_active] = False
		kb[req.ID][is_complete] = False

		if hint.ID in kb_consistent:
			kb_consistent.remove( req.ID )

		rospy.loginfo( f"(mark_wrong_id) discarded ID={req.ID}" )



def shut_msg( ):
	rospy.loginfo( "stopping ... " )



if __name__ == "__main__":
	rospy.init_node( "cluedo_kb" )
	rospy.on_shutdown( shut_msg )

	rospy.loginfo( "cluedo_kb initialization..." )
	kb = list( )
	kb_consistent = list( )
	for i in range(0, 6):
		kb.append( ["", "", "", True, False] )
		kb_consistent.append( i )
	rospy.loginfo( "cluedo_kb initialization... done" )

	rospy.loginfo( "cluedo_kb subscriber /oracle_hint..." )
	rospy.Subscriber( "oracle_hint", ErlOracle, add_hint )

	rospy.loginfo( "cluedo_kb client /get_id..." )
	srv_get_id = rospy.Service( "/get_id", GetId, get_id )

	rospy.loginfo( "cluedo_kb client /mark_wrong_id..." )
	srv_get_id = rospy.Service( "/mark_wrong_id", MarkWrongId, mark_wrong_id )

	rospy.loginfo( "cluedo_kb starting..." )
	rospy.spin( )
