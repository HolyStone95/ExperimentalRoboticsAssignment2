
/** @ package erl_assignment_2
*
*	@file manipulation.cpp
*	@brief This node directly interacts with moveIT for moving the arm
*
*	@author Iacopo Pietrasanta
*	@version 1.0.0
*
*	Subscribes to: <BR>
* 		/clock 							[rosgraph_msgs/Clock]
* 		/execute_trajectory/feedback	[moveit_msgs/ExecuteTrajectoryActionFeedback]
* 		/execute_trajectory/result 		[moveit_msgs/ExecuteTrajectoryActionResult]
* 		/execute_trajectory/status 		[actionlib_msgs/GoalStatusArray]
* 		/move_group/feedback 			[moveit_msgs/MoveGroupActionFeedback]
* 		/move_group/result 				[moveit_msgs/MoveGroupActionResult]
* 		/move_group/status 				[actionlib_msgs/GoalStatusArray]
* 		/pickup/feedback 				[moveit_msgs/PickupActionFeedback]
* 		/pickup/result 					[moveit_msgs/PickupActionResult]
* 		/pickup/status 					[actionlib_msgs/GoalStatusArray]
* 		/place/feedback 				[moveit_msgs/PlaceActionFeedback]
* 		/place/result 					[moveit_msgs/PlaceActionResult]
* 		/place/status 					[actionlib_msgs/GoalStatusArray]
* 		/tf 							[tf2_msgs/TFMessage]
* 		/tf_static 						[tf2_msgs/TFMessage]
*
*	Publishes to: <BR>
*		/attached_collision_object 		[moveit_msgs/AttachedCollisionObject]
* 		/execute_trajectory/cancel 		[actionlib_msgs/GoalID]
*		/execute_trajectory/goal		[moveit_msgs/ExecuteTrajectoryActionGoal]
*		/move_group/cancel 				[actionlib_msgs/GoalID]
* 		/move_group/goal 				[moveit_msgs/MoveGroupActionGoal]
* 		/pickup/cancel 					[actionlib_msgs/GoalID]
* 		/pickup/goal 					[moveit_msgs/PickupActionGoal]
* 		/place/cancel 					[actionlib_msgs/GoalID]
* 		/place/goal 					[moveit_msgs/PlaceActionGoal]
* 		/rosout 						[rosgraph_msgs/Log]
* 		/trajectory_execution_event 	[std_msgs/String]

*	Services: <BR>
*   	/manipulation
*
*	Client Services: <BR>
*   	/go_to_point		[std_srvs/SetBool]
*    	/manipulation		[std_srvs/SetBool]
*		/get_id				[erl_assignment_2_msgs/GetId]
*		/oracle_solution	[erl2/Oracle]
*		/mark_wrong_id		[erl_assignment_2_msgs/MarkWrongId]
]
*
*	Action Services: <BR>
*    	None
*
*	Description: <BR>
*		This node interfaces the robotic arm with the MoveIt nodes, allowing all
*		the manipulation necessary actions.
*
*/
#include "ros/ros.h"
#include "std_srvs/SetBool.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


/**
 * @brief   our move group interface
 */
moveit::planning_interface::MoveGroupInterface *move_group_interface;

/**
 * @brief   manupulation Service server
 */
ros::ServiceServer srv_manip;



/**
 * \brief   move the robot in one pose
 *
 * \details Depending on the request boolean value, this function moves the robot
 *					arm in two possible positions. Pose "home" represents an initial,
 *					rest position
 *
 * \return  void
 */
void move_arm(bool gather_hint)
{
	if(gather_hint)
	{
		// gather_hint
		/**
 		* @brief   manipulation Service server
 		*/
		move_group_interface->setNamedTarget( "gather_hint" );
	}
	else
	{
		// home
		move_group_interface->setNamedTarget( "home" );
	}

	move_group_interface->move( );
}



/**
 * \brief   manipulation Service callback implementatiomn
 *
 * \details This function represents the callback of the maipulaiton
 *          service, it calls the move_arm function.
 *
 * \param[in]     req   A request of type std_srvs::SetBool,
 *                      enabling/disabling certain behaviours
 * \param[in]     res   A response of type std_srvs::SetBool,
 *                      indicating a successfull run of triggered
 *                      service
 *
 * \return  void
 */
bool cbk_manip(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
	move_arm(req.data);

	res.success = true;
	return true;
}


/// main function
int main(int argc, char* argv[])
{
	ros::init(argc, argv, "manipulation");
	ros::NodeHandle nh;

	ros::AsyncSpinner spinner(2);
	spinner.start( );

	// define the robot pkanning group
	static const std::string PLANNING_GROUP = "arm";
	moveit::planning_interface::MoveGroupInterface move_group_interfacei( PLANNING_GROUP );
	move_group_interface = &move_group_interfacei;

	// set planning time
	move_group_interface->setPlanningTime(10.0);


	/**
	* @brief   before starting, move the robot in the init pose
	*/
	move_arm(false);

	// srv for /manipulation service
	srv_manip = nh.advertiseService("/manipulation", cbk_manip);

	ros::waitForShutdown( );

	return 0;
}
