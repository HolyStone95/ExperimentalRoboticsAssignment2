# Experimental Robotics Laboratory - second assignment

This package contains the source code implemented during the second assignment of the Experimental Robotics Laboratory, course of the first semester of the second year of Robotics Engineering Master Degree course of University of Genoa, Italy.

## Index

- Index:
  - [Required packages](#required-packages)
  - [Assignment request](#assignment-request)
  - [Software Architecture and Functioning](#software-architecture-and-functioning)
  - [ROS node description: An overview](#ros-node-description:-an-overview)
    - [The go_to_point.py node](#the-go_to_point.py-node)
    - [The main.py node](#the-main.py-node)
    - [The cluedo_kb.py node](#the-cluedo_kb.py-node)
    - [The action_interface.cpp node](#the-action_interface.cpp-node)
    - [The manipulation.cpp node](#the-manipulation.cpp-node)
    - [The my_simulation.cpp node](#the-my_simulation.cpp-node)
    - [Rqt_graph](#rqt_graph)
    - [UML temporal diagram](#uml-temporal-diagram)
  - [How to launch and additional documentation](#how-to-launch-and-additional-documentation)
  - [System limitations](#system-limitations)
  - [Possible Technical Improvements](#possible-technical-improvements)
  - [Contact](#contact)

## Required packages

  * **ROS**
  * **ROSPlan Framework**
  * **MoveIt Frameowrk**

## Assignment request

This assignment improves the first one by adding a real robot model into a physically simulated world, the hint are now present in the environment, the environment can be navigated, and a robotic arm needs to be manipulated.
The plan now is not carried out by a **smach** state machine, but with **ROSPlan** framework. The robotic arm is manipulated by **MoveIt**. The environment has some obstacle walls that the robot needs to avoid. Again the goal of the game, as for the first assignment, is to find a CONSISTENT true hypothesis.

There are four different positions in the environment (x,y,z respectively) that contains hints:
If the *cluedo_link* ( the end-effector ) of the robot is reasonably close, this will trigger the oracle for the generation of a hint

> **REMARK** x and y coordinates where known a priori as shown in the table below   

  | room  | x,y coordinates  | z coordinate |
  |--|--|--|
  | FirstMarkerPosition | ( -3,0 ) | 0.75 v 1.25  |
  | SecondMarkerPosition | ( +3,0 ) | 0.75 v 1.25 |
  | ThirdMarkerPosition | ( 0,-3 ) | 0.75 v 1.25  |
  | FourthMarkerPosition | ( 0, +3 ) | 0.75 v 1.25 |

There are different values for z, meaning that the robot needs to be able to reach them with it's arm, precisely with its *cluedo_link*.

Please consider that **consistent hypothesis** have been defined as COMPLETED but NOT INCONSISTENT

> *REMARK* A consistent hypothesis is  defined as *completed* when there occurs one role for each class (i.e., one occourence of what, one occourence for who, one occourence for where ).
A straightforward example of such hypothesis is [ID2][12], whose definition is here below reported

```txt
ID2_1: ['where', 'Study']
ID2_2: ['who', 'Col.Mustard']
ID2_3: ['what', 'Rope']
```

As in the first assignment:
- only one ID source is the trustable one.
- Whenever a robot gets a complete hypothesis, it should go in the center of the arena
- Once the center has been reached, it should «tell» its solution (as in the first assignment).
- If the solution is the **correct one**, the game ends

## Software Architecture and Functioning

The most relevant aspects of the project and a brief video tutorial on how to launch the simulation can be found here below

https://user-images.githubusercontent.com/61761835/187249845-1b03e627-d32e-4464-b7d3-0f172419d2f9.mp4

<p align="right">(<a href="#top">back to top</a>)</p>

## ROS node description: An overview

Here there is the UML components diagram of the project

<img src="https://github.com/HolyStone95/ExperimentalRoboticsAssignment2/blob/master/media/component_diagrams/v3/component_diagram.jpg" >

As shown in the above component diagram, this software architecture relies on the synergy of various modules:

- cluedo_kb.py              <!-- PLEASE INSERT HERE -->

- go_to_point.py            <!-- PLEASE INSERT HERE -->

- main.py                  <!-- PLEASE INSERT HERE -->

- action_interface.cpp     <!-- PLEASE INSERT HERE -->

- manipulation.cpp       <!-- PLEASE INSERT HERE -->

### The go_to_point.py node


<p align="center">
<img src="https://github.com/HolyStone95/ExperimentalRoboticsAssignment2/blob/master/media/component_diagrams/v1/erl_assignment_go_to_point_py.jpg" width= 500 height=500>
</p>

It's a ROS service, used for navigation of the robot in the environment. It receives a desired position in form of planar coordinates and, given the fact that it can computes the position of the robot thanks to */odom* it publishes the necessary velocities in order to achieve the movement on */cmd_vel* topic.
Node interfaces:
```Plain txt
Node [/go_to_point]
Publications:
 * /cmd_vel [geometry_msgs/Twist]
 * /rosout [rosgraph_msgs/Log]

Subscriptions:
 * /clock [rosgraph_msgs/Clock]
 * /odom [nav_msgs/Odometry]

Services:
 * /go_to_point
 * /go_to_point/get_loggers
 * /go_to_point/set_logger_level

```
### The main.py node

<p align="center">
<img src="https://github.com/HolyStone95/ExperimentalRoboticsAssignment2/blob/master/media/component_diagrams/v1/erl_assignment_2_main_py.jpg" width= 500 height=500>
</p>

As already said this node represents the main phaser of the project:

- it generates a problem: a [pddl problem][115] is published on a topic
- establishes a plan: a planner is called for  publishing the plan to a topic
- parses a plan: At this stage the PDDL plan is converted into ROS messages, ready to be executed
- dispatches a plan:  for being executed

Node Interfaces:
```Plain txt
Node [/main]
Publications:
 * /rosout [rosgraph_msgs/Log]

Subscriptions:
 * /clock [rosgraph_msgs/Clock]

Services:
 * /main/get_loggers
 * /main/set_logger_level

```
### The cluedo_kb.py node

<p align="center">
<img src="https://github.com/HolyStone95/ExperimentalRoboticsAssignment2/blob/master/media/component_diagrams/v1/erl_assignment_2_cluedo_kb_py.jpg" width= 500 height=500>
</p>

This node represents the knowledge base of the robot, allowing for storing and processing of hypotheses, and interrogation of the oracle in order to check if the problem is being resolved.

> ***REMARK*** the KB listens in on the oracle's topic and as soon as the oracle transmits the clue, the KB adds the message to the ontology without the need for an explicit request

Node interfaces:
```Plain txt
Node [/cluedo_kb]
Publications:
 * /rosout [rosgraph_msgs/Log]

Subscriptions:
 * /oracle_hint [erl2/ErlOracle]

Services:
 * /cluedo_kb/get_loggers
 * /cluedo_kb/set_logger_level
 * /get_id
 * /mark_wrong_id
```

### The action_interface.cpp node

<p align="center">
<img src="https://github.com/HolyStone95/ExperimentalRoboticsAssignment2/blob/master/media/component_diagrams/v3/erl_assignment_2_action_interface_cpp.jpg" width= 500 height=500>
</p>

action_interface.cpp implements all rosplan actions in a single ROS node, moreover:

1. leave_temple
2. shift_gripper
3. gather_hint
4. go_to_wp
5. reach_temple
6. check_consistent_hypo
7. query_hypo


``` Plain txt
; States evaluated: 54
; Cost: 14.013
; Time 0.00
0.000: (leave_temple tp wp1)  [1.000]
1.001: (shift_gripper wp1)  [1.000]
2.002: (gather_hint wp1)  [1.000]
3.003: (go_to_wp wp1 wp2)  [1.000]
4.004: (shift_gripper wp2)  [1.000]
5.005: (gather_hint wp2)  [1.000]
6.006: (go_to_wp wp2 wp3)  [1.000]
7.007: (shift_gripper wp3)  [1.000]
8.008: (gather_hint wp3)  [1.000]
9.009: (go_to_wp wp3 wp4)  [1.000]
10.010: (shift_gripper wp4)  [1.000]
11.011: (gather_hint wp4)  [1.000]
12.012: (reach_temple wp4 tp)  [1.000]
12.012: (check_consistent_hypo wp1)  [1.000]
13.013: (query_hypo tp)  [1.000]
```

Node interfaces:
```Plain txt
Node [/rpi_leave_temple]
Publications:
 * /rosout                                         [rosgraph_msgs/Log]
 * /rosplan_knowledge_base/pddl_action_parameters  [rosplan_knowledge_msgs/DomainFormula]
 * /rosplan_plan_dispatcher/action_feedback        [rosplan_dispatch_msgs/ActionFeedback]

Subscriptions:
 * /clock [rosgraph_msgs/Clock]
 * /rosplan_plan_dispatcher/action_dispatch [rosplan_dispatch_msgs/ActionDispatch]

Services:
 * /rpi_leave_temple/get_loggers
 * /rpi_leave_temple/set_logger_level

--------------------------------------------------------------------------------
Node [/rpi_shift_gripper]
Publications:
 * /rosout [rosgraph_msgs/Log]
 * /rosplan_knowledge_base/pddl_action_parameters  [rosplan_knowledge_msgs/DomainFormula]
 * /rosplan_plan_dispatcher/action_feedback        [rosplan_dispatch_msgs/ActionFeedback]

Subscriptions:
 * /clock [rosgraph_msgs/Clock]
 * /rosplan_plan_dispatcher/action_dispatch        [rosplan_dispatch_msgs/ActionDispatch]

Services:
 * /rpi_shift_gripper/get_loggers
 * /rpi_shift_gripper/set_logger_level

--------------------------------------------------------------------------------
Node [/rpi_gather_hint]
Publications:
 * /rosout [rosgraph_msgs/Log]
 * /rosplan_knowledge_base/pddl_action_parameters  [rosplan_knowledge_msgs/DomainFormula]
 * /rosplan_plan_dispatcher/action_feedback        [rosplan_dispatch_msgs/ActionFeedback]

Subscriptions:
 * /clock [rosgraph_msgs/Clock]
 * /rosplan_plan_dispatcher/action_dispatch        [rosplan_dispatch_msgs/ActionDispatch]

Services:
 * /rpi_gather_hint/get_loggers
 * /rpi_gather_hint/set_logger_level

--------------------------------------------------------------------------------
Node [/rpi_go_to_wp]
Publications:
 * /rosout [rosgraph_msgs/Log]
 * /rosplan_knowledge_base/pddl_action_parameters [rosplan_knowledge_msgs/DomainFormula]
 * /rosplan_plan_dispatcher/action_feedback       [rosplan_dispatch_msgs/ActionFeedback]

Subscriptions:
 * /clock [rosgraph_msgs/Clock]
 * /rosplan_plan_dispatcher/action_dispatch       [rosplan_dispatch_msgs/ActionDispatch]

Services:
 * /rpi_go_to_wp/get_loggers
 * /rpi_go_to_wp/set_logger_level

--------------------------------------------------------------------------------
Node [/rpi_reach_temple]
Publications:
 * /rosout [rosgraph_msgs/Log]
 * /rosplan_knowledge_base/pddl_action_parameters [rosplan_knowledge_msgs/DomainFormula]
 * /rosplan_plan_dispatcher/action_feedback       [rosplan_dispatch_msgs/ActionFeedback]

Subscriptions:
 * /clock [rosgraph_msgs/Clock]
 * /rosplan_plan_dispatcher/action_dispatch       [rosplan_dispatch_msgs/ActionDispatch]

Services:
 * /rpi_reach_temple/get_loggers
 * /rpi_reach_temple/set_logger_level

--------------------------------------------------------------------------------
Node [/rpi_check_consistent_hypo]
Publications:
 * /rosout [rosgraph_msgs/Log]
 * /rosplan_knowledge_base/pddl_action_parameters  [rosplan_knowledge_msgs/DomainFormula]
 * /rosplan_plan_dispatcher/action_feedback        [rosplan_dispatch_msgs/ActionFeedback]

Subscriptions:
 * /clock [rosgraph_msgs/Clock]
 * /rosplan_plan_dispatcher/action_dispatch        [rosplan_dispatch_msgs/ActionDispatch]

Services:
 * /rpi_check_consistent_hypo/get_loggers
 * /rpi_check_consistent_hypo/set_logger_level

--------------------------------------------------------------------------------
Node [/rpi_query_hypo]
Publications:
 * /rosout [rosgraph_msgs/Log]
 * /rosplan_knowledge_base/pddl_action_parameters [rosplan_knowledge_msgs/DomainFormula]
 * /rosplan_plan_dispatcher/action_feedback       [rosplan_dispatch_msgs/ActionFeedback]

Subscriptions:
 * /clock [rosgraph_msgs/Clock]
 * /rosplan_plan_dispatcher/action_dispatch       [rosplan_dispatch_msgs/ActionDispatch]

Services:
 * /rpi_query_hypo/get_loggers
 * /rpi_query_hypo/set_logger_level

```

### The manipulation.cpp node

Concerning the `manipulation_cpp` node:

<p align="center">
<img src="https://github.com/HolyStone95/ExperimentalRoboticsAssignment2/blob/master/media/component_diagrams/v1/erl_assignment_2_manipulation_cpp.jpg" width= 500 height=500>
</p>

This node is simply devoted to control the robot manipulator arm by directly interacting with the MoveIt! framework


Node interfaces:
```Plain txt
Node [/manipulation]
Publications:
 * /attached_collision_object [moveit_msgs/AttachedCollisionObject]
 * /execute_trajectory/cancel [actionlib_msgs/GoalID]
 * /execute_trajectory/goal [moveit_msgs/ExecuteTrajectoryActionGoal]
 * /move_group/cancel [actionlib_msgs/GoalID]
 * /move_group/goal [moveit_msgs/MoveGroupActionGoal]
 * /pickup/cancel [actionlib_msgs/GoalID]
 * /pickup/goal [moveit_msgs/PickupActionGoal]
 * /place/cancel [actionlib_msgs/GoalID]
 * /place/goal [moveit_msgs/PlaceActionGoal]
 * /rosout [rosgraph_msgs/Log]
 * /trajectory_execution_event [std_msgs/String]

Subscriptions:
 * /clock [rosgraph_msgs/Clock]
 * /execute_trajectory/feedback [moveit_msgs/ExecuteTrajectoryActionFeedback]
 * /execute_trajectory/result [moveit_msgs/ExecuteTrajectoryActionResult]
 * /execute_trajectory/status [actionlib_msgs/GoalStatusArray]
 * /move_group/feedback [moveit_msgs/MoveGroupActionFeedback]
 * /move_group/result [moveit_msgs/MoveGroupActionResult]
 * /move_group/status [actionlib_msgs/GoalStatusArray]
 * /pickup/feedback [moveit_msgs/PickupActionFeedback]
 * /pickup/result [moveit_msgs/PickupActionResult]
 * /pickup/status [actionlib_msgs/GoalStatusArray]
 * /place/feedback [moveit_msgs/PlaceActionFeedback]
 * /place/result [moveit_msgs/PlaceActionResult]
 * /place/status [actionlib_msgs/GoalStatusArray]
 * /tf [tf2_msgs/TFMessage]
 * /tf_static [tf2_msgs/TFMessage]

Services:
 * /manipulation
 * /manipulation/get_loggers
 * /manipulation/set_logger_level
```
### The my_simulation.cpp node

<p align="center">
<img src="https://github.com/HolyStone95/ExperimentalRoboticsAssignment2/blob/master/media/component_diagrams/v1/erl2_my_simulation_cpp.jpg" width= 500 height=500>
</p>

This node is the one provided by the professor, that control the hints generation flow.

Node interfaces:
```Plain txt
Node [/my_simulation]
Publications:
 * /oracle_hint [erl2/ErlOracle]
 * /rosout [rosgraph_msgs/Log]
 * /visualization_marker [visualization_msgs/MarkerArray]

Subscriptions:
 * /gazebo/link_states [gazebo_msgs/LinkStates]

Services:
 * /my_simulation/get_loggers
 * /my_simulation/set_logger_level
 * /oracle_solution

```

### Rqt_graph

<img src= "https://github.com/HolyStone95/ExperimentalRoboticsAssignment2/blob/master/media/rqt/rosgraph_nodes_topics_all.png" />

### UML temporal diagram

> :warning: Due to the size of the original image, it is shown only a raw preview, opened with the standard pc's imahe viewer. If you want to download the original file, you can find it [here][111]

<img src= "https://github.com/HolyStone95/ExperimentalRoboticsAssignment2/blob/master/media/miscellaneous/raw_preview.png" />

## How to launch and additional documentation

To test the project, first of all:

- Open a shell and run:

```sh
roslaunch erl_moveit_pkg run_detectibot.launch 2>/dev/null

```

- Open a second shell and run

```sh
roslaunch erl_assignment_2 run_rosplan.launch

```

- Open a third shell and type:

```sh
rosrun erl_assignment_2 main.py


```

Additional documentation of this project can be found opening in the browser the file *_build/html/index.html*

## System limitations

Here below, some of the major system limitations are listed:

- the robot model is not optimal, causing the robot oscillating during navigation.

- the navigation module is not suitable for environments with obstacles as it needs to make a straight line between the starting point and the target.

- since there is no unit that deals explicitly with the marker topology, changing such a topology requires the modification of several parts of the architecture including:
  - the main node that takes into account the topology for being able to do replanning
  - the pddl models, in particular the problem file
  - the oracle, represented by the simulation.cpp node, having hard-coded markers, must be modified to support a new topology

- The architecture is a little heavy in my opinion, with some nodes that are to much centralized. The architecture is vulnerable to centralized crash problems due to the architecture not completely exploiting modularity potentials in its implementation. For example the main node called *action_interface* could be divided and split into different nodes enabling the program to still execute some actions even if another part crashes.

## Possible Technical Improvements

- The current KB is being chosen because there was problems and limitations in using ARMOR due to the nature of the generation flow of ID, decided a priori. By modification of the ID generation flow and by creating an interface in order to initially filter the data, it is fair to say that the integration with ARMOR would be preferable.

- The navigation module used here is extremely rudimental and just enough to perform the necessary navigation required by this assignment, but a better navigation should be implemented, for example using a move base, much more solid, much more capable of obstacle avoidance and of reaching easily certain points in planar space. In addition to that the robot needs a lot of manouvering space to rotate; There should be the need of seeking an appropriate navigation algorithm to reduce the necessary manoeuvring space

- The current robot model is quite unstable. It should be adjusted so that it does not oscillate during its movements. This is due to the simplicity of its mobile base with respect to the shape of its robotic arm. A more robust and stable model should eliminate some of these issues.

- Possible improvements in the manipulation controls used by MoveIt for obtaining a better, more robust, more precise manipulation.

## Contact

Iacopo Pietrasanta - s4119821@studenti.unige.it

Project Link: [https://github.com/HolyStone95/ExperimentalRoboticsAssignment2](https://github.com/HolyStone95/ExperimentalRoboticsAssignment2)

<p align="right">(<a href="#top">back to top</a>)</p>
