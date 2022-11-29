echo "---> Generating a Problem"
rosservice call /rosplan_problem_interface/problem_generation_server
echo "... ok!  "

echo "--> Planning"
rosservice call /rosplan_planner_interface/planning_server
echo "... ok!  "

echo "--> Executing the Plan"
rosservice call /rosplan_parsing_interface/parse_plan
echo "... ok!  "

echo "--> Dispatching actions"
rosservice call /rosplan_plan_dispatcher/dispatch_plan
echo "... ok!  "


`# rosservice call /rosplan_plan_dispatcher/dispatch_plan ` \
`# rosservice call /rosplan_problem_interface/problem_generation_server` \
`# rosservice call /rosplan_planner_interface/planning_server` \
`# rosservice call /rosplan_parsing_interface/parse_plan_from_file "plan_path: '/root/ros_ws/src/explab_2nd/common/planning_plan_from_parse.pddl'"` \

