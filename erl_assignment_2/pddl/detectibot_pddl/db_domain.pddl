(define (domain db_domain)

(:requirements 
	:strips
	:typing
	:fluents
	:disjunctive-preconditions
	:durative-actions
	:adl
)

(:types
	waypoint
	temple
)

(:predicates
	(at ?wp - waypoint)                ;; robot at a certain point
	(has_been_at ?wp - waypoint)       ;; robot has visited a certain location 
	(true_hypo)                        ;; true hypothesis, the real goal of the robot
	(gripper_inplace)                  ;; gripper in the right position for gathering hints
	(consistent_hypo)                  ;; complete hypothesis, ready to be tested 
	(at_temple ?tp - temple)           ;; robot stays in Temple, ready for querying 
	(gathered_hint ?wp - waypoint)     ;; Hint got collected 
	(not_gathered_hint ?wp - waypoint)
	(not_has_been_at ?wp - waypoint)
	(not_gripper_inplace)              ;; gripper not in the right position for gathering hints
	
	
	
)


;; Move to any waypoint
(:durative-action go_to_wp
	:parameters (?from ?to - waypoint)
	:duration ( = ?duration 1)
	:condition (and
		(at start(at ?from))
		(at start(not_gripper_inplace))
		(at start(gathered_hint ?from ))
		(at start(not_has_been_at ?to))
		)
	:effect (and
		(at end(at ?to))
		(at start (not (at ?from)))
		;;(at end (not_gripper_inplace)
		(at end(not(not_has_been_at ?from)))
		(at end (has_been_at ?to))
	
		
		
		
)
)

;; Move the gripper so that it can grab the hint
(:durative-action shift_gripper
	:parameters (?wp - waypoint)
	:duration ( = ?duration 1)
	:condition (and
		(at start(at ?wp))
		(at start(not_gripper_inplace))
		(at start(not_gathered_hint ?wp))
		)
	:effect (and
		(at end( gripper_inplace))
		(at end (not (not_gripper_inplace)))
		)
) 
		

;; collect the hint within the marker 
(:durative-action gather_hint
	:parameters (?wp - waypoint)
	:duration ( = ?duration 1)
	:condition (and 
				(at start(at ?wp))
				(at start(gripper_inplace))
				)
					
				
	:effect (and 
				(at end( gathered_hint ?wp))
				(at end(not_gripper_inplace))
				(at end(not(gripper_inplace)))
				(at end(not(not_gathered_hint ?wp)))
			)
)


;; reach the temple for querying the oracle 
(:durative-action reach_temple
	:parameters ( ?from - waypoint ?to -temple)
	:duration ( = ?duration 1)
	:condition (and
			   (at start(at ?from))
			   (at start (forall (?wp - waypoint) ( gathered_hint ?wp)))
			   )
	:effect (and
		(at start(not (at ?from)))
		(at end (at_temple ?to))
		(at end (at_temple ?to))
		)
)


;; leave the temple for starting exploring the waypoints
(:durative-action leave_temple
	:parameters ( ?from - temple ?to - waypoint)
	:duration ( = ?duration 1)
	:condition(and
				(at start (at_temple ?from))
				;; (at start(not_gripper_inplace))
				)
	:effect(and
		(at start (not(at_temple ?from)))
		(at end(at ?to))
		)
)


(:durative-action check_consistent_hypo
	:parameters(?wp - waypoint)
	:duration (= ?duration 1)
	:condition (and 
			   (at start (forall (?wp - waypoint) ( gathered_hint ?wp)))
				)
	:effect ( and
			(at end (consistent_hypo))
			)
)			

(:durative-action query_hypo
	:parameters(?tp - temple)
	:duration (= ?duration 1)
	:condition (and 
				(at start(at_temple ?tp))
				(at start(consistent_hypo))
				)
				
	:effect (and
			(at end(true_hypo))
			)
)

)
