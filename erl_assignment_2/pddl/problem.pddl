(define (problem task)
(:domain db_domain)
(:objects
    wp1 wp2 wp3 wp4 - waypoint
    tp - temple
)
(:init





    (at_temple tp)


    (not_gathered_hint wp1)
    (not_gathered_hint wp2)
    (not_gathered_hint wp3)
    (not_gathered_hint wp4)

    (not_has_been_at wp1)
    (not_has_been_at wp2)
    (not_has_been_at wp3)
    (not_has_been_at wp4)

    (not_gripper_inplace)

)
(:goal (and
    (true_hypo)
))
)
