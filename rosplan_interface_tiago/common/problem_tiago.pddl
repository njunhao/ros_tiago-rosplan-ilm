(define (problem task)
(:domain tiago)
(:objects wp0 wp1 wp2 - waypoint r1 - robot p1 - person o1 o2 - obj)
(:init
    (robot_at r1 wp0)
    (object_at o1 wp1)
    (object_at o2 wp2)
    (emptyhand r1)
    (person_not_found p1)
)
(:goal (and
    (task_received p1)
))
)
