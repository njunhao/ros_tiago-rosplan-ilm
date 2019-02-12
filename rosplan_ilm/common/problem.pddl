(define (problem task)
(:domain tiago)
(:objects
    wp0 wp1 wp2 - waypoint
    r1 - robot
    o1 o2 - obj
    p1 - person
)
(:init
    (robot_at r1 wp0)


    (emptyhand r1)

    (localised r1)

    (object_at o1 wp1)
    (object_at o2 wp2)




    (person_not_found p1)




)
(:goal (and
    (person_found p1)
))
(:metric maximize)
)
