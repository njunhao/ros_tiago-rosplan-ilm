(define (problem task)
(:domain turtlebot_energy)
(:objects
    wp0 wp1 wp2 wp3 wp4 - waypoint
    kenny - robot
)
(:init
    (robot_at kenny wp0)
    (docked kenny)
    (dock_at wp0)

	(= (energy kenny) 120)

)
(:goal (and
    (visited wp0)
    (visited wp1)
    (visited wp2)
    (visited wp3)
    (visited wp4)
)))