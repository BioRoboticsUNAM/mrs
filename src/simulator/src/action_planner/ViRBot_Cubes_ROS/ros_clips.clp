
;************************************************
;*                                              *
;*      ros_clips.clp                           *
;*                                              *
;*      Jesus Savage                            *
;*                                              *
;*              Bio-Robotics Laboratory         *
;*              UNAM, 2020                      *
;*                                              *
;*                                              *
;************************************************


(defrule clips-alive
        ?f <- (alive clips)
        =>
        (retract ?f)
        (printout t "ROS clips alive 0 ROS")
)


;(defrule clips-alive-finish
	;(declare (salience 100))
	;(finish-planner ?name ?num_pln)
        ;?f <- (alive clips)
        ;=>
        ;(retract ?f)
        ;(printout t "ROS clips alive 1 ROS")
;)

;(defrule send-ros
	;(declare (salience 100))
	;;?f <-  (step ?n)
	;?f1 <- (send-ROS ?system ?action ?command ?t ?num)
	;=>
        ;;(retract ?f ?f1)
        ;(retract ?f1)
        ;(printout t "ROS " ?system " " ?action " " ?command " " ?t " " ?num " ROS")
;)


(defrule send-ros-num_plans
        (declare (salience 100))
        ?f <-  (get-num-plans-total)
        ?f1 <- (send-ROS ?system num_plans-total ?npl ?num ?t ?n)
        =>
        (retract ?f ?f1)
        ;(retract ?f1)
        (printout t "ROS " ?system " num_plans-total " ?npl " " ?num " " ?t " ROS")
)


(defrule send-ros-num_plans-number
	(declare (salience 100))
        ?f <-  (get-num-plans-number ?npl)
        ?f1 <- (send-ROS ?system num_plans-number ?npl ?num ?t ?n)
        =>
        (retract ?f ?f1)
        (printout t "ROS " ?system " num_plans-number " ?npl " " ?num " " ?t " ROS")
)


;(defrule step-ros
	;?f <- (step ?num)
	;=>
	;(retract ?f)
	;(printout t "ROS step " ?num " ROS")
;)



(defrule send-plan-two-arguments
	;(declare (salience 100))
        ;?f <-  (step ?id ?nm)
        ?f1 <- (send-ROS ?system ?id&:(neq ?id num_plans-total) ?nm ?action ?arg1 ?arg2)
        =>
        (retract ?f1)
        (printout t "ROS " ?system " plan " ?id " " ?nm " " ?action " " ?arg1 " " ?arg2 " ROS")
)



(defrule send-plan-one-argument
        ;(declare (salience 100))
        ;?f <-  (step ?id ?nm)
        ?f1 <- (send-ROS ?system ?id ?nm ?action ?argument)
        =>
        (retract ?f1)
        (printout t "ROS " ?system " plan " ?id " " ?nm " " ?action " "  ?argument " ROS")
)


(defrule send-plan-three-arguments
        (declare (salience 100))
        ;?f <-  (step ?id ?nm)
        ?f1 <- (send-ROS ?system ?id ?nm ?action ?arg1 ?arg2 ?arg3)
        =>
        (retract ?f1)
        (printout t "ROS " ?system " plan " ?id " " ?nm " " ?action " "  ?arg1 " " ?arg2 " " ?arg3  " ROS")
)



