
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

; It sends a message that clips is alive
(defrule clips-alive-start
          (declare (salience 1000))
          (ros-node ?node)
        =>
          (printout ROS "clips alive 0" crlf)
          (rospub ?node "clips alive 0")
)



;(defrule clips-alive
        ;?f <- (alive clips)
	;(ros-node ?node)
        ;=>
        ;(retract ?f)
        ;(rospub ?node "clips alive 0")
;)



(defrule clips-alive
	(declare (salience -900))
        ?f <- (alive clips)
        (ros-node ?node)
        =>
        (retract ?f)
        (printout ROS "clips alive 0" crlf)
        (rospub ?node "clips alive 0")
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
	(ros-node ?node)
        =>
        (retract ?f ?f1)
        ;(retract ?f1)
	(bind ?var (str-cat ?system " num_plans-total " ?npl " " ?num " " ?t " "  ?n))
	(rospub ?node ?var)
	(printout ROS ?var crlf)
)


(defrule send-ros-num_plans-number
	(declare (salience 100))
        (ros-node ?node)
        ?f <-  (get-num-plans-number ?npl)
        ?f1 <- (send-ROS ?system num_plans-number ?npl ?num ?t ?n)
        =>
        (retract ?f ?f1)
        ( bind ?var (str-cat ?system " num_plans-number " ?npl " " ?num " " ?t ))
	(rospub ?node ?var)
	(printout ROS ?var crlf)
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
        (ros-node ?node)
        ?f1 <- (send-ROS ?system ?id&:(neq ?id num_plans-total) ?nm ?action ?arg1 ?arg2)
        =>
        (retract ?f1)
        (bind ?var (str-cat ?system " plan " ?id " " ?nm " " ?action " " ?arg1 " " ?arg2 ))
	(rospub ?node ?var)
	(printout ROS ?var crlf)
)



(defrule send-plan-one-argument
        (ros-node ?node)
        ?f1 <- (send-ROS ?system ?id ?nm ?action ?argument)
        =>
        (retract ?f1)
	(bind ?var (str-cat ?system " plan " ?id " " ?nm " " ?action " "  ?argument))
	(rospub ?node ?var)
	(printout ROS ?var crlf)
)

(defrule send-plan-three-arguments
        (declare (salience 100))
        (ros-node ?node)
        ?f1 <- (send-ROS ?system ?id ?nm ?action ?arg1 ?arg2 ?arg3)
        =>
        (retract ?f1)
        (bind ?var (str-cat ?system " plan " ?id " " ?nm " " ?action " "  ?arg1 " " ?arg2 " " ?arg3))
	(rospub ?node ?var)
	(printout ROS ?var crlf)
)


;(defrule keep-clips-alive
	;(declare (salience -1000))
	;?f1 <- (keep-clips-alive)
	;=>
	;(retract ?f1)
	;(assert (keep-clips-alive))
;)



