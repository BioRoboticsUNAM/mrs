;********************************************************
;*							*
;*	ROS_virbot.clp					*
;*							*
;*							*
;*			University of Mexico		*
;*			Jesus Savage-Carmona		*
;*							*
;*			11/22/22			*
;*			11/22/22			*
;*							*
;********************************************************


(defglobal ?*num-vision* = 0)
(defglobal ?*plan_number* = 0)
(defglobal ?*plan_time* = 30000)
;(defglobal ?*plan_number_new* = 0)



;************************************************
;*                                              *
;*      Rules                                   *
;*                                              *
;************************************************



(defrule go-zone-not-same-zone-ROS
	?f1 <- (num-sentences 1)
	?f <- (ptrans (actor ?actor)(obj ?actor)(to ?room))
	;(not (attend ))
	( Room (name ?room)(zone ?zone))
	( item (type Robot) (name ?actor))
	=>
	(retract ?f ?f1)
	; it sends the robot to the asqued zone
	(bind ?*plan_number* (+ 1 ?*plan_number*))
	(bind ?*plan_number_new* (+ 1 ?*plan_number_new*))
	(assert (plan (name ptrans)(id ?*plan_number* )(number 1 )(actions goto ?room ?zone)) )
  	(assert (attempt (name ptrans) (id ?*plan_number*)(move robot)(room ?room)(zone ?zone)(on room)(number 1 )))
	(assert (finish-planner ptrans ?*plan_number* ?*plan_number_new*))
)



(defrule delete-goto-ROS
        ?f <- (plans ?id ?num ?success goto)
        ?f1 <- (plan (id ?id)(number ?num )(actions goto ?room ?zone))
        ?f2 <- (item (type Robot) (name ?actor))
        =>
        (retract ?f ?f1)
        ; it modifies the robot position
	(if (eq ?success 1) then
	 	(modify ?f2 (room ?room)) 
	)
)


(defrule delete-grab-ROS
        ?f <- (plans ?id ?num ?success grab)
        ?f1 <- (plan (id ?id)(number ?num )(actions grab ?object))
        ?f2 <- (item (type Robot) (name ?actor))
	?f3 <- (item (type Objects)(name ?object))
        =>
        (retract ?f ?f1)
        ; it modifies the robot position
        (if (eq ?success 1) then
                (modify ?f2 (hands ?object))
                (modify ?f3 (room robot))
        )
)


(defrule delete-drop-ROS
        ?f <- (plans ?id ?num ?success dropped)
        ?f1 <- (plan (id ?id)(number ?num )(actions drop ?object))
        ?f2 <- (item (type Robot) (name ?actor)(room ?zone-robot))
	?f3 <- (item (type Objects)(name ?object))
        =>
        (retract ?f ?f1)
        ; it modifies the robot position
        (if (eq ?success 1) then
                (modify ?f2 (hands nil))
                (modify ?f3 (room ?zone-robot))
        )
)


(defrule delete-find-object-ROS
        ?f <- (plans ?id ?num ?success find-object)
        ?f1 <- (plan (id ?id)(number ?num )(actions find-object ?object))
        =>
        (retract ?f ?f1)
)


(defrule delete-mv-ROS
        ?f <- (plans ?id ?num ?success mv) 
        ?f1 <- (plan (id ?id)(number ?num )(actions mv ?object))
        =>
        (retract ?f ?f1)
)


(defrule delete-go-place-ROS
        ?f <- (plans ?id ?num ?success go) 
        ?f1 <- (plan (id ?id)(number ?num )(actions go ?place))
        =>
        (retract ?f ?f1)
)



(defrule delete-attempt-atrans-ROS
	(declare (salience 100))
	?f <- (attempts ?id ?success)
	?f1 <- (attempt (name atrans) (id ?id )(move ?obj)(room ?room-human)(zone ?zone-human)(on ?human))
        ?f2 <- (item (type Objects) (name ?obj)(room ?room-obj)(zone ?zone))
        ?f3 <- (item (type Human) (name ?human)(room ?room-human)(zone ?zone-human))
	=>
	(retract ?f)
	(if (eq ?success 1) then
		(retract ?f1)
		(modify ?f2 (room ?room-human)(zone ?zone-human)(possession ?human))
		(modify ?f3 (objs ?obj))
	 else
		(assert (fail-plan ?id))
		(retract ?f1) ; modify this later
	)
)


(defrule update-ownership-object
        (item (type Objects) (name ?obj)(possession ?human))
        ?f1 <- (item (type Human) (name ?human)(objs ?obj))
        ?f2 <- (item (type Human) (name ?human1&:(neq ?human1 ?human))(objs ?obj))
        =>
        (modify ?f2 (objs nil))
)



(defrule delete-attempt-generic-ROS
        ?f <- (attempts ?id ?success)
        ?f1 <- (attempt (id ?id))
        =>
        (retract ?f)
        (if (eq ?success 1) then
                (retract ?f1)
         else
                (assert (fail-plan ?id))
                (retract ?f1) ; modify this later
        )
)


(defrule clear-first-upper-obj-ROS
	(declare (salience 100))
	(num-sentences 1)
        (ptrans (actor ?actor)(obj ?obj)(to ?place))
        (item (type Objects) (name ?obj)(room ?room)(zone ?zone)(upper ?upper-obj&:(neq ?upper-obj nothing)))
        =>
        (assert (goal (move ?upper-obj)(on freespace)))
)

(defrule clear-atrans-first-upper-obj-ROS
        (declare (salience 100))
	(num-sentences 1)
        (atrans (actor ?actor)(obj ?obj)(to ?place))
        (item (type Objects) (name ?obj)(room ?room)(zone ?zone)(upper ?upper-obj&:(neq ?upper-obj nothing)))
        =>
        (assert (goal (move ?upper-obj)(on freespace)))
)

(defrule clear-upper-objs-ROS
        (declare (salience 200))
	(num-sentences 1)
        (ptrans (actor ?actor)(obj ?obj)(to ?place))
        ?goal <- (goal (move ?obj1) (on freespace))
        ?f1 <- (item (type Objects) (name ?obj1)(room ?room)(zone ?zone)(upper ?upper-obj&:(neq ?upper-obj nothing)))
        =>
        (assert (goal (move ?upper-obj)(on freespace)))
        ;(modify ?f1 (upper nothing))
)



(defrule clear-atrans-upper-objs-ROS
        (declare (salience 200))
	(num-sentences 1)
        (atrans (actor ?actor)(obj ?obj)(to ?place))
	?goal <- (goal (move ?obj1) (on freespace))
        ?f1 <- (item (type Objects) (name ?obj1)(room ?room)(zone ?zone)(upper ?upper-obj&:(neq ?upper-obj nothing)))
        =>
        (assert (goal (move ?upper-obj)(on freespace)))
	;(modify ?f1 (upper nothing))
)



(defrule move-to-free-space-ROS
        (declare (salience 100))
	(num-sentences 1)
	?goal <- (goal (move ?obj1) (on freespace))
	(ptrans (actor ?actor)(obj ?obj)(to ?place))
	?f1 <- (item (type Objects) (name ?obj)(room ?room)(zone ?zone))
        (item (type Robot) (name ?actor))
        ?f2 <- (item (type Objects) (name ?obj1)(room ?room)(zone ?zone)(upper nothing)(lower ?obj))
        =>
        (retract ?goal)
	(modify ?f1 (upper nothing))
	(modify ?f2 (lower base))
	(bind ?*plan_number* (+ 1 ?*plan_number*))
	(bind ?*plan_number_new* (+ 1 ?*plan_number_new*))
        (printout t ?obj1 " will be moved onto free space in room " ?room crlf)
        (assert (plan (name ptrans)(id ?*plan_number*) (number 1 )(actions goto ?room ?zone)(duration ?*plan_time*)) )
        (assert (plan (name ptrans)(id ?*plan_number*) (number 2 )(actions find-object ?obj1)(duration ?*plan_time*)) )
        (assert (plan (name ptrans)(id ?*plan_number*) (number 3 )(actions grab ?obj1 )(duration ?*plan_time*)) )
        (assert (plan (name ptrans)(id ?*plan_number*) (number 4 )(actions find-object freespace)(duration ?*plan_time*)) )
        (assert (plan (name ptrans)(id ?*plan_number*) (number 5 )(actions go freespace)(duration ?*plan_time*)) )
        (assert (plan (name ptrans)(id ?*plan_number*) (number 6 )(actions drop ?obj1 )(duration ?*plan_time*)) )
        (assert (attempt (name ptrans) (id ?*plan_number*) (move ?obj1)(room ?room)(zone ?zone)(on freespace)(number 6 ) ))
)


(defrule atrans-move-to-free-space-ROS
        (declare (salience 100))
        ?goal <- (goal (move ?obj1) (on freespace))
        (atrans (actor ?actor)(obj ?obj)(to ?place))
        ?f1 <- (item (type Objects) (name ?obj)(room ?room)(zone ?zone))
        (item (type Robot) (name ?actor))
        ?f2 <- (item (type Objects) (name ?obj1)(room ?room)(zone ?zone)(upper nothing)(lower ?obj))
        =>
        (retract ?goal)
        (modify ?f1 (upper nothing))
        (modify ?f2 (lower base))
        (bind ?*plan_number* (+ 1 ?*plan_number*))
        (bind ?*plan_number_new* (+ 1 ?*plan_number_new*))
        (printout t ?obj1 " will be moved onto free space in room " ?room crlf)
        (assert (plan (name atrans)(id ?*plan_number*) (number 1 )(actions goto ?room ?zone)(duration ?*plan_time*)) )
        (assert (plan (name atrans)(id ?*plan_number*) (number 2 )(actions find-object ?obj1)(duration ?*plan_time*)) )
        (assert (plan (name atrans)(id ?*plan_number*) (number 3 )(actions grab ?obj1 )(duration ?*plan_time*)) )
        (assert (plan (name atrans)(id ?*plan_number*) (number 4 )(actions find-object freespace)(duration ?*plan_time*)) )
        (assert (plan (name atrans)(id ?*plan_number*) (number 5 )(actions go freespace)(duration ?*plan_time*)) )
        (assert (plan (name atrans)(id ?*plan_number*) (number 6 )(actions drop ?obj1 )(duration ?*plan_time*)) )
        (assert (attempt (name atrans) (id ?*plan_number*) (move ?obj1)(room ?room)(zone ?zone)(on freespace)(number 6 ) ))
)



(defrule move-to-free-space-objects-ROS
        (declare (salience 100))
        ?goal <- (goal (move ?obj1) (on freespace))
        ?goal2 <- (goal (move ?obj2) (on freespace))
        (ptrans (actor ?actor)(obj ?obj)(to ?place))
        (item (type Robot) (name ?actor))
        ?f2 <- (item (type Objects) (name ?obj1)(room ?room)(zone ?zone)(upper nothing)(lower ?obj2))
        ?f1 <- (item (type Objects) (name ?obj2)(room ?room)(zone ?zone))
        =>
        (retract ?goal)
        (modify ?f1 (upper nothing))
        (modify ?f2 (lower base))
        (bind ?*plan_number* (+ 1 ?*plan_number*))
	(bind ?*plan_number_new* (+ 1 ?*plan_number_new*))
        (printout t ?obj1 " will be moved onto free space in room " ?room crlf)
        (assert (plan (name ptrans)(id ?*plan_number*) (number 1 )(actions goto ?room ?zone)(duration ?*plan_time*)) )
        (assert (plan (name ptrans)(id ?*plan_number*) (number 2 )(actions find-object ?obj1)(duration ?*plan_time*)) )
        (assert (plan (name ptrans)(id ?*plan_number*) (number 3 )(actions grab ?obj1 )(duration ?*plan_time*)) )
        (assert (plan (name ptrans)(id ?*plan_number*) (number 4 )(actions find-object freespace)(duration ?*plan_time*)) )
        (assert (plan (name ptrans)(id ?*plan_number*) (number 5 )(actions go freespace)(duration ?*plan_time*)) )
        (assert (plan (name ptrans)(id ?*plan_number*) (number 6 )(actions drop ?obj1 )(duration ?*plan_time*)) )
        (assert (attempt (name ptrans) (id ?*plan_number*) (move ?obj1)(room ?room)(zone ?zone)(on freespace)(number 6 ) ))
)


(defrule atrans-move-to-free-space-objects-ROS
        (declare (salience 100))
        ?goal <- (goal (move ?obj1) (on freespace))
        ?goal2 <- (goal (move ?obj2) (on freespace))
        (atrans (actor ?actor)(obj ?obj)(to ?place))
        (item (type Robot) (name ?actor))
        ?f2 <- (item (type Objects) (name ?obj1)(room ?room)(zone ?zone)(upper nothing)(lower ?obj2))
        ?f1 <- (item (type Objects) (name ?obj2)(room ?room)(zone ?zone))
        =>
        (retract ?goal)
        (modify ?f1 (upper nothing))
        (modify ?f2 (lower base))
        (bind ?*plan_number* (+ 1 ?*plan_number*))
        (bind ?*plan_number_new* (+ 1 ?*plan_number_new*))
        (printout t ?obj1 " will be moved onto free space in room " ?room crlf)
        (assert (plan (name atrans)(id ?*plan_number*) (number 1 )(actions goto ?room ?zone)(duration ?*plan_time*)) )
        (assert (plan (name atrans)(id ?*plan_number*) (number 2 )(actions find-object ?obj1)(duration ?*plan_time*)) )
        (assert (plan (name atrans)(id ?*plan_number*) (number 3 )(actions grab ?obj1 )(duration ?*plan_time*)) )
        (assert (plan (name atrans)(id ?*plan_number*) (number 4 )(actions find-object freespace)(duration ?*plan_time*)) )
        (assert (plan (name atrans)(id ?*plan_number*) (number 5 )(actions go freespace)(duration ?*plan_time*)) )
        (assert (plan (name atrans)(id ?*plan_number*) (number 6 )(actions drop ?obj1 )(duration ?*plan_time*)) )
        (assert (attempt (name atrans) (id ?*plan_number*) (move ?obj1)(room ?room)(zone ?zone)(on freespace)(number 6 ) ))
)



(defrule ptrans-object-unknown-ROS
        ?f <- (num-sentences 1)
        ?f1 <- (ptrans (actor ?actor)(obj ?obj)(to ?place))
        (item (type Robot) (name ?actor))
        (item (type Objects) (name ?obj)(room nil)(upper nothing))
        =>
        ; it asks where the objects is
        (retract ?f)
        (bind ?*plan_number* (+ 1 ?*plan_number*))
        (bind ?*plan_number_new* (+ 1 ?*plan_number_new*))
        (assert (plan (name state)(id ?*plan_number* )(number 1 )(actions ask ?obj room)) )
        (assert (attempt (name state) (id ?*plan_number* ) (state ?obj)(room unknown)(zone any)(number 1 )))
        (assert (finish-planner state ?*plan_number* ?*plan_number_new*))
)






(defrule exec-ptrans-object-direct-ROS
	?f1 <- (num-sentences 1)
        ?f <- (ptrans (actor ?actor)(obj ?obj)(to ?place))
	(item (type Robot) (name ?actor))
	(item (type Objects) (name ?obj)(room ?room)(zone ?zone)(upper nothing))
	( Room (name ?place)(zone ?zone-place))
        =>
	; it sends the robot to the asqued zone
	(retract ?f ?f1)
	(bind ?*plan_number* (+ 1 ?*plan_number*))
	(bind ?*plan_number_new* (+ 1 ?*plan_number_new*))
        (assert (plan (name ptrans)(id ?*plan_number* )(number 1 )(actions goto ?room ?zone)) )
        (assert (plan (name ptrans)(id ?*plan_number* ) (number 2 )(actions find-object ?obj)) )
        (assert (plan (name ptrans)(id ?*plan_number* ) (number 3 )(actions grab ?obj )) )
        (assert (plan (name ptrans)(id ?*plan_number* ) (number 4 )(actions goto ?place ?zone-place)) )
        (assert (plan (name ptrans)(id ?*plan_number* ) (number 5 )(actions find-object freespace)) )
        (assert (plan (name ptrans)(id ?*plan_number* ) (number 6 )(actions go freespace )) )
        (assert (plan (name ptrans)(id ?*plan_number* ) (number 7 )(actions drop ?obj )) )
        (assert (attempt (name ptrans) (id ?*plan_number* ) (move ?obj)(room ?room)(zone ?zone)(on floor)(number 7 )))
        (assert (finish-planner ptrans ?*plan_number* ?*plan_number_new*))
)



(defrule exec-ptrans-human-ROS
	?f1 <- (num-sentences 1)
        ?f <- (ptrans (actor ?actor)(obj ?actor)(to ?human))
	(item (type Robot) (name ?actor))
        (item (type Human) (name ?human)(room ?room)(zone ?zone))
        ;( Room (name ?place)(zone ?zone-place))
        =>
        ; it sends the robot to the asqued zone
	(retract ?f ?f1)
	(bind ?*plan_number* (+ 1 ?*plan_number*))
	(bind ?*plan_number_new* (+ 1 ?*plan_number_new*))
        (assert (plan (name ptrans)(id ?*plan_number* )(number 1 )(actions goto ?room ?zone)) )
        (assert (plan (name ptrans)(id ?*plan_number* ) (number 2 )(actions find-object ?human)) )
        (assert (attempt (name ptrans) (id ?*plan_number* ) (move ?actor)(room ?room)(zone ?zone)(on room)(number 2 )))
        (assert (finish-planner ptrans ?*plan_number* ?*plan_number_new*))
)



; Go to the kitchen, find Mary, and give her an apple
; Go to the studio, find Mary, and give her a book
; ptrans (actor robot)(obj robot)(to studio))
; atrans (actor robot)(obj book)(to mother))
; attend (actor robot)(obj mother)(from studio))


(defrule exec-ptrans-attend-atrans-ROS
	;(declare (salience 200))
	?f <- (num-sentences 3)
 	?f1 <- (ptrans (actor ?actor)(obj ?actor)(to ?room))
	(item (type Robot) (name ?actor))
	?f2 <- (attend (actor ?actor)(obj ?human))
        ;(item (type Human) (name ?human)(room ?room-human)(zone ?zone-human))
	(item (type Human) (name ?human)(room ?room-human&:(neq ?room-human nil))(zone ?zone-human))
        ?f3 <- (atrans (actor ?actor)(obj ?obj)(to ?human))
        ;(item (type Objects) (name ?obj)(room ?room)(upper nothing))
	(item (type Objects) (name ?obj)(room ?room&:(neq ?room nil))(upper nothing))
        ( Room (name ?room)(zone ?zone-place))
        =>
	(retract ?f ?f1 ?f2 ?f3)
	(bind ?*plan_number* (+ 1 ?*plan_number*))
	(bind ?*plan_number_new* (+ 1 ?*plan_number_new*))
        (assert (plan (name ptrans-attend-atrans)(id ?*plan_number* )(number 1 )(actions goto ?room ?zone-place)) )
        (assert (plan (name ptrans-attend-atrans)(id ?*plan_number* ) (number 2 )(actions find-object ?obj)) )
        (assert (plan (name ptrans-attend-atrans)(id ?*plan_number* ) (number 3 )(actions grab ?obj )) )
        (assert (plan (name ptrans-attend-atrans)(id ?*plan_number* ) (number 4 )(actions find-object ?human)) )
        (assert (plan (name ptrans-attend-atrans)(id ?*plan_number* ) (number 5 )(actions go ?human )) )
        (assert (plan (name ptrans-attend-atrans)(id ?*plan_number* ) (number 6 )(actions drop ?obj )) )
        (assert (attempt (name ptrans-attend-atrans) (id ?*plan_number* )(move ?obj)(room ?room-human)(zone ?zone-human)(on ?human)(number 6 )))
        (assert (finish-planner ptrans-attend-atrans ?*plan_number* ?*plan_number_new*))
)



;(ptrans (actor robot) (obj robot) (from robots_place) (to kitchen))
;(attend (actor robot) (obj father))

(defrule exec-ptrans-attend-ROS
        ?f <- (num-sentences 2)
        ?f1 <- (ptrans (actor ?actor)(obj ?actor)(to ?room))
        (item (type Robot) (name ?actor))
        ?f2 <- (attend (actor ?actor)(obj ?human))
        ;(item (type Human) (name ?human)(room ?room-human)(zone ?zone-human))
        (item (type Human) (name ?human)(room ?room-human&:(neq ?room-human nil))(zone ?zone-human))
        ( Room (name ?room)(zone ?zone-place))
        =>
        (retract ?f ?f1 ?f2)
        (bind ?*plan_number* (+ 1 ?*plan_number*))
        (bind ?*plan_number_new* (+ 1 ?*plan_number_new*))
        (assert (plan (name ptrans-attend-atrans)(id ?*plan_number* )(number 1 )(actions goto ?room ?zone-place)) )
        (assert (plan (name ptrans-attend-atrans)(id ?*plan_number* ) (number 2 )(actions find-object ?human)) )
        (assert (plan (name ptrans-attend-atrans)(id ?*plan_number* ) (number 3 )(actions go ?human )) )
        (assert (attempt (name ptrans-attend-atrans) (id ?*plan_number* )(move ?actor)(room ?room-human)(zone ?zone-human)(on ?human)(number 3 )))
        (assert (finish-planner ptrans-attend-atrans ?*plan_number* ?*plan_number_new*))
)






(defrule exec-atrans-object-recepient-ROS
	?f <- (num-sentences 1)
        ?f1 <- (atrans (actor ?actor)(obj ?obj)(to ?human))
	(not (attend ))
        (item (type Robot) (name ?actor))
        (item (type Objects) (name ?obj)(room ?room)(zone ?zone)(upper nothing))
        (item (type Human) (name ?human)(room ?room-human)(zone ?zone-human))
        ( Room (name ?place)(zone ?zone-place))
        =>
        ; it sends the robot to the asqued zone
        (retract ?f ?f1)
        (bind ?*plan_number* (+ 1 ?*plan_number*))
        (bind ?*plan_number_new* (+ 1 ?*plan_number_new*))
        (assert (plan (name atrans)(id ?*plan_number* )(number 1 )(actions goto ?room ?zone)) )
        (assert (plan (name atrans)(id ?*plan_number* ) (number 2 )(actions find-object ?obj)) )
        (assert (plan (name atrans)(id ?*plan_number* ) (number 3 )(actions grab ?obj )) )
        (assert (plan (name atrans)(id ?*plan_number* ) (number 4 )(actions goto ?room-human ?zone-human)) )
        (assert (plan (name atrans)(id ?*plan_number* ) (number 5 )(actions find-object ?human)) )
        (assert (plan (name atrans)(id ?*plan_number* ) (number 6 )(actions go ?human )) )
        (assert (plan (name atrans)(id ?*plan_number* ) (number 7 )(actions drop ?obj )) )
        (assert (attempt (name atrans) (id ?*plan_number* )(move ?obj)(room ?room-human)(zone ?zone-human)(on ?human)(number 7 )))
        (assert (finish-planner atrans ?*plan_number* ?*plan_number_new*))
)



(defrule delete-num-sentences-ROS
        (declare (salience -200))
        ?f <- (num-sentences ?n)
	=>
	(retract ?f)
)



(defrule state-location-ROS
	(declare (salience 300))
	?f <- (num-sentences 1)
	?f2 <-(state (attribute location)(obj ?object)(value ?place))
	?f1 <- (item (type Objects)(name ?object))
        =>
	(retract ?f2)
        ; it modifies the object position
        (modify ?f1 (room ?place))
)



(defrule qtrans-object-ROS
        ?f <- (num-sentences 1)
        ?f1 <- (qtrans (obj ?obj)(question where))
        (item (type Objects) (name ?obj)(room ?room)(zone ?zone))
        =>
        ; it answer where the objects is
        (retract ?f ?f1)
        (bind ?*plan_number* (+ 1 ?*plan_number*))
        (bind ?*plan_number_new* (+ 1 ?*plan_number_new*))
        (assert (plan (name state)(id ?*plan_number* )(number 1 )(actions answer ?obj ?room ?zone)))
        (assert (attempt (name answer) (id ?*plan_number* )(question where)(zone ?zone)(number 1 )))
        (assert (finish-planner state ?*plan_number* ?*plan_number_new*))
)


(defrule qtrans-human-ROS
        ?f <- (num-sentences 1)
        ?f1 <- (qtrans (obj ?human)(question where))
	(item (type Human) (name ?human)(room ?room)(zone ?zone))
        =>
        ; it answer where the objects is
        (retract ?f ?f1)
        (bind ?*plan_number* (+ 1 ?*plan_number*))
        (bind ?*plan_number_new* (+ 1 ?*plan_number_new*))
        (assert (plan (name state)(id ?*plan_number* )(number 1 )(actions answer ?human ?room ?zone)))
        (assert (attempt (name answer) (id ?*plan_number* )(question where)(zone ?zone)(number 1 )))
        (assert (finish-planner state ?*plan_number* ?*plan_number_new*))
)



(defrule qtrans-Robot-ROS
        ?f <- (num-sentences 1)
        ?f1 <- (qtrans (obj ?robot)(question where))
        (item (type Robot) (name ?robot)(room ?room)(zone ?zone))
        =>
        ; it answer where the robot is
        (retract ?f ?f1)
        (bind ?*plan_number* (+ 1 ?*plan_number*))
        (bind ?*plan_number_new* (+ 1 ?*plan_number_new*))
        (assert (plan (name state)(id ?*plan_number* )(number 1 )(actions answer ?robot ?room ?zone)))
        (assert (attempt (name answer) (id ?*plan_number* )(question where)(zone ?zone)(number 1 )))
        (assert (finish-planner state ?*plan_number* ?*plan_number_new*))
)



(defrule qtrans-who-object-ROS
        ?f <- (num-sentences 1)
        ?f1 <- (qtrans (obj ?obj)(question who))
        (item (type Objects) (name ?obj)(possession ?human)(zone ?zone))
        =>
        ; it answer where the objects is
        (retract ?f ?f1)
        (bind ?*plan_number* (+ 1 ?*plan_number*))
        (bind ?*plan_number_new* (+ 1 ?*plan_number_new*))
        (assert (plan (name state)(id ?*plan_number* )(number 1 )(actions answer ?obj ?human ?zone)))
        (assert (attempt (name answer) (id ?*plan_number* )(question who)(zone ?zone)(number 1 )))
        (assert (finish-planner state ?*plan_number* ?*plan_number_new*))
)



(defrule exec-atrans-no-recipient-ROS
        ?f <- (num-sentences 1)
        (atrans (actor ?actor)(to nil))
        (item (type Robot) (name ?actor))
        =>
 	; it asks who is the human to receive the object
        (retract ?f)
        (bind ?*plan_number* (+ 1 ?*plan_number*))
        (bind ?*plan_number_new* (+ 1 ?*plan_number_new*))
        (assert (plan (name state)(id ?*plan_number* )(number 1 )(actions ask human recipient)) )
        (assert (attempt (name state) (id ?*plan_number* ) (state recipient)(human unknown)(number 1 )))
        (assert (finish-planner state ?*plan_number* ?*plan_number_new*))

)



(defrule state-recipient-ROS
        (declare (salience 300))
        (num-sentences 1)
        ?f2 <- (state (attribute recipient)(value ?human))
	?f1 <- (atrans (actor ?actor)(to nil))
        =>
        (retract ?f2)
        ; it modifies the object position
        (modify ?f1 (to ?human))
)


