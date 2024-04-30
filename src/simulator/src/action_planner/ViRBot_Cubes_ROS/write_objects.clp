;********************************************************
;*                                                      *
;*      write_objects.clp                               *
;*                                                      *
;*                      University of Mexico            *
;*                      J. Savage                       *
;*                                                      *
;*                      6-6-2022                        *
;*                      6-6-2022                        *
;********************************************************


(defglobal ?*file_name_objects* = "../../data/objects/objects.txt")
(defglobal ?*file_name_initial_state* = "virbot_initial_state.txt")


(defrule open-file
        ?f <- (initial-fact)
        =>
        (retract ?f)
	(open ?*file_name_objects* obj "w")
	(open ?*file_name_initial_state* initial "w")
        (assert (file_objects obj))
        (assert (file_initial_state initial))
	(printout t "Open file " ?*file_name_objects* crlf)
	(printout t "Open file " ?*file_name_initial_state* crlf)
)



(defrule writting_objects_obj_file
        (declare (salience 100))
        ?f1 <- (item (type Objects) (name ?object) (pose ?x ?y ?z))
	(file_objects ?file)
        =>
        (printout t ?object " x " ?x " " ?y " " ?z  crlf)
	;(format ?file "?object %f %f" ?object ?x ?y)
        ;(printout ?file crlf)
	(printout obj ?object " " ?x " " ?y crlf)

)

(defrule writting_rooms_obj_file
        (declare (salience 90))
        ?f1 <- (Room (name ?object) (center ?x ?y))
        (file_objects ?file)
        =>
        (printout t ?object " x " ?x " " ?y crlf)
        ;(format ?file "?object %f %f" ?object ?x ?y)
        ;(printout ?file crlf)
        (printout obj ?object " " ?x " " ?y crlf)

)


(defrule writting_furniture_obj_file
        (declare (salience 100))
        ?f1 <- (item (type Furniture) (name ?object) (pose ?x ?y ?z))
        (file_objects ?file)
        =>
        (printout t ?object " x " ?x " " ?y " " ?z  crlf)
        (printout obj ?object " " ?x " " ?y crlf)

)



(defrule writting_objects_initial_file
        (declare (salience 90))
        ?f1 <- (item (type Objects) (name ?object)(room ?room)(pose ?x ?y ?z))
        (file_initial_state ?file)
        =>
        (printout t "object " ?object " " ?room " x " ?x " " ?y " " ?z  crlf)
        (printout initial "object " ?object " " ?room " " ?x " " ?y crlf)

)


(defrule writting_furniture_initial_file
        (declare (salience 90))
        ?f1 <- (item (type Furniture) (name ?object)(room ?room) (pose ?x ?y ?z))
        (file_initial_state ?file)
        =>
        (printout t "furniture " ?object " " ?room  " x " ?x " " ?y " " ?z  crlf)
        (printout initial "furniture " ?object " " ?room " " ?x " " ?y crlf)

)



(defrule writting_rooms_initial_file
        (declare (salience 90))
        ?f1 <- (Room (name ?object)(center ?x ?y))
        (file_initial_state ?file)
        =>
        (printout t "room " ?object " x " ?x " " ?y crlf)
        (printout initial "room " ?object " " ?x " " ?y crlf)

)





(defrule clossing_objects_file
	?f <- (file_objects ?file)
	?f1 <- (file_initial_state ?file_initial)
        =>
	(retract ?f ?f1)
	(close ?file)
	(close ?file_initial)
        (printout t "clossing files" crlf)
	(exit)
)

