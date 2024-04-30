
;************************************************
;*						*
;*	Initial state 				*
;*						*
;*                      J.Savage, UNAM          *
;*						*
;*                      1/5/20                  *
;*                                              *
;************************************************




(deffacts Initial-state-objects-rooms-zones-actors


; Objects definitions
	( item (type Objects) (name deposit)(room bedroom)(image table)( attributes no-pick brown)(pose 6.183334 7.000000 0.0))
	( item (type Objects) (name storage)(room livingroom)(image table)( attributes no-pick brown)(pose 3.183334 2.000000 0.0))
	( item (type Objects) (name blockA)(room bedroom)(zone deposit)(image blockA)(attributes pick)(pose 0.34 0.30 0.0))
	( item (type Objects) (name blockB)(room bedroom)(zone deposit)(image blockB)(attributes pick)(pose 0.34 0.40 0.0))
	( item (type Objects) (name blockC)(room bedroom)(zone deposit)(image blockC)(attributes pick)(pose 0.34 0.20 0.0))
	( item (type Objects) (name blockD)(room livingroom)(zone storage)(image blockD)(attributes pick)(pose 0.9 1.50 0.0))
	( item (type Objects) (name blockE)(room livingroom)(zone storage)(image blockE)(attributes pick)(pose 0.9 1.40 0.0))
	( item (type Objects) (name blockF)(room livingoom)(zone storage)(image blockF)(attributes pick)(pose 0.9 1.60 0.0))
	( item (type Objects) (name freespace)(room any)(zone any)(image none)(attributes none)(pose 0.0 0.0 0.0))

; Rooms definitions
	( Room (name livingroom)(zone deposit)(zones dummy1 frontexit frontentrance storage dummy2)(center 0.60 1.40))
	( Room (name kitchen)(zone deposit)(zones dummy1 frontexit frontentrance deposit dummy2)(center 1.70 1.40))
	( Room (name bedroom)(zone deposit)(zones dummy1 frontexit frontentrance deposit dummy2)(center 0.30 0.60))
	( Room (name corridor)(zone deposit)(zones dummy1 frontexit frontentrance deposit dummy2)(center 0.9 1.00))
	( Room (name childsbedroom)(zone deposit)(zones dummy1 frontexit frontentrance deposit dummy2)(center 1.0 0.35))
	( Room (name storage)(zone deposit)(zones dummy1 frontexit frontentrance deposit dummy2)(center 1.6 0.60))

; Robots definitions
	( item (type Robot) (name robot)(zone frontexit)(pose 1.048340 1.107002 0.0))

; Furniture definitions
	( item (type Furniture) (name cubestable)(zone bedroom)(image table)( attributes no-pick brown)(pose 6.183334 7.000000 0.0))

; Doors definitions
	( item (type Door) (name outsidedoor) (status closed) )

	( Arm (name left))

;stacks definitions
        (stack bedroom deposit blockB blockA blockC)
        (stack livingroom storage blockE blockD blockF)

        (real-stack bedroom deposit blockB blockA blockC)
        (real-stack livingroom storage blockE blockD blockF)


	;(goal-stack 1 livingroom storage blockC blockF blockD blockA blockE blockB)


	(goal-stack 1 bedroom deposit blockD blockE blockF)
	(goal-stack 2 livingroom storage blockC blockA blockB)

	;(goal-stack 1 corridor deposit blockA blockE blockC)
	;(goal-stack 2 childsbedroom deposit blockD blockB blockF)
	;(goal-stack 3 kitchen deposit blockD blockA)

        (plan (name cubes) (number 0)(duration 0))

)



