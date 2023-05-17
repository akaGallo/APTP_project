(define (domain emergency_services_logistics_domain5_plansys2)

    ; -------------------------------- DEFINE REQUIREMENTS --------------------------------

    (:requirements :strips :equality :typing :fluents :durative-actions)

    ; ----------------------------------- DEFINE TYPES ------------------------------------

    (:types
        movable - object                                      ; an object that can be moved and/or is located at a location
        area - object                                         ; an area in the world
        depot location - area                                 ; a depot location for all objects to start at, a location in the world
        injured_person robotic_agent - movable                ; a person who is injured, an autonomous robot that can perform tasks  
        carrier box content - movable                         ; a carrier for transporting objects, a box for containing contents, a content that can be transported in a box
        food medicine tool - content                          ; contents that can be delivered to injured people
    )

    ; -------------------------------- DEFINE PREDICATES ----------------------------------

    (:predicates
        (located_at ?m - movable ?l - area)        ; whether an object movable is at a particular area
	    (is_at ?c - content ?l - depot)                ; whether a content is at the depot
        (ready ?r - robotic_agent)                  ; whether a robotic agent is ready for performing a task
        (is_empty ?b - box)                            ; whether a box is empty
        (has_content ?b - box ?c - content)         ; whether a box has a certain type of content
        (loaded ?r - robotic_agent ?b - box)        ; whether a robot has a box loaded with content
        (has_food ?p - injured_person ?f - food)           ; whether an injured person has received a food 
        (has_medicine ?p - injured_person ?m - medicine)   ; whether an injured person has received a medicine
        (has_tool ?p - injured_person ?t - tool)           ; whether an injured person has received a tool
    )

    ; --------------------------------- DEFINE FUNCTIONS ----------------------------------

    (:functions
        (carrier_capacity ?a - carrier)             ; a function that defines the actual capacity of a carrier to carry boxes
        (max_capacity ?a - carrier)                       ; a function that defines the maximum capacity of a carrier to carry boxes
    )

    ; ------------------------------ DEFINE DURATIVE ACTIONS -------------------------------

    ; FILL A BOX WITH A CERTAIN TYPE OF CONTENT
    (:durative-action fill_box
        :parameters (?l - depot ?r - robotic_agent ?b - box ?c - content)
        :duration (= ?duration 3)
        :condition (and
            (at start (is_at ?c ?l))
            (at start (is_empty ?b))
            (at start (ready ?r))
            (over all (located_at ?r ?l))
            (over all (located_at ?b ?l))
        )
        :effect (and 
            (at start (not (is_at ?c ?l)))
            (at start (not (ready ?r)))
            (at end (ready ?r))
            (at end (not (is_empty ?b)))
            (at end (has_content ?b ?c))
        )
    )

    ; LOAD A FILLED BOX ONTO A ROBOT
    (:durative-action load_carrier
        :parameters (?l - depot ?r - robotic_agent ?a - carrier ?b - box ?c - content)
        :duration (= ?duration 5)
        :condition (and 
            (at start (located_at ?b ?l))
            (at start (> (carrier_capacity ?a) 0))
            (at start (ready ?r))
            (over all (located_at ?r ?l))
            (over all (located_at ?a ?l))
            (over all (has_content ?b ?c))
        )
        :effect (and
            (at start (not (located_at ?b ?l)))
            (at start (not (ready ?r)))
            (at end (ready ?r))
            (at end (loaded ?r ?b))
            (at end (decrease (carrier_capacity ?a) 1))
        )
    )

    ; MOVE A ROBOT FROM THE DEPOT/A LOCATION TO ANOTHER LOCATION CARRYING A FILLED BOX
    (:durative-action move_carrier_with_box
        :parameters (?from - area ?to - location ?r - robotic_agent ?a - carrier)
        :duration (= ?duration 10)
        :condition (and 
            (at start (located_at ?r ?from))
            (at start (located_at ?a ?from))
            (at start (ready ?r))
            (over all (< (carrier_capacity ?a) (max_capacity ?a)))
        )
        :effect (and 
            (at start (not (located_at ?r ?from)))
            (at start (not (located_at ?a ?from)))
            (at start (not (ready ?r)))
            (at end (ready ?r))
            (at end (located_at ?r ?to))
            (at end (located_at ?a ?to))
        )
    )

    ; UNLOAD A BOX FROM A CARRIER
    (:durative-action unload_carrier
        :parameters (?l - location ?r - robotic_agent ?a - carrier ?b - box)
        :duration (= ?duration 5)
        :condition (and 
            (at start (loaded ?r ?b))
            (at start (ready ?r))
            (over all (located_at ?r ?l))
            (over all (located_at ?a ?l))
        )
        :effect (and
            (at start (not (ready ?r)))
            (at start (not (loaded ?r ?b)))
            (at end (located_at ?b ?l))
            (at end (ready ?r))
            (at end (increase (carrier_capacity ?a) 1))
        )
    )

    ; DELIVER A FOOD TO AN INJURED PERSON
    (:durative-action deliver_food
        :parameters (?l - location ?r - robotic_agent ?p - injured_person ?b - box ?f - food)
        :duration (= ?duration 1)
        :condition (and
            (at start (has_content ?b ?f))
            (at start (ready ?r))
            (over all (located_at ?r ?l))
            (over all (located_at ?p ?l))
            (over all (located_at ?b ?l))
        )
        :effect (and
            (at start (not (has_content ?b ?f)))
            (at start (not (ready ?r)))
            (at end (ready ?r))
            (at end (has_food ?p ?f))
        )
    )

    ; DELIVER A MEDICINE TO AN INJURED PERSON
    (:durative-action deliver_medicine
        :parameters (?l - location ?r - robotic_agent ?p - injured_person ?b - box ?m - medicine)
        :duration (= ?duration 1)
        :condition (and
            (at start (has_content ?b ?m))
            (at start (ready ?r))
            (over all (located_at ?r ?l))
            (over all (located_at ?p ?l))
            (over all (located_at ?b ?l))
        )
        :effect (and
            (at start (not (has_content ?b ?m)))
            (at start (not (ready ?r)))
            (at end (ready ?r))
            (at end (has_medicine ?p ?m))
        )
    )

    ; DELIVER A TOOL TO AN INJURED PERSON
    (:durative-action deliver_tool
        :parameters (?l - location ?r - robotic_agent ?p - injured_person ?b - box ?t - tool)
        :duration (= ?duration 1)
        :condition (and
            (at start (has_content ?b ?t))
            (at start (ready ?r))
            (over all (located_at ?r ?l))
            (over all (located_at ?p ?l))
            (over all (located_at ?b ?l))
        )
        :effect (and
            (at start (not (has_content ?b ?t)))
            (at start (not (ready ?r)))
            (at end (ready ?r))
            (at end (has_tool ?p ?t))
        )
    )

    ; MOVE A ROBOT FROM A LOCATION TO THE DEPOT CARRYING AN EMPTY BOX
    (:durative-action move_carrier_without_box
        :parameters (?from - location ?to - depot ?r - robotic_agent ?a - carrier)
        :duration (= ?duration 6)
        :condition (and
            (at start (located_at ?r ?from))
            (at start (located_at ?a ?from))
            (at start (ready ?r))
            (over all (= (carrier_capacity ?a) (max_capacity ?a)))
        )
        :effect (and 
            (at start (not (located_at ?r ?from)))
            (at start (not (located_at ?a ?from)))
            (at start (not (ready ?r)))
            (at end (ready ?r))
            (at end (located_at ?r ?to))
            (at end (located_at ?a ?to))
        )
    )
)
