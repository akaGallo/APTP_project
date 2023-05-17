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
        compartment - object                                  ; a space within a carrier that can contain a box
        pos1 pos2 pos3 pos4 - compartment                     ; 4 spaces for containing boxes within a carrier
    )

    ; -------------------------------- DEFINE PREDICATES ----------------------------------

    (:predicates
        (located_at ?m - movable ?l - area)        ; whether an object movable is at a particular area
	    (is_at ?c - content ?l - area)              ; whether a content is at a particular location
        (ready ?r - robotic_agent)                  ; whether a robotic agent is ready for performing a task
        (is_empty ?b - box)                         ; whether a box is empty
        (has_content ?b - box ?c - content)         ; whether a box has a certain type of content
        (is_free ?s - compartment)                  ; whether a compartment is free (not occupied by a box)
        (is_occupied ?s - compartment ?b - box)     ; whether a compartment is occupied by box
        (has_food ?p - injured_person ?f - food)           ; whether an injured person has received a food 
        (has_medicine ?p - injured_person ?m - medicine)   ; whether an injured person has received a medicine
        (has_tool ?p - injured_person ?t - tool)           ; whether an injured person has received a tool
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
        :parameters (?l - depot ?r - robotic_agent ?a - carrier ?s - compartment ?b - box ?c - content)
        :duration (= ?duration 5)
        :condition (and 
            (at start (located_at ?b ?l))
            (at start (is_free ?s))
            (at start (ready ?r))
            (over all (located_at ?r ?l))
            (over all (located_at ?a ?l))
            (over all (has_content ?b ?c))
        )
        :effect (and
            (at start (not (located_at ?b ?l)))
            (at start (not (ready ?r)))
            (at end (not (is_free ?s)))
            (at end (is_occupied ?s ?b))
            (at end (ready ?r))
        )
    )

    ; MOVE A ROBOT FROM THE DEPOT/A LOCATION TO ANOTHER LOCATION CARRYING A FILLED BOX
    (:durative-action move_carrier_with_box
        :parameters (?from - area ?to - location ?r - robotic_agent ?a - carrier ?s - compartment ?b - box)
        :duration (= ?duration 10)
        :condition (and 
            (at start (located_at ?r ?from))
            (at start (located_at ?a ?from))
            (at start (ready ?r))
            (over all (is_occupied ?s ?b))
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
        :parameters (?l - location ?r - robotic_agent ?a - carrier ?s - compartment ?b - box)
        :duration (= ?duration 5)
        :condition (and 
            (at start (is_occupied ?s ?b))
            (at start (ready ?r))
            (over all (located_at ?r ?l))
            (over all (located_at ?a ?l))
        )
        :effect (and
            (at start (not (ready ?r)))
            (at start (not (is_occupied ?s ?b)))
            (at end (located_at ?b ?l))
            (at end (is_free ?s))
            (at end (ready ?r))
        )
    )

    ; UNFILL A CERTAIN TYPE OF CONTENT FROM A BOX
    (:durative-action unfill_box
        :parameters (?l - location ?r - robotic_agent ?b - box ?c - content)
        :duration (= ?duration 1)
        :condition (and 
            (at start (has_content ?b ?c))
            (at start (ready ?r))
            (over all (located_at ?r ?l))
            (over all (located_at ?b ?l))
        )
        :effect (and
            (at start (is_at ?c ?l))
            (at start (not (ready ?r)))
            (at end (not (has_content ?b ?c)))
            (at end (is_empty ?b))
            (at end (ready ?r))
        )
    )

    ; DELIVER A FOOD TO AN INJURED PERSON
    (:durative-action deliver_food
        :parameters (?l - location ?r - robotic_agent ?p - injured_person ?f - food)
        :duration (= ?duration 1)
        :condition (and
            (at start (ready ?r))
            (over all (located_at ?r ?l))
            (over all (located_at ?p ?l))
            (over all (is_at ?f ?l))
        )
        :effect (and
            (at start (not (ready ?r)))
            (at end (ready ?r))
            (at end (has_food ?p ?f))
        )
    )

    ; DELIVER A MEDICINE TO AN INJURED PERSON
    (:durative-action deliver_medicine
        :parameters (?l - location ?r - robotic_agent ?p - injured_person ?m - medicine)
        :duration (= ?duration 1)
        :condition (and
            (at start (ready ?r))
            (over all (located_at ?r ?l))
            (over all (located_at ?p ?l))
            (over all (is_at ?m ?l))
        )
        :effect (and
            (at start (not (ready ?r)))
            (at end (ready ?r))
            (at end (has_medicine ?p ?m))
        )
    )

    ; DELIVER A TOOL TO AN INJURED PERSON
    (:durative-action deliver_tool
        :parameters (?l - location ?r - robotic_agent ?p - injured_person ?t - tool)
        :duration (= ?duration 1)
        :condition (and
            (at start (ready ?r))
            (over all (located_at ?r ?l))
            (over all (located_at ?p ?l))
            (over all (is_at ?t ?l))
        )
        :effect (and
            (at start (not (ready ?r)))
            (at end (ready ?r))
            (at end (has_tool ?p ?t))
        )
    )

    ; MOVE A ROBOT FROM A LOCATION TO THE DEPOT CARRYING AN EMPTY BOX
    (:durative-action move_carrier_without_box
        :parameters (?from - location ?to - depot ?r - robotic_agent ?a - carrier ?p1 - pos1 ?p2 - pos2 ?p3 - pos3 ?p4 - pos4)
        :duration (= ?duration 6)
        :condition (and
            (at start (located_at ?r ?from))
            (at start (located_at ?a ?from))
            (at start (ready ?r))
            (over all (is_free ?p1))
            (over all (is_free ?p2))
            (over all (is_free ?p3))
            (over all (is_free ?p4))
        )
        :effect (and 
            (at start (not (ready ?r)))
            (at start (not (located_at ?r ?from)))
            (at start (not (located_at ?a ?from)))
            (at end (located_at ?r ?to))
            (at end (located_at ?a ?to))
            (at end (ready ?r))
        )
    )
)