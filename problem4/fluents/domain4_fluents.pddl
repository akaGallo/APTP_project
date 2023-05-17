(define (domain emergency_services_logistics_domain4_fluents)

    ; -------------------------------- DEFINE REQUIREMENTS --------------------------------

    (:requirements :typing :fluents :durative-actions :duration-inequalities)

    ; ----------------------------------- DEFINE TYPES ------------------------------------

    (:types
        location - object                                    ; a location in the world
        movable - object                                     ; an object that can be moved and/or is located at a location
        injured_person robotic_agent - movable               ; a person who is injured, an autonomous robot that can perform tasks
        carrier box content - movable                        ; a carrier for transporting objects, a box for containing contents, a content that can be transported in a box
        food medicine tool - content                         ; contents that can be delivered to injured people
    )

    ; --------------------------------- DEFINE CONSTANTS ----------------------------------

    (:constants
        depot - location                                     ; a constant depot location for all objects to start at
    )

    ; -------------------------------- DEFINE PREDICATES ----------------------------------

    (:predicates
        (located_at ?m - movable ?l - location)   ; whether an object movable is at a particular location
        (is_empty ?b - box)                        ; whether a box is empty
        (has_content ?b - box ?c - content)        ; whether a box has a certain type of content
        (loaded ?r - robotic_agent ?b - box)       ; whether a robot has a box loaded with content
        (need_food ?p - injured_person)               ; whether an injured person needs food
        (need_medicine ?p - injured_person)           ; whether an injured person needs medicine
        (need_tool ?p - injured_person)               ; whether an injured person needs a tool
        (has_food ?p - injured_person)                    ; whether an injured person has received a food
        (has_medicine ?p - injured_person)                ; whether an injured person has received a medicine
        (has_tool ?p - injured_person)                    ; whether an injured person has received a tool
        (ready ?r - robotic_agent)                 ; whether a robotic agent is ready for performing a task
    )

    ; --------------------------------- DEFINE FUNCTIONS ----------------------------------

    (:functions
        (carrier_capacity ?a - carrier)            ; a function that defines the actual capacity of a carrier to carry boxes
        (max_capacity ?a - carrier)                      ; a function that defines the maximum capacity of a carrier to carry boxes
    )

    ; ------------------------------ DEFINE DURATIVE ACTIONS ------------------------------

    ; FILL A BOX WITH A CERTAIN TYPE OF CONTENT
    (:durative-action fill_box
        :parameters (?r - robotic_agent ?b - box ?c - content)
        :duration (= ?duration 1)
        :condition (and 
            (over all (located_at ?r depot))
            (over all (located_at ?b depot))
            (at start (located_at ?c depot))
            (at start (is_empty ?b))
            (at start (ready ?r))
        )
        :effect (and 
            (at start (not (located_at ?c depot)))
            (at start (not (is_empty ?b)))
            (at start (not (ready ?r)))
            (at end (ready ?r))
            (at end (has_content ?b ?c))
        )
    )

    ; LOAD A FILLED BOX ONTO A ROBOT
    (:durative-action load_carrier
        :parameters (?r - robotic_agent ?a - carrier ?b - box ?c - content)
        :duration (and (>= ?duration 2) (<= ?duration 5))
        :condition (and 
            (at start (located_at ?b depot))
            (at start (> (carrier_capacity ?a) 0))
            (at start (ready ?r))
            (over all (located_at ?r depot))
            (over all (located_at ?a depot))
            (over all (has_content ?b ?c))
        )
        :effect (and
            (at start (not (located_at ?b depot)))
            (at start (not (ready ?r)))
            (at end (ready ?r))
            (at end (loaded ?r ?b))
            (at end (decrease (carrier_capacity ?a) 1))
        )
    )

    ; MOVE A ROBOT FROM THE DEPOT/A LOCATION TO ANOTHER LOCATION CARRYING A FILLED BOX
    (:durative-action move_carrier_with_box
        :parameters (?from ?to - location ?r - robotic_agent ?a - carrier)
        :duration (and (>= ?duration 7) (<= ?duration 12))
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
        :duration (and (>= ?duration 2) (<= ?duration 5))
        :condition (and 
            (at start (loaded ?r ?b))
            (at start (ready ?r))
            (over all (located_at ?r ?l))
            (over all (located_at ?a ?l))
        )
        :effect (and
            (at start (located_at ?b ?l))
            (at start (not (ready ?r)))
            (at end (ready ?r))
            (at end (not (loaded ?r ?b)))
            (at end (increase (carrier_capacity ?a) 1))
        )
    )

    ; DELIVER A FOOD TO AN INJURED PERSON
    (:durative-action deliver_food
        :parameters (?l - location ?r - robotic_agent ?p - injured_person ?b - box ?f - food)
        :duration (= ?duration 1)
        :condition (and
            (at start (has_content ?b ?f))
            (at start (need_food ?p))
            (at start (ready ?r))
            (over all (located_at ?r ?l))
            (over all (located_at ?p ?l))
            (over all (located_at ?b ?l))
        )
        :effect (and
            (at start (not (has_content ?b ?f)))
            (at start (is_empty ?b))
            (at start (not (ready ?r)))
            (at end (ready ?r))
            (at end (located_at ?f ?l))
            (at end (not (need_food ?p)))
            (at end (has_food ?p))
        )
    )

    ; DELIVER A MEDICINE TO AN INJURED PERSON
    (:durative-action deliver_medicine
        :parameters (?l - location ?r - robotic_agent ?p - injured_person ?b - box ?m - medicine)
        :duration (= ?duration 1)
        :condition (and
            (at start (has_content ?b ?m))
            (at start (need_medicine ?p))
            (at start (ready ?r))
            (over all (located_at ?r ?l))
            (over all (located_at ?p ?l))
            (over all (located_at ?b ?l))
        )
        :effect (and
            (at start (not (has_content ?b ?m)))
            (at start (is_empty ?b))
            (at start (not (ready ?r)))
            (at end (ready ?r))
            (at end (located_at ?m ?l))
            (at end (not (need_medicine ?p)))
            (at end (has_medicine ?p))
        )
    )

    ; DELIVER A TOOL TO AN INJURED PERSON
    (:durative-action deliver_tool
        :parameters (?l - location ?r - robotic_agent ?p - injured_person ?b - box ?t - tool)
        :duration (= ?duration 1)
        :condition (and
            (at start (has_content ?b ?t))
            (at start (need_tool ?p))
            (at start (ready ?r))
            (over all (located_at ?r ?l))
            (over all (located_at ?p ?l))
            (over all (located_at ?b ?l))
        )
        :effect (and
            (at start (not (has_content ?b ?t)))
            (at start (is_empty ?b))
            (at start (not (ready ?r)))
            (at end (ready ?r))
            (at end (located_at ?t ?l))
            (at end (not (need_tool ?p)))
            (at end (has_tool ?p))
        )
    )

    ; MOVE A ROBOT FROM A LOCATION TO THE DEPOT CARRYING AN EMPTY BOX
    (:durative-action move_carrier_without_box
        :parameters (?from - location ?r - robotic_agent ?a - carrier)
        :duration (and (>= ?duration 5) (<= ?duration 10))
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
            (at end (located_at ?r depot))
            (at end (located_at ?a depot))
        )
    )
)