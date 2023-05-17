(define (domain emergency_services_logistics_domain4_simple)

    ; -------------------------------- DEFINE REQUIREMENTS --------------------------------

    (:requirements :typing :equality :durative-actions :duration-inequalities)

    ; ----------------------------------- DEFINE TYPES ------------------------------------

    (:types
        location - object                                     ; a location in the world
        movable - object                                      ; an object that can be moved and/or is located at a location
        injured_person robotic_agent - movable                ; a person who is injured, an autonomous robot that can perform tasks
        carrier box content - movable                         ; a carrier for transporting objects, a box for containing contents, a content that can be transported in a box
        food medicine tool - content                          ; contents that can be delivered to injured people
        compartment - object                                  ; a space within a carrier that can contain a box
    )

    ; --------------------------------- DEFINE CONSTANTS ----------------------------------

    (:constants
        depot - location                                      ; a constant depot location for all objects to start at
        pos1 pos2 pos3 pos4 - compartment                     ; 4 constant spaces for containing boxes within a carrier
    )

    ; -------------------------------- DEFINE PREDICATES ----------------------------------

    (:predicates
        (located_at ?m - movable ?l - location)    ; whether an object movable is at a particular location
        (is_empty ?b - box)                         ; whether a box is empty
        (has_content ?b - box ?c - content)         ; whether a box has a certain type of content
        (is_free ?s - compartment)                  ; whether a compartment is free (not occupied by a box)
        (is_occupied ?s - compartment ?b - box)     ; whether a compartment is occupied by box
        (need_food ?p - injured_person)                ; whether an injured person needs food
        (need_medicine ?p - injured_person)            ; whether an injured person needs medicine
        (need_tool ?p - injured_person)                ; whether an injured person needs a tool
        (has_food ?p - injured_person)                     ; whether an injured person has received a food
        (has_medicine ?p - injured_person)                 ; whether an injured person has received a medicine
        (has_tool ?p - injured_person)                     ; whether an injured person has received a tool
        (ready ?r - robotic_agent)                  ; whether a robotic agent is ready for performing a task
    )

    ; ------------------------------ DEFINE DURATIVE ACTIONS -------------------------------

    ; FILL A BOX WITH A CERTAIN TYPE OF CONTENT
    (:durative-action fill_box
        :parameters (?r - robotic_agent ?b - box ?c - content)
        :duration (= ?duration 1)
        :condition (and 
            (over all (located_at ?r depot))
            (over all (located_at ?b depot))
            (at start (ready ?r))
            (at start (located_at ?c depot))
            (at start (is_empty ?b))
        )
        :effect (and 
            (at start (not (located_at ?c depot)))
            (at start (not (ready ?r)))
            (at end (not (is_empty ?b)))
            (at end (has_content ?b ?c))
            (at end (ready ?r))
        )
    )

    ; LOAD A FILLED BOX ONTO A ROBOT
    (:durative-action load_carrier
        :parameters (?r - robotic_agent ?a - carrier ?s - compartment ?b - box ?c - content)
        :duration (and (>= ?duration 2) (<= ?duration 5))
        :condition (and 
            (over all (located_at ?r depot))
            (over all (located_at ?a depot))
            (over all (has_content ?b ?c))
            (at start (located_at ?b depot))
            (at start (is_free ?s))
            (at start (ready ?r))
        )
        :effect (and
            (at start (not (located_at ?b depot)))
            (at start (not (ready ?r)))
            (at end (not (is_free ?s)))
            (at end (is_occupied ?s ?b))
            (at end (ready ?r))
        )
    )    

    ; MOVE A ROBOT FROM THE DEPOT/A LOCATION TO ANOTHER LOCATION CARRYING A FILLED BOX
    (:durative-action move_carrier_with_box
        :parameters (?from ?to - location ?r - robotic_agent ?a - carrier ?s - compartment ?b - box)
        :duration (and (>= ?duration 7) (<= ?duration 12))
        :condition (and 
            (over all (is_occupied ?s ?b))
            (at start (located_at ?r ?from))
            (at start (located_at ?a ?from))
            (at start (ready ?r))
        )
        :effect (and 
            (at start (not (located_at ?r ?from)))
            (at start (not (located_at ?a ?from)))
            (at start (not (ready ?r)))
            (at end (located_at ?r ?to))
            (at end (located_at ?a ?to))
            (at end (ready ?r))
        )
    )

    ; UNLOAD A BOX FROM A CARRIER
    (:durative-action unload_carrier
        :parameters (?l - location ?r - robotic_agent ?a - carrier ?s - compartment ?b - box)
        :duration (and (>= ?duration 2) (<= ?duration 5))
        :condition (and 
            (over all (located_at ?r ?l))
            (over all (located_at ?a ?l))
            (at start (is_occupied ?s ?b))
            (at start (ready ?r))
        )
        :effect (and
            (at start (located_at ?b ?l))
            (at start (not (ready ?r)))
            (at end (not (is_occupied ?s ?b)))
            (at end (is_free ?s))
            (at end (ready ?r))
        )
    )

    ; UNFILL A CERTAIN TYPE OF CONTENT FROM A BOX
    (:durative-action unfill_box
        :parameters (?l - location ?r - robotic_agent ?b - box ?c - content)
        :duration (= ?duration 1)
        :condition (and 
            (over all (located_at ?r ?l))
            (over all (located_at ?b ?l))
            (at start (has_content ?b ?c))
            (at start (ready ?r))
        )
        :effect (and
            (at start (located_at ?c ?l))
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
            (over all (located_at ?r ?l))
            (over all (located_at ?p ?l))
            (over all (located_at ?f ?l))
            (at start (need_food ?p))
            (at start (ready ?r))
        )
        :effect (and
            (at start (not (ready ?r)))
            (at start (not (need_food ?p)))
            (at end (has_food ?p))
            (at end (ready ?r))
        )
    )

    ; DELIVER A MEDICINE TO AN INJURED PERSON
    (:durative-action deliver_medicine
        :parameters (?l - location ?r - robotic_agent ?p - injured_person ?m - medicine)
        :duration (= ?duration 1)
        :condition (and
            (over all (located_at ?r ?l))
            (over all (located_at ?p ?l))
            (over all (located_at ?m ?l))
            (at start (need_medicine ?p))
            (at start (ready ?r))
        )
        :effect (and
            (at start (not (ready ?r)))
            (at start (not (need_medicine ?p)))
            (at end (has_medicine ?p))
            (at end (ready ?r))
        )
    )

    ; DELIVER A TOOL TO AN INJURED PERSON
    (:durative-action deliver_tool
        :parameters (?l - location ?r - robotic_agent ?p - injured_person ?t - tool)
        :duration (= ?duration 1)
        :condition (and
            (over all (located_at ?r ?l))
            (over all (located_at ?p ?l))
            (over all (located_at ?t ?l))
            (at start (need_tool ?p))
            (at start (ready ?r))
        )
        :effect (and
            (at start (not (ready ?r)))
            (at start (not (need_tool ?p)))
            (at end (has_tool ?p))
            (at end (ready ?r))
        )
    )

    ; MOVE A ROBOT FROM A LOCATION TO THE DEPOT CARRYING AN EMPTY BOX
    (:durative-action move_carrier_without_box
        :parameters (?from - location ?r - robotic_agent ?a - carrier)
        :duration (and (>= ?duration 5) (<= ?duration 10))
        :condition (and
            (over all (is_free pos1))
            (over all (is_free pos2))
            (over all (is_free pos3))
            (over all (is_free pos4))
            (at start (located_at ?r ?from))
            (at start (located_at ?a ?from))
            (at start (ready ?r))
        )
        :effect (and
            (at start (not (ready ?r)))
            (at start (not (located_at ?r ?from)))
            (at start (not (located_at ?a ?from)))
            (at end (located_at ?r depot))
            (at end (located_at ?a depot))
            (at end (ready ?r))
        )
    )
)