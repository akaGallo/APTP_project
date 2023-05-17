(define (domain emergency_services_logistics_domain2_fluents)

    ; -------------------------------- DEFINE REQUIREMENTS --------------------------------

    (:requirements :strips :typing :equality :fluents :negative-preconditions)

    ; ----------------------------------- DEFINE TYPES ------------------------------------

    (:types
        location - object                                   ; a location in the world
        movable - object                                    ; an object that can be moved and/or is located at a location
        injured_person robotic_agent - movable              ; a person who is injured, an autonomous robot that can perform tasks
        carrier box content - movable                       ; a carrier for transporting objects, a box for containing contents, a content that can be transported in a box
        food medicine tool - content                        ; contents that can be delivered to injured people
    )

    ; --------------------------------- DEFINE CONSTANTS ----------------------------------

    (:constants
        depot - location                                    ; a constant depot location for all objects to start at
    )

    ; -------------------------------- DEFINE PREDICATES ----------------------------------

    (:predicates
        (located_at ?m - movable ?l - location)  ; whether an object movable is at a particular location
        (is_empty ?b - box)                       ; whether a box is empty
        (has_content ?b - box ?c - content)       ; whether a box has a certain type of content
        (loaded ?r - robotic_agent ?b - box)      ; whether a robot has a box loaded with content
        (need_food ?p - injured_person)              ; whether an injured person needs food
        (need_medicine ?p - injured_person)          ; whether an injured person needs medicine
        (need_tool ?p - injured_person)              ; whether an injured person needs a tool
    )

    ; --------------------------------- DEFINE FUNCTIONS ----------------------------------

    (:functions
        (carrier_capacity ?a - carrier)           ; a function that defines the capacity of a carrier to carry boxes
    )

    ; ---------------------------------- DEFINE ACTIONS ------------------------------------

    ; FILL A BOX WITH A CERTAIN TYPE OF CONTENT
    (:action fill_box
        :parameters (?r - robotic_agent ?b - box ?c - content)
        :precondition (and
            (located_at ?r depot)
            (located_at ?b depot)
            (located_at ?c depot)
            (is_empty ?b)
        )
        :effect (and
            (not (located_at ?c depot))
            (not (is_empty ?b))
            (has_content ?b ?c)
        )
    )

    ; LOAD A FILLED BOX ONTO A ROBOT
    (:action load_carrier
        :parameters (?r - robotic_agent ?a - carrier ?b - box)
        :precondition (and
            (located_at ?r depot)
            (located_at ?a depot)
            (located_at ?b depot)
            (not (is_empty ?b))
            (not (loaded ?r ?b))
            (> (carrier_capacity ?a) 0) ; THE ROBOTIC AGENT CAN LOAD UP TO 4 BOXES ONTO A CARRIER, WHICH ALL MUST BE AT THE SAME LOCATION.
        )
        :effect (and
            (not (located_at ?b depot))
            ; FOR EACH ROBOTIC AGENT WE NEED TO COUNT AND KEEP TRACK OF:
            (loaded ?r ?b)                      ; i) WHICH BOXES ARE ON EACH CARRIER;
            (decrease (carrier_capacity ?a) 1)  ; ii) HOW MANY BOXES THERE ARE IN TOTAL ON EACH CARRIER, SO THAT CARRIERS CANNOT BE OVERLOADED.
        )
    )

    ; MOVE A ROBOT FROM THE DEPOT/A LOCATION TO ANOTHER LOCATION CARRYING A FILLED BOX
    (:action move_carrier_with_box
        :parameters (?from ?to - location ?r - robotic_agent ?a - carrier)
        :precondition (and
            (not (= ?to depot)) ; THE ROBOTIC AGENT CAN MOVE THE CARRIER TO A LOCATION WHERE THERE ARE PEOPLE NEEDING SUPPLIES.
            (not (= ?from ?to))
            (located_at ?r ?from)
            (located_at ?a ?from)
            (exists (?b - box) (loaded ?r ?b)) ; THE ROBOTIC AGENT MAY CONTINUE MOVING THE CARRIER TO ANOTHER LOCATION, UNLOADING ADDITIONAL BOXES
        )
        :effect (and
            (not (located_at ?r ?from))
            (located_at ?r ?to)
            (not (located_at ?a ?from))
            (located_at ?a ?to)
        )
    )

    ; UNLOAD A BOX FROM A CARRIER
    (:action unload_carrier
        :parameters (?l - location ?r - robotic_agent ?a - carrier ?b - box)
        :precondition (and
            (located_at ?r ?l)
            (located_at ?a ?l)
            (loaded ?r ?b)
        )
        :effect (and
            (not (loaded ?r ?b))
            (located_at ?b ?l)
            (increase (carrier_capacity ?a) 1) ; THE ROBOTIC AGENT CAN UNLOAD ONE OR MORE BOX FROM THE CARRIER TO A LOCATION WHERE IT IS.
        )
    )
    
    ; DELIVER A FOOD TO AN INJURED PERSON
    (:action deliver_food
        :parameters (?l - location ?r - robotic_agent ?p - injured_person ?b - box ?f - food)
        :precondition (and
            (located_at ?p ?l)
            (located_at ?r ?l)
            (located_at ?b ?l)
            (has_content ?b ?f)
            (need_food ?p)
        )
        :effect (and
            (not (has_content ?b ?f))
            (is_empty ?b)
            (located_at ?f ?l)
            (not (need_food ?p))
        )
    )
    
    ; DELIVER A MEDICINE TO AN INJURED PERSON
    (:action deliver_medicine
        :parameters (?l - location ?r - robotic_agent ?p - injured_person ?b - box ?m - medicine)
        :precondition (and
            (located_at ?p ?l)
            (located_at ?r ?l)
            (located_at ?b ?l)
            (has_content ?b ?m)
            (need_medicine ?p)
        )
        :effect (and
            (not (has_content ?b ?m))
            (is_empty ?b)
            (located_at ?m ?l)
            (not (need_medicine ?p))
        )
    )

    ; DELIVER A TOOL TO AN INJURED PERSON
    (:action deliver_tool
        :parameters (?l - location ?r - robotic_agent ?p - injured_person ?b - box ?t - tool)
        :precondition (and
            (located_at ?p ?l)
            (located_at ?r ?l)
            (located_at ?b ?l)
            (has_content ?b ?t)
            (need_tool ?p)
        )
        :effect (and
            (not (has_content ?b ?t))
            (is_empty ?b)
            (located_at ?t ?l)
            (not (need_tool ?p))
        )
    )

    ; MOVE A ROBOT FROM A LOCATION TO THE DEPOT CARRYING AN EMPTY BOX
    (:action move_carrier_without_box
        :parameters (?from - location ?r - robotic_agent ?a - carrier)
        :precondition (and
            (not (= ?from depot))
            (located_at ?r ?from)
            (located_at ?a ?from)
            (forall (?b - box) (not (loaded ?r ?b))) ; THE ROBOTIC AGENT DOES NOT HAVE TO RETURN TO DEPOT UNTILL AFTER ALL BOXES ON THE CARRIER HAVE BEEN DELIVERED.
        )
        :effect (and
            (not (located_at ?r ?from))
            (located_at ?r depot)
            (not (located_at ?a ?from))
            (located_at ?a depot)
        )
    )
)