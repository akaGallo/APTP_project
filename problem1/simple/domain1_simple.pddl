(define (domain emergency_services_logistics_domain1_simple)

    ; -------------------------------- DEFINE REQUIREMENTS --------------------------------

    (:requirements :strips :typing :equality :negative-preconditions)

    ; ----------------------------------- DEFINE TYPES ------------------------------------
 
    (:types
        location - object                                   ; a location in the world
        movable - object                                    ; an object that can be moved and/or is located at a location
        injured_person robotic_agent - movable              ; a person who is injured, an autonomous robot that can perform tasks
        box content - movable                               ; a box for containing contents, a content that can be transported in a box
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
        (is_loaded ?r - robotic_agent ?b - box)   ; whether a robot has a box loaded with content
        (is_unloaded ?r - robotic_agent)          ; whether a robot is not currently loaded with a box
        (need_food ?p - injured_person)              ; whether an injured person needs food
        (need_medicine ?p - injured_person)          ; whether an injured person needs medicine
        (need_tool ?p - injured_person)              ; whether an injured person needs a tool
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
            (not (is_loaded ?r ?b))
        )
        :effect (and
            (not (located_at ?c depot))
            (not (is_empty ?b))
            (has_content ?b ?c)
        )
    )
    
    ; LOAD A FILLED BOX ONTO A ROBOT
    (:action load_robot
        :parameters (?r - robotic_agent ?b - box)
        :precondition (and  
            (located_at ?r depot)
            (located_at ?b depot)
            (is_unloaded ?r)
            (not (is_empty ?b))
        )
        :effect (and
            (not (located_at ?b depot))
            (not (is_unloaded ?r))
            (is_loaded ?r ?b)
        )
    )
    
    ; MOVE A ROBOT FROM DEPOT TO ANOTHER LOCATION CARRYING A FILLED BOX
    (:action move_robot_with_content
        :parameters (?to - location ?r - robotic_agent ?b - box ?c - content)
        :precondition (and  
            (not (= ?to depot))
            (located_at ?r depot)
            (is_loaded ?r ?b)
            (has_content ?b ?c)
        )
        :effect (and 
            (not (located_at ?r depot))
            (located_at ?r ?to)
            (located_at ?b ?to)
            (located_at ?c ?to)
        )
    )
    
    ; DELIVER A FOOD TO AN INJURED PERSON
    (:action deliver_food
        :parameters (?l - location ?p - injured_person ?r - robotic_agent ?b - box ?f - food)
        :precondition (and  
            (located_at ?p ?l)
            (located_at ?r ?l)
            (is_loaded ?r ?b)
            (has_content ?b ?f)
            (need_food ?p)
        )
        :effect (and
            (not (has_content ?b ?f))
            (is_empty ?b)
            (not (need_food ?p))
        )
    )

    ; DELIVER A MEDICINE TO AN INJURED PERSON
    (:action deliver_medicine
        :parameters (?l - location ?p - injured_person ?r - robotic_agent ?b - box ?m - medicine)
        :precondition (and  
            (located_at ?p ?l)
            (located_at ?r ?l)
            (is_loaded ?r ?b)
            (has_content ?b ?m)
            (need_medicine ?p)
        )
        :effect (and
            (not (has_content ?b ?m))
            (is_empty ?b)
            (not (need_medicine ?p))
        )
    )

    ; DELIVER A TOOL TO AN INJURED PERSON
    (:action deliver_tool
        :parameters (?l - location ?p - injured_person ?r - robotic_agent ?b - box ?t - tool)
        :precondition (and  
            (located_at ?p ?l)
            (located_at ?r ?l)
            (is_loaded ?r ?b)
            (has_content ?b ?t)
            (need_tool ?p)
        )
        :effect (and
            (not (has_content ?b ?t))
            (is_empty ?b)
            (not (need_tool ?p))
        )
    )

    ; MOVE A ROBOT FROM A LOCATION TO THE DEPOT CARRYING AN EMPTY BOX
    (:action move_robot_without_content
        :parameters (?from - location ?r - robotic_agent ?b - box)
        :precondition (and  
            (not (= ?from depot))
            (located_at ?r ?from)
            (is_loaded ?r ?b)
            (is_empty ?b)
        )
        :effect (and 
            (not (located_at ?r ?from))
            (located_at ?r depot)
            (not (located_at ?b ?from))
            (located_at ?b depot)
        )
    )
    
    ; UNLOAD AN EMPTY BOX FROM THE ROBOT
    (:action unload_robot
        :parameters (?r - robotic_agent ?b - box)
        :precondition (and  
            (located_at ?r depot)
            (located_at ?b depot)
            (is_loaded ?r ?b)
            (is_empty ?b)
        )
        :effect (and 
            (not (is_loaded ?r ?b))
            (is_unloaded ?r)
        )
    )
)
