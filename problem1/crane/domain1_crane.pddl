(define (domain emergency_services_logistics_domain1_crane)

    ; -------------------------------- DEFINE REQUIREMENTS --------------------------------

    (:requirements :strips :typing :equality :negative-preconditions)

    ; ----------------------------------- DEFINE TYPES ------------------------------------
    
    (:types
        location - object                                    ; a location in the world
        movable - object                                     ; an object that can be moved and/or is located at a location
        injured_person robotic_agent - movable               ; a person who is injured, an autonomous robot that can perform tasks
        box content - movable                                ; a box for containing contents, a content that can be transported in a box
        food medicine tool - content                         ; contents that can be delivered to injured people
        crane - object                                       ; a crane located at depot that can pick up/down and move boxes and contents
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
        (is_loaded ?r - robotic_agent ?b - box)    ; whether a robot has a box loaded with content
        (is_unloaded ?r - robotic_agent)           ; whether a robot is not currently loaded with a box
        (holding_box ?g - crane ?b - box)          ; whether a crane is holding a box
        (holding_content ?g - crane ?c - content)  ; whether a crane is holding a content
        (is_free ?g - crane)                       ; whether a crane is not holding anything
        (is_delivered ?c - content)                   ; whether a content has been delivered to an injured person
        (need_food ?p - injured_person)               ; whether an injured person needs food
        (need_medicine ?p - injured_person)           ; whether an injured person needs medicine
        (need_tool ?p - injured_person)               ; whether an injured person needs a tool
    )

    ; ---------------------------------- DEFINE ACTIONS ------------------------------------

    ; A CRANE PICKS UP A CONTENT TO BE FILLED INTO A BOX
    (:action crane_pick_up_content
        :parameters (?g - crane ?c - content)
        :precondition (and  
            (located_at ?c depot)
            (exists (?b - box) (and (located_at ?b depot) (is_empty ?b)))
            (is_free ?g)
        )
        :effect (and
            (not (located_at ?c depot))
            (not (is_free ?g))
            (holding_content ?g ?c)
        )
    )

    ; A CRANE FILLS A CONTENT INTO A BOX
    (:action crane_fill_box
        :parameters (?g - crane ?b - box ?c - content)
        :precondition (and  
            (located_at ?b depot)
            (holding_content ?g ?c)
            (is_empty ?b)
            (not (exists (?r - robotic_agent) (is_loaded ?r ?b)))
        )
        :effect (and
            (not (holding_content ?g ?c))
            (is_free ?g)
            (not (is_empty ?b))
            (has_content ?b ?c)
        )
    )

    ; A CRANE PICKS UP A FILLED BOX TO BE LOADED ONTO A ROBOT
    (:action crane_pick_up_box
        :parameters (?g - crane ?b - box)
        :precondition (and
            (located_at ?b depot)
            (exists (?r - robotic_agent) (and (located_at ?r depot) (is_unloaded ?r)))
            (not (is_empty ?b))
            (is_free ?g)
        )
        :effect (and
            (not (located_at ?b depot))
            (not (is_free ?g))
            (holding_box ?g ?b)
        )
    )
    
    ; A CRANE LOADS A FILLED BOX ONTO A ROBOT
    (:action crane_load_robot
        :parameters (?g - crane ?r - robotic_agent ?b - box)
        :precondition (and  
            (located_at ?r depot)
            (is_unloaded ?r)
            (holding_box ?g ?b)
        )
        :effect (and 
            (not (is_unloaded ?r))
            (is_loaded ?r ?b)
            (not (holding_box ?g ?b))
            (is_free ?g)
        )
    )

    ; A ROBOT MOVES FROM THE DEPOT TO ANOTHER LOCATION CARRYING A FILLED BOX
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

    ; A ROBOT UNFILLS A CONTENT FROM A BOX
    (:action unfill_box
        :parameters (?l - location ?r - robotic_agent ?b - box ?c - content)
        :precondition (and 
            (located_at ?r ?l)
            (is_loaded ?r ?b)
            (has_content ?b ?c)
        )
        :effect (and 
            (not (has_content ?b ?c))
            (is_empty ?b)
        )
    )

    ; A ROBOT DELIVERS A FOOD TO AN INJURED PERSON
    (:action deliver_food
        :parameters (?l - location ?r - robotic_agent ?p - injured_person ?f - food)
        :precondition (and
            (located_at ?p ?l)
            (located_at ?r ?l)
            (located_at ?f ?l)
            (exists (?b - box) (and (is_loaded ?r ?b) (is_empty ?b)))
            (not (is_delivered ?f))
            (need_food ?p)
        )
        :effect (and
            (is_delivered ?f)
            (not (need_food ?p))
        )
    )

    ; A ROBOT DELIVERS A MEDICINE TO AN INJURED PERSON
    (:action deliver_medicine
        :parameters (?l - location ?r - robotic_agent ?p - injured_person ?m - medicine)
        :precondition (and
            (located_at ?p ?l)
            (located_at ?r ?l)
            (located_at ?m ?l)
            (exists (?b - box) (and (is_loaded ?r ?b) (is_empty ?b)))
            (not (is_delivered ?m))
            (need_medicine ?p)
        )
        :effect (and
            (is_delivered ?m)
            (not (need_medicine ?p))
        )
    )

    ; A ROBOT DELIVERS A TOOL TO AN INJURED PERSON
    (:action deliver_tool
        :parameters (?l - location ?r - robotic_agent ?p - injured_person ?t - tool)
        :precondition (and
            (located_at ?p ?l)
            (located_at ?r ?l)
            (located_at ?t ?l)
            (exists (?b - box) (and (is_loaded ?r ?b) (is_empty ?b)))
            (not (is_delivered ?t))
            (need_tool ?p)
        )
        :effect (and
            (is_delivered ?t)
            (not (need_tool ?p))
        )
    )

    ; A ROBOT MOVES FROM A LOCATION TO THE DEPOT CARRYING AN EMPTY BOX
    (:action move_robot_without_content
        :parameters (?from - location ?r - robotic_agent ?b - box)
        :precondition (and
            (not (= ?from depot))
            (located_at ?r ?from)
            (is_loaded ?r ?b)
            (is_empty ?b)
            (not (exists (?c - content) (and (located_at ?c ?from) (not (is_delivered ?c)))))
        )
        :effect (and
            (not (located_at ?r ?from))
            (located_at ?r depot)
            (not (located_at ?b ?from))
            (located_at ?b depot)
        )
    )
    
    ; A CRANE UNLOADS A BOX FROM A ROBOT
    (:action crane_unload_robot
        :parameters (?g - crane ?r - robotic_agent ?b - box)
        :precondition (and
            (located_at ?r depot)
            (is_loaded ?r ?b)
            (is_empty ?b)
            (is_free ?g)
        )
        :effect (and
            (not (is_loaded ?r ?b))
            (is_unloaded ?r)
            (not (is_free ?g))
            (holding_box ?g ?b)
        )
    )

    ; A CRANE PUTS DOWN A BOX TO BE FILLED AGAIN
    (:action crane_put_down_box
        :parameters (?g - crane ?b - box)
        :precondition (and
            (located_at ?b depot)
            (is_empty ?b)
            (holding_box ?g ?b)
        )
        :effect (and
            (not (holding_box ?g ?b))
            (is_free ?g)
        )
    )
)
