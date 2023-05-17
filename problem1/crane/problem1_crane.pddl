(define (problem emergency_services_logistics_problem1_crane)
    (:domain emergency_services_logistics_domain1_crane)

    ; ----------------------------------- DEFINE OBJECTS -----------------------------------
    
    (:objects
        Trento Povo - location
        Chiara Matteo Marta - injured_person
        robot - robotic_agent
        crane - crane
        box1 box2 - box
        apple pear - food
        morphine paracetamol - medicine
        bandaid gauze - tool
    )

    ; -------------------------------- DEFINE INITIAL STATE --------------------------------

    (:init
        ; THERE ARE NO INJURED PEOPLE AT THE DEPOT.
        (located_at Chiara Trento) (located_at Matteo Trento) (located_at Marta Povo)
        
        ; A SINGLE ROBOTIC AGENT IS LOCATED AT THE DEPOT TO DELIVER BOXES.
        (located_at robot depot) 
        
        ; INITIALLIY ALL BOXES ARE LOCATED AT A SINGLE LOCATION THAT WE MAY CALL THE DEPOT.
        (located_at box1 depot) (located_at box2 depot)

        ; ALL THE CONTENTS TO LOAD IN THE BOXES ARE INITIALLY LOCATED AT THE DEPOT.
        (located_at apple depot) (located_at pear depot)
        (located_at morphine depot) (located_at paracetamol depot)
        (located_at bandaid depot) (located_at gauze depot)

        (is_unloaded robot) (is_free crane)

        (is_empty box1) (is_empty box2)

        (need_medicine Chiara)
        (need_food Matteo) (need_tool Matteo)
        (need_food Marta) (need_medicine Marta) (need_tool Marta)
    )

    ; ------------------------------------ DEFINE GOAL --------------------------------------

    (:goal 
        (and
            ; CERTAIN PEOPLE HAVE CERTAIN CONTENTS (e.g., medicine, food, tool). 
            (not (need_medicine Chiara))
            ; SOME PEOPLE MIGHT NOT NEED FOOD, MEDICINE, OR TOOL.
            (not (need_food Matteo)) (not (need_tool Matteo))
            ; SOME PEOPLE MIGHT NEED BOTH FOOD AND MEDICINE, OR FOOD AND TOOL, OR THREE OF THEM, AND SO ON.
            (not (need_food Marta)) (not (need_medicine Marta)) (not (need_tool Marta))
        )
    )
)