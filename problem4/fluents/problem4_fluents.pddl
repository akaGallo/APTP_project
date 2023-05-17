(define (problem emergency_services_logistics_problem4_fluents)
    (:domain emergency_services_logistics_domain4_fluents)

    ; ----------------------------------- DEFINE OBJECTS -----------------------------------

    (:objects
        Trento Povo - location
        Chiara Matteo Marta - injured_person
        robot - robotic_agent
        helicopter - carrier
        box1 box2 box3 box4 - box
        apple pear - food
        morphine - medicine
        bandaid - tool
    )

    ; --------------------------------- DEFINE INITIAL STATE -------------------------------

    (:init
        ; THERE ARE NO INJURED PEOPLE AT THE DEPOT.
        (located_at Chiara Trento) (located_at Matteo Trento) (located_at Marta Povo)
        
        ; A SINGLE ROBOTIC AGENT IS LOCATED AT THE DEPOT TO DELIVER BOXES.
        (located_at robot depot)
        
        (located_at helicopter depot)

        ; INITIALLIY ALL BOXES ARE LOCATED AT A SINGLE LOCATION THAT WE MAY CALL THE DEPOT.
        (located_at box1 depot) (located_at box2 depot) (located_at box3 depot) (located_at box4 depot)

        ; ALL THE CONTENTS TO LOAD IN THE BOXES ARE INITIALLY LOCATED AT THE DEPOT.
        (located_at apple depot) (located_at pear depot)
        (located_at morphine depot)
        (located_at bandaid depot)

        (ready robot)
        
        ; THE ROBOTIC AGENT IS INITIALLY EMPTY.
        (= (carrier_capacity helicopter) 4)
        (= (max_capacity helicopter) 4)

        (is_empty box1) (is_empty box2) (is_empty box3) (is_empty box4)

        (need_food Chiara)
        (need_medicine Matteo)
        (need_food Marta) (need_tool Marta)
    )

    ; ------------------------------------- DEFINE GOAL -------------------------------------

    (:goal 
        (and
            ; CERTAIN PEOPLE HAVE CERTAIN CONTENTS (e.g., medicine, food, tool). 
            (has_food Chiara)
            ; SOME PEOPLE MIGHT NOT NEED FOOD, MEDICINE, OR TOOL.
            (has_medicine Matteo)
            ; SOME PEOPLE MIGHT NEED BOTH FOOD AND MEDICINE, OR FOOD AND TOOL, OR THREE OF THEM, AND SO ON.
            (has_food Marta) (has_tool Marta)

        )
    )

    ; ------------------------------------ DEFINE METRIC ------------------------------------

    (:metric minimize (total-time)) ; specify metric to minimize total time for plan completion
)