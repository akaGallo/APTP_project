(define (problem emergency_services_logistics_problem2_fluents)
    (:domain emergency_services_logistics_domain2_fluents)

    ; ----------------------------------- DEFINE OBJECTS -----------------------------------

    (:objects
        Trento Povo - location
        Chiara Matteo Marta - injured_person
        robot - robotic_agent
        helicopter - carrier ; THOUGH A SINGLE CARRIER IS NEEDED FOR THE SINGLE ROBOTIC AGENT, THERE SHOULD STILL BE A SEPARATE TYPE FOR CARRIERS.
        box1 box2 box3 box4 box5 - box
        apple pear - food
        morphine paracetamol - medicine
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
        (located_at box1 depot) (located_at box2 depot) (located_at box3 depot)
        (located_at box4 depot) (located_at box5 depot)

        ; ALL THE CONTENTS TO LOAD IN THE BOXES ARE INITIALLY LOCATED AT THE DEPOT.
        (located_at apple depot) (located_at pear depot)
        (located_at morphine depot) (located_at paracetamol depot)
        (located_at bandaid depot)

        ; THE CAPACITY OF THE CARRIERS (SAME FOR ALL CARRIERS) SHOULD BE PROBLEM SPECIFIC. THUS, IT SHOULD BE DEFINED IN THE PROBLEM FILE.
        (= (carrier_capacity helicopter) 4) ; FIX THE CAPACITY OF THE ROBOTIC AGENT TO BE 4 / THE ROBOTIC AGENT IS INITIALLY EMPTY.

        (is_empty box1) (is_empty box2) (is_empty box3) (is_empty box4) (is_empty box5)

        (need_medicine Chiara)
        (need_food Matteo) (need_tool Matteo)
        (need_food Marta) (need_medicine Marta)
    )

    ; ------------------------------------- DEFINE GOAL -------------------------------------

    (:goal 
        (and
            ; CERTAIN PEOPLE HAVE CERTAIN CONTENTS (e.g., medicine, food, tool). 
            (not (need_medicine Chiara))
            ; SOME PEOPLE MIGHT NOT NEED FOOD, MEDICINE, OR TOOL.
            (not (need_food Matteo)) (not (need_tool Matteo))
            ; SOME PEOPLE MIGHT NEED BOTH FOOD AND MEDICINE, OR FOOD AND TOOL, OR THREE OF THEM, AND SO ON.
            (not (need_food Marta)) (not (need_medicine Marta))
        )    
    )
)