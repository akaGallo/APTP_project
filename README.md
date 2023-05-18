# Automated Planning: Theory and Practice - Project

This repository represents a student project by Carlo Marotta for the "Automated Planning" course of the Master's in Artificial Intelligence Systems at the University of Trento, for the academic year 2022-2023.

The repository consists of the following components:

- Assignment PDF: This document provides the project's assignment details.
- Problem Folders: There are 5 separate folders for each of the five problems described in the assignment.
- Report PDF: This document contains the project report.
- README file: it serves as a guide to install all the necessary materials required to run the code, and how to execute it.
- 
Here's a brief description of the five different tasks:

* Problem 1: it involves developing the basic structure of the problem using the Planning Domain Definition Language (PDDL). The problem revolves around a robotic agent responsible for delivering necessary contents to injured people. To run this problem, you will need the Planutils environment and the FastDownward planner.

* Problem 2: it is an extension of the previous problem, introducing a carrier that enables the robotic agent to transport plus contents more efficiently. It also utilizes PDDL.

* Problem 3: the PDDL problem from Problem 2 is converted to the Hierarchical Task Network Planning Domain Definition Language (HDDL). Tasks and methods are introduced and made compatible with the existing actions. To run this task, you will need the Panda planner.

* Problem 4: it extends Problem 2 by introducing a temporal domain. Each action is assigned a specific duration and time constraints are incorporated. The goal is to minimize the required time. To run this task, you can use either the Optic planner or the TemporalFastDownward planner.

* Problem 5: it further extends Problem 4 by employing a more sophisticated planner that allows the definition of C++ code for each action. Once a plan is obtained, which minimizes the required time, it is possible to simulate the proposed solution by running the plan using the implemented C++ codes. To run this problem, you will need ROS2 and PlanSys2.

The mentioned planners are available and ready to install on Linux. It is recommended to use a Linux machine for the best experience. Alternatively, you can install [Ubuntu 22.04](https://ubuntu.com/download/desktop) for the [VirtualBox](https://www.virtualbox.org) directly on MacOS. Instructions for installing it are provided in the subsequent section. If you are using Linux, you can proceed directly to the section that explains the installation process for the planners on Linux.
