# <p align="center">Automated Planning: Theory and Practice - Project</p> 

<p align="justify">This repository represents a student project by Carlo Marotta for the "Automated Planning" course of the Master's in Artificial Intelligence Systems at the University of Trento, for the academic year 2022-2023.</p>

The repository consists of the following components:

- Assignment PDF: This document provides the project's assignment details.
- Problem Folders: There are 5 separate folders for each of the five problems described in the assignment.
- Report PDF: This document contains the project report.
- README file: it serves as a guide to install all the necessary materials required to run the code, how to execute it and the obtained results in form of screenshot.

Here's a brief description of the five different tasks:

* <p align="justify">Problem 1: it involves developing the basic structure of the problem using the Planning Domain Definition Language (PDDL). The problem revolves around a robotic agent responsible for delivering necessary contents to injured people. To run this problem, you will need the Planutils environment and the FastDownward planner.</p>

* <p align="justify">Problem 2: it is an extension of the previous problem, introducing a carrier that enables the robotic agent to transport plus contents more efficiently. It also utilizes PDDL.</p>

* <p align="justify">Problem 3: the PDDL problem from Problem 2 is converted to the Hierarchical Task Network Planning Domain Definition Language (HDDL). Tasks and methods are introduced and made compatible with the existing actions. To run this task, you will need the Panda planner.</p>

* <p align="justify">Problem 4: it extends Problem 2 by introducing a temporal domain. Each action is assigned a specific duration and time constraints are incorporated. The goal is to minimize the required time. To run this task, you can use either the Optic planner or the Temporal Fast Downward planner.</p>

* <p align="justify">Problem 5: it further extends Problem 4 by employing a more sophisticated planner that allows the definition of C++ code for each action. Once a plan is obtained, which minimizes the required time, it is possible to simulate the proposed solution by running the plan using the implemented C++ codes. To run this problem, you will need ROS2 and PlanSys2.</p>

The mentioned planners are available and ready to install on Linux. It is recommended to use a Linux machine for the best experience. Alternatively, you can install [Ubuntu 22.04](https://ubuntu.com/download/desktop) for the [VirtualBox](https://www.virtualbox.org) directly on MacOS. Instructions for installing it are provided in the subsequent section. If you are using Linux, you can proceed directly to the section that explains the installation process for the planners on Linux.

****
# Install all the planners in Ubuntu 22.04 using VirtualBox for MacOS
<p align="justify">At the beginning, to update your packages and prepare for the next steps, please run the following commands:</p>

```bash
sudo apt update
sudo apt upgrade
sudo apt install -y python3-pip
```

## GO and SINGULARITY
<p align="justify">We require Singularity, a container platform, for installation. After installing Singularity, you can install Planutils using pip3.</p>

1. Install GO
```bash
sudo apt update
sudo apt upgrade
sudo apt-get install -y curl vim git build-essential libseccomp-dev pkg-config squashfs-tools cryptsetup
export VERSION=1.20.3 OS=linux ARCH=amd64  # change this with the more recent in the following GO website (https://go.dev/dl/)
sudo rm -r /usr/local/go
wget -O /tmp/go${VERSION}.${OS}-${ARCH}.tar.gz https://dl.google.com/go/go${VERSION}.${OS}-${ARCH}.tar.gz && \
sudo tar -C /usr/local -xzf /tmp/go${VERSION}.${OS}-${ARCH}.tar.gz
echo 'export GOPATH=${HOME}/go' >> ~/.bashrc
echo 'export PATH=/usr/local/go/bin:${PATH}:${GOPATH}/bin' >> ~/.bashrc
source ~/.bashrc
curl -sfL https://install.goreleaser.com/github.com/golangci/golangci-lint.sh |
sh -s -- -b $(go env GOPATH)/bin v1.21.0
mkdir -p ${GOPATH}/src/github.com/sylabs
cd ${GOPATH}/src/github.com/sylabs
```

2. Install Singularity
```bash
git clone https://github.com/sylabs/singularity.git
cd singularity
git checkout v3.6.3
cd ${GOPATH}/src/github.com/sylabs/singularity
./mconfig
cd ./builddir
make
sudo make install
```
  
3. Change Singularity mount hostfs option
```bash
cd /usr/local/etc/singularity
sudo chmod a+rwx singularity.conf
vim singularity.conf

# Press the 'shift+I' keys to enter insert mode.
# Scroll down the file until you locate the line containing the phrase "mount hostfs = no".
# Modify the line by replacing "mount hostfs = no" with "mount hostfs = yes".
# Press the 'esc' key to exit insert mode.
# Type ':x' and press enter to save the changes and exit the file.
```

## Apptainer and PLANUTILS
1. Install Apptainer
<p align="justify">To enable Fast Downward support in your system, you need to install Apptainer beforehand. Installing Apptainer facilitates the containerization of Fast Downward, enabling you to create self-contained and portable environments for running Fast Downward effectively.</p>

```bash
sudo apt update
sudo apt upgrade
sudo apt install -y software-properties-common
sudo add-apt-repository -y ppa:apptainer/ppa
sudo apt update
sudo apt install -y apptainer
```

2. Install Planutils
<p align="justify">Planutils is a suite of planners that simplifies the installation and usage of planners within a virtual environment. Once installed, you can conveniently use the suite of planners within the provided virtual environment.</p>

```bash
sudo pip install planutils
planutils activate
planutils setup
exit
```

3. Install DOWNWARD, FF, LAMA, LAMA-FIRST, ENHSP-2020, OPTIC, TFD:
```bash
sudo planutils install -y downward ff lama lama-first enhsp-2020 optic tfd
```

## PANDA
Panda is a planner based on Java. it can be donwloaded and run using the following commands:
1. Update the packages information and install java
```bash
sudo apt-get update
sudo apt install default-jre
```
2. Download PANDA planner in your workspace
```bash
cd Desktop/
wget https://www.uni-ulm.de/fileadmin/website_uni_ulm/iui.inst.090/panda/PANDA.jar
```

## PLANSYS2
<p align="justify">PlanSys2 is based on ROS2. To compile a PlanSys2 project you will need ROS2 and 2 more packages that are required to build the dependencies of the project (Rosdep) and to compile it (Colcon for ROS2). Follow the ensuing steps to install everything.
To compile a PlanSys2 project, you must have ROS2 installed, along with two additional packages necessary for building the project's dependencies (Rosdep) and compiling it (Colcon for ROS2). Follow the steps below to install all the required components.</p>

### ROS2
<p align="justify">Begin by installing ROS2. Make sure to follow the installation instructions specific to your operating system, which can be found on the official ROS2 website.
Once ROS2 is successfully installed, proceed to install Rosdep. Rosdep is a package manager for ROS that manages system dependencies required by ROS packages. Install Rosdep by executing the appropriate commands for your operating system. Detailed instructions can be found in the ROS documentation.</p>

1. Set locale and Setup sources
  ```bash
locale
sudo apt update
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale

sudo apt install -y software-properties-common
sudo add-apt-repository universe
  ```

2. Add the ROS 2 GPG key with apt and add the repository to your sources list, then install ROS-Base.
  ```bash
sudo apt update
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade

sudo apt install ros-humble-ros-base
  ```

### COLCON for ROS2
Get the files from the ROS2 repository and add its key to apt.
  ```bash
sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install python3-colcon-common-extensions
  ```

### ROSDEP
Install rosdep.
  ```bash
sudo apt-get install python3-rosdep
sudo rosdep init
rosdep update
  ```

***
# Running the planners
<p align="justify">For running the planners on the domain and problem files of the assignment in this repository, you will need to run the following code:</p>

  ```bash
cd Desktop
git clone https://github.com/akaGallo/APTP_project
  ```

## PROBLEM 1
<p align="justify">To run the Problem 1, you will need the Planutils environment and the FastDownward planner for the usage of FF, LAMA, LAMA-FIRST and DOWNWARD.</p>

  ```bash
cd Desktop/APTP_project/problem1
  ```
  
### SIMPLE
  ```bash
cd simple
# for each of the following wait the end of its execution
planutils run ff domain1_simple.pddl problem1_simple.pddl
planutils run lama domain1_simple.pddl problem1_simple.pddl
planutils run lama-first domain1_simple.pddl problem1_simple.pddl
planutils run downward domain1_simple.pddl problem1_simple.pddl “—search astar(goalcount)”
  ```

### CRANE
  ```bash
cd crane
# for each of the following wait the end of its execution
planutils run ff domain1_crane.pddl problem1_crane.pddl
planutils run lama domain1_crane.pddl problem1_crane.pddl
planutils run lama-first domain1_crane.pddl problem1_crane.pddl
planutils run downward domain1_crane.pddl problem1_crane.pddl “—search astar(goalcount)”
  ```

## PROBLEM 2
<p align="justify">To run the Problem 2, you have to use again FF, LAMA and LAMA-FIRST for the SIMPLE case, while for the FLUENTS case you will need the ENHSP-2020 planner.</p>

  ```bash
cd Desktop/APTP_project/problem2
  ```
  
### SIMPLE
  ```bash
cd simple
# for each of the following wait the end of its execution
planutils run ff domain2_simple.pddl problem2_simple.pddl
planutils run lama domain2_simple.pddl problem2_simple.pddl
planutils run lama-first domain2_simple.pddl problem2_simple.pddl
planutils run downward domain2_simple.pddl problem2_simple.pddl “—search astar(goalcount)”
  ```

### FLUENTS
  ```bash
cd fluents
# for each of the following wait the end of its execution
planutils run enhsp-2020 “-o domain2_fluents.pddl -f problem2_fluents.pddl -planner opt-blind”
planutils run enhsp-2020 “-o domain2_fluents.pddl -f problem2_fluents.pddl -planner opt-hmax”
planutils run enhsp-2020 “-o domain2_fluents.pddl -f problem2_fluents.pddl -planner opt-hrmax”
planutils run enhsp-2020 “-o domain2_fluents.pddl -f problem2_fluents.pddl -planner sat-aibr”
planutils run enhsp-2020 “-o domain2_fluents.pddl -f problem2_fluents.pddl -planner sat-hadd”
planutils run enhsp-2020 “-o domain2_fluents.pddl -f problem2_fluents.pddl -planner sat-hmrp”
planutils run enhsp-2020 “-o domain2_fluents.pddl -f problem2_fluents.pddl -planner sat-hmrph”
planutils run enhsp-2020 “-o domain2_fluents.pddl -f problem2_fluents.pddl -planner sat-hmrphj”
planutils run enhsp-2020 “-o domain2_fluents.pddl -f problem2_fluents.pddl -planner sat-hradd”
  ```

## PROBLEM 3
<p align="justify">To run Problem 3, make sure to have the PANDA planner available in your workspace. Load the PANDA planner before proceeding further.</p>

  ```bash
cd Desktop/APTP_project/problem3
  ```
  
### HTN1
  ```bash
cp Desktop/PANDA.jar Desktop/APTP_project/problem3/htn1
cd htn1
java -jar PANDA.jar -parser hddl domain3_htn1.hddl problem3_htn1.hddl
  ```

### HTN2
  ```bash
cp Desktop/PANDA.jar Desktop/APTP_project/problem3/htn2
cd htn2
java -jar PANDA.jar -parser hddl domain3_htn2.hddl problem3_htn2.hddl
  ```

## PROBLEM 4
<p align="justify">To run the Problem 4, you will need the OPTIC planner.</p>

  ```bash
cd Desktop/APTP_project/problem4
  ```
  
### SIMPLE
  ```bash
cd simple
# for each of the following wait the end of its execution
planutils run optic “-N domain4_simple.pddl problem4_simple.pddl”
planutils run optic “-N -W1,1 domain4_simple.pddl problem4_simple.pddl”
planutils run optic “-N -E -W1,1 domain4_simple.pddl problem4_simple.pddl”
  ```

### FLUENTS
  ```bash
cd fluents
# for each of the following wait the end of its execution
planutils run optic “-N domain4_fluents.pddl problem4_fluents.pddl”
planutils run optic “-N -W1,1 domain4_fluents.pddl problem4_fluents.pddl”
planutils run optic “-N -E -W1,1 domain4_fluents.pddl problem4_fluents.pddl”
  ```

## PROBLEM 5
<p align="justify">To run PlanSys2, you need to have two separate terminals running simultaneously.</p>

  ```bash
cd Desktop/APTP_project/problem5
  ```
  
### SIMPLE
1. Open the TERMINAL 1 and run the following code:
<p align="justify">Proceed with the following steps in terminal one to build the dependencies, compile the project, and host the PlanSys2 planner based on ROS.</p>

  ```bash
cd plansys2_problem5_simple
bash terminal1.sh
  ```
  
2. Then, open the TERMINAL 2 and execute the final output:
<p align="justify">After setting up terminal 1, open a new terminal for terminal 2, that is dedicated to running the PlanSys2_terminal, which facilitates sending desired data (instances, predicates, goals) to the planner, computing a plan, and executing it.</p>

  ```bash
cd plansys2_problem5_simple
bash terminal2.sh
# [INFO] [...] [terminal]: No problem file specified.
# ROS2 Planning System console. Type "quit" to finish
source pddl/problem5_problem
get plan
run
```

### FLUENTS
1. Open the TERMINAL 1 and run the following code:
  ```bash
cd plansys2_problem5_fluents
bash terminal1.sh
  ```
  
2. Then, open the TERMINAL 2 and execute the final output:
  ```bash
cd plansys2_problem5_simple
bash terminal2.sh
# [INFO] [...] [terminal]: No problem file specified.
# ROS2 Planning System console. Type "quit" to finish
source pddl/problem5_problem
get plan
run
```
