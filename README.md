# <p align="center">Automated Planning: Theory and Practice - Project</p> 

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
****
# Installing all the planners for Ubuntu 22.04 on VirtualBox for MacOS

At the beginning, to update your packages and prepare for the next steps, please run the following commands:
```bash
sudo apt update
sudo apt upgrade
sudo apt install python3-pip
```

## GO and SINGULARITY
We require Singularity, a container platform, for installation. After installing Singularity, you can install Planutils using pip3.

1. Install GO
```bash
sudo apt update
sudo apt upgrade
sudo apt-get install -y curl vim git build-essential libseccomp-dev pkg-config squashfs-tools cryptsetup
export VERSION=1.20.3 OS=linux ARCH=amd64  # change this with the more recent in [GO](https://go.dev/dl/) website
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
To enable Fast Downward support in your system, you need to install Apptainer beforehand. Installing Apptainer facilitates the containerization of Fast Downward, enabling you to create self-contained and portable environments for running Fast Downward effectively.
```bash
sudo apt update
sudo apt upgrade
sudo apt install -y software-properties-common
sudo add-apt-repository -y ppa:apptainer/ppa
sudo apt update
sudo apt install -y apptainer
```

2. Install Planutils
Planutils is a suite of planners that simplifies the installation and usage of planners within a virtual environment. Once installed, you can conveniently use the suite of planners within the provided virtual environment.
```bash
sudo pip install planutils
planutils activate
planutils setup
exit
```

3. Install the DOWNWARD, FF, LAMA, LAMA-FIRST, ENHSP-2020, OPTIC, TFD
```bash
sudo planutils install -y downward ff lama lama-first enhsp-2020 optic tfd
```

## PANDA
Panda is a planner based on Java. it can be donwloaded and run using the following commands
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
PlanSys2 is based on ROS2. To compile a PlanSys2 project you will need ROS2 and 2 more packages that are required to build the dependencies of the project (Rosdep) and to compile it (Colcon for ROS2). Follow the ensuing steps to install everything

To compile a PlanSys2 project, you must have ROS2 installed, along with two additional packages necessary for building the project's dependencies (Rosdep) and compiling it (Colcon for ROS2). Follow the steps below to install all the required components.

### ROS2
Begin by installing ROS2. Make sure to follow the installation instructions specific to your operating system, which can be found on the official ROS2 website.
Once ROS2 is successfully installed, proceed to install Rosdep. Rosdep is a package manager for ROS that manages system dependencies required by ROS packages. Install Rosdep by executing the appropriate commands for your operating system. Detailed instructions can be found in the ROS documentation.

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

2. Add the ROS 2 GPG key with apt and add the repository to your sources list, then install ROS-Base
  ```bash
sudo apt update
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade

sudo apt install ros-humble-ros-base
  ```

### COLCON for ROS2
Get the files from the ROS2 repository and add its key to apt
  ```bash
sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install python3-colcon-common-extensions
  ```

### ROSDEP
Install rosdep
  ```bash
sudo apt-get install python3-rosdep
sudo rosdep init
rosdep update
  ```

***
# Running the planners
For running the planners on the domain and problem files of the assignment in this repository, you will need to run the following code:
  ```bash
cd Desktop
git clone https://github.com/akaGallo/EmergencyServicesLogistics-APTP2023.git planning
  ```
  
SONO ARRIVATO QUIIIIIIIIIII

## PLANUTILS
To run one of the planner installed in planutils follow the ensuing procedure
1. Get to the folder of one of the following tasks
  ```bash
cd <your_workspace>/planning/{task1|task2|task4}
  ```

2. Activate planutils 
```bash
activate planutils
```

3. Run the correct planner for that task according to the following list:
- Task1 & Task 2: downward
- Task4: optic or tfd (temporal fast downward)
```bash
<downward|tfd|optic> taskX_domain.pddl taskX_problem.pddl  #Change the X with the number of the task
```

## PANDA
To run the panda planner follow the ensuing procedure:
1. Get to the folder of task 3
  ```bash
cd <your_workspace>/planning/task3
  ```

2. Run the planner jar file previusly downloaded from its path
```bash
 java -jar /mnt/c/Users/<your_user>/<your_workspace>/PANDA.jar -parser hddl task3_domain_htn.hddl task3_problem_htn.hddl
```

## PLANSYS2 
To run PlanSys2 two terminals are required. You can easily manage two Ubuntu terminals inside the same application using Windows Terminal.
#### TERMINAL 1
Terminal one is used to build the dependencies, compile the project and host the PlanSys2 planner based on ROS. To do so follow the ensuing procedure
1. Get inside the project folder
  ```bash
cd <your_workspace>/planning/task5/plansys2_task5/
  ```

2. Install ROS2 infrastrucutre for the current terminal
  ```bash
source /opt/ros/humble/setup.bash
  ```

3. Compile the project a first time, it could lead to errors
  ```bash
colcon build --symlink-install
  ```

4. Now, install all the dependencies required by that project
  ```bash
rosdep install --from-paths ./ --ignore-src -r -y
  ```

5. Compile the project again, this time the compilation will be sucessful. If problem arises try to compile the code multiple times
  ```bash 
colcon build --symlink-install
  ```

6. Integrate ROS2 infrastrucutre with the PlanSys2 compiled in the previous step
  ```bash
source install/setup.bash
  ```

7. Luch ROS2. Now this terminal will be used only to show the results
  ```bash
ros2 launch plansys2_task5 plansys2_task5_launch.py
  ```
  
All the above listed procedure for Terminal1 is automatically exetcuted by running the file `compile_and_run.sh` present in the plansys2_task5 folder. Therefore, you can just run the two following commands to execute all the procedure
  ```bash
cd <your_workspace>/planning/task5/plansys2_task5/
bash compile_and_run.sh
  ```

#### TERINAL 2
Once terminal 1 has been set up, open a new terminal. Terminal two is used to run the PlanSys2_terminal, which is used to push into the planner all the wanted data (instances, predicates, goal), to compute a plan and to run it.

1. Repeat step 1, 2 and 6 (of Terminal1) also for this terminal
  ```bash
cd <your_workspace>/planning/task5/plansys2_task5/
source /opt/ros/humble/setup.bash
source install/setup.bash
  ```

2. Run ROS2-terminal
  ```bash
ros2 run plansys2_terminal plansys2_terminal
  ```

3. Source the problem data (using the absolute path from your system root)
  ```bash
source /mnt/c/Users/<you_user>/<your_workspace>/downloaded/plansys2_task/pddl/task5_problem 1  #Update this path according to your system
  ```

4. Get a plan
  ```bash
get plan
  ```

5. Run the plan. The status is visualized on the this terminal, the sum-up is visualized on the first terminal
  ```bash
run
  ```
  
The first two steps of the above listed procedure for Terminal2 are automatically exetcuted by running the file `run_terminal.sh` present in the plansys2_task5 folder. Therefore, you can just run the two following commands to execute all the procedure
  ```bash
cd <your_workspace>/planning/task5/plansys2_task5/
bash run_terminal.sh
# One in the terminal run
source /mnt/c/Users/<your_user>/<your_workspace>/plansys2_task5/pddl/task5_problem 1  #Update this path according to your system
# If at this points error like 'Could not add the predicate <name_of_predicate>' start to manifest, please stop both the terminals (pressing CTRL+C) and repeat the procedure for terminal1 and terminal2. This will fix the problem. 
# If no problem arises, run the following commands
get plan
run
# If after the command 'run' nothing happens on Terminal2 then look at Terminal1. If here a message like 'process has died' is present. Then stop both the terminals (pressing CTRL+C) and repeat the procedure for terminal1 and terminal2. This will fix the problem. 
  ```
