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
Planutils is a suite of planners that simplifies the installation and usage of planners within a virtual environment. It requires Singularity, a container platform, for installation. After installing Singularity, you can install Planutils using pip3. Once installed, you can conveniently use the suite of planners within the provided virtual environment.

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
First, on Ubuntu based containers install software-properties-common package to obtain add-apt-repository command. On Ubuntu Desktop/Server derived systems skip this step.
```bash
sudo apt update
sudo apt upgrade
sudo apt install -y software-properties-common
sudo add-apt-repository -y ppa:apptainer/ppa
sudo apt update
sudo apt install -y apptainer
```

2. Install Planutils
```bash
sudo pip install planutils
planutils activate
planutils setup
exit
```

5. Install the DOWNWARD, FF, LAMA, LAMA-FIRST, ENHSP-2020, OPTIC, TFD
```bash
sudo planutils install -y downward ff lama lama-first enhsp-2020 optic tfd
```

## PANDA
Panda is a planner based on Java. it can be donwloaded and ran using the following commands
1. Update the packages information and install java
```bash
apt-get update
apt install default-jre
```
2. Download PANDA planner in you workspace
```bash
cd <your_workspace>
wget https://www.uni-ulm.de/fileadmin/website_uni_ulm/iui.inst.090/panda/PANDA.jar
```

## PLANSYS2
PlanSys2 is based on ROS2. To compile a PlanSys2 project you will need ROS2 and 2 more packages that are required to build the dependencies of the project (Rosdep) and to compile it (Colcon for ROS2). Follow the ensuing steps to install everything

### ROS2
1. Set locale
  ```bash
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings
  ```
  
2. Setup sources
  ```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
  ```

3. Add the ROS 2 GPG key with apt
  ```bash
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  ```

4. Add the repository to your sources list
  ```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  ```

5. Update your apt repository caches after setting up the repositories and verify that all the currently installed packages are up to date before installing ROS2
  ```bash
sudo apt update
sudo apt upgrade
  ```

6. Install ROS-Base (Bare Bones): Communication libraries, message packages, command line tools. This command will not install GUI tools (they are useless since we are working from WSL terminal).
  ```bash
sudo apt install ros-humble-ros-base
  ```

### COLCON for ROS2
1. Get the files from the ROS2 repository and add its key to apt
  ```bash
sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
  ```

2. Update your apt repository caches after setting up the repositories and the install the package using apt
  ```bash
sudo apt update
sudo apt install python3-colcon-common-extensions
  ```

### ROSDEP
1. Install rosdep
  ```bash
sudo apt-get install python3-rosdep
  ```

2. Initialize and update it
  ```bash
sudo rosdep init
rosdep update
  ```
