# Mega Build for Coman Orocos Interface

Follow steps from section 2: if you want to run simulations, from 1: if you want to run the coman through orocos  

# Section 1: Building dependencies

Follow the following commands:  

$ source ~/citk/systems/cogimon-minimal-trusty-nightly/bin/setup-cogimon-env.sh

$ cd ~/

$ git clone https://c4science.ch/diffusion/6862/coman_workspace.git

$ cd ~/coman_workspace/iit-rtt-coman/yaml-cpp

$ mkdir build && cd build

$ cmake ../ && make -j8

$ mkdir -p ~/coman_workspace/coman_shared/build && cd ~/coman_workspace/coman_shared/build

$ cmake ../ && make -j8

$ cd ../../iit-rtt-coman/microstrain

$ mkdir build && cd build

$ cmake ../ && make -j8

$ cd ../../rtt_coman

$ mkdir build && cd build

$ cmake ../ && make -j8

# Section 2: Building Walking Code

# Remember to check CMakeLists.txt in walking-rtt to make RealRobot Build or SimRobot Build  

Change ops file urdf and srdf load paths by replacing /home/biorob/ with /home/user/ in walking-rtt/Walking/ops/  

path to ops file :

gedit ~/coman_workspace/walking-rtt/Walking/ops/TestComanWalking.ops

Look for the urdf_path and srdf_path and change the paths by adding your current username to it in place of /home/biorob/

# Follow the following commands:  

# IF SIMULATION : 

Ctrl + Shift + T (In a new Terminal)  

$ roscore

Ctrl + Shift + T (In a new Terminal)  

$ source ~/citk/systems/cogimon-minimal-trusty-nightly/bin/setup-cogimon-env.sh

$ cd ~/coman_workspace

$ mkdir build && cd build

$ cmake ../walking-rtt/

$ source ../walking-rtt/setup-cogimon-walkingrtt-env.sh

$ deployer-gnulinux

#SimRobot (Replace $HOME with /home/user/)

Deployer [S]> loadService("this", "scripting")

Deployer [S]> scripting.runScript("$HOME/coman_workspace/walking-rtt/Walking/ops/TestComanWalking.ops")

#RealRobot (Replace $HOME with /home/user/)

Deployer [S]> loadService("this", "scripting")

Deployer [S]> scripting.runScript("$HOME/coman_workspace/walking-rtt/Walking/ops/coman_walking_real.ops")

Ctrl + Shift + T (In a new Terminal) 

$ gzclient