#Make sure all pre-requisites have been followed and dependencies installed, check the CMakeLists.txt in case certain directories need to be linked for building. Use the set() command to export the dir path.

#Edit setup-cogimon-walkingrtt-env.sh by changing the {$PATH} to the directory where this repository is cloned
export RTT_COMPONENT_PATH={$PATH}/build/orocos:/opt/ros/indigo/lib/orocos:$RTT_COMPONENT_PATH

#Source setup file from walking-rtt directory to terminal before running
source setup-cogimon-walkingrtt-env.sh

#Edit TestComanWalking.ops file from the walking-rtt/Walking/ops/ directory by replacing {$CITK_DIR} to your citk directory
#line 33: coman.loadURDFAndSRDF("{$CITK_DIR}/citk/systems/cogimon-minimal-trusty-nightly/share/gazebo/models/cogimon/iit-coman/model.urdf", "{$CITK_DIR}/citk/systems/cogimon-minimal-trusty-nightly/share/gazebo/models/cogimon/iit-coman/coman.srdf")

#Run deployer-gnulinux and copy the scripts found in the file "coman.txt"
~$ deployer-gnulinux
  Deployer [S] >> loadService("this", "scripting")
  = true
  Deployer [S] >> scripting.runScript("{$PATH}/walking-rtt/Walking/ops/TestComanWalking.ops")

#Initialize Roscore
~$ roscore

#Start Gazebo
~$ gzclient

#Exit the deployer to kill the process by using ctrl + 4 command, exit the gzclient before restarting the deployer

######################For Developers : ########################
#If any changes are made to the src/include files, 
~$ cd {$PATH}/build
~$ cmake ../walking-rtt
~$ make

#Follow the above steps to test
#=============================
#	Happy Coding =)
#=============================




