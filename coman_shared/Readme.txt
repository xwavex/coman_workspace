
	Welcome to the COMAN libraries.

== Quick Start ==

By default the libraries and examples are installed in subdirectories of this directory.
If you want to change the install the libraries in some other place (such as the system
/usr/local area) you can change the cmake install prefix.

To build and install the libraries:

	mkdir build; cd build
	cmake ..
	make
	make install

Alternately, to build examples at the same time...

	cmake .. -DBUILD_example_basic=ON -DBUILD_example_microstrain=ON

The examples expect the COMAN_ROOT environment variable to be set:

	#Assuming you are in the same directory as this readme...
	export COMAN_ROOT=$(pwd)

== Organization ==

./src		Source code for coman libraries and applications
./examples	Examples you can base your own applications on
./utils		Scripts for getting dependencies, updating firmware


./src/robolli		Library for using the coman's motor controller
./src/microstrain	Library for using the coman's IMU
./src/os		Helper library for threading, ipc and realtime
./src/coman		Library for coman kinematics

