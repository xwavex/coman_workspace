====== Pre-release notes =====

  The examples are built using scons, with no need to build the
coman libraries.

>cd $COMAN_ROOT/examples
>scons
>cd $COMAN_ROOT/examples/walking
>scons

The examples will be in their respective subfolders.




==============================
This folder contains some examples that can be used as a starting point.

Each example has a cmake file configured as a standalone project. They can be
built either as part of the coman libraries, or they can be built as if they
were a project independent of the coman libraries.

Obviously if you build them independently, the coman libraries must have been 
previously built.

These example cmake files expect the shell environment variable $COMAN_ROOT to 
be set to the path where coman lib and include files are. If make install is
run the binaries will be installed in $COMAN_ROOT/bin

	#Assuming coman libraries are in ~/coman
	export COMAN_ROOT=~/coman
	cd ~/coman/examples/basic
	mkdir build; cd build
	cmake ..
	make
	make install


