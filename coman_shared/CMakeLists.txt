project(coman_shared)
cmake_minimum_required(VERSION 2.8)

##################################################################################
########## Setup stuff. You probably want to skip to the next section. ###########
##################################################################################
set(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH}
  "${CMAKE_CURRENT_SOURCE_DIR}/utils/cmake"
  "${CMAKE_CURRENT_SOURCE_DIR}/utils/cmake/modules" #Local Find* modules
  )

#Install into source folder unless user sets something different
SET(COMAN_ROOT ${PROJECT_SOURCE_DIR} CACHE PATH "Prefix prepended to install directories")
SET(CMAKE_INSTALL_PREFIX "${COMAN_ROOT}" CACHE INTERNAL "Prefix prepended to install directories" FORCE)

#A macro to allow clean, readable inclusion of subdirectories
macro(coman_optional_build name description path ONOFF)
  option(BUILD_${name} ${description} ${ONOFF})
  if( BUILD_${name} )
    #We this need to make include files available examples... a bit brute force
    include_directories("${PROJECT_SOURCE_DIR}/${path}")
    add_subdirectory(${path})
  endif()
endmacro(coman_optional_build name description path ONOFF)

##################################################################################
##########   Libraries and applications to compile are specified here  ###########
##################################################################################

#Force a flag on the compiler. Useful for tracking include/library order.
#add_definitions(-v)

include_directories(${PROJECT_SOURCE_DIR}/include) #Everybody gets the base include directory

find_package(XenomaiPosix)
if(XENOMAI_POSIX_FOUND)
  option(USE_REALTIME "Compile program for realtime" ON)
  execute_process(COMMAND xeno-config --skin=posix --cflags OUTPUT_VARIABLE XENO_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
  execute_process(COMMAND xeno-config --skin=posix --ldflags OUTPUT_VARIABLE XENO_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${XENO_CFLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${XENO_CFLAGS}")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${XENO_LDFLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${XENO_LDFLAGS} -lnative")
endif (XENOMAI_POSIX_FOUND)

option(USE_ZMQ "Use ZMQ for IPC" OFF)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -O3")

coman_optional_build(robolli "Build RoboLLi networked controller drivers" src/robolli ON)
coman_optional_build(microstrain "Build Microstrain IMU drivers" src/microstrain ON)
coman_optional_build(os "Build COMAN OS Abstraction Layer" src/os ON)
coman_optional_build(coman "Build COMAN library (kinematics, coordination, etc)" src/coman ON)
coman_optional_build(example_basic "Build coman basic example" examples/basic OFF)
coman_optional_build(example_microstrain "Build IMU test example" examples/microstrain OFF)
coman_optional_build(example_walking "Build COMAN walking example" examples/walking OFF)
coman_optional_build(example_impendance "Build COMAN impedance example" examples/impedance OFF)
coman_optional_build(xddp_console "Build COMAN xddp_console" examples/xddp_console OFF)

