	project(coman_walking_example)

################################################################
#    Coman project header. Don't touch unless you know how     # 
################################################################
cmake_minimum_required(VERSION 2.8)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

IF("$ENV{COMAN_ROOT}" STREQUAL "")
  message(FATAL_ERROR "!!! No COMAN_ROOT environment variable. !!!\n"
          "Try: >export COMAN_ROOT=/location/of/coman/libraries\n" $ENV{COMAN_ROOT})
ELSE()
  message("Install root is: " $ENV{COMAN_ROOT})
  set(COMAN_ROOT $ENV{COMAN_ROOT}) 
ENDIF()
 
#Install into source folder unless user sets something different
SET(MY_INSTALL_PREFIX ${COMAN_ROOT} CACHE PATH "Prefix prepended to install directories")
SET(CMAKE_INSTALL_PREFIX "${MY_INSTALL_PREFIX}" CACHE INTERNAL "Prefix prepended to install directories" FORCE)


set(CMAKE_INCLUDE_CURRENT_DIR ON)

##################################################################
#    Project files.  Adjust to fit your project.                 #
##################################################################

include_directories("${COMAN_ROOT}/include")
link_directories("${COMAN_ROOT}/lib")

add_definitions("-std=c++0x")

set(SOURCES
Boards_ctrl_walk.cpp
RTControl.cpp
GlobalExt.cpp
Matrix.cpp
Stabilizer.cpp
TestJoint.cpp
test_main.cpp
)

set(LIBRARIES
m
dl
stdc++
pthread
rt
coman_os
robolli
coman_microstrain
yaml-cpp
)

add_definitions(-DEPFL -DDEMO_TYPE=2)
add_executable(coman_walking_example ${SOURCES})

target_link_libraries(coman_walking_example ${LIBRARIES} ${XTRA_LIBRARIES})
install(TARGETS coman_walking_example DESTINATION bin)
