#
# This is based on orocos sample cmakelists, modified slightly
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.8.3
#
cmake_minimum_required(VERSION 2.8.11)
 
#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
include(ExternalProject)
project(Walking-rtt)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

FIND_PACKAGE(Eigen3 REQUIRED)
#find_package(iDynTree REQUIRED)
find_package(rtt_ros)
find_package(rtt_sensor_msgs REQUIRED)
find_package(rtt_roscomm REQUIRED)
find_package(orocos-ocl REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)
find_package(srdfdom_advr REQUIRED)
find_package(XBotCoreModel REQUIRED)
find_package(catkin REQUIRED 
  rtt_ros 
  rtt_roscomm)

find_package(PkgConfig REQUIRED)
pkg_search_module(TINYXML REQUIRED tinyxml)
if(NOT TINYXML_INCLUDE_DIR)
        find_path("/usr/include" tinyxml.h)
        find_library(TINYXML_LIBRARY NAMES tinyxml)
        set(TINYXML_LIBRARIES ${TINYXML_LIBRARY})
    set(TINYXML_INCLUDE_DIRS ${TINYXML_INCLUDE_DIR})
    set(TINYXML_LIBRARY_DIR "/usr/lib/x86_64-linux-gnu")
endif()
message(STATUS "TINYXML_VERSION: ${TINYXML_VERSION}")
include_directories(BEFORE SYSTEM ${TINYXML_INCLUDE_DIRS})
link_directories(${TINYXML_LIBRARY_DIR})
message(STATUS "TINYXML_LIBRARIES: ${TINYXML_LIBRARIES}")
message(STATUS "TINYXML_LIBRARY_DIR: ${TINYXML_LIBRARY_DIR}")
message(STATUS "TINYXML_INCLUDE_DIR: ${TINYXML_INCLUDE_DIRS}")
message(STATUS "TINYXML_LIBRARY: ${TINYXML_LIBRARY}")

find_package(RST-RT REQUIRED)
message(STATUS "RST-RT version: ${RST-RT_VERSION}")
include_directories(BEFORE SYSTEM ${RST-RT_INCLUDE_DIRS})
add_definitions(${RST-RT_CFLAGS})
link_directories(${RST-RT_LIBRARY_DIR} ${RST-RT_LIBRARY_DIRS})

find_package(PkgConfig REQUIRED)
pkg_search_module(TINYXML REQUIRED tinyxml)
if(NOT TINYXML_INCLUDE_DIR)
        find_path("/usr/include" tinyxml.h)
        find_library(TINYXML_LIBRARY NAMES tinyxml)
        set(TINYXML_LIBRARIES ${TINYXML_LIBRARY})
    set(TINYXML_INCLUDE_DIRS ${TINYXML_INCLUDE_DIR})
    set(TINYXML_LIBRARY_DIR "/usr/lib/x86_64-linux-gnu")
endif()
message(STATUS "TINYXML_VERSION: ${TINYXML_VERSION}")
include_directories(BEFORE SYSTEM ${TINYXML_INCLUDE_DIRS})
link_directories(${TINYXML_LIBRARY_DIR})
message(STATUS "TINYXML_LIBRARIES: ${TINYXML_LIBRARIES}")
message(STATUS "TINYXML_LIBRARY_DIR: ${TINYXML_LIBRARY_DIR}")
message(STATUS "TINYXML_INCLUDE_DIR: ${TINYXML_INCLUDE_DIRS}")
message(STATUS "TINYXML_LIBRARY: ${TINYXML_LIBRARY}")


find_package(Boost REQUIRED COMPONENTS thread)
INCLUDE_DIRECTORIES(include ${EIGEN3_INCLUDE_DIR})
include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/include/eigen3/Eigen
)
# current source and include:
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

include_directories(
${PROJECT_SOURCE_DIR}/Walking/include
${USE_OROCOS_INCLUDE_DIRS}
${RST-RT_INCLUDE_DIRS}
${Boost_INCLUDE_DIR}
${Eigen_INCLUDE_DIRS}
${orocos_kdl_INCLUDE_DIRS}
${USE_OROCOS_INCLUDE_DIRS}
${RST-RT_INCLUDE_DIRS}
${TINYXML_INCLUDE_DIRS}
${srdfdom_advr_INCLUDE_DIRS}
${XBotCoreModel_INCLUDE_DIRS}
"~/coman_workspace/coman_shared/include"
)

include_directories("${PROJECT_SOURCE_DIR}/includeall")
include_directories("${PROJECT_SOURCE_DIR}/includeall/state_estimation_include")
 
# Set the CMAKE_PREFIX_PATH in case you're not using Orocos through ROS
# for helping these find commands find RTT.
find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

add_definitions(-DREAL_ROBOT)

find_package(rtt-core-extensions REQUIRED)
include_directories(${RTT-CORE-EXTENSIONS_INCLUDE_DIRS})
# ${idyntree_INCLUDE_DIRS}
link_directories(${RTT-CORE-EXTENSIONS_LIBRARY_DIRS})
# ${iDynTree_LIBRARIES}

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
 
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#
 
 
# Creates a component library libexample-<target>.so
# and installs in the directory lib/orocos/example/
#

AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/Walking/src SOURCES0)
AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/srcall/ SOURCESALL)
AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/srcall/state_estimation_src/ SOURCES_SEST)
set(SOURCES ${SOURCESALL} ${SOURCES0} ${SOURCES_SEST})

orocos_component(${CMAKE_PROJECT_NAME}   ${SOURCES}) # ...you may add multiple source files


# You may add multiple orocos_component statements.
 
#
# Building a typekit (recommended):
#
# Creates a typekit library libexample-types-<target>.so
# and installs in the directory lib/orocos/example/types/
#
#orocos_typegen_headers(example-types.hpp) # ...you may add multiple header files
#
# You may only have *ONE* orocos_typegen_headers statement !
 
#
# Building a normal library (optional):
#
# Creates a library libsupport-<target>.so and installs it in
# lib/
#
#orocos_library(support support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.
 
 
#
# Building a Plugin or Service (optional):
#
# Creates a plugin library libexample-service-<target>.so or libexample-plugin-<target>.so
# and installs in the directory lib/orocos/example/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
#orocos_service(example-service example-service.cpp) # ...only one service per library !
#orocos_plugin(example-plugin example-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin/orocos_service statements.

# target_link_libraries()
message(STATUS "Checking for OCL Library Dirs")
message(STATUS ${orocos-ocl_LIBRARY_DIRS})
target_link_libraries(
${CMAKE_PROJECT_NAME} 
${RST-RT_LIBRARIES} 
${RTT-CORE-EXTENSIONS_LIBRARIES} 
${USE_OROCOS_LIBRARIES} 
${OROCOS-RTT_LIBRARIES} 
${orocos-ocl_LIBRARY_DIRS}/librtt_rostopic-gnulinux.so 
${orocos-ocl_LIBRARY_DIRS}/librtt_rosclock-gnulinux.so 
${catkin_LIBRARIES}
${USE_OROCOS_LIBRARIES}
${orocos_kdl_LIBRARIES}
${OROCOS-RTT_LIBRARIES}
${Boost_LIBRARIES}
${RST-RT_LIBRARIES}
${TINYXML_LIBRARIES}
${srdfdom_advr_LIBRARIES}
${XBotCoreModel_LIBRARIES}
)

#
# Additional headers (not in typekit):
#
# Installs in the include/orocos/example/ directory
#
# orocos_install_headers( example-component.hpp ) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.
 
#
# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#
orocos_generate_package()
