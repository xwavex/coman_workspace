
set(THIS_LIB coman_microstrain)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
include_directories(${PROJECT_SOURCE_DIR}/src/os
	"~/coman_workspace/coman_shared/include")

set(ThisLib_SOURCES 
imu_3DM-GX3-25.cpp
)

set(HEADERS_TO_INSTALL
imu_3DM-GX3-25.h
)

add_library(${THIS_LIB} STATIC ${ThisLib_SOURCES})
#target_link_libraries(${THIS_LIB} ${RTLIBS})

install(TARGETS ${THIS_LIB} ARCHIVE DESTINATION lib)
install(FILES ${HEADERS_TO_INSTALL} DESTINATION include)

