set(THIS_LIB coman_microstrain)


if (USE_REALTIME) 
  add_definitions(-DRT_ENV)
#  set(RTLIBS rtdm)
endif (USE_REALTIME)


set(CMAKE_INCLUDE_CURRENT_DIR ON)
include_directories(${PROJECT_SOURCE_DIR}/src/os)

set(ThisLib_SOURCES 
imu_3DM-GX3-25.cpp
)

set(HEADERS_TO_INSTALL
imu_3DM-GX3-25.h
)

add_library(${THIS_LIB} SHARED ${ThisLib_SOURCES})
#target_link_libraries(${THIS_LIB} ${RTLIBS})

install(TARGETS ${THIS_LIB} ARCHIVE DESTINATION LIBRARY DESTINATION lib)
install(FILES ${HEADERS_TO_INSTALL} DESTINATION include)

