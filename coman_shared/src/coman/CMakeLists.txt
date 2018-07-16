set(THIS_LIB coman)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(ThisLib_SOURCES 
Kinematics/FwInvK.cpp
Kinematics/Matrix.cpp
)

set(HEADERS_TO_INSTALL
Kinematics/libKinematics.h
)

add_library(${THIS_LIB} STATIC ${ThisLib_SOURCES})
target_link_libraries(${THIS_LIB})

install(TARGETS ${THIS_LIB} ARCHIVE DESTINATION lib)
install(FILES ${HEADERS_TO_INSTALL} DESTINATION include)

