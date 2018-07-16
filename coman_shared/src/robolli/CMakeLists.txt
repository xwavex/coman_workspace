set(THIS_LIB robolli)

if (USE_REALTIME) 
  add_definitions(-DRT_ENV)
endif (USE_REALTIME)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
include_directories(${PROJECT_SOURCE_DIR}/src/os)

set(ThisLib_SOURCES 
Boards_iface.cpp
Boards_ctrl_ext.cpp
CommProtocol.cpp
DSP_board.cpp
)

set(HEADERS_TO_INSTALL
Boards_iface.h
Boards_ctrl_ext.h
Boards_exception.h
CommProtocol.hpp
definitions.h
broadcast_data.h
DSP_board.h
)

add_library(${THIS_LIB} SHARED ${ThisLib_SOURCES})
target_link_libraries(${THIS_LIB})

install(TARGETS ${THIS_LIB} ARCHIVE DESTINATION LIBRARY DESTINATION lib)
install(FILES ${HEADERS_TO_INSTALL} DESTINATION include)

