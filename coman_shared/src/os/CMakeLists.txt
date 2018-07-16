set(THIS_LIB coman_os)
set(THIS_LIB_RT coman_os_rt)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(ThisLib_SOURCES 
nrt_serial.cpp
nrt_thread.cpp
)

set(RT_SOURCES
rt_thread.cpp
rt_serial.cpp
rt_ipc.cpp
)

set(HEADERS_TO_INSTALL
utils.h
thread_util.h
nrt_serial.h
)

set(RT_HEADERS_TO_INSTALL
rt_ipc.h
rt_serial.h
)

add_library(${THIS_LIB} SHARED ${ThisLib_SOURCES})
install(TARGETS ${THIS_LIB} ARCHIVE DESTINATION LIBRARY DESTINATION lib )
install(FILES ${HEADERS_TO_INSTALL} DESTINATION include)

if (XENOMAI_POSIX_FOUND)
  add_library(${THIS_LIB_RT} SHARED ${ThisLib_SOURCES} ${RT_SOURCES})
  #target_link_libraries(${THIS_LIB}_rt)
  install(TARGETS ${THIS_LIB_RT} DESTINATION lib)
  install(FILES ${RT_HEADERS_TO_INSTALL} DESTINATION include)
endif (XENOMAI_POSIX_FOUND)


