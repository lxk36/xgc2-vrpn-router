cmake_minimum_required(VERSION 3.10)
project(xgc2_vrpn_router VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)

find_path(VRPN_INCLUDE_DIR vrpn_Tracker.h)
find_library(VRPN_LIBRARY NAMES vrpn)
find_library(QUAT_LIBRARY NAMES quat)

if(NOT VRPN_INCLUDE_DIR OR NOT VRPN_LIBRARY)
  message(FATAL_ERROR "VRPN headers/library not found. Install libvrpn-dev.")
endif()

add_executable(xgc2-vrpn-router src/main.cpp)
target_include_directories(xgc2-vrpn-router PRIVATE ${VRPN_INCLUDE_DIR})
target_link_libraries(xgc2-vrpn-router PRIVATE ${VRPN_LIBRARY} Threads::Threads)
if(QUAT_LIBRARY)
  target_link_libraries(xgc2-vrpn-router PRIVATE ${QUAT_LIBRARY})
endif()

install(TARGETS xgc2-vrpn-router RUNTIME DESTINATION bin)
install(FILES config/router.conf DESTINATION share/xgc2-vrpn-router)
install(FILES systemd/xgc2-vrpn-router.service DESTINATION /lib/systemd/system)
