cmake_minimum_required(VERSION 3.0.2)
project(vrpn_router)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geometry_msgs
  roscpp
  xmlrpcpp
)

catkin_package(
  CATKIN_DEPENDS diagnostic_msgs geometry_msgs roscpp xmlrpcpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(vrpn_router_node
  src/vrpn_router_node.cpp
)
add_dependencies(vrpn_router_node ${catkin_EXPORTED_TARGETS})
target_compile_features(vrpn_router_node PUBLIC cxx_std_17)
target_link_libraries(vrpn_router_node ${catkin_LIBRARIES})

install(TARGETS vrpn_router_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(VRPN REQUIRED)
  find_package(Threads REQUIRED)

  catkin_add_gtest(vrpn_router_core_tests test/core_tests.cpp)
  if(TARGET vrpn_router_core_tests)
    target_link_libraries(vrpn_router_core_tests ${catkin_LIBRARIES})
  endif()

  add_executable(vrpn_router_test_vrpn_server_node
    test/vrpn_test_server_node.cpp
  )
  add_dependencies(vrpn_router_test_vrpn_server_node ${catkin_EXPORTED_TARGETS})
  target_compile_features(vrpn_router_test_vrpn_server_node PUBLIC cxx_std_17)
  target_link_libraries(vrpn_router_test_vrpn_server_node
    ${catkin_LIBRARIES}
    ${VRPN_SERVER_LIBRARIES}
    Threads::Threads
  )

  add_rostest(test/vrpn_router_e2e.test DEPENDENCIES vrpn_router_node)
  add_rostest(test/vrpn_router_protocol_e2e.test
    DEPENDENCIES vrpn_router_node vrpn_router_test_vrpn_server_node
  )
endif()
