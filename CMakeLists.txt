cmake_minimum_required(VERSION 3.0.2)
project(vrpn_router)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geometry_msgs
  roscpp
  tf2
  xmlrpcpp
)

catkin_package(
  CATKIN_DEPENDS diagnostic_msgs geometry_msgs roscpp tf2 xmlrpcpp
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

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
