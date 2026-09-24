cmake_minimum_required(VERSION 3.10)
project(wamv_utils LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  nav_msgs
  roscpp
  tf
)

catkin_package(
  CATKIN_DEPENDS gazebo_msgs nav_msgs roscpp tf
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(wamv_state_converter wamv_state_converter.cpp)
target_link_libraries(wamv_state_converter ${catkin_LIBRARIES})
add_dependencies(wamv_state_converter ${catkin_EXPORTED_TARGETS})

install(
  TARGETS wamv_state_converter
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
