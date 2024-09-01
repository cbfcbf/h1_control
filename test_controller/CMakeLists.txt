cmake_minimum_required(VERSION 3.0.2)
project(test_controller)

# Set C++ standard to C++11 or higher
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find catkin dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_interface
  hardware_interface
  pluginlib
  legged_common
)

# Declare catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp controller_interface hardware_interface pluginlib legged_common
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Add the library
add_library(${PROJECT_NAME} src/test_controller.cpp)

# Link against catkin libraries
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Install the library
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install include files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
