cmake_minimum_required(VERSION 3.10)
project(gravity_compensation_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  urdf
  pluginlib
  controller_interface
  hardware_interface
  realtime_tools
  eigen_conversions
  franka_hw
)

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

# Declare catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/gravity_compensation_controller.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} 
  Eigen3::Eigen
  pinocchio::pinocchio
  pinocchio::pinocchio_headers
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install plugin description
install(FILES gravity_compensation_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install launch files
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
