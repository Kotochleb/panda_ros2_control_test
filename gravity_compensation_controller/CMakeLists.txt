cmake_minimum_required(VERSION 3.20)
project(gravity_compensation_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow -Werror=format)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(gravity_compensation_controller_parameters
  src/gravity_compensation_controller_parameter.yaml
)

add_library(gravity_compensation_controller SHARED
  src/gravity_compensation_controller.cpp
)
target_compile_features(gravity_compensation_controller PUBLIC cxx_std_17)
target_include_directories(gravity_compensation_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/gravity_compensation_controller>
)
target_link_libraries(gravity_compensation_controller PUBLIC gravity_compensation_controller_parameters
  Eigen3::Eigen
  pinocchio::pinocchio
  pinocchio::pinocchio_headers
)
ament_target_dependencies(gravity_compensation_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(gravity_compensation_controller PRIVATE "gravity_compensation_controller_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface gravity_compensation_controller_plugins.xml)

ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  gravity_compensation_controller/trajecotry_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include/gravity_compensation_controller
)
install(TARGETS gravity_compensation_controller gravity_compensation_controller_parameters
  EXPORT export_gravity_compensation_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_gravity_compensation_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
