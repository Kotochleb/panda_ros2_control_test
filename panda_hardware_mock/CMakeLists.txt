cmake_minimum_required(VERSION 3.16)
project(panda_hardware_mock LANGUAGES CXX)

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

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(panda_hardware_mock_parameters
  src/panda_hardware_mock_parameter.yaml
)

add_library(panda_hardware_mock SHARED
  src/panda_hardware_mock.cpp
)
target_compile_features(panda_hardware_mock PUBLIC cxx_std_17)
target_include_directories(panda_hardware_mock PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/panda_hardware_mock>
)
target_link_libraries(panda_hardware_mock PUBLIC panda_hardware_mock_parameters)
ament_target_dependencies(panda_hardware_mock PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(panda_hardware_mock PRIVATE "panda_hardware_mock_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface panda_hardware_mock_plugins.xml)

ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  panda_hardware_mock/trajecotry_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include/panda_hardware_mock
)
install(TARGETS panda_hardware_mock panda_hardware_mock_parameters
  EXPORT export_panda_hardware_mock
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_panda_hardware_mock HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
