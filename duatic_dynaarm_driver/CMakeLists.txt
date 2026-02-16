cmake_minimum_required(VERSION 3.8)
project(duatic_dynaarm_driver)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wno-c++20-extensions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)

set(PACKAGE_DEPENDENCIES
  rclcpp
  pluginlib
  hardware_interface
  urdf
  ethercat_sdk_master
  rsl_drive_sdk
  duatic_duadrive_interface
)

foreach(Dependency IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/dynaarm_hardware_interface.cpp
  src/dynaarm_mock_hardware_interface.cpp
  src/kinematic_translation.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${YAML_CPP_INCLUDE_DIR}
)

ament_target_dependencies(${PROJECT_NAME} ${PACKAGE_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES})

ament_export_dependencies(${PACKAGE_DEPENDENCIES} yaml-cpp)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories("include/${PROJECT_NAME}")

pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)


install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  config urdf
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)

endif()

ament_package()
