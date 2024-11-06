cmake_minimum_required(VERSION 3.8)
project(stretch_nav2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set (lib_name stretch_nav2_core)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)

set(dep_pkgs
  rclcpp
  nav2_costmap_2d
  pluginlib
)
# include Cpp "include" directory
include_directories(include)

add_library(${lib_name} SHARED
  stretch_nav2/human_layer.cpp
)
include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY map DESTINATION share/${PROJECT_NAME})
install(DIRECTORY stretch_nav2 DESTINATION share/${PROJECT_NAME})
install(TARGETS ${lib_name}
  DESTINATION lib)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  stretch_nav2/simple_commander_demo.py
  DESTINATION lib/${PROJECT_NAME}
)
pluginlib_export_plugin_description_file(nav2_costmap_2d human_layer.xml)
pluginlib_export_plugin_description_file(nav2_costmap_2d gradient_layer.xml)

ament_target_dependencies(${lib_name} ${dep_pkgs})
ament_package()
