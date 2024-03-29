cmake_minimum_required(VERSION 3.8)
project(nesfr7_simple_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(nesfr_bridge REQUIRED)
find_package(nesfr_teleop REQUIRED)
find_package(joy REQUIRED)
find_package(nesfr_arm_bringup REQUIRED)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME})

#
# reference: https://roboticsbackend.com/ros2-package-for-both-python-and-cpp-nodes/
#
# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/joy_switch.py
  nesfr7.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  nesfr7.service
  DESTINATION share/${PROJECT_NAME}
)

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

ament_package()
