cmake_minimum_required(VERSION 3.5)
project(top_aimer)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(libbase REQUIRED)
find_package(armor REQUIRED)
find_package(angle_solver REQUIRED)
find_package(selector REQUIRED)
find_package(base_interfaces REQUIRED)
find_package(binocular REQUIRED)

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
add_definitions(-DTOP_CFG="${CMAKE_CURRENT_LIST_DIR}/TopAimer.yaml")
include_directories(include)

add_executable(top_aimer_node 
      src/Aimer.cpp
      src/RosAimer.cpp
)

ament_target_dependencies(top_aimer_node
      rclcpp
      libbase
      armor
      angle_solver
      selector
      base_interfaces
      binocular
)

install(TARGETS
  top_aimer_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
