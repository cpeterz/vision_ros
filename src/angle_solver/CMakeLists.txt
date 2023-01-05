cmake_minimum_required(VERSION 3.5)
project(angle_solver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(libbase REQUIRED)
find_package(armor REQUIRED)

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

include_directories(include)

add_definitions(-DANGLE_CFG="${CMAKE_CURRENT_SOURCE_DIR}/AngleSolver.yaml")

add_library(angle_solver SHARED
        include/angle_solver/AngleSolver.hpp
        src/AngleSolver.cpp
)

target_include_directories(angle_solver PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)        


ament_target_dependencies(angle_solver
        PUBLIC
        libbase
        armor
        rclcpp
)


ament_export_targets(angle_solver HAS_LIBRARY_TARGET)
ament_export_dependencies(
        libbase
        armor
        rclcpp
)

install(
        DIRECTORY include/
        DESTINATION include
)

install(
        TARGETS angle_solver # 告诉ros2有这么个目标（可执行文件或者库）
        EXPORT angle_solver
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
ament_package()
