cmake_minimum_required(VERSION 3.5)
project(port)

option(USE_SERIAL OFF)

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
find_package(std_msgs REQUIRED)
find_package(base_interfaces REQUIRED)
#find_package(pthread REQUIRED)
find_package(libbase REQUIRED)
if(USE_SERIAL)
  find_package(Boost REQUIRED )
endif()

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

set(PORT_SRC src/CanPort.cpp)
set(PORT_DEPEND )


if (USE_SERIAL)
  set(PORT_SRC ${PORT_SRC} src/SerialPort.cpp)
  set(PORT_DEPEND ${PORT_DEPEND} libbase ${Boost_LIBRARIES})
  add_definitions(-DPORT_CFG="${CMAKE_CURRENT_LIST_DIR}/../Port.yaml")
endif()

add_library(port SHARED
    ${PORT_SRC}
    include/port/CanPort.hpp
)
# 添加头文件地址

target_include_directories(port PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(port
    PUBLIC
    rclcpp
    libbase
    ${PORT_DEPEND}
    )
# 为了使下游文件可以访问
ament_export_targets(port HAS_LIBRARY_TARGET)
ament_export_dependencies(
    rclcpp
)

# 注册 导出头文件
install(
  DIRECTORY include/
  DESTINATION include
)
# 注册 导出库文件
install(
    TARGETS port
    EXPORT port
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
ament_package()

##CanPort

