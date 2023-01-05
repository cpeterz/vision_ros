cmake_minimum_required(VERSION 3.5)
project(deepdetector)

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
find_package(libbase REQUIRED)
find_package(armor REQUIRED)
find_package(binocular REQUIRED)
find_package(lightoptimizer REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(base_interfaces REQUIRED)
find_package(OpenVINO REQUIRED)
# find_package(OpenCV 3.4.13 REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

#component所用cmake
add_library(deep_detector_component SHARED
 src/DeepDetector.cpp
 src/RosDeepDetector.cpp
)

add_definitions(-DDEEP_CFG="${CMAKE_CURRENT_LIST_DIR}/DeepDetector.yaml")
add_definitions(-DMODEL_DIR="${CMAKE_CURRENT_LIST_DIR}/model/")
add_definitions(-DARMOR_CFG="${CMAKE_CURRENT_LIST_DIR}/ArmorDetector.yaml")

target_link_libraries(deep_detector_component openvino::runtime)

ament_target_dependencies(deep_detector_component
  "libbase"
  "base_interfaces"
  "cv_bridge"
  "rclcpp_components"
  "armor"
  "binocular"
  "lightoptimizer"
  # "OpenCV"
)



install(TARGETS 
  deep_detector_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

rclcpp_components_register_nodes(deep_detector_component "wmj::DeepDetector_Node")


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
