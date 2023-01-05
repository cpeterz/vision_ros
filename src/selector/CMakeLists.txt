cmake_minimum_required(VERSION 3.5)
project(selector)

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
find_package(port REQUIRED)
find_package(armor REQUIRED)
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

# 添加源文件，生成库
add_library(selector SHARED
        include/selector/Selector.hpp
        src/Selector.cpp
        )
# 添加头文件地址
target_include_directories(selector PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        )

# 用于代替传统的target_link_libraries
ament_target_dependencies(selector
        PUBLIC
        armor
        port
        )

# 为了使下游文件可以访问
ament_export_targets(selector HAS_LIBRARY_TARGET)
ament_export_dependencies(
        armor
        port 
        ) 

# 注册 导出头文件
install(
        DIRECTORY include/
        DESTINATION include
        )


# 注册 导出库文件
install(
        TARGETS selector # 告诉ros2有这么个目标（可执行文件或者库）
        EXPORT selector
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
        )


        
ament_package()
