cmake_minimum_required(VERSION 3.8)
project(nodemanage)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CONFIG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../config/${PROJECT_NAME}/)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(nodemanage src/nodemanage.cpp)
target_include_directories(nodemanage PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(nodemanage PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(nodemanage yaml-cpp)   # 链接 yaml-cpp
ament_target_dependencies(
  nodemanage
  "rclcpp"
  "yaml-cpp"
  "lifecycle_msgs"
  "std_msgs"
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
#install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION ${CONFIG_DIR})
install(
        DIRECTORY ${CONFIG_DIR} # 顶层配置目录
        DESTINATION share/${PROJECT_NAME}/config
        USE_SOURCE_PERMISSIONS
)
install(TARGETS nodemanage
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
