cmake_minimum_required(VERSION 3.8)
project(bagrecorder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CONFIG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../config/${PROJECT_NAME}/)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_storage REQUIRED)

add_executable(bagrecorder_exe src/bagrecorder.cpp)
target_include_directories(bagrecorder_exe PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(bagrecorder_exe PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
        bagrecorder_exe
        "example_interfaces"
        "rosbag2_cpp"
        "rosbag2_storage"
        "rosbag2_transport"
        "rclcpp"
        "std_msgs"
        "std_srvs"
        "sensor_msgs"
)
# 显式地链接 rosbag2_cpp 库
target_link_libraries(bagrecorder_exe
        rosbag2_cpp::rosbag2_cpp
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION ${CONFIG_DIR})
install(
        DIRECTORY ${CONFIG_DIR} # 顶层配置目录
        DESTINATION share/${PROJECT_NAME}/config
        USE_SOURCE_PERMISSIONS
)
#install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(TARGETS bagrecorder_exe
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
