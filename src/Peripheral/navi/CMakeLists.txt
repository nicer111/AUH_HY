cmake_minimum_required(VERSION 3.8)
project(navi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CONFIG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../config/${PROJECT_NAME}/)
set(SERIAL_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../Third/serial)
file(GLOB SERIAL_SOURCES
        ${SERIAL_ROOT_DIR}/src/*.cc
        ${SERIAL_ROOT_DIR}/src/impl/*.cc
        ${SERIAL_ROOT_DIR}/src/impl/list_ports/*.cc
)

add_library(serial STATIC
        ${SERIAL_SOURCES}
)

target_include_directories(serial PUBLIC
        ${SERIAL_ROOT_DIR}/include
        ${SERIAL_ROOT_DIR}/include/impl
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

add_executable(navi src/navi.cpp)
target_link_libraries(navi
        serial  # 第三方库
)

target_include_directories(navi PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(navi PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  navi
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "tf2"
  "tf2_geometry_msgs"
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION ${CONFIG_DIR})
install(
        DIRECTORY ${CONFIG_DIR} # 顶层配置目录
        DESTINATION share/${PROJECT_NAME}/config
        USE_SOURCE_PERMISSIONS
)

install(TARGETS navi
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
