cmake_minimum_required(VERSION 3.8)
project(va500p)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CONFIG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../config/${PROJECT_NAME}/)
message(STATUS "CONFIG_DIR路径：${CONFIG_DIR}")
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
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

add_executable(va500p src/va500p.cpp)
target_link_libraries(va500p
        serial  # 第三方库
)

target_include_directories(va500p PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(va500p PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  va500p
  "rclcpp"
  "rclcpp_lifecycle"
  "lifecycle_msgs"
  "std_msgs"
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION ${CONFIG_DIR})
install(
        DIRECTORY ${CONFIG_DIR} # 顶层配置目录
        DESTINATION share/${PROJECT_NAME}/config
        USE_SOURCE_PERMISSIONS
)

install(TARGETS va500p
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
