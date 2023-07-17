cmake_minimum_required(VERSION 3.10.0)
project(can_bridge_node)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(carla_msgs REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/
  ${PROJECT_SOURCE_DIR}/inc/
)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

EXECUTE_PROCESS(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
MESSAGE(STATUS "cpu architecture: ${ARCHITECTURE}")
if(${ARCHITECTURE} STREQUAL "x86_64")
  set(USBCAN_LIB ${PROJECT_SOURCE_DIR}/usbcan/controlcan/libcontrolcan_x64.so)
elseif(${ARCHITECTURE} STREQUAL "aarch64")
  set(USBCAN_LIB ${PROJECT_SOURCE_DIR}/usbcan/controlcan/libcontrolcan_arm64.so)
else()
  MESSAGE(FATAL_ERROR "invalid cpu architecture!")
endif()

MESSAGE(STATUS "usbcan lib path: " ${USBCAN_LIB})
set(EXECUTABLE_NAME ${PROJECT_NAME}_bin)
MESSAGE(STATUS "executable name: " ${EXECUTABLE_NAME})

add_executable(${EXECUTABLE_NAME}  src/can_bridge_node.cc)
ament_target_dependencies(${EXECUTABLE_NAME}
  rclcpp 
  std_msgs
  carla_msgs
)

target_link_libraries(${EXECUTABLE_NAME} ${USBCAN_LIB})

# install binary files
install(TARGETS
${EXECUTABLE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  usbcan
  DESTINATION lib/${PROJECT_NAME}
)

# install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# install config files.
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# install assets files.
# install(DIRECTORY
#   assets
#   DESTINATION share/${PROJECT_NAME}/
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()