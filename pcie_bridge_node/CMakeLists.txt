cmake_minimum_required(VERSION 3.10.0)
project(pcie_bridge_node)

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
find_package(cv_bridge REQUIRED)

# set(OpenCV_DIR "/usr/local/share/OpenCV")
find_package(OpenCV REQUIRED)
message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

include_directories(
  ${PROJECT_SOURCE_DIR}/
  ${PROJECT_SOURCE_DIR}/inc/
)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

EXECUTE_PROCESS(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
MESSAGE(STATUS "cpu architecture: ${ARCHITECTURE}")

set(AUMO_S2_LIB ${PROJECT_SOURCE_DIR}/aumo_s2_sdk/linux/libControlPlay.a)
set(EXECUTABLE_NAME ${PROJECT_NAME}_bin)
MESSAGE(STATUS "executable name: " ${EXECUTABLE_NAME})

add_executable(${EXECUTABLE_NAME}  src/pcie_bridge_node.cc)
ament_target_dependencies(${EXECUTABLE_NAME}
  rclcpp 
  std_msgs
  cv_bridge
)
target_link_libraries(${EXECUTABLE_NAME} ${AUMO_S2_LIB})

add_executable(test_canfd_bin  src/test_canfd.cc)
ament_target_dependencies(test_canfd_bin
  rclcpp 
  std_msgs
  cv_bridge
)
target_link_libraries(test_canfd_bin ${AUMO_S2_LIB})

add_executable(test_raw12_bin  src/test_raw12.cc)
target_link_libraries(test_raw12_bin
    ${OpenCV_LIBS}
    -ldl
    -lrt
    -lpthread
)

add_executable(test_video_inject  src/test_video_inject.cc)
target_link_libraries(test_video_inject
    ${AUMO_S2_LIB}
    -ldl
    -lrt
    -lpthread
)

# install binary files
install(TARGETS ${EXECUTABLE_NAME}   DESTINATION  lib/${PROJECT_NAME})
install(TARGETS test_canfd_bin   DESTINATION  lib/${PROJECT_NAME})
install(TARGETS test_raw12_bin   DESTINATION  lib/${PROJECT_NAME})
install(TARGETS test_video_inject   DESTINATION  lib/${PROJECT_NAME})

# install library
install(DIRECTORY  aumo_s2_sdk  DESTINATION lib/${PROJECT_NAME})

# install launch files.
install(DIRECTORY   launch  DESTINATION share/${PROJECT_NAME}/)

# install config files.
install(DIRECTORY  config  DESTINATION share/${PROJECT_NAME}/)

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