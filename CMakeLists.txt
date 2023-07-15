cmake_minimum_required(VERSION 3.5)
project(hobot_visualization)

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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

# x3|x5ultra|x86
set(PREFIX_PATH x3)
set(SYS_ROOT ${CMAKE_SYSROOT})

if(PLATFORM_X3)
  message("build platform X3")
  add_definitions(-DPLATFORM_X3)
  set(PREFIX_PATH x3)
elseif(PLATFORM_X5ultra)
  message("build platform X5ultra")
  add_definitions(-DPLATFORM_X5ultra)
  set(PREFIX_PATH x5ultra)
elseif(PLATFORM_X86)
  message("build platform x86")
  add_definitions(-DPLATFORM_X86)
  set(PREFIX_PATH x86)
  set(SYS_ROOT ${THIRD_PARTY})
else()
  if (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
    message("invalid platform, build platform X3 default")
    add_definitions(-DPLATFORM_X3)
    set(PREFIX_PATH x3)
  elseif (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
    message("build platform X86")
    add_definitions(-DPLATFORM_X86)
    set(PREFIX_PATH x86)
    set(SYS_ROOT ${THIRD_PARTY})
  endif()
endif()

message("PREFIX_PATH is " ${PREFIX_PATH})
message("SYS_ROOT is " ${SYS_ROOT})

include_directories(include
  ${PROJECT_SOURCE_DIR}
  ${SYS_ROOT}/usr/include
)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/vis_render_node.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}
  ai_msgs
  rclcpp
  visualization_msgs
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/launch/
  DESTINATION share/${PROJECT_NAME}/launch)

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

ament_package()
