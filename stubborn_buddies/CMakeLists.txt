cmake_minimum_required(VERSION 3.5)
project(stubborn_buddies)

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
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(stubborn_buddies_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.

include_directories(
  include
  ${lifecycle_msgs_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS})

add_library(lifecycle_watchdog SHARED
  src/lifecycle_watchdog.cpp)
target_compile_definitions(lifecycle_watchdog
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(lifecycle_watchdog
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "stubborn_buddies_msgs"
  "rclcpp_lifecycle")
rclcpp_components_register_nodes(lifecycle_watchdog "lifecycle_watchdog::LifecycleWatchdog")
set(node_plugins "${node_plugins}lifecycle_watchdog::LifecycleWatchdog;$<TARGET_FILE:lifecycle_watchdog>\n")

add_library(lifecycle_heartbeat SHARED
  src/lifecycle_heartbeat.cpp)
target_compile_definitions(lifecycle_heartbeat
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(lifecycle_heartbeat
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "stubborn_buddies_msgs"
  "rclcpp_lifecycle")
rclcpp_components_register_nodes(lifecycle_heartbeat "lifecycle_heartbeat::LifecycleHeartbeat")
set(node_plugins "${node_plugins}lifecycle_heartbeat::LifecycleHeartbeat;$<TARGET_FILE:lifecycle_heartbeat>\n")

add_library(lifecycle_talker SHARED
  src/lifecycle_talker.cpp)
target_compile_definitions(lifecycle_talker
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(lifecycle_talker
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "rclcpp_lifecycle"
  "stubborn_buddies_msgs")
rclcpp_components_register_nodes(lifecycle_talker "lifecycle_talker::LifecycleTalker")
set(node_plugins "${node_plugins}lifecycle_talker::LifecycleTalker;$<TARGET_FILE:lifecycle_talker>\n")

# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

add_executable(linktime_composition
  src/linktime_composition.cpp)

set(libs
  lifecycle_heartbeat
  lifecycle_watchdog
  lifecycle_talker)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set(libs
    "-Wl,--no-as-needed"
    ${libs}
    "-Wl,--as-needed")
endif()

target_link_libraries(linktime_composition ${libs})
ament_target_dependencies(linktime_composition
  "class_loader"
  "rclcpp"
  "rclcpp_components")

#ament_export_include_directories(include)
ament_export_targets(lifecycle_heartbeat HAS_LIBRARY_TARGET)
ament_export_targets(lifecycle_watchdog HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp 
  rclcpp_lifecycle
  stubborn_buddies_msgs
  rclcpp_components)

install(TARGETS
  lifecycle_heartbeat
  EXPORT lifecycle_heartbeat
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

install(TARGETS
  lifecycle_watchdog
  EXPORT lifecycle_watchdog
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

install(TARGETS
  lifecycle_talker
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

install(TARGETS
  linktime_composition
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
