cmake_minimum_required(VERSION 3.5)

project(pcl_point_cloud_transport)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(PCL REQUIRED QUIET COMPONENTS common io)
find_package(pcl_conversions REQUIRED)
find_package(pluginlib REQUIRED)
find_package(point_cloud_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(dependencies
  pcl_conversions
  pluginlib
  point_cloud_transport
  rclcpp
  sensor_msgs
  std_msgs
)

include_directories(include ${PCL_COMMON_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  SHARED
  src/manifest.cpp
  src/pcl_publisher.cpp
  src/pcl_sensor_msgs_pointcloud2_type_adapter.cpp
  src/pcl_subscriber.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

add_executable(pclcontainer_subscriber
  examples/subscriber.cpp
  src/pcl_sensor_msgs_pointcloud2_type_adapter.cpp
)
ament_target_dependencies(pclcontainer_subscriber
  pcl_conversions
  point_cloud_transport
  rclcpp
  sensor_msgs
)
target_link_libraries(pclcontainer_subscriber ${PCL_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# Install executables
install(
  TARGETS pclcontainer_subscriber
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(point_cloud_transport pcl_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} PCL)
ament_export_dependencies(${dependencies})
ament_package()
