cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)

project(draco_point_cloud_transport)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(point_cloud_interfaces REQUIRED)
find_package(point_cloud_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(Draco REQUIRED)

set(dependencies
  pluginlib
  point_cloud_interfaces
  point_cloud_transport
  rclcpp
  rcpputils
  sensor_msgs
  std_msgs
)


include_directories(include ${DRACO_INCLUDE_DIR})

add_library(${PROJECT_NAME}
  SHARED
  src/cloud.cpp
  src/draco_publisher.cpp
  src/draco_subscriber.cpp
  src/manifest.cpp
)

<<<<<<< HEAD
target_link_libraries(${PROJECT_NAME} ${DRACO_LIBRARY})

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
=======
target_link_libraries(${PROJECT_NAME} PRIVATE
  ${DRACO_LIBRARIES}
  ${dependencies}
)
target_include_directories(${PROJECT_NAME} PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${DRACO_INCLUDE_DIR}
)
>>>>>>> 5126976 (Change Linking to DRACO_LIBRARIES as it was changes in draco (#57))

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(point_cloud_transport draco_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
