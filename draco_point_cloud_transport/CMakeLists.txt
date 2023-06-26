cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)

project(draco_point_cloud_transport)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(point_cloud_interfaces REQUIRED)
find_package(point_cloud_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(Draco REQUIRED)

set(dependencies
  pluginlib
  point_cloud_interfaces
  point_cloud_transport
  rclcpp
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

target_link_libraries(${PROJECT_NAME} ${DRACO_LIBRARY})

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

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

# TODO: Fix tests
# if (CATKIN_ENABLE_TESTING)
#   find_package(roslint REQUIRED)

#   # catkin_lint - checks validity of package.xml and CMakeLists.txt
#   # ROS buildfarm calls this without any environment and with empty rosdep cache,
#   # so we have problems reading the list of packages from env
#   # see https://github.com/ros-infrastructure/ros_buildfarm/issues/923
#   if(DEFINED ENV{ROS_HOME})
#     #catkin_lint: ignore_once env_var
#     set(ROS_HOME "$ENV{ROS_HOME}")
#   else()
#     #catkin_lint: ignore_once env_var
#     set(ROS_HOME "$ENV{HOME}/.ros")
#   endif()
#   #catkin_lint: ignore_once env_var
#   if(DEFINED ENV{ROS_ROOT} AND EXISTS "${ROS_HOME}/rosdep/sources.cache")
#     roslint_custom(catkin_lint "-W2" .)
#   endif()

#   # Roslint C++ - checks formatting and some other rules for C++ files

#   file(GLOB_RECURSE ROSLINT_INCLUDE include/*.h include/*.hpp)
#   file(GLOB_RECURSE ROSLINT_SRC src/*.cpp src/*.hpp src/*.h)
#   #file(GLOB_RECURSE ROSLINT_TEST test/*.cpp)

#   set(ROSLINT_CPP_OPTS "--extensions=h,hpp,hh,c,cpp,cc;--linelength=120;--filter=\
#     -build/header_guard,-readability/namespace,-whitespace/braces,-runtime/references,\
#     -build/c++11,-readability/nolint,-readability/todo,-legal/copyright,-build/namespaces")
#   roslint_cpp(${ROSLINT_INCLUDE} ${ROSLINT_SRC})

#   roslint_add_test()
# endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
