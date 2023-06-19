cmake_minimum_required(VERSION 3.10.2)
project(draco_point_cloud_transport)

find_package(builtin_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(point_cloud_transport REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Use Draco as a standalone cmake project, it gets built thanks to catkin
# package.xml.
# There are some find_package/naming issues due to name draco/Draco mismatch.
# Currently, with caktin tools only DracoConfig.cmake is found with just two
# variables defined, draco_INCLUDE_DIR and draco_LIBRARY_DIR. In that case we
# try to reconstruct the rest from FindDraco.cmake (module mode).
find_package(Draco REQUIRED)
if(draco_INCLUDE_DIR)
  set(draco_FOUND TRUE)
  set(draco_LIBRARY_DIRS "${draco_LIBRARY_DIR}")
  find_path(draco_INCLUDE_DIRS draco/point_cloud/point_cloud.h PATHS "${draco_INCLUDE_DIR}/..")
  find_library(draco_LIBRARIES NAMES draco PATHS "${draco_LIBRARY_DIR}")
  message("Draco found: ${draco_INCLUDE_DIRS}, ${draco_LIBRARIES}")
endif()

set(msg_files
  "msg/CompressedPointCloud2.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES sensor_msgs std_msgs builtin_interfaces
)

include_directories(include SYSTEM ${draco_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/cloud.cpp
  src/draco_publisher.cpp
  src/draco_subscriber.cpp
)
ament_target_dependencies(${PROJECT_NAME} pluginlib point_cloud_transport sensor_msgs std_msgs)

add_library(${PROJECT_NAME}_plugin
  src/cloud.cpp
  src/draco_publisher.cpp
  src/draco_subscriber.cpp
  src/manifest.cpp
)
ament_target_dependencies(${PROJECT_NAME} pluginlib point_cloud_transport sensor_msgs std_msgs)
class_loader_hide_library_symbols(${PROJECT_NAME}_plugin)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_plugin
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(FILES draco_plugins.xml
  DESTINATION DESTINATION share/${PROJECT_NAME}
)

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

ament_package()
