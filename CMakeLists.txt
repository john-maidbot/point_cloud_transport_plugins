cmake_minimum_required(VERSION 2.8.3)
project(draco_point_cloud_transport)

# Use Draco as a standalone cmake project, it gets built thanks to catkin
# package.xml.
find_package(catkin REQUIRED
        COMPONENTS
        dynamic_reconfigure
        message_generation
        point_cloud_transport
        sensor_msgs
        std_msgs)

# There are some find_package/naming issues due to name draco/Draco mismatch.
# Currently, with cakting tools only DracoConfig.cmake is found with just two
# variables defined, draco_INCLUDE_DIR and draco_LIBRARY_DIR. In that case we
# try to reconstruct the rest from FindDraco.cmake (module mode).
find_package(Draco REQUIRED)
if(draco_INCLUDE_DIR)
  set(draco_FOUND TRUE)
  set(draco_INCLUDE_DIRS "${draco_INCLUDE_DIR}/..")
  set(draco_LIBRARY_DIRS "${draco_LIBRARY_DIR}")
  set(draco_LIBRARIES draco)
endif()

add_message_files(
        FILES
        CompressedPointCloud2.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        sensor_msgs
)

generate_dynamic_reconfigure_options(cfg/DracoPublisher.cfg cfg/DracoSubscriber.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS dynamic_reconfigure point_cloud_transport sensor_msgs std_msgs message_runtime
  # Checks draco_INCLUDE_DIRS and draco_LIBRARIES.
  DEPENDS draco
)

include_directories(include
        ${catkin_INCLUDE_DIRS}
        ${draco_INCLUDE_DIRS})

link_directories(${draco_LIBRARY_DIRS})

add_library(${PROJECT_NAME}
        src/draco_publisher.cpp
        src/draco_subscriber.cpp
        src/manifest.cpp
        src/conversion_utilities.cpp
        src/DracotoPC2.cpp
        src/PC2toDraco.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg draco_point_cloud_transport_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${draco_LIBRARIES})
class_loader_hide_library_symbols(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
)

# add xml file
install(FILES draco_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
