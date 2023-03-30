cmake_minimum_required(VERSION 3.0.2)
project(rotation_controller)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        pluginlib
        controller_interface
)

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        pluginlib
        CATKIN_DEPENDS
        roscpp
        std_msgs
)

include_directories(
    include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/rotation_controller.cpp
)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
)

# Mark other files for installation
install(
        FILES rotation_controller_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)