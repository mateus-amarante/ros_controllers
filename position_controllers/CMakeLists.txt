cmake_minimum_required(VERSION 3.0.2)
project(position_controllers)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  forward_command_controller
  hardware_interface
  pluginlib
  rostime
)

# Declare a catkin package
catkin_package(
    CATKIN_DEPENDS forward_command_controller hardware_interface
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)


###########
## Build ##
###########

# Specify header include paths
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME}
  src/joint_position_controller.cpp
  src/joint_group_position_controller.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(roscpp REQUIRED)

  include_directories(${controller_manager_INCLUDE_DIRS})
  include_directories(${std_msgs_INCLUDE_DIRS})
  include_directories(${sensor_msgs_INCLUDE_DIRS})
  include_directories(${roscpp_INCLUDE_DIRS})

  add_executable(dummy_robot test/dummy_robot.cpp)
  target_link_libraries(dummy_robot ${catkin_LIBRARIES} ${controller_manager_LIBRARIES} ${roscpp_LIBRARIES})

  add_rostest_gtest(position_controllers_test
    test/position_controllers.test
    test/position_controllers_test.cpp
  )
  target_link_libraries(position_controllers_test ${catkin_LIBRARIES} ${roscpp_LIBRARIES})

endif()


#############
## Install ##
#############

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install targets
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install plugins
install(FILES position_controllers_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
