
add_executable(aruco_node "aruco_ros.cpp")

include_directories(. /home/x/Libs/eigen-3.3)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs tf roslaunch message_generation rosbag)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

# add_service_files(DIRECTORY srv FILES TwoInts.srv)
# generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp message_runtime
)

target_link_libraries(aruco_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

LINK_LIBRARIES(${PROJECT_NAME} ${REQUIRED_LIBRARIES}  ${OpenCV_LIBS} )

#install(DIRECTORY launch config
 # DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Check the validity of the supplied launch files.
# roslaunch_add_file_check(launch)
