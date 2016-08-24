INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src ${PROJECT_SOURCE_DIR}/src/include_MindVision ${GNULIBS_INCLUDE_DIR} .)
LINK_LIBRARIES(${PROJECT_NAME} ${REQUIRED_LIBRARIES} )

 


ADD_EXECUTABLE(aruco_create_markermap aruco_create_markermap.cpp)


ADD_EXECUTABLE(aruco_simple_markermap aruco_simple_markermap.cpp)
ADD_EXECUTABLE(aruco_test_markermap aruco_test_markermap.cpp pcdwriter.cpp MVCamera.cpp)
ADD_EXECUTABLE(aruco_markermap_pix2meters aruco_markermap_pix2meters.cpp)
 

INSTALL(TARGETS     aruco_markermap_pix2meters     aruco_simple_markermap aruco_test_markermap    RUNTIME DESTINATION bin)


find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs tf roslaunch message_generation)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp message_runtime
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_test_markermap ${catkin_LIBRARIES}  libMVSDK.so)
#set_target_properties(${PROJECT_NAME}_test_markermap PROPERTIES
#                      OUTPUT_NAME ${PROJECT_NAME}_test_markermap PREFIX "")

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Check the validity of the supplied launch files.
roslaunch_add_file_check(launch)
