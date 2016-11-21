add_library(ros_bridge SHARED cloud_odom_ros_subscriber.cpp)
target_link_libraries(ros_bridge
                      identifiable
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES})