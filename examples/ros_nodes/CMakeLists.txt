add_executable(show_objects_node show_objects_node.cpp)
target_link_libraries(show_objects_node
                      ros_bridge
                      image_labeler
                      projections
                      ground_remove
                      visualization
                      ${MY_QT_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES}
                      ${CMAKE_THREAD_LIBS_INIT})

if (${PCL_FOUND})
  add_executable(save_clusters_node save_clusters_node.cpp)
  target_link_libraries(save_clusters_node
                        ros_bridge
                        image_labeler
                        projections
                        ground_remove
                        cloud_saver
                        ${Boost_LIBRARIES}
                        ${catkin_LIBRARIES}
                        ${OpenCV_LIBS}
                        ${PCL_LIBRARIES})
endif()
