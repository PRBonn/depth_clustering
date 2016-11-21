add_executable(show_objects_node show_objects_node.cpp)
target_link_libraries(show_objects_node
                      ros_bridge
                      image_labeler
                      projections
                      ground_remove
                      visualization
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBS}
                      ${PCL_LIBRARIES}
                      ${CMAKE_THREAD_LIBS_INIT})

add_custom_command(
    TARGET show_objects_node POST_BUILD
    COMMAND ln -sf ${PROJECT_SOURCE_DIR}/bin/show_objects_node show_objects_node
)

if (${PCL_FOUND})
  add_executable(save_clusters_node save_clusters_node.cpp)
  target_link_libraries(save_clusters_node
                        ros_bridge
                        image_labeler
                        projections
                        ground_remove
                        visualization
                        cloud_saver
                        ${Boost_LIBRARIES}
                        ${catkin_LIBRARIES}
                        ${OpenCV_LIBS}
                        ${PCL_LIBRARIES})
endif()
