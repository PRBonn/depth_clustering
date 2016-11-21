add_executable(show_objects_kitti show_objects_kitti.cpp)
target_link_libraries(show_objects_kitti
                      identifiable
                      image_labeler
                      velodyne_utils
                      projections
                      ground_remove
                      folder_reader
                      visualization
                      ${Boost_LIBRARIES}
                      ${PCL_LIBRARIES}
                      ${OpenCV_LIBS}
                      ${CMAKE_THREAD_LIBS_INIT})

add_executable(show_objects_moosman show_objects_moosman.cpp)
target_link_libraries(show_objects_moosman
                      identifiable
                      image_labeler
                      velodyne_utils
                      projections
                      ground_remove
                      folder_reader
                      visualization
                      ${Boost_LIBRARIES}
                      ${PCL_LIBRARIES}
                      ${OpenCV_LIBS}
                      ${CMAKE_THREAD_LIBS_INIT})
