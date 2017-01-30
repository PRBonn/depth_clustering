add_executable(show_objects_kitti show_objects_kitti.cpp)
target_link_libraries(show_objects_kitti
                      identifiable
                      image_labeler
                      velodyne_utils
                      projections
                      ground_remove
                      folder_reader
                      visualization
                      ${MY_QT_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${PCL_LIBRARIES}
                      ${CMAKE_THREAD_LIBS_INIT})

add_executable(show_objects_moosmann show_objects_moosmann.cpp)
target_link_libraries(show_objects_moosmann
                      identifiable
                      image_labeler
                      velodyne_utils
                      projections
                      ground_remove
                      folder_reader
                      visualization
                      ${MY_QT_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${PCL_LIBRARIES}
                      ${OpenCV_LIBS}
                      ${CMAKE_THREAD_LIBS_INIT})
