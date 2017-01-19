add_library(ground_remove SHARED depth_ground_remover.cpp)
target_link_libraries(ground_remove
                      identifiable
                      velodyne_utils
                      difference
                      ${PCL_LIBRARIES}
                      ${OpenCV_LIBS})