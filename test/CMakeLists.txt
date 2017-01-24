
#############
## Testing ##
#############

# Add gtest based cpp test target and link libraries

add_library(test_help_funcs SHARED helper_funcs.cpp)
target_link_libraries(test_help_funcs ${PCL_LIBRARIES})

add_executable(depth_clustering_test
                 test_main.cpp
                 test_identifiable.cpp
                 test_pose.cpp
                 test_cloud.cpp
                 test_rich_point.cpp
                 test_sender_client.cpp
                 test_image_clusterer.cpp
                 test_radians.cpp
                 test_proj_params.cpp
                 test_cloud_projection.cpp
                 test_angle_diff.cpp
                 test_line_dist_diff.cpp
                 test_ground_remove.cpp)

target_link_libraries(depth_clustering_test
                    test_help_funcs
                    identifiable
                    projections
                    image_labeler
                    ground_remove
                    difference
                    ${catkin_LIBRARIES}
                    ${PCL_LIBRARIES}
                    ${OpenCV_LIBS}
                    ${GTEST_LIBRARIES})

# this will go away once we get rid of catkin
add_test(test_clustering
         ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/depth_clustering_test)