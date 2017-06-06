add_library(difference SHARED
            angle_diff.cpp
            line_dist_diff.cpp
            normal_diff.cpp)

target_link_libraries(difference normal_computer)
