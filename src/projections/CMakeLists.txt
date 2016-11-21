add_library(projections SHARED
            projection_params.cpp
            cloud_projection.cpp
            ring_projection.cpp
            spherical_projection.cpp)

target_link_libraries(projections ${OpenCV_LIBS})