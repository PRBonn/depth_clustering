add_library(cloud SHARED cloud.cpp rich_point.cpp)
target_link_libraries(cloud
                      projections
                      ${OpenCV_LIBS}
                      ${PCL_LIBRARIES})

add_library(velodyne_utils SHARED velodyne_utils.cpp)
target_link_libraries(velodyne_utils
                      projections
                      cloud
                      ${PCL_LIBRARIES}
                      ${OpenCV_LIBS})

find_package(Boost REQUIRED COMPONENTS filesystem regex)
add_library(folder_reader SHARED folder_reader.cpp)
target_link_libraries(folder_reader
                      ${Boost_LIBRARIES})

add_library(bbox bbox.cpp)
target_link_libraries(bbox cloud)


