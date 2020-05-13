## Copyright (C) 2020  I. Bogoslavskyi, C. Stachniss
##
## Permission is hereby granted, free of charge, to any person obtaining a
## copy of this software and associated documentation files (the "Software"),
## to deal in the Software without restriction, including without limitation
## the rights to use, copy, modify, merge, publish, distribute, sublicense,
## and/or sell copies of the Software, and to permit persons to whom the
## Software is furnished to do so, subject to the following conditions:
##
## The above copyright notice and this permission notice shall be included in
## all copies or substantial portions of the Software.
##
## THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
## IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
## FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
## AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
## LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
## FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
## DEALINGS IN THE SOFTWARE.

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


