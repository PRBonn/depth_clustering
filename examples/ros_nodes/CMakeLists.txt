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
