## Copyright (C) 2020  I. Bogoslavskyi, C. Stachniss
##
## GNU-GPL licence that follows one of libQGLViewer.

add_library(visualization SHARED visualizer.cpp)
target_link_libraries(visualization
                      identifiable
                      ${QGLVIEWER_LIBRARY}
                      ${MY_QT_LIBRARIES}
                      ${OPENGL_gl_LIBRARY}
                      ${OPENGL_glu_LIBRARY})
if(${PCL_FOUND})
  add_library(cloud_saver SHARED cloud_saver.cpp)
  target_link_libraries(cloud_saver ${PCL_LIBRARIES} ${OpenCV_LIBS})
endif()
