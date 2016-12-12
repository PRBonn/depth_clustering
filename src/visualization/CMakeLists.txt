add_library(visualization SHARED visualizer.cpp)
target_link_libraries(visualization
                      identifiable
                      ${PCL_LIBRARIES}
                      ${QGLVIEWER_LIBRARY}
                      ${QT_QTOPENGL_LIBRARY}
                      ${QT_QTGUI_LIBRARY}
                      ${QT_QTCORE_LIBRARY}
                      ${OPENGL_gl_LIBRARY}
                      ${OPENGL_glu_LIBRARY})
if(${PCL_FOUND})
  add_library(cloud_saver SHARED cloud_saver.cpp)
  target_link_libraries(cloud_saver ${PCL_LIBRARIES} ${OpenCV_LIBS})
endif()
