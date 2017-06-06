add_library(viewer SHARED viewer.cpp)
target_link_libraries(viewer
                      drawable
                      ${QGLVIEWER_LIBRARY}
                      ${MY_QT_LIBRARIES}
                      ${OPENGL_gl_LIBRARY}
                      ${OPENGL_glu_LIBRARY})