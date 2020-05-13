## Copyright (C) 2020  I. Bogoslavskyi, C. Stachniss
##
## GNU-GPL licence that follows one of libQGLViewer.

add_library(viewer SHARED viewer.cpp)
target_link_libraries(viewer
                      drawable
                      ${QGLVIEWER_LIBRARY}
                      ${MY_QT_LIBRARIES}
                      ${OPENGL_gl_LIBRARY}
                      ${OPENGL_glu_LIBRARY})
