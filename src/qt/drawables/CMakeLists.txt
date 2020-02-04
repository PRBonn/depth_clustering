add_library(drawable SHARED
            drawable_cloud.cpp
            object_painter.cpp
            drawable_polygon3d.cpp
            drawable_cube.cpp)
target_link_libraries(drawable
                      ${OpenCV_LIBS}
                      ${OPENGL_gl_LIBRARY}
                      ${OPENGL_glu_LIBRARY})
