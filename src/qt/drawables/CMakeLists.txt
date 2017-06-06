add_library(drawable SHARED
            drawable_cloud.cpp
            drawable_cube.cpp)
target_link_libraries(drawable
                      ${OPENGL_gl_LIBRARY}
                      ${OPENGL_glu_LIBRARY})