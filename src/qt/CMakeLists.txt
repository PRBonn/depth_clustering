cmake_minimum_required (VERSION 2.8.12 FATAL_ERROR)

add_subdirectory(drawables)
add_subdirectory(viewer)
add_subdirectory(utils)
add_subdirectory(widgets)

add_executable(qt_gui_app main.cpp)
target_link_libraries(qt_gui_app
                      qt_widgets)
