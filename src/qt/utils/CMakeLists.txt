add_library(qt_utils utils.cpp)
target_link_libraries(qt_utils
                      ${MY_QT_LIBRARIES}
                      ${OpenCV_LIBS})
