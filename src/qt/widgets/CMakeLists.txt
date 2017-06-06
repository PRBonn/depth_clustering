set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(QT_SOURCES
    base_viewer_widget.cpp
    opengl_folder_player.cpp
    )

set(QT_HEADERS
    base_viewer_widget.h
    opengl_folder_player.h
    )

set(QT_UI_FILES
    ui/opengl_folder_player.ui
    )


message(STATUS "QT LIBS: ${MY_QT_LIBRARIES}")

if (Qt4_FOUND)
  QT4_WRAP_CPP(QT_HEADERS_MOC ${QT_HEADERS})
  QT4_WRAP_UI(QT_UI_FILES_MOC ${QT_UI_FILES})
elseif(Qt5_FOUND)
  QT5_WRAP_CPP(QT_HEADERS_MOC ${QT_HEADERS})
  QT5_WRAP_UI(QT_UI_FILES_MOC ${QT_UI_FILES})
else()
  message(FATAL_ERROR "Qt not found. So cannot moc.")
endif()

add_library(qt_widgets
            ${QT_SOURCES}
            ${QT_UI_FILES_MOC}
            ${QT_HEADERS_MOC})

target_link_libraries(qt_widgets
                      viewer
                      drawable
                      qt_utils
                      identifiable
                      ground_remove
                      image_labeler
                      velodyne_utils
                      folder_reader
                      bbox
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBS}
                      ${QGLVIEWER_LIBRARY}
                      ${MY_QT_LIBRARIES}
                      ${OPENGL_gl_LIBRARY}
                      ${OPENGL_glu_LIBRARY})

