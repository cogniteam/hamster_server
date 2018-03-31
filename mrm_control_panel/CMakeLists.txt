cmake_minimum_required(VERSION 2.8.3)

project(mrm_control_panel)

MACRO (QT4_WRAP_UI2 outfiles )
  QT5_EXTRACT_OPTIONS(ui_files ui_options ${ARGN})

  FOREACH (it ${ui_files})
    GET_FILENAME_COMPONENT(outfile ${it} NAME_WE)
    GET_FILENAME_COMPONENT(infile ${it} ABSOLUTE)
    SET(outfile ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/${outfile}.h)
    ADD_CUSTOM_COMMAND(OUTPUT ${outfile}
      COMMAND ${QT_UIC_EXECUTABLE}
      ARGS ${ui_options} -o ${outfile} ${infile}
      MAIN_DEPENDENCY ${infile})
    SET(${outfiles} ${${outfiles}} ${outfile})
  ENDFOREACH (it)

ENDMACRO (QT4_WRAP_UI2)


find_package(catkin REQUIRED COMPONENTS 
    rqt_gui 
    rqt_gui_cpp 
    tf 
    image_transport
    ackermann_msgs)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

find_package(Boost COMPONENTS system thread REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

set(mrm_control_panel_SRCS
    src/mrm_control_panel/ControlPlugin.cpp
    src/mrm_control_panel/ControlWidget.cpp
    src/mrm_control_panel/RobotWidget.cpp
    src/mrm_control_panel/VideoWidget.cpp
    src/mrm_control_panel/MapWidget.cpp
    
    src/mrm_control_panel/TeleoperatorWidget.cpp
    src/mrm_control_panel/TeleoperatorInstanceWidget.cpp
    
    src/mrm_control_panel/MapGraphicsView.cpp
    src/mrm_control_panel/MapGraphicsItem.cpp
    src/mrm_control_panel/RobotGraphicsItem.cpp
    
    src/mrm_control_panel/RobotSelectionManager.cpp
    
    src/mrm_control_panel/Parameters.cpp
)

set(mrm_control_panel_HDRS
    include/mrm_control_panel/ControlWidget.h
    include/mrm_control_panel/RobotWidget.h
    include/mrm_control_panel/VideoWidget.h
    include/mrm_control_panel/MapWidget.h
    
    include/mrm_control_panel/VelocityWidget.h
    include/mrm_control_panel/KeyboardTeleop.h
    include/mrm_control_panel/TeleoperatorWidget.h
    include/mrm_control_panel/TeleoperatorInstanceWidget.h
    
    include/mrm_control_panel/MapGraphicsItem.h
    include/mrm_control_panel/RobotGraphicsItem.h
)

set(mrm_control_panel_UIS
    ui/mrm_robot_widget.ui
    ui/mrm_video_widget.ui
    ui/mrm_map_widget.ui
    ui/mrm_control_widget.ui
    ui/TeleoperatorInstance.ui
    ui/mrm_operator_widget.ui
)

set(mrm_control_panel_INCLUDE_DIRECTORIES
  include ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  INCLUDE_DIRS ${mrm_control_panel_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rqt_gui rqt_gui_cpp cv_bridge image_transport
)

file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)

catkin_python_setup()

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
# QT5_WRAP_CPP(mrm_control_panel_MOCS ${mrm_control_panel_HDRS})

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  QT5_WRAP_CPP(mrm_control_panel_MOCS ${mrm_control_panel_HDRS})
else()
  QT4_WRAP_CPP(mrm_control_panel_MOCS ${mrm_control_panel_HDRS})
endif()

# QT5_WRAP_UI(mrm_control_panel_UIS_H ${mrm_control_panel_UIS})

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
    QT5_WRAP_UI(mrm_control_panel_UIS_H ${mrm_control_panel_UIS})
else()
    QT4_WRAP_UI(mrm_control_panel_UIS_H ${mrm_control_panel_UIS})
endif()

include_directories(${mrm_control_panel_INCLUDE_DIRECTORIES} 
    ${catkin_INCLUDE_DIRS}
    include
)

add_library(clipper
    src/polygons/clipper.cpp
)

target_link_libraries(clipper
    ${catkin_LIBRARIES} 
    ${Boost_LIBRARIES} 
    ${OpenCV_LIBRARIES}
)
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(clipper Qt5::Widgets)
else()
  target_link_libraries(clipper ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()


add_library(${PROJECT_NAME} 
    ${mrm_control_panel_SRCS} 
    ${mrm_control_panel_MOCS} 
    ${mrm_control_panel_UIS_H} 
    ${QT_RESOURCES_CPP}
)

target_link_libraries(${PROJECT_NAME} 
    ${catkin_LIBRARIES} 
    ${QT_QTCORE_LIBRARY} 
    ${QT_QTGUI_LIBRARY}
    ${Boost_LIBRARIES} 
    ${OpenCV_LIBRARIES}
    clipper
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# install(PROGRAMS scripts/rqt_marble
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
