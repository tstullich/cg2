CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

SOURCE_GROUP( "Header Files" REGULAR_EXPRESSION "$^" )
SOURCE_GROUP( "Source Files" REGULAR_EXPRESSION "$^" )
SOURCE_GROUP( "Generated" REGULAR_EXPRESSION "ui_|qrc_|moc_.+cpp$" )

FIND_PACKAGE( OpenGL )
FIND_PACKAGE( GLUT )
FIND_PATH( GLM_INCLUDE_DIR glm/glm.hpp HINTS /usr/include /usr/local/include /opt/local/include )
SET( Qt5Widgets_DIR "/usr/local/opt/qt5/lib/cmake/Qt5Widgets" )
FIND_PACKAGE( Qt5Widgets )
INCLUDE_DIRECTORIES( ${GLM_INCLUDE_DIR} ${GLUT_INCLUDE_DIR} )

SET( app_SOURCES main.cpp QGLViewerWidget.cpp )
SET( app_HEADERS QGLViewerWidget.h )
SET( app_QT_HEADERS QGLViewerWidget.h )
QT5_WRAP_CPP( app_HEADERS_MOC ${app_QT_HEADERS} )

ADD_EXECUTABLE( minqgl ${app_SOURCES} ${app_HEADERS_MOC} ${app_HEADERS} ) 
QT5_USE_MODULES( minqgl Widgets OpenGL )
TARGET_LINK_LIBRARIES( minqgl ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} )