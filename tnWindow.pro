greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TEMPLATE      = app
HEADERS       = tnGLWidget.h \
                tnWindow.h \
                Utility.h \
                Vector.h \
                Matrix.h
SOURCES       = tnGLWidget.cpp \
                main.cpp \
                tnWindow.cpp
                Utility.cpp \
                Vector.cpp \
                Matrix.cpp
RESOURCES     = tnWindow.qrc
FORMS         = tnWindow.ui
QT           += opengl


