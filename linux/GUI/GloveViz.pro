#-------------------------------------------------
#
# Project created by QtCreator 2011-12-22T13:01:52
#
#-------------------------------------------------

QT       += core gui svg opengl xml

TARGET   = GloveViz
TEMPLATE = app

SOURCES += main.cpp\
        glovevizmainwindow.cpp \
        glovesvgpainter.cpp \
        seriallineconnector.cpp

HEADERS  += glovevizmainwindow.h \
    glovesvgpainter.h \
    seriallineconnector.h

RESOURCES = resources.qrc

FORMS    += glovevizmainwindow.ui









