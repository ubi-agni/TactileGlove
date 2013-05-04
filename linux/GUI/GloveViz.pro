#-------------------------------------------------
#
# Project created by QtCreator 2011-12-22T13:01:52
#
#-------------------------------------------------

QT       += core gui svg opengl xml

TARGET = GloveViz
TEMPLATE = app

INCLUDEPATH+=/usr/include/libxml2 /usr/include/cairo
LIBS+=-L/usr/lib/kde -lkdeui -lcairo -lsvg-cairo -lsvg

SOURCES += main.cpp\
        glovevizmainwindow.cpp \
    svgglview.cpp \
    svgwindow.cpp \
    glovesvgpainter.cpp \
    seriallineconnector.cpp

HEADERS  += glovevizmainwindow.h \
    svgglview.h \
    svgwindow.h \
    glovesvgpainter.h \
    seriallineconnector.h

FORMS    += glovevizmainwindow.ui









