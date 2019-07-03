TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt
SOURCES += main.cpp

#Libraries
unix: CONFIG += link_pkgconfig

#OpenCV
unix: PKGCONFIG += opencv

##V4L2
unix:!macx: LIBS += -lpthread
unix:!macx: LIBS += -lv4l2

HEADERS += \
    skeleton.h
