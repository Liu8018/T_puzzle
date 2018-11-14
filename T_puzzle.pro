TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        main.cpp \
    dstpatternfinder.cpp \
    cornerpoint.cpp \
    t_puzzlesolver.cpp

INCLUDEPATH += usr/include\
               usr/include/opencv \
               usr/include/opencv2
LIBS += /usr/lib64/libopencv_imgproc.so \
        /usr/lib64/libopencv_highgui.so \
        /usr/lib64/libopencv_core.so \
        /usr/lib64/libopencv_imgcodecs.so

HEADERS += \
    dstpatternfinder.h \
    cornerpoint.h \
    t_puzzlesolver.h
