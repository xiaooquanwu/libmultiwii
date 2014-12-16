TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

#QMAKE_CXXFLAGS += -std=c++11
#QMAKE_CXXFLAGS += -pthread
#QMAKE_CXXFLAGS += -Wl
#QMAKE_CXXFLAGS += --no-as-needed

QMAKE_CFLAGS += -std=c89

SOURCES += \
     main.c \
    serial.c \
    multiwii.c

HEADERS += \
    serial.h \
    multiwii.h \
    dlib.hxx

# LIBS += \
#    -lpthread

#include(deployment.pri)
# qtcAddDeployment()
