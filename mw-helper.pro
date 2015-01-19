TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

#QMAKE_CXXFLAGS += -std=c++11
#QMAKE_CXXFLAGS += -pthread
#QMAKE_CXXFLAGS += -Wl
#QMAKE_CXXFLAGS += --no-as-needed

QMAKE_CFLAGS += -std=c89

INCLUDEPATH += ./include/

SOURCES += \
    test/00init/main.c \
    src/serial.c \
    src/multiwii.c

HEADERS += \
    include/serial.h \
    include/multiwii.h \
    include/dlib.hxx

# LIBS += \
#    -lpthread

#include(deployment.pri)
# qtcAddDeployment()
