TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

#QMAKE_CXXFLAGS += -std=c++11
#QMAKE_CXXFLAGS += -pthread
#QMAKE_CXXFLAGS += -Wl
#QMAKE_CXXFLAGS += --no-as-needed

SOURCES += \
     main.cpp \
    serial.cpp \
    multiwii.cpp

HEADERS += \
    serial.h \
    multiwii.h

LIBS += \
    -lpthread

include(deployment.pri)
qtcAddDeployment()
