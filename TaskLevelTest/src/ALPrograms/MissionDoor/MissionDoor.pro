#-------------------------------------------------
#
# Project created by QtCreator 2014-02-11T10:41:29
#
#-------------------------------------------------

TEMPLATE = app
CONFIG   += console
CONFIG   -= app_bundle

QMAKE_CXXFLAGS += -I/usr/include/xenomai/cobalt
QMAKE_CXXFLAGS += -I/usr/include/xenomai
QMAKE_CXXFLAGS += -D_GNU_SOURCE -D_REENTRANT -D__COBALT__

QMAKE_LFLAGS += /usr/lib/xenomai/bootstrap.o -Wl,--wrap=main -Wl,--dynamic-list=/usr/lib/dynlist.ld
QMAKE_LFLAGS += -L/usr/lib -lcobalt -lpthread -lrt

INCLUDEPATH += \
    /usr/include/xenomai/cobalt \
    /usr/include/xenomai \
    /usr/include/xenomai/alchemy \
    ../../../share/Headers


LIBS        += \
    -lalchemy \
    -lcopperplate \
    -lcobalt \
    -lpthread \
    -lrt \
    -lpcan \
    -L../../../share/Libs       -lKINE_DRC_HUBO2 \
    -L../../../share/Libs	-lik_math2 \

SOURCES += main.cpp \
    joint.cpp \
    BasicFiles/BasicTrajectory.cpp \
    taskmotion.cpp \
    ManualCAN.cpp

HEADERS += \
    joint.h \
    BasicFiles/BasicMath.h \
    BasicFiles/BasicMatrix.h \
    BasicFiles/BasicTrajectory.h \
    taskGeneral.h \
    taskmotion.h \
    RBLog.h \
    ManualCAN.h \
    MissionDoor.h

