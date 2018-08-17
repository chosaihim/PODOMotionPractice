TEMPLATE = app
CONFIG  += console
CONFIG  -= app_bundle

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

LIBS    += \
    -lalchemy \
    -lcopperplate \
    -lcobalt \
    -lpthread \
    -lrt \
    -lpcan\
   -L../../../share/Libs       -lKINE_DRC_HUBO2 \
   -L../../../share/Libs	-lik_math2 \

SOURCES +=main.cpp\
    joint.cpp \
    taskmotion.cpp \
    BasicTrajectory.cpp \
    ManualCAN.cpp   \

HEADERS +=taskmotion.h \
    taskGeneral.h \
    BasicTrajectory.h \
    BasicMatrix.h \
    BasicMath.h \
    ManualMove.h \
    liftbox.h \
    ManualCAN.h \
    ../../__Daemon/RBLog.h
