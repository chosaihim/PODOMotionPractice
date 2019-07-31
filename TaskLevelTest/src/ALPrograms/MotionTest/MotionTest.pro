QT += core
QT -= gui

QMAKE_CXXFLAGS += -I/usr/include/xenomai/cobalt
QMAKE_CXXFLAGS += -I/usr/include/xenomai
QMAKE_CXXFLAGS += -D_GNU_SOURCE -D_REENTRANT -D__COBALT__

QMAKE_LFLAGS += /usr/lib/xenomai/bootstrap.o -Wl,--wrap=main -Wl,--dynamic-list=/usr/lib/dynlist.ld
QMAKE_LFLAGS += -L/usr/lib -lcobalt -lpthread -lrt


CONFIG += c++11

TARGET = MotionTest
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app


#*********************

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
    -lpcan \
    -L../../../share/Libs	-lik_math4 \
    -L../../../share/Libs    -lKINE_DRC_HUBO4\
    -L../../../share/Libs       -lKINE_DRC_HUBO2 \
    -L../../../share/Libs	-lik_math2


SOURCES += main.cpp \
    StateMachine.cpp \
    solver/ldl.cpp \
    solver/matrix_support.cpp \
    solver/solver.cpp \
    solver/util.cpp \
    ManualCAN.cpp \
    manualwalking.cpp \
    controller.cpp

HEADERS += \
    BasicFiles/BasicSetting.h \
    BasicFiles/BasicJoint.h \
    StateMachine.h \
    solver/solver.h \
    ManualCAN.h \
    Definitions.h \
    BasicFiles/BasicMath.h \
    #approachbox.h \
    motiontest.h