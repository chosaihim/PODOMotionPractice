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
        -lpcan \
       -L../../../share/Libs	-lik_math4 \
       -L../../../share/Libs    -lKINE_DRC_HUBO4

SOURCES += main.cpp \
    StateMachine.cpp \
    solver/ldl.cpp \
    solver/matrix_support.cpp \
    solver/solver.cpp \
    solver/util.cpp \
    JoyStick/RBJoystick.cpp \
    ManualCAN.cpp

HEADERS += \
    BasicFiles/BasicSetting.h \
    BasicFiles/BasicJoint.h \
    StateMachine.h \
    Definitions.h \
    solver/solver.h \
    kine_drc_hubo.h \
    JoyStick/joystickvariable.h \
    JoyStick/joystickclass.h \
    ManualCAN.h



unix:!macx: LIBS += -L$$PWD/../../../share/Libs/ -lik_math4

INCLUDEPATH += $$PWD/../../../share/Libs
DEPENDPATH += $$PWD/../../../share/Libs

unix:!macx: PRE_TARGETDEPS += $$PWD/../../../share/Libs/libik_math4.a
