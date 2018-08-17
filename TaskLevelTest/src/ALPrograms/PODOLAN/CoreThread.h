#ifndef CORETHREAD_H
#define CORETHREAD_H

#include <sys/mman.h>
#include <fcntl.h>

#include <QThread>
#include <QByteArray>
#include <QTimer>

#include "PODOServer.h"

#define MACHINE_RETURN_RING_SIZE    20

class CoreThread : public QThread
{
    Q_OBJECT
public:
    CoreThread();

protected:
    void run();
};


class CoreWorker : public QObject
{
    Q_OBJECT
public:
    CoreWorker();

private slots:
    void onPODO2GUI();
    void onGUI2PODO();
    void onCheckTCMD();
    void onPODO2Field();
    void onField2PODO();
    void onVISION2PODO();


private:
    PODO_GUI_Server             *serverPODOGUI;
    DaemonServerStateMachine    *serverMachine;
    PODO_VISION_Server          *serverVISION;

    LAN_PODO2GUI            DATA_PODO;

    void    SendtoGUI();
    void    ReadfromGUI();
    void    ReadfromField();
    void    ReadfromVISION();


    MachineReturnData   MachineReturnRing[MACHINE_RETURN_RING_SIZE];
    int MachineReturnRingFront;
    int MachineReturnRingBack;
};



#endif // CORETHREAD_H
