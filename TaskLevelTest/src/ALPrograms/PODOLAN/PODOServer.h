#ifndef PODOSERVER_H
#define PODOSERVER_H

#include <LAN/RBTCPServer.h>
#include <QDataStream>
#include "../../../share/Headers/LANData/RBLANData.h"
#include "../../../share/Headers/LANData/ROSLANData.h"
#include "../../../share/Headers/LANData/VisionLANData.h"

typedef struct{
    int             STATE_COMMAND;
    char            STATE_PARA_CHAR[10];
    int             STATE_PARA_INT[10];
    float           STATE_PARA_FLOAT[10];
} MachineReturnData;


class PODO_GUI_Server : public RBTCPServer
{
    Q_OBJECT
public:
    PODO_GUI_Server();

    QByteArrays dataReceived;
    QByteArray  RBData;

protected slots:
    virtual void    RBReadData();

private:
    int         dataSize;
};


class PODO_ROS_Server : public RBTCPServer
{
    Q_OBJECT
public:
    PODO_ROS_Server();

    QByteArrays dataReceived;
    QByteArray  RBData;

protected slots:
    virtual void    RBReadData();

private:
    int         dataSize;
};

class PODO_VISION_Server : public RBTCPServer
{
    Q_OBJECT
public:
    PODO_VISION_Server();

    QByteArrays dataReceived;
    QByteArray  RBData;

protected slots:
    virtual void    RBReadData();

private:
    int         dataSize;
};


class DaemonServerStateMachine : public RBTCPServer
{
    Q_OBJECT
public:
    DaemonServerStateMachine();

protected slots:
    virtual void    RBReadData();

private:
    int         dataSize;
public:
    QByteArrays dataReceived;
    QByteArray  RBData;
};



#endif // PODOSERVER_H
