#ifndef VISIONTHREAD_H
#define VISIONTHREAD_H

#include <sys/mman.h>
#include <fcntl.h>

#include <QThread>
#include <QByteArray>
#include <QTimer>

#include "PODOServer.h"

class VisionThread : public QThread
{
    Q_OBJECT
public:
    VisionThread();

protected:
    void run();
};


class VisionWorker : public QObject
{
    Q_OBJECT
public:
    VisionWorker();

private slots:
    void onNewConnection();

    void onVISION2PODO();

private:
    PODO_VISION_Server      *serverPODOVISION;
    KINECT_DATA             receiveData;

    void    ReadfromVISION();


};


#endif // VISIONTHREAD_H
