#include "VisionThread.h"


extern pRBCORE_SHM      sharedData;
extern pUSER_SHM        sharedUSER;

VisionThread::VisionThread()
{
}

void VisionThread::run(){

    VisionWorker worker;
    QTimer timerVISION2PODO;


    connect(&timerVISION2PODO, SIGNAL(timeout()), &worker, SLOT(onVISION2PODO()));
    timerVISION2PODO.start(50);

    exec();
}



VisionWorker::VisionWorker(){
    serverPODOVISION = new PODO_VISION_Server();
    serverPODOVISION->RBServerOpen(QHostAddress::AnyIPv4, 5500);

    connect(serverPODOVISION, SIGNAL(SIG_NewConnection()), this, SLOT(onNewConnection()));

}

void VisionWorker::onNewConnection(){

}



void VisionWorker::ReadfromVISION(){
    FILE_LOG(logINFO) << "Read from VISION";
    QByteArray tempData = serverPODOVISION->dataReceived[0];
    serverPODOVISION->dataReceived.pop_front();

    memcpy(&receiveData, tempData.data(), sizeof(receiveData));

    sharedUSER->M2G.V2G.pos[0] = receiveData.pos_x;
    sharedUSER->M2G.V2G.pos[1] = receiveData.pos_y;
    sharedUSER->M2G.V2G.pos[2] = receiveData.pos_z;
    sharedUSER->M2G.V2G.ori[0] = receiveData.ori_w;
    sharedUSER->M2G.V2G.ori[1] = receiveData.ori_x;
    sharedUSER->M2G.V2G.ori[2] = receiveData.ori_y;
    sharedUSER->M2G.V2G.ori[3] = receiveData.ori_z;
}


void VisionWorker::onVISION2PODO(){
    if(serverPODOVISION->dataReceived.size() > 0){
        ReadfromVISION();
    }
}

