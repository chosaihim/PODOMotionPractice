#include "CoreThread.h"


extern pRBCORE_SHM      sharedData;
extern pUSER_SHM        sharedUSER;

CoreThread::CoreThread()
{
}

void CoreThread::run(){
    QTimer timerPODO2GUI, timerGUI2PODO, timerTCMD, timerField2PODO, timerPODO2Field, timerVISION2PODO;
    CoreWorker worker;

    connect(&timerPODO2GUI, SIGNAL(timeout()), &worker, SLOT(onPODO2GUI()));
    connect(&timerGUI2PODO, SIGNAL(timeout()), &worker, SLOT(onGUI2PODO()));
    connect(&timerTCMD, SIGNAL(timeout()), &worker, SLOT(onCheckTCMD()));
    connect(&timerPODO2Field, SIGNAL(timeout()), &worker, SLOT(onPODO2Field()));
    connect(&timerField2PODO, SIGNAL(timeout()), &worker, SLOT(onField2PODO()));
    connect(&timerVISION2PODO, SIGNAL(timeout()),&worker, SLOT(onVISION2PODO()));

    timerPODO2GUI.start(50);
    timerGUI2PODO.start(50);
    timerTCMD.start(30);
    timerPODO2Field.start(50);
    timerField2PODO.start(50);
    timerVISION2PODO.start(50);

    exec();
}



CoreWorker::CoreWorker(){
    MachineReturnRingFront = MachineReturnRingBack = 0;

    serverPODOGUI = new PODO_GUI_Server();
    serverPODOGUI->RBServerOpen(QHostAddress::AnyIPv4, 4000);

    serverMachine = new DaemonServerStateMachine();
    serverMachine->RBServerOpen(QHostAddress::AnyIPv4, 4500);

    serverVISION = new PODO_VISION_Server();
    serverVISION->RBServerOpen(QHostAddress::AnyIPv4, 5500);

}

void CoreWorker::ReadfromVISION()
{
    QByteArray tempData = serverVISION->dataReceived[0];
    serverVISION->dataReceived.pop_front();

    KINECT_DATA receiveData;
    memcpy(&receiveData, tempData, sizeof(KINECT_DATA));

    sharedData->V2M.pos[0] = receiveData.pos_x;
    sharedData->V2M.pos[1] = receiveData.pos_y;

    sharedUSER->M2G.V2G.pos[0] = receiveData.pos_x;
    sharedUSER->M2G.V2G.pos[1] = receiveData.pos_y;
    sharedUSER->M2G.V2G.pos[2] = receiveData.pos_z;
    sharedUSER->M2G.V2G.ori[0] = receiveData.ori_w;
    sharedUSER->M2G.V2G.ori[1] = receiveData.ori_x;
    sharedUSER->M2G.V2G.ori[2] = receiveData.ori_y;
    sharedUSER->M2G.V2G.ori[3] = receiveData.ori_z;
}

void CoreWorker::SendtoGUI(){
    memcpy(&(DATA_PODO.CoreData), sharedData, sizeof(RBCORE_SHM));
    memcpy(&(DATA_PODO.UserM2G), &(sharedUSER->M2G), sizeof(MOTION2GUI));

    QByteArray SendData = QByteArray::fromRawData((char*)&DATA_PODO, sizeof(LAN_PODO2GUI));
    serverPODOGUI->RBSendData(SendData);
}

void CoreWorker::ReadfromGUI(){
    QByteArray tempData = serverPODOGUI->dataReceived[0];
    serverPODOGUI->dataReceived.pop_front();

    LAN_GUI2PODO tempDATA;
    memcpy(&tempDATA, tempData, sizeof(LAN_GUI2PODO));
    memcpy(&(sharedUSER->G2M), &(tempDATA.UserG2M), sizeof(GUI2MOTION));

    int target = tempDATA.UserCMD.COMMAND_TARGET;
    for(int i=0; i<MAX_COMMAND_DATA; i++){
        sharedData->COMMAND[target].USER_PARA_CHAR[i]    = tempDATA.UserCMD.COMMAND_DATA.USER_PARA_CHAR[i];
        sharedData->COMMAND[target].USER_PARA_INT[i]     = tempDATA.UserCMD.COMMAND_DATA.USER_PARA_INT[i];
        sharedData->COMMAND[target].USER_PARA_FLOAT[i]   = tempDATA.UserCMD.COMMAND_DATA.USER_PARA_FLOAT[i];
        sharedData->COMMAND[target].USER_PARA_DOUBLE[i]  = tempDATA.UserCMD.COMMAND_DATA.USER_PARA_DOUBLE[i];
    }
    sharedData->COMMAND[target].USER_COMMAND = tempDATA.UserCMD.COMMAND_DATA.USER_COMMAND;
    //printf("READ from GUI target = %d\n",tempDATA.UserCMD.COMMAND_TARGET);
}


void CoreWorker::ReadfromField(){
    QByteArray tempData = serverMachine->dataReceived[0];
    serverMachine->dataReceived.pop_front();

    USER_COMMAND tempDATA;
    memcpy(&tempDATA, tempData, sizeof(USER_COMMAND));

    int target = tempDATA.COMMAND_TARGET;
    for(int i=0; i<MAX_COMMAND_DATA; i++){
        sharedData->COMMAND[target].USER_PARA_CHAR[i]    = tempDATA.COMMAND_DATA.USER_PARA_CHAR[i];
        sharedData->COMMAND[target].USER_PARA_INT[i]     = tempDATA.COMMAND_DATA.USER_PARA_INT[i];
        sharedData->COMMAND[target].USER_PARA_FLOAT[i]   = tempDATA.COMMAND_DATA.USER_PARA_FLOAT[i];
        sharedData->COMMAND[target].USER_PARA_DOUBLE[i]  = tempDATA.COMMAND_DATA.USER_PARA_DOUBLE[i];
    }
    sharedData->COMMAND[target].USER_COMMAND = tempDATA.COMMAND_DATA.USER_COMMAND;
}

void CoreWorker::onPODO2GUI(){
    if(serverPODOGUI->RBConnectionState == RBLAN_CS_CONNECTED){
        SendtoGUI();
    }
}

void CoreWorker::onGUI2PODO(){
    if(serverPODOGUI->dataReceived.size() > 0){
        ReadfromGUI();
    }
}

void CoreWorker::onVISION2PODO()
{
    if(serverVISION->dataReceived.size() > 0)
    {
        ReadfromVISION();
    }
}

void CoreWorker::onCheckTCMD(){
    if(serverMachine->RBConnectionState == RBLAN_CS_CONNECTED){
        if(sharedData->STATE_COMMAND != -1){
            int tempIndex = (MachineReturnRingFront+1)%MACHINE_RETURN_RING_SIZE;
            if(tempIndex == MachineReturnRingBack){
                FILE_LOG(logWARNING) << "Ring Overflow..";
                return;
            }else{
                FILE_LOG(logSUCCESS) << "State Command Push: " << sharedData->STATE_COMMAND;
                MachineReturnRing[MachineReturnRingFront].STATE_COMMAND = sharedData->STATE_COMMAND;
                for(int i=0; i<10; i++){
                    MachineReturnRing[MachineReturnRingFront].STATE_PARA_CHAR[i] = sharedData->STATE_PARA_CHAR[i];
                    MachineReturnRing[MachineReturnRingFront].STATE_PARA_INT[i] = sharedData->STATE_PARA_INT[i];
                    MachineReturnRing[MachineReturnRingFront].STATE_PARA_FLOAT[i] = sharedData->STATE_PARA_FLOAT[i];
                }
                MachineReturnRingFront = tempIndex;
            }
            sharedData->STATE_COMMAND = -1;
        }
    }
}

void CoreWorker::onPODO2Field(){
    if(serverMachine->RBConnectionState == RBLAN_CS_CONNECTED){
        if(MachineReturnRingBack != MachineReturnRingFront){
            MachineReturnData tempData;
            tempData.STATE_COMMAND = MachineReturnRing[MachineReturnRingBack].STATE_COMMAND;
            for(int i=0; i<10; i++){
                tempData.STATE_PARA_CHAR[i] = MachineReturnRing[MachineReturnRingBack].STATE_PARA_CHAR[i];
                tempData.STATE_PARA_INT[i] = MachineReturnRing[MachineReturnRingBack].STATE_PARA_INT[i];
                tempData.STATE_PARA_FLOAT[i] = MachineReturnRing[MachineReturnRingBack].STATE_PARA_FLOAT[i];
            }
            FILE_LOG(logWARNING) << "State Command Send: " << tempData.STATE_COMMAND;
            QByteArray SendData = QByteArray::fromRawData((char*)&tempData, sizeof(MachineReturnData));
            serverMachine->RBSendData(SendData);
            MachineReturnRingBack = (MachineReturnRingBack+1)%MACHINE_RETURN_RING_SIZE;
        }
    }
}

void CoreWorker::onField2PODO(){
    if(serverMachine->dataReceived.size() > 0){
        ReadfromField();
    }
}
