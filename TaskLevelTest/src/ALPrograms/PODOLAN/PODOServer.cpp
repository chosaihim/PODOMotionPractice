#include "PODOServer.h"

PODO_GUI_Server::PODO_GUI_Server()
{
    //dataSize = sizeof(USER_COMMAND);
    dataSize = sizeof(LAN_GUI2PODO);
    dataReceived.clear();
    RBData.resize(dataSize);
}

void PODO_GUI_Server::RBReadData(){
    QDataStream in(RBTcpClient);
    in.setVersion(QDataStream::Qt_5_2);
    //printf("available size = %d\n",RBTcpClient->bytesAvailable());

    if(RBTcpClient->bytesAvailable() < dataSize)
        return;

    while(RBTcpClient->bytesAvailable() >= dataSize){
        in.readRawData(RBData.data(), dataSize);
        dataReceived.push_back(RBData);
    }
}


// =========================================

PODO_ROS_Server::PODO_ROS_Server()
{
    dataSize = sizeof(LAN_ROS2PODO);
    dataReceived.clear();
    RBData.resize(dataSize);
}

void PODO_ROS_Server::RBReadData(){
    QDataStream in(RBTcpClient);
    in.setVersion(QDataStream::Qt_5_2);

    if(RBTcpClient->bytesAvailable() < dataSize)
        return;

    while(RBTcpClient->bytesAvailable() >= dataSize){
        in.readRawData(RBData.data(), dataSize);
        dataReceived.push_back(RBData);
    }
}


// =========================================

// =========================================

PODO_VISION_Server::PODO_VISION_Server()
{
    dataSize = sizeof(KINECT_DATA);
    dataReceived.clear();
    RBData.resize(dataSize);
}

void PODO_VISION_Server::RBReadData(){
    QDataStream in(RBTcpClient);
    in.setVersion(QDataStream::Qt_5_2);

    if(RBTcpClient->bytesAvailable() < dataSize)
        return;

    while(RBTcpClient->bytesAvailable() >= dataSize){
        in.readRawData(RBData.data(), dataSize);
        dataReceived.push_back(RBData);
    }
}


// =========================================
DaemonServerStateMachine::DaemonServerStateMachine()
{
    dataSize = sizeof(COMMAND_STRUCT);
    dataReceived.clear();
    RBData.resize(dataSize);
}

void DaemonServerStateMachine::RBReadData(){
    QDataStream in(RBTcpClient);
    in.setVersion(QDataStream::Qt_5_2);

    if(RBTcpClient->bytesAvailable() < dataSize)
        return;

    while(RBTcpClient->bytesAvailable() >= dataSize){
        in.readRawData(RBData.data(), dataSize);
        dataReceived.push_back(RBData);
    }
}
