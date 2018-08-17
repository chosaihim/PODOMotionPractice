#include "RBIMUSensor.h"

#include <QSettings>


using namespace std;
RBIMUSensor::RBIMUSensor()
{
    delX = delY = delZ = ROLL = PITCH = YAW = 0.0;

    Quat_IMU0 = 1.0;
    Quat_IMU1 = 0.0;
    Quat_IMU2 = 0.0;
    Quat_IMU3 = 0.0;

    QString settingFile = "configs/Daemon_IMU_offset.ini";
    QSettings settings(settingFile, QSettings::NativeFormat);

    ROLL_OFFSET = settings.value("roll", "").toFloat();
    PITCH_OFFSET = settings.value("pitch", "").toFloat();
}

void RBIMUSensor::RBIMU_AddCANMailBox(){
    canHandler->RBCAN_AddMailBox(ID_RCV_DATA1);
    canHandler->RBCAN_AddMailBox(ID_RCV_DATA2);
    canHandler->RBCAN_AddMailBox(ID_RCV_INFO);
    canHandler->RBCAN_AddMailBox(ID_RCV_PARA);
    //canHandler->RBCAN_AddMailBox(ID_RCV_STAT);
}

void RBIMUSensor::RBBoard_GetDBData(DB_IMU db){
    BOARD_ID        = db.BOARD_ID;
    BOARD_NAME      = db.BOARD_NAME;
    SENSOR_ID       = db.SENSOR_ID;
    CAN_CHANNEL     = db.CAN_CHANNEL;
    SENSOR_TYPE     = db.SENSOR_TYPE;
    ID_RCV_DATA1    = db.ID_RCV_DATA1;
    ID_RCV_DATA2    = db.ID_RCV_DATA2;
    ID_RCV_STAT     = db.ID_RCV_STAT;
    ID_RCV_INFO     = db.ID_RCV_INFO;
    ID_RCV_PARA     = db.ID_RCV_PARA;
}

int RBIMUSensor::RBBoard_CANCheck(int _canr){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;		// board no.
    mb.data[1] = 0x01;		// command
    mb.data[2] = _canr;	// CAN communication rate(msec)
    mb.dlc = 3;
    mb.id = COMMAND_CANID;

    if( canHandler->RBCAN_WriteData(mb) == true ){
        usleep(15*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_INFO;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA){
            std::cout << ">>> RMIMU: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[32minitialized.\033[0m[ch " << CAN_CHANNEL << "]\n";
            ConnectionStatus = true;
            mb.status = RBCAN_NODATA;
            return true;
        }else{
            std::cout << ">>> RMIMU: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[31mfailed \033[0mto initialize.[ch " << CAN_CHANNEL << "]\n";
            ConnectionStatus = false;
            return false;
        }
    }
    else return false;
}

int RBIMUSensor::RBBoard_RequestStatus(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;		// board no.
    mb.data[1] = 0x02;		// command
    mb.dlc = 2;
    mb.id = COMMAND_CANID;

    return canHandler->RBCAN_WriteData(mb);
}

int RBIMUSensor::RBBoard_LoadDefaultValue(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;			// board no.
    mb.data[1] = 0xFA;			// command
    mb.dlc = 2;
    mb.id = COMMAND_CANID;

    return canHandler->RBCAN_WriteData(mb);
}

int RBIMUSensor::RBIMU_RequestNulling(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;			// board no.
    mb.data[1] = 0x81;			// command
    mb.dlc = 2;
    mb.id = COMMAND_CANID;

    return canHandler->RBCAN_WriteData(mb);
}

int RBIMUSensor::RBIMU_RequestCalibration(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;			// board no.
    mb.data[1] = 0x82;			// command
    mb.dlc = 2;
    mb.id = COMMAND_CANID;

    return canHandler->RBCAN_WriteData(mb);
}

int RBIMUSensor::RBIMU_RequestParameter(unsigned char _prf){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;			// board no.
    mb.data[1] = 0x24;			// command
    mb.data[2] = _prf;		    // parameter request
    // _prf = 0x01 : ACC_X_GAIN  ACC_Y_GAIN  ACC_Z_GAIN
    // _prf = 0x02 : ACC_X_BIAS  ACC_Y_BIAS  ACC_Z_BIAS
    mb.dlc = 3;
    mb.id = COMMAND_CANID;

    return canHandler->RBCAN_WriteData(mb);
}

int RBIMUSensor::RBIMU_SetBoardNumber(int _newbno){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;						// board no.
    mb.data[1] = 0xA8;						// command
    mb.data[2] = _newbno;					// new board number
    mb.dlc = 3;
    mb.id = COMMAND_CANID;

    return canHandler->RBCAN_WriteData(mb);
}

int RBIMUSensor::RBIMU_RequestData(int _type){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = SENSOR_ID;				// Sensor board no. if _sbno = 0xFF : all sensor boards ??
    mb.data[1] = _type;              // Request Angle and Rate data
    mb.data[2] = 0x01;              // Extra value to indicate an IMU sensor, otherwise this CAN message would be
                                    // the same as one of the FT request data messages.
    mb.dlc = 3;
    mb.id = SENSOR_REQUEST_CANID;

    return canHandler->RBCAN_WriteDataDirectly(mb);
}

int RBIMUSensor::RBIMU_ReadData(void){
//    RBCAN_MB mb;
//    RBCAN_MB mb2;

//    // Read IMU Data1
//    mb.channel = CAN_CHANNEL;
//    mb.id = ID_RCV_DATA1;
//    canHandler->RBCAN_ReadData(&mb);
//    if(mb.status != RBCAN_NODATA)
//    {
////        ROLL = (double)((short)((mb.data[1]<<8)|mb.data[0]))/100.0f + ROLL_OFFSET;
////        PITCH = (double)((short)((mb.data[3]<<8)|mb.data[2]))/100.0f + -1.0;//PITCH_OFFSET;
////        ROLL_VEL = (double)((short)((mb.data[5]<<8)|mb.data[4]))/100.0f;
////        PITCH_VEL = (double)((short)((mb.data[7]<<8)|mb.data[6]))/100.0f;
//        //return true;
//        ACC_X = (double)((short)((mb.data[1]<<8)|mb.data[0]))/100.0f;
//        ACC_Y = (double)((short)((mb.data[3]<<8)|mb.data[2]))/100.0f;
//        ACC_Z = (double)((short)((mb.data[5]<<8)|mb.data[4]))/100.0f;
//        //TEMP = (double)((short)((mb.data[7]<<8)|mb.data[6]))/100.0f;

//        mb.status = RBCAN_NODATA;
//    }

////    // Read IMU Data2
////    mb2.channel = CAN_CHANNEL;
////    mb2.id = ID_RCV_DATA2;
////    canHandler->RBCAN_ReadData(&mb2);
////    if(mb2.status != RBCAN_NODATA){
////        ACC_X = (double)((short)((mb2.data[1]<<8)|mb2.data[0]))/100.0f + ROLL_OFFSET;//FOG Edit
////        ACC_Y = (double)((short)((mb2.data[3]<<8)|mb2.data[2]))/100.0f + -1.0;//PITCH_OFFSET;//FOG Edit
////        ACC_Z = (double)((short)((mb2.data[5]<<8)|mb2.data[4]))/100.0f;
////        YAW_VEL = (double)((short)((mb2.data[7]<<8)|mb2.data[6]))/100.0f;
////        //return true;//FOG Edit
////        mb.status = RBCAN_NODATA;
////    }
//    return true;



    RBCAN_MB mb;
    RBCAN_MB mb2;

    // Read IMU Data1
    mb.channel = CAN_CHANNEL;
    mb.id = ID_RCV_DATA1;
    canHandler->RBCAN_ReadData(&mb);
    if(mb.status != RBCAN_NODATA)
    {
        ROLL = (double)((short)((mb.data[1]<<8)|mb.data[0]))/100.0f;
        PITCH = (double)((short)((mb.data[3]<<8)|mb.data[2]))/100.0f;
        ROLL_VEL = (double)((short)((mb.data[5]<<8)|mb.data[4]))/100.0f;
        PITCH_VEL = (double)((short)((mb.data[7]<<8)|mb.data[6]))/100.0f;
        //return RBIMU_SUCCESS;
        mb.status = RBCAN_NODATA;
    }

    // Read IMU Data2
    mb2.channel = CAN_CHANNEL;
    mb2.id = ID_RCV_DATA2;
    canHandler->RBCAN_ReadData(&mb2);
    if(mb2.status != RBCAN_NODATA)
    {
        // ACC offset
        // Robot 1
        // X >> -0.8
        // Y >> -0.4
        // Robot 2
        // X >> 0.0
        // Y >> 0.0
        ACC_X = (double)((short)((mb2.data[1]<<8)|mb2.data[0]))/100.0f + ROLL_OFFSET;//FOG Edit
        ACC_Y = (double)((short)((mb2.data[3]<<8)|mb2.data[2]))/100.0f + PITCH_OFFSET;//FOG Edit
        ACC_Z = (double)((short)((mb2.data[5]<<8)|mb2.data[4]))/100.0f;
        YAW_VEL = (double)((short)((mb2.data[7]<<8)|mb2.data[6]))/100.0f;
        //return RBIMU_SUCCESS;//FOG Edit
        mb.status = RBCAN_NODATA;
    }

    float R2D = 57.2957802f;
    float D2R = 0.0174533f;

    double velX = ROLL_VEL*D2R;
    double velY = PITCH_VEL*D2R;
    double velZ = YAW_VEL*D2R;

    delX = velX + velY*sin(ROLL*D2R)*tan(PITCH*D2R) + velZ*cos(ROLL*D2R)*tan(PITCH*D2R);
    delY = velY*cos(ROLL*D2R) - velZ*sin(ROLL*D2R);
    delZ = velY*sin(ROLL*D2R)/(cos(PITCH*D2R)) + velZ*cos(ROLL*D2R)/(cos(PITCH*D2R)); // it does not return

    if(isnan(delX)) delX = 0;
    if(isnan(delY)) delY = 0;
    if(isnan(delZ)) delZ = 0;

    double l = sqrt(velX*velX + velY*velY + velZ*velZ);
    double p[4] = {cos(0.005*l/2.0),velX*sin(0.005*l/2.0)/l,velY*sin(0.005*l/2.0)/l,velZ*sin(0.005*l/2.0)/l};
    double Q[4];
    for(int i=0;i<4;i++)
    {
        if(isnan(p[i]))
        {
            p[0] = 1; p[1] = 0; p[2] = 0; p[3] = 0;
            std::cout << "p -isnan" << endl;
            break;
        }
    }
    if(isnan(Quat_IMU0)||isnan(Quat_IMU1)||isnan(Quat_IMU2)||isnan(Quat_IMU3)){
        Quat_IMU0 = 1; Quat_IMU1 = 0; Quat_IMU2 = 0; Quat_IMU3 = 0;
        std::cout<<"Q - isnan"<<std::endl;
    }

    Q[0] = Quat_IMU0*p[0] - Quat_IMU1*p[1] - Quat_IMU2*p[2] - Quat_IMU3*p[3];//hamiltonian form
    Q[1] = Quat_IMU0*p[1] + Quat_IMU1*p[0] + Quat_IMU2*p[3] - Quat_IMU3*p[2];
    Q[2] = Quat_IMU0*p[2] - Quat_IMU1*p[3] + Quat_IMU2*p[0] + Quat_IMU3*p[1];
    Q[3] = Quat_IMU0*p[3] + Quat_IMU1*p[2] - Quat_IMU2*p[1] + Quat_IMU3*p[0];

//    Q[0] = Quat_IMU0*p[0] - Quat_IMU1*p[1] - Quat_IMU2*p[2] - Quat_IMU3*p[3];//JPL form
//    Q[1] = Quat_IMU0*p[1] + Quat_IMU1*p[0] - Quat_IMU2*p[3] + Quat_IMU3*p[2];
//    Q[2] = Quat_IMU0*p[2] + Quat_IMU1*p[3] + Quat_IMU2*p[0] - Quat_IMU3*p[1];
//    Q[3] = Quat_IMU0*p[3] - Quat_IMU1*p[2] + Quat_IMU2*p[1] + Quat_IMU3*p[0];

    double ll = sqrt(Q[0]*Q[0] + Q[1]*Q[1] + Q[2]*Q[2] + Q[3]*Q[3]);

    Quat_IMU0 = Q[0]/ll;
    Quat_IMU1 = Q[1]/ll;
    Quat_IMU2 = Q[2]/ll;
    Quat_IMU3 = Q[3]/ll;

    G_ROLL_VEL = delX;
    G_PITCH_VEL = delX;
    G_YAW_VEL = delX;

    // Quaternion Integration -> Global Euler ZYX angle (Yaw Roll Pitch)
    YAW   = R2D*atan2(2.0*(Q[1]*Q[2] + Q[0]*Q[3]),Q[0]*Q[0] + Q[1]*Q[1] - Q[2]*Q[2] - Q[3]*Q[3]);//Q[1];//delX*0.002;//0.001;
    PITCH = R2D*asin(-2.0*(Q[1]*Q[3] - Q[0]*Q[2]));//Q[2];//delY*0.002;//0.001;
    ROLL  = R2D*atan2(2.0*(Q[2]*Q[3] + Q[0]*Q[1]) , Q[0]*Q[0] - Q[1]*Q[1] - Q[2]*Q[2] + Q[3]*Q[3]);//Q[3];//delZ*0.002;//0.001;



    Quat_IMU0 = Q[0]/ll;
    Quat_IMU1 = Q[1]/ll;
    Quat_IMU2 = Q[2]/ll;
    Quat_IMU3 = Q[3]/ll;

    L_ROLL_VEL = velX;
    L_PITCH_VEL = velY;
    L_YAW_VEL = velZ;

    IMU_ROLL_NULL   = ROLL;
    IMU_PITCH_NULL  = PITCH;
    IMU_YAW_NULL    = YAW;


    return true;
}
