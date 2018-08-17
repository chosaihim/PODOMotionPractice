#include <iostream>
#include <sys/mman.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>

#include <QSettings>

#include "RBCAN.h"
#include "RBRawLAN.h"
#include "RBDataBase.h"
#include "../../share/Headers/LANData/GazeboLANData.h"

#include "RBProcessManager.h"

#include "RBMotorController.h"
#include "RBFTSensor.h"
#include "RBIMUSensor.h"
#include "RBSmartPower.h"
#include "RBOpticFlowSensor.h"
#include "RBFOGSensor.h"



using namespace std;


QString     settingFile;


// Basic --------
int     IS_WORKING = false;
int     IS_CHILD = false;
int     IS_CAN_OK = false;
int     IS_RS232_OK = false;

pRBCORE_SHM             sharedData;
pRBCAN                  canHandler;
pRBLAN                  lanHandler;
RBProcessManager        *pmHandler;
OpticalDisplacement     ODHandler;
int *GAZEBO_FLAG;

// Daemon Options
int     __IS_GAZEBO = false;
int     __IS_FOG = false;
int     __IS_ROS = true;
int     __IS_EXF_R = false;     // extra finger right
int     __IS_EXF_L = false;     // extra finger left
float   EXF_R_Modifier[5] = {0.0, };
float   EXF_L_Modifier[5] = {0.0, };


// Initialize --------
int     RBCore_Initialize();
int     RBCore_SMInitialize();
int     RBCore_DBInitialize();
int     RBCore_CANInitialize();
int     RBCore_LANInitialize();
int     RBCore_ThreadInitialize();
int     RBCore_PMInitialize();
int     RBCore_Termination();
void    RBCore_RTThreadCon(void *);
void    *RBCore_NRTThreadCon(void *);
RT_TASK rtTaskCon;
ulong   nrtTaskCon;

// Database --------
DB_GENERAL      RBDataBase::_DB_GENERAL;
DB_MC           RBDataBase::_DB_MC[MAX_MC];
DB_FT           RBDataBase::_DB_FT[MAX_FT];
DB_IMU          RBDataBase::_DB_IMU[MAX_IMU];
DB_SP           RBDataBase::_DB_SP[MAX_SP];
DB_OF           RBDataBase::_DB_OF[MAX_OF];
DB_AL           RBDataBase::_DB_AL[MAX_AL];

int     _VERSION;
int     _NO_OF_AL;
int     _NO_OF_COMM_CH;
int     _NO_OF_MC;
int     _NO_OF_FT;
int     _NO_OF_IMU;
int     _NO_OF_SP;
int     _NO_OF_OF;


// Devices --------
RBMotorController   _DEV_MC[MAX_MC];
RBFTSensor          _DEV_FT[MAX_FT];
RBIMUSensor         _DEV_IMU[MAX_IMU];
RBSmartPower        _DEV_SP[MAX_SP];
RBOpticFlowSensor   _DEV_OF[MAX_OF];
RBFOGSensor         _DEV_FOG;

int _CANOUT_ENABLED = false;
int _ENCODER_ENABLED = false;
int _SENSOR_ENABLED = false;
int _REFERENCE_ENABLED = false;


int _ALCommandCnt[MAX_AL] = {0,};

long _ThreadCnt = 0;

bool StatusReadFlag[MAX_MC] = {0,};
bool ErrorClearStart = false;

// Thread Functions
void    THREAD_ReadSensor();
void    THREAD_ReadEncoder();
void    THREAD_ReadTemperature();
void    THREAD_ReadVoltage();
void    THREAD_ReadHomeError();

void    THREAD_RequestSensor();
void    THREAD_RequestEncoder();
void    THREAD_RequestTemperature();
void    THREAD_RequestVoltage();

// Command Functions
void    RBCMD_InitCheckDevice();
void    RBCMD_InitWheel();
void    RBCMD_InitFindHome();
void    RBCMD_InitFetOnOff();
void    RBCMD_InitControlOnOff();
void    RBCMD_InitSetFingerModifier();

void    RBCMD_AttrSwitchingMode();
void    RBCMD_AttrFrictionCompensation();
void    RBCMD_AttrControlMode();

void    RBCMD_SensorEncoderReset();
void    RBCMD_SensorEncoderOnOff();
void    RBCMD_SensorSensorOnOff();
void    RBCMD_SensorFTNull();
void    RBCMD_SensorIMUNull();
void    RBCMD_SensorIMUOffsetSet();
void    RBCMD_SensorFOGNull();
void    RBCMD_SensorFOGUSBReset();
void    RBCMD_SensorOFNull();
void    RBCMD_SensorOFLampOnOff();

void    RBCMD_MotionRefOnOff();
void    RBCMD_MotionMove();
void    RBCMD_MotionGainOverride();
void    RBCMD_MotionErrorClear();

void    RBCMD_CANEnableDisable();


// Debugging data
#define ROW_data_debug 6000
#define COL_data_debug 50
float   JW_Data_Debug[COL_data_debug][ROW_data_debug];
int sJW_Data_Debug_Index = 0;
FILE *fpDebug;

void JW_save()
{
    if(sJW_Data_Debug_Index < ROW_data_debug)
    {
        JW_Data_Debug[0][sJW_Data_Debug_Index] = _DEV_MC[MC_ID_CH_Pairs[REB].id].Joints[MC_ID_CH_Pairs[REB].ch].Reference;
        JW_Data_Debug[1][sJW_Data_Debug_Index] = _DEV_MC[MC_ID_CH_Pairs[REB].id].Joints[MC_ID_CH_Pairs[REB].ch].CurrentPosition;
        JW_Data_Debug[2][sJW_Data_Debug_Index] = _DEV_MC[MC_ID_CH_Pairs[REB].id].Joints[MC_ID_CH_Pairs[REB].ch].CurrentStatus.B[1];

        JW_Data_Debug[4][sJW_Data_Debug_Index] = _DEV_MC[MC_ID_CH_Pairs[LEB].id].Joints[MC_ID_CH_Pairs[LEB].ch].Reference;
        JW_Data_Debug[5][sJW_Data_Debug_Index] = _DEV_MC[MC_ID_CH_Pairs[LEB].id].Joints[MC_ID_CH_Pairs[LEB].ch].CurrentPosition;
        JW_Data_Debug[6][sJW_Data_Debug_Index] = _DEV_MC[MC_ID_CH_Pairs[LEB].id].Joints[MC_ID_CH_Pairs[LEB].ch].CurrentStatus.B[1];

        JW_Data_Debug[8][sJW_Data_Debug_Index] = _DEV_MC[MC_ID_CH_Pairs[RWP].id].Joints[MC_ID_CH_Pairs[RWP].ch].Reference;
        JW_Data_Debug[9][sJW_Data_Debug_Index] = _DEV_MC[MC_ID_CH_Pairs[RWP].id].Joints[MC_ID_CH_Pairs[RWP].ch].CurrentPosition;
        JW_Data_Debug[10][sJW_Data_Debug_Index] = _DEV_MC[MC_ID_CH_Pairs[RWP].id].Joints[MC_ID_CH_Pairs[RWP].ch].CurrentStatus.B[1];

        JW_Data_Debug[12][sJW_Data_Debug_Index] = _DEV_MC[MC_ID_CH_Pairs[LWP].id].Joints[MC_ID_CH_Pairs[LWP].ch].Reference;
        JW_Data_Debug[13][sJW_Data_Debug_Index] = _DEV_MC[MC_ID_CH_Pairs[LWP].id].Joints[MC_ID_CH_Pairs[LWP].ch].CurrentPosition;
        JW_Data_Debug[14][sJW_Data_Debug_Index] = _DEV_MC[MC_ID_CH_Pairs[LWP].id].Joints[MC_ID_CH_Pairs[LWP].ch].CurrentStatus.B[1];

        sJW_Data_Debug_Index++;
        if(sJW_Data_Debug_Index >= ROW_data_debug) sJW_Data_Debug_Index = 0;
    }
}



void CatchSignals(int _signal)
{
    switch(_signal)
    {
    case SIGHUP:     // shell termination
    case SIGINT:     // Ctrl-c
    case SIGTERM:    // "kill" from shell
    case SIGKILL:
    case SIGSEGV:
        if(__IS_GAZEBO)
            lanHandler->RBLanClose();
        else
            canHandler->Finish();
        usleep(1000*1000);

        for(int i=1; i<_NO_OF_AL; i++){
            pmHandler->CloseAL(i);
        }
        IS_WORKING = false;
        break;
    }
    usleep(1000*1000);
}

void CheckArguments(int argc, char *argv[]){
    int opt = 0;
    int extrafinger = 0;
    while((opt = getopt(argc, argv, "g:f:r:e:")) != -1){
        switch(opt){
        case 'g':
            if(strcmp(optarg, "true")==0 || strcmp(optarg, "TRUE")==0){
                __IS_GAZEBO = true;
            }else if(strcmp(optarg, "false")==0 || strcmp(optarg, "FALSE")==0){
                __IS_GAZEBO = false;
            }else{
                FILE_LOG(logERROR) << opt;
                FILE_LOG(logERROR) << "Invalid option for Gazebo";
                FILE_LOG(logERROR) << "Valid options are \"true\", \"TRUE\", \"false\", \"FALSE\"";
                FILE_LOG(logERROR) << "Use default value";
            }
            break;
        case 'f':
            if(strcmp(optarg, "true")==0 || strcmp(optarg, "TRUE")==0){
                __IS_FOG = true;
            }else if(strcmp(optarg, "false")==0 || strcmp(optarg, "FALSE")==0){
                __IS_FOG = false;
            }else{
                FILE_LOG(logERROR) << opt;
                FILE_LOG(logERROR) << "Invalid option for FOG";
                FILE_LOG(logERROR) << "Valid options are \"true\", \"TRUE\", \"false\", \"FALSE\"";
                FILE_LOG(logERROR) << "Use default value";
            }
            break;
        case 'r':
            if(strcmp(optarg, "true")==0 || strcmp(optarg, "TRUE")==0){
                __IS_ROS = true;
            }else if(strcmp(optarg, "false")==0 || strcmp(optarg, "FALSE")==0){
                __IS_ROS = false;
            }else{
                FILE_LOG(logERROR) << opt;
                FILE_LOG(logERROR) << "Invalid option for ROS";
                FILE_LOG(logERROR) << "Valid options are \"true\", \"TRUE\", \"false\", \"FALSE\"";
                FILE_LOG(logERROR) << "Use default value";
            }
            break;
        case 'e':
            extrafinger = atoi(optarg);
            if(extrafinger & 0x01 == 0x01)      __IS_EXF_R = true;
            else                                __IS_EXF_R = false;
            if((extrafinger>>1) & 0x01 == 0x01) __IS_EXF_L = true;
            else                                __IS_EXF_L = false;
            break;
        case '?':
            if(optopt == 'g'){
                FILE_LOG(logERROR) << "Option for Gazebo";
                FILE_LOG(logERROR) << "Valid options are \"true\", \"TRUE\", \"false\", \"FALSE\"";
            }else if(optopt == 'f'){
                FILE_LOG(logERROR) << "Option for FOG";
                FILE_LOG(logERROR) << "Valid options are \"true\", \"TRUE\", \"false\", \"FALSE\"";
            }
        }
    }


    cout << endl;
    FILE_LOG(logERROR) << "=========Daemon Setting==========";
    FILE_LOG(logWARNING) << argv[0];
    if(__IS_GAZEBO)     FILE_LOG(logWARNING) << "Daemon for Gazebo";
    else                FILE_LOG(logWARNING) << "Daemon for Robot";
    if(__IS_FOG)        FILE_LOG(logWARNING) << "FOG is used";
    else                FILE_LOG(logWARNING) << "FOG is not used";
    if(__IS_ROS)        FILE_LOG(logWARNING) << "ROS is used";
    else                FILE_LOG(logWARNING) << "ROS is not used";
    if(__IS_EXF_R)      FILE_LOG(logWARNING) << "Extra Right Finger is used";
    if(__IS_EXF_L)      FILE_LOG(logWARNING) << "Extra Left Finger is used";
    FILE_LOG(logERROR) << "=================================";
    cout << endl;
}

int main(int argc, char *argv[])
{
    // Copyright
    cout << endl;
    cout << " \033[31m######################################################################\n";
    cout << " #                                                                    #\n";
    cout << " #  PODO Version 2.2                                                  #\n";
    cout << " #  Copyright 2016 Rainbow Robotics Co.                               #\n";
    cout << " #                                                                    #\n";
    cout << " #  Main developer: Jeongsoo Lim                                      #\n";
    cout << " #  E-mail: yjs0497@kaist.ac.kr                                       #\n";
    cout << " #                                                                    #\n";
    cout << " #  We touch the core!                                                #\n";
    cout << " #                                                                    #\n";
    cout << " ######################################################################\n";

    // Termination signal
    signal(SIGTERM, CatchSignals);       // "kill" from shell
    signal(SIGINT,  CatchSignals);       // Ctrl-c
    signal(SIGHUP,  CatchSignals);       // shell termination
    signal(SIGKILL, CatchSignals);
    signal(SIGSEGV, CatchSignals);

    // Block memory swapping
    mlockall(MCL_CURRENT|MCL_FUTURE);

    CheckArguments(argc, argv);

    if(RBCore_Initialize() == false){
        FILE_LOG(logERROR) << "Core Initialization Failed..";
        return 0;
    }


    // set EXF setting in shared memory
    sharedData->EXF_R_Enabled = __IS_EXF_R;
    sharedData->EXF_L_Enabled = __IS_EXF_L;

    settingFile = "configs/FingerConfig.ini";
    QSettings settings(settingFile, QSettings::NativeFormat);
    if(__IS_EXF_R){
        EXF_R_Modifier[0] = settings.value("exfr0", "").toFloat();
        EXF_R_Modifier[1] = settings.value("exfr1", "").toFloat();
        EXF_R_Modifier[2] = settings.value("exfr2", "").toFloat();
        EXF_R_Modifier[3] = settings.value("exfr3", "").toFloat();
        EXF_R_Modifier[4] = settings.value("exfr4", "").toFloat();
    }
    if(__IS_EXF_L){
        EXF_L_Modifier[0] = settings.value("exfl0", "").toFloat();
        EXF_L_Modifier[1] = settings.value("exfl1", "").toFloat();
        EXF_L_Modifier[2] = settings.value("exfl2", "").toFloat();
        EXF_L_Modifier[3] = settings.value("exfl3", "").toFloat();
        EXF_L_Modifier[4] = settings.value("exfl4", "").toFloat();
    }

    // set defalut 1.0 value if it is not set yet
    for(int i=0; i<5; i++){
        if(EXF_R_Modifier[i] <= 0.01){
            settings.setValue(QString().sprintf("exfr%d", i), 1.0);
            EXF_R_Modifier[i] = 1.0;
        }
        if(EXF_L_Modifier[i] <= 0.01){
            settings.setValue(QString().sprintf("exfl%d", i), 1.0);
            EXF_L_Modifier[i] = 1.0;
        }
    }

    for(int i=0; i<5; i++){
        sharedData->EXF_R_Modifier[i] = EXF_R_Modifier[i];
        sharedData->EXF_L_Modifier[i] = EXF_L_Modifier[i];
    }



    while(IS_WORKING){
        usleep(100*1000);

        switch(sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND){
        case DAEMON_PROCESS_CREATE:
            FILE_LOG(logINFO) << "CMD: DAEMON_PROCESS_CREATE";
            pmHandler->OpenAL(sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_INT[0]);
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_PROCESS_KILL:
            FILE_LOG(logINFO) << "CMD: DAEMON_PROCESS_KILL";
            pmHandler->CloseAL(sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_INT[0]);
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_INIT_CHECK_DEVICE:
            FILE_LOG(logINFO) << "CMD: DAEMON_INIT_CHECK_DEVICE";
            if(__IS_GAZEBO) {FILE_LOG(logINFO) << "Gazebo doesn't need DAEMON_INIT_CHECK_DEVICE";}
            else{
                if(IS_CAN_OK)   {RBCMD_InitCheckDevice();}
                else            {FILE_LOG(logWARNING) << "CAN device not set";}
            }
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_INIT_FIND_HOME:
            FILE_LOG(logINFO) << "CMD: DAEMON_INIT_FIND_HOME";
            if(__IS_GAZEBO) {FILE_LOG(logINFO) << "Gazebo doesn't need DAEMON_INIT_FIND_HOME";}
            else{
                if(IS_CAN_OK)   {RBCMD_InitFindHome();}
                else            {FILE_LOG(logWARNING) << "CAN device not set";}
            }
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_INIT_FET_ONOFF:
            FILE_LOG(logINFO) << "CMD: DAEMON_INIT_FET_ONOFF";
            if(__IS_GAZEBO) {FILE_LOG(logINFO) << "Gazebo doesn't need DAEMON_INIT_FET_ONOFF";}
            else{
                if(IS_CAN_OK)   {RBCMD_InitFetOnOff();}
                else            {FILE_LOG(logWARNING) << "CAN device not set";}
            }
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_INIT_CONTROL_ONOFF:
            FILE_LOG(logINFO) << "CMD: DAEMON_INIT_CONTROL_ONOFF";
            if(__IS_GAZEBO) {FILE_LOG(logINFO) << "Gazebo doesn't need DAEMON_INIT_CONTROL_ONOFF";}
            else{
                if(IS_CAN_OK)   {RBCMD_InitControlOnOff();}
                else            {FILE_LOG(logWARNING) << "CAN device not set";}
            }
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_INIT_SET_FINGER_MODIFIER:
            FILE_LOG(logINFO) << "CMD: DAEMON_INIT_SET_FINGER_MODIFIER";

            RBCMD_InitSetFingerModifier();

            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_ATTR_SWITCHING_MODE:
            FILE_LOG(logINFO) << "CMD: DAEMON_ATTR_SWITCHING_MODE";
            if(__IS_GAZEBO) {FILE_LOG(logINFO) << "Gazebo doesn't need DAEMON_ATTR_SWITCHING_MODE";}
            else{
                if(IS_CAN_OK)   {RBCMD_AttrSwitchingMode();}
                else            {FILE_LOG(logWARNING) << "CAN device not set";}
            }
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_ATTR_FRICTION_COMPENSATION:
            FILE_LOG(logINFO) << "CMD: DAEMON_ATTR_FRICTION_COMPENSATION";
            if(__IS_GAZEBO) {FILE_LOG(logINFO) << "Gazebo doesn't need DAEMON_ATTR_FRICTION_COMPENSATION";}
            else{
                if(IS_CAN_OK)   {RBCMD_AttrFrictionCompensation();}
                else            {FILE_LOG(logWARNING) << "CAN device not set";}
            }
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_ATTR_CONTROL_MODE:
            FILE_LOG(logINFO) << "CMD: DAEMON_ATTR_CONTROL_MODE";
            if(__IS_GAZEBO) {FILE_LOG(logINFO) << "Gazebo doesn't need DAEMON_ATTR_CONTROL_MODE";}
            else{
                if(IS_CAN_OK)   {RBCMD_AttrControlMode();}
                else            {FILE_LOG(logWARNING) << "CAN device not set";}
            }
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_SENSOR_ENCODER_RESET:
            FILE_LOG(logINFO) << "CMD: DAEMON_SENSOR_ENCODER_RESET";
            if(__IS_GAZEBO) {FILE_LOG(logINFO) << "Gazebo doesn't need DAEMON_SENSOR_ENCODER_RESET";}
            else{
                if(IS_CAN_OK)   {RBCMD_SensorEncoderReset();}
                else            {FILE_LOG(logWARNING) << "CAN device not set";}
            }
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_SENSOR_ENCODER_ONOFF:
            FILE_LOG(logINFO) << "CMD: DAEMON_SENSOR_ENCODER_ONOFF";
            if(__IS_GAZEBO) {FILE_LOG(logINFO) << "Gazebo doesn't need DAEMON_SENSOR_ENCODER_ONOFF";}
            else{
                if(IS_CAN_OK)   {RBCMD_SensorEncoderOnOff();}
                else            {FILE_LOG(logWARNING) << "CAN device not set";}
            }
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_SENSOR_SENSOR_ONOFF:
            FILE_LOG(logINFO) << "CMD: DAEMON_SENSOR_SENSOR_ONOFF";
            if(__IS_GAZEBO) {FILE_LOG(logINFO) << "Gazebo doesn't need DAEMON_SENSOR_SENSOR_ONOFF";}
            else{
                if(IS_CAN_OK)   {RBCMD_SensorSensorOnOff();}
                else            {FILE_LOG(logWARNING) << "CAN device not set";}
            }
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_SENSOR_FT_NULL:
            FILE_LOG(logINFO) << "CMD: DAEMON_SENSOR_FT_NULL";
            if(__IS_GAZEBO) {FILE_LOG(logINFO) << "Gazebo doesn't need DAEMON_SENSOR_FT_NULL";}
            else{
                if(IS_CAN_OK)
                {
                    RBCMD_SensorFTNull();
                }
                else            {FILE_LOG(logWARNING) << "CAN device not set";}
            }
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_SENSOR_IMU_NULL:
            FILE_LOG(logINFO) << "CMD: DAEMON_SENSOR_IMU_NULL";
            if(__IS_GAZEBO) {FILE_LOG(logINFO) << "Gazebo doesn't need DAEMON_SENSOR_IMU_NULL";}
            else{
                if(IS_CAN_OK)   {RBCMD_SensorIMUNull();}
                else            {FILE_LOG(logWARNING) << "CAN device not set";}
            }
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_SENSOR_IMU_OFFSET_SET:
            FILE_LOG(logINFO) << "CMD: DAEMON_SENSOR_IMU_OFFSET_SET";
            if(__IS_GAZEBO) {FILE_LOG(logINFO) << "Gazebo doesn't need DAEMON_SENSOR_IMU_OFFSET_SET";}
            else{
                if(IS_CAN_OK)   {RBCMD_SensorIMUOffsetSet();}
                else            {FILE_LOG(logWARNING) << "CAN device not set";}
            }
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_SENSOR_FOG_NULL:
            FILE_LOG(logINFO) << "CMD: DAEMON_SENSOR_FOG_NULL";
            if(__IS_GAZEBO) {FILE_LOG(logINFO) << "Gazebo doesn't need DAEMON_SENSOR_FOG_NULL";}
            else{
                if(IS_RS232_OK) {RBCMD_SensorFOGNull();}
                else            {FILE_LOG(logWARNING) << "RS232 device not set";}
            }
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_SENSOR_FOG_USB_RESET:
            FILE_LOG(logINFO) << "CMD: DAEMON_SENSOR_FOG_USB_RESET";
            if(__IS_GAZEBO) {FILE_LOG(logINFO) << "Gazebo doesn't need DAEMON_SENSOR_FOG_USB_RESET";}
            else{
                if(IS_RS232_OK) {RBCMD_SensorFOGUSBReset();}
                else            {FILE_LOG(logWARNING) << "RS232 device not set";}
            }
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_SENSOR_OF_NULL:
            FILE_LOG(logINFO) << "CMD: DAEMON_SENSOR_OF_NULL";
            if(__IS_GAZEBO) {FILE_LOG(logINFO) << "Gazebo doesn't need DAEMON_SENSOR_OF_NULL";}
            else{
                if(IS_CAN_OK)   {RBCMD_SensorOFNull();}
                else            {FILE_LOG(logWARNING) << "CAN device not set";}
            }
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_SENSOR_OF_LAMP_ONOFF:
            FILE_LOG(logINFO) << "CMD: DAEMON_SENSOR_OF_LAMP_ONOFF";
            if(__IS_GAZEBO) {FILE_LOG(logINFO) << "Gazebo doesn't need DAEMON_SENSOR_OF_LAMP_ONOFF";}
            else{
                if(IS_CAN_OK)   {RBCMD_SensorOFLampOnOff();}
                else            {FILE_LOG(logWARNING) << "CAN device not set";}
            }
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_MOTION_REF_ONOFF:
            FILE_LOG(logINFO) << "CMD: DAEMON_MOTION_REF_ONOFF";
            RBCMD_MotionRefOnOff();

            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_MOTION_MOVE:
            FILE_LOG(logINFO) << "CMD: DAEMON_MOTION_MOVE";
            RBCMD_MotionMove();

            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_MOTION_GAIN_OVERRIDE:
            FILE_LOG(logINFO) << "CMD: DAEMON_MOTION_GAIN_OVERRIDE";
            RBCMD_MotionGainOverride();

            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_MOTION_ERROR_CLEAR:
            FILE_LOG(logINFO) << "CMD: DAEMON_MOTION_ERROR_CLEAR";
            if(__IS_GAZEBO) {FILE_LOG(logINFO) << "Gazebo doesn't need DAEMON_MOTION_ERROR_CLEAR";}
            else{
                if(IS_CAN_OK)   {RBCMD_MotionErrorClear();}
                else            {FILE_LOG(logWARNING) << "CAN device not set";}
            }
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case DAEMON_CAN_ENABLE_DISABLE:
            FILE_LOG(logINFO) << "CMD: DAEMON_CAN_ENABLE_DISABLE";
            if(__IS_GAZEBO) {FILE_LOG(logINFO) << "Gazebo doesn't need DAEMON_CAN_ENABLE_DISABLE";}
            else{
                if(IS_CAN_OK)   {RBCMD_CANEnableDisable();}
                else            {FILE_LOG(logWARNING) << "CAN device not set";}
            }
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case 888:
            FILE_LOG(logINFO) << "Command: SAVE_DEBUG_DATA";
            fpDebug = fopen("dataDaemon.txt","w");
            for(int i=0;i<ROW_data_debug;i++){
                for(int j=0;j<COL_data_debug;j++)fprintf(fpDebug,"%g\t", JW_Data_Debug[j][i]);
                fprintf(fpDebug,"\n");
            }
            fclose(fpDebug);
            sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = NO_ACT;
            break;

        }


//        // Command acceptance check for AL -------------------------------
//        for(int i=2; i<_NO_OF_AL; i++){ // i=0 Daemon, i=1 PODOLAN
//            if(sharedData->COMMAND[i].USER_COMMAND != 0 &&
//                sharedData->COMMAND[i].USER_COMMAND != 100 &&
//                sharedData->CommandAccept[i] == false){ // NO_ACT should be 0 or 100
//                    _ALCommandCnt[i]++;
//            }else{
//                _ALCommandCnt[i] = 0;
//            }

//            // AL didn't accept command for (100ms X 5 = 500ms)
//            if(_ALCommandCnt[i] > 5){
//                sharedData->ErrorInform2GUI |= (1<<i);
//                sharedData->CommandAccept[i] = true;
//            }
//        }
        // ----------------------------------------------------------------
    }


    RBCore_Termination();

    usleep(1000*1000);
    return 0;
}



void RBCore_RTThreadCon(void *)
{
    rt_task_set_periodic(NULL, TM_NOW, (RT_TIMER_PERIOD_MS)*1000000);

    RTIME   th_start, th_stop;
    int     waitCnt, waitOK;
    double  timeGap = 0.0;
    double  finalJointRef[MOTOR_2CH];

    while(IS_WORKING)
    {
        //rt_task_wait_period(NULL);

        while(*GAZEBO_FLAG != 1)
        {
            usleep(5*1000);
        }
        *GAZEBO_FLAG = 0;

        // Read Sensor & Encoder =====================================
        if(canHandler->IsWorking()){
            THREAD_ReadSensor();
            THREAD_ReadEncoder();
            THREAD_ReadTemperature();
            THREAD_ReadVoltage();
            THREAD_ReadHomeError();

            sharedData->CAN_Enabled = _CANOUT_ENABLED;
            sharedData->ENC_Enabled = _ENCODER_ENABLED;
            sharedData->SEN_Enabled = _SENSOR_ENABLED;
            sharedData->REF_Enabled = _REFERENCE_ENABLED;
        }
        // ===========================================================

        // Change Flag ===============================================
        for(int i=0; i<_NO_OF_MC; i++){
            for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
                sharedData->ACK_SIGNAL[i][j] = false;
            }
        }
        for(int i=0; i<_NO_OF_AL; i++){
            sharedData->SYNC_SIGNAL[i] = true;
        }
        // ===========================================================

        // Wait Reference ============================================
        th_start = rt_timer_read();
        waitCnt = 0;
        waitOK = false;
        while(1){
            // check the all WriteDoneFlag are enabled
            int notRead = 0;
            for(int i=0; i<_NO_OF_MC; i++){
                for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
                    if(sharedData->ACK_SIGNAL[i][j] == false && sharedData->MotionOwner[i][j] != RBCORE_PODO_NO){
                        notRead++;
                    }
                }
            }
            if(notRead == 0){
                th_stop = rt_timer_read();
                timeGap = (double)(th_stop - th_start) / (double)1000000.0;
                waitOK = true;
                break;
            }

            if(waitCnt%500 == 0){
                th_stop = rt_timer_read();
                timeGap = (double)(th_stop - th_start) / (double)1000000.0;
                if(timeGap > 2.5){
                    waitOK = false;
//                    FILE_LOG(logWARNING) << "Over 2.5msec";
                    break;
                }
            }
            waitCnt++;
            usleep(2);
        }
        // ===========================================================


        // Write CAN Reference =======================================
        for(int i=0; i<_NO_OF_MC; i++){
            // skip if the extra finger is not set
            if(!__IS_EXF_R && (i==25 || i==26))     continue;
            if(!__IS_EXF_L && (i==27 || i==28))     continue;


            for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
                int motionOwner = sharedData->MotionOwner[i][j];

                _DEV_MC[i].RBJoint_MoveJoint(j);
                sharedData->JointReference[RBCORE_PODO_NO][i][j] = _DEV_MC[i].MoveJoints[j].RefAngleCurrent;

                finalJointRef[j] = sharedData->JointReference[motionOwner][i][j];
                _DEV_MC[i].Joints[j].Reference = finalJointRef[j];
                sharedData->ENCODER[i][j].CurrentReference = finalJointRef[j];
            }
            if(canHandler->IsWorking() && _CANOUT_ENABLED){ // CANOUT
                _DEV_MC[i].RBBoard_SendReference();
            }
        }
        //JW_save();
        // ===========================================================


        // Manual CAN ================================================
        if(canHandler->IsWorking()){
            for(int i=0; i<MAX_MANUAL_CAN; i++){
                if(sharedData->ManualCAN[i].status == MANUALCAN_NEW){
                    sharedData->ManualCAN[i].status = MANUALCAN_WRITING;
                    RBCAN_MB mb;
                    mb.channel = sharedData->ManualCAN[i].channel;
                    mb.id = sharedData->ManualCAN[i].id;
                    mb.dlc = sharedData->ManualCAN[i].dlc;
                    for(int j=0; j<mb.dlc; j++)
                        mb.data[j] = sharedData->ManualCAN[i].data[j];

                    if(mb.data[0] != 0x6F || _CANOUT_ENABLED)  // CANOUT
                        canHandler->RBCAN_WriteData(mb);

                    //FILE_LOG(logINFO) << "MANUAL_CAN: " << mb.id << " == " << (int)mb.data[0] << ", " << (int)mb.data[1] << ", " << (int)mb.data[2] << ", " << (int)mb.data[3] << ", " << (int)mb.data[4] << ", " << (int)mb.data[5] << ", " << (int)mb.data[6] << ", " << (int)mb.data[7];
                    sharedData->ManualCAN[i].status = MANUALCAN_EMPTY;
                }
            }
        }
        // ===========================================================


        _ThreadCnt++;
        // Request Sensor & Encoder ==================================
        if(canHandler->IsWorking()){
            THREAD_RequestSensor();
            THREAD_RequestEncoder();
            THREAD_RequestTemperature();
            THREAD_RequestVoltage();
        }
        // ===========================================================
    }

}

void *RBCore_NRTThreadCon(void *)
{
    DRC_GAZEBO_SENSOR   GazeboSensor;
    DRC_GAZEBO_JOINT    GazeboJoint;
    char                *GazeboJoint_and_Type;
    DRC_GAZEBO_GO_CMD   GazeboGain;
    char                *GazeboGain_and_Type;

    GazeboJoint_and_Type = (char*)malloc(sizeof(DRC_GAZEBO_JOINT) + sizeof(int));
    GazeboGain_and_Type = (char*)malloc(sizeof(DRC_GAZEBO_GO_CMD) + sizeof(int));

    RTIME   th_start, th_stop;
    int     waitCnt, waitOK;
    double  timeGap = 0.0;
    double  finalJointRef[MOTOR_2CH];

    while(IS_WORKING)
    {
        usleep(10);
        if(lanHandler->ConnectionStatus){
            if(lanHandler->RBLanReadData((char*)(&GazeboSensor), sizeof(GazeboSensor), 0x00) == LAN_NEW_DATA)
            {

                sharedData->Sim_Time_sec = GazeboSensor.Sim_Time.sec;
                sharedData->Sim_Time_nsec = GazeboSensor.Sim_Time.nsec;

                // Read Sensor & Encoder =====================================
                for(int i=0; i<_NO_OF_FT; i++){
                    sharedData->FT[i].Fx = GazeboSensor.FTSensor[i].force[0];
                    sharedData->FT[i].Fy = GazeboSensor.FTSensor[i].force[1];
                    sharedData->FT[i].Fz = GazeboSensor.FTSensor[i].force[2];
                    sharedData->FT[i].Mx = GazeboSensor.FTSensor[i].torque[0];
                    sharedData->FT[i].My = GazeboSensor.FTSensor[i].torque[1];
                    sharedData->FT[i].Mz = GazeboSensor.FTSensor[i].torque[2];
                }
                for(int i=0; i<_NO_OF_IMU; i++){
                    sharedData->IMU[i].Roll      = GazeboSensor.IMUSensor[0];
                    sharedData->IMU[i].Pitch     = GazeboSensor.IMUSensor[1];
                    sharedData->IMU[i].Yaw       = GazeboSensor.IMUSensor[2];
                    sharedData->IMU[i].RollVel   = GazeboSensor.IMUSensor[3];
                    sharedData->IMU[i].PitchVel  = GazeboSensor.IMUSensor[4];
                    sharedData->IMU[i].YawVel    = GazeboSensor.IMUSensor[5];
                    sharedData->IMU[i].AccX      = GazeboSensor.IMUSensor[6];
                    sharedData->IMU[i].AccY      = GazeboSensor.IMUSensor[7];
                    sharedData->IMU[i].AccZ      = GazeboSensor.IMUSensor[8];
                }

                double tempDouble;
                for(int i=0; i<NO_OF_JOINTS; i++){
                    tempDouble = sharedData->ENCODER[MC_GetID(i)][MC_GetCH(i)].CurrentPosition;
                    sharedData->ENCODER[MC_GetID(i)][MC_GetCH(i)].CurrentPosition = GazeboSensor.JointCurrentPosition[i];
                    sharedData->ENCODER[MC_GetID(i)][MC_GetCH(i)].CurrentVelocity = (sharedData->ENCODER[MC_GetID(i)][MC_GetCH(i)].CurrentPosition - tempDouble) / (double)RT_TIMER_PERIOD_MS * 1000.0;
                }


                sharedData->FOG.Roll     = sharedData->IMU[0].Roll;
                sharedData->FOG.Pitch    = sharedData->IMU[0].Pitch;
                sharedData->FOG.Yaw      = sharedData->IMU[0].Yaw;
                sharedData->FOG.RollVel     = sharedData->IMU[0].RollVel*RBCORE_PI/180.;
                sharedData->FOG.PitchVel    = sharedData->IMU[0].PitchVel*RBCORE_PI/180.;
                sharedData->FOG.YawVel      = sharedData->IMU[0].YawVel*RBCORE_PI/180.0;

                sharedData->CAN_Enabled = _CANOUT_ENABLED;
                sharedData->ENC_Enabled = _ENCODER_ENABLED;
                sharedData->SEN_Enabled = _SENSOR_ENABLED;
                sharedData->REF_Enabled = _REFERENCE_ENABLED;
                // ===========================================================

                // Change Flag ===============================================
                for(int i=0; i<_NO_OF_MC; i++){
                    for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
                        sharedData->ACK_SIGNAL[i][j] = false;
                    }
                }
                for(int i=0; i<_NO_OF_AL; i++){
                    sharedData->SYNC_SIGNAL[i] = true;
                }
                // ===========================================================

                // Wait Reference ============================================
                th_start = rt_timer_read();
                waitCnt = 0;
                waitOK = false;
                while(1)
                {
                    // check the all WriteDoneFlag are enabled
                    int notRead = 0;
                    for(int i=0; i<_NO_OF_MC; i++){
                        for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
                            if(sharedData->ACK_SIGNAL[i][j] == false && sharedData->MotionOwner[i][j] != RBCORE_PODO_NO){
                                notRead++;
                            }
                        }
                    }
                    if(notRead == 0){
                        th_stop = rt_timer_read();
                        timeGap = (double)(th_stop - th_start) / (double)1000000.0;
                        waitOK = true;
                        break;
                    }

                    if(waitCnt%500 == 0){
                        th_stop = rt_timer_read();
                        timeGap = (double)(th_stop - th_start) / (double)1000000.0;
                        if(timeGap > 2.5)
                        {
                            waitOK = false;
                            FILE_LOG(logWARNING) << "Over 2.5msec";
                            break;
                        }

                    }
                    waitCnt++;
                    usleep(2);
                }
                // ===========================================================

                // Write LAN Reference =======================================
                for(int i=0; i<_NO_OF_MC; i++){
                    for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
                        int motionOwner = sharedData->MotionOwner[i][j];

                        _DEV_MC[i].RBJoint_MoveJoint(j);
                        sharedData->JointReference[RBCORE_PODO_NO][i][j] = _DEV_MC[i].MoveJoints[j].RefAngleCurrent;

                        finalJointRef[j] = sharedData->JointReference[motionOwner][i][j];
                        _DEV_MC[i].Joints[j].Reference = sharedData->ENCODER[i][j].CurrentReference = finalJointRef[j];
                    }
                }
                for(int i=0; i<NO_OF_JOINTS; i++){
                    GazeboJoint.JointReference[i] = _DEV_MC[MC_GetID(i)].Joints[MC_GetCH(i)].Reference;
                }
                int type = GAZEBO_TYPE_JOINT;
                //memcpy(GazeboJoint_and_Type, &type, sizeof(int));
                //memcpy(&(GazeboJoint_and_Type[sizeof(int)]), &GazeboJoint, sizeof(DRC_GAZEBO_JOINT));

                lanHandler->RBLanWriteData(&type, sizeof(int));
                lanHandler->RBLanWriteData(&GazeboJoint, sizeof(DRC_GAZEBO_JOINT));
                //lanHandler->RBLanWriteData(GazeboJoint_and_Type, sizeof(GazeboJoint_and_Type));
                // ===========================================================

                for(int i=0; i<MAX_MANUAL_CAN; i++){
                    if(sharedData->ManualCAN[i].status == MANUALCAN_NEW){
                        sharedData->ManualCAN[i].status = MANUALCAN_WRITING;

                        int id = sharedData->ManualCAN[i].id;
                        int dlc = sharedData->ManualCAN[i].dlc;
                        int bno = -1;
                        for(int j=0; j<_NO_OF_MC; j++){
                            if(_DEV_MC[j].ID_SEND_GENERAL == id){
                                bno = j;
                                break;
                            }
                        }

                        if(bno >= 0){
                            if(dlc > 0){
                                if(sharedData->ManualCAN[i].data[0] == 0x6F){    // Gain Override
                                    int ch = sharedData->ManualCAN[i].data[1]-1;
                                    int joint = -1;
                                    for(int j=0; j<NO_OF_JOINTS; j++){
                                        if(MC_GetID(j) == bno && MC_GetCH(j) == ch){
                                            joint = j;
                                            break;
                                        }
                                    }
                                    //FILE_LOG(logWARNING) << "ManualCAN JOINT: " << joint;
                                    if(joint >= 0){
                                        int gain = sharedData->ManualCAN[i].data[2];
                                        int timeMS = (int)(sharedData->ManualCAN[i].data[3] | (sharedData->ManualCAN[i].data[4] << 8));

                                        int type = GAZEBO_TYPE_GAINOVERRIDE;
                                        GazeboGain.gain = gain;
                                        GazeboGain.joint = joint;
                                        GazeboGain.timeMs = timeMS;
                                        FILE_LOG(logWARNING) << "GainOverride: " << joint << " , " << gain << ", " << timeMS;
                                        lanHandler->RBLanWriteData(&type, sizeof(int));
                                        lanHandler->RBLanWriteData(&GazeboGain, sizeof(DRC_GAZEBO_GO_CMD));
                                    }
                                }else if(sharedData->ManualCAN[i].data[0] == 0x11){ // Home
                                    int ch = sharedData->ManualCAN[i].data[1]-1;
                                    int joint = -1;
                                    if(ch == -1){
                                        FILE_LOG(logWARNING) << "Only support single channel";
                                    }else{
                                        for(int j=0; j<NO_OF_JOINTS; j++){
                                            if(MC_GetID(j) == bno && MC_GetCH(j) == ch){
                                                joint = j;
                                                break;
                                            }
                                        }
                                        if(joint >= 0){
                                            int type = GAZEBO_TYPE_HOME;
                                            FILE_LOG(logWARNING) << "Find Home: " << joint;
                                            lanHandler->RBLanWriteData(&type, sizeof(int));
                                            lanHandler->RBLanWriteData(&joint, sizeof(int));
                                        }
                                    }
                                }
                            }

                        }
                        sharedData->ManualCAN[i].status = MANUALCAN_EMPTY;
                    }
                }
            }
        }else{ // without Gazebo connection
            // No Sensor Data

            // Change Flag ===============================================
            for(int i=0; i<_NO_OF_MC; i++){
                for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
                    sharedData->ACK_SIGNAL[i][j] = false;
                }
            }
            for(int i=0; i<_NO_OF_AL; i++){
                sharedData->SYNC_SIGNAL[i] = true;
            }
            // ===========================================================

            // Wait Reference ============================================
            th_start = rt_timer_read();
            waitCnt = 0;
            waitOK = false;
            while(1){
                // check the all WriteDoneFlag are enabled
                int notRead = 0;
                for(int i=0; i<_NO_OF_MC; i++){
                    for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
                        if(sharedData->ACK_SIGNAL[i][j] == false && sharedData->MotionOwner[i][j] != RBCORE_PODO_NO){
                            notRead++;
                        }
                    }
                }
                if(notRead == 0){
                    th_stop = rt_timer_read();
                    timeGap = (double)(th_stop - th_start) / (double)1000000.0;
                    waitOK = true;
                    break;
                }

                if(waitCnt%500 == 0){
                    th_stop = rt_timer_read();
                    timeGap = (double)(th_stop - th_start) / (double)1000000.0;
                    if(timeGap > 2.5){
                        waitOK = false;
                        FILE_LOG(logWARNING) << "Over 2.5msec";
                        break;
                    }
                }
                waitCnt++;
                usleep(2);
            }
            // ===========================================================

            // Move Reference ============================================
            for(int i=0; i<_NO_OF_MC; i++){
                for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
                    int motionOwner = sharedData->MotionOwner[i][j];

                    _DEV_MC[i].RBJoint_MoveJoint(j);
                    sharedData->JointReference[RBCORE_PODO_NO][i][j] = _DEV_MC[i].MoveJoints[j].RefAngleCurrent;

                    finalJointRef[j] = sharedData->JointReference[motionOwner][i][j];
                    _DEV_MC[i].Joints[j].Reference = sharedData->ENCODER[i][j].CurrentReference = finalJointRef[j];
                }
            }
            // ===========================================================

            // No Manual Control

            usleep(5*1000);
        }
    }
    return NULL;
}




void RBCMD_InitCheckDevice(){
    for(int i=0; i<_NO_OF_MC; i++){
        _DEV_MC[i].RBBoard_CANCheck(RT_TIMER_PERIOD_MS);
        for(int j=0; j<MOTOR_2CH; j++){
            sharedData->ENCODER[i][j].BoardConnection = _DEV_MC[i].ConnectionStatus;
        }
    }
    for(int i=0; i<_NO_OF_FT; i++){
        _DEV_FT[i].RBBoard_CANCheck(RT_TIMER_PERIOD_MS);
        sharedData->FT[i].BoardConnection = _DEV_FT[i].ConnectionStatus;
    }
    for(int i=0; i<_NO_OF_IMU; i++){
        _DEV_IMU[i].RBBoard_CANCheck(RT_TIMER_PERIOD_MS);
        sharedData->IMU[i].BoardConnection = _DEV_IMU[i].ConnectionStatus;
    }
    for(int i=0; i<_NO_OF_SP; i++){
        _DEV_SP[i].RBBoard_CANCheck(RT_TIMER_PERIOD_MS);
        sharedData->SP[i].BoardConnection = _DEV_SP[i].ConnectionStatus;
    }
    for(int i=0; i<_NO_OF_OF; i++){
        _DEV_OF[i].RBBoard_CANCheck(RT_TIMER_PERIOD_MS);
        sharedData->OF.BoardConnection = _DEV_OF[i].ConnectionStatus;
    }

    RBCMD_InitWheel();
}

void RBCMD_InitWheel(){
    for(int i=RWH; i<=LWH; i++){
        int id = MC_ID_CH_Pairs[i].id;
        int ch = MC_ID_CH_Pairs[i].ch;
        _DEV_MC[id].RBJoint_SetLowerPosLimit(ch+1, 0, 0);
        _DEV_MC[id].RBJoint_SetUpperPosLimit(ch+1, 0, 0);
        _DEV_MC[id].RBJoint_SetPositionCommandMode(1, ch+1);
    }
}

void RBCMD_InitFindHome(){
    int special_for_finger = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_INT[0];
    int right_left = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_INT[1];
    int id = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    int ch = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1];

    if(special_for_finger == 999){
        if((right_left & 0x01) == 0x01){
            // right
            _DEV_MC[21].RBJoint_ResetEncoder(2);
            sharedData->MotionOwner[21][1] = RBCORE_PODO_NO;
            for(int k=0; k<_NO_OF_AL; k++){
                sharedData->JointReference[k][21][1] = 0.0;
            }
            _DEV_MC[21].Joints[1].Reference = 0.0;
            _DEV_MC[21].MoveJoints[1].RefAngleCurrent = 0.0;

            _DEV_MC[21].RBJoint_EnableFETDriver(2, 1);
            usleep(100);
            _DEV_MC[21].RBJoint_FindHome(2);

            if(__IS_EXF_R){
                _DEV_MC[25].RBJoint_ResetEncoder(1);
                _DEV_MC[26].RBJoint_ResetEncoder(1);
                sharedData->MotionOwner[25][0] = RBCORE_PODO_NO;
                sharedData->MotionOwner[26][0] = RBCORE_PODO_NO;
                for(int k=0; k<_NO_OF_AL; k++){
                    sharedData->JointReference[k][25][0] = 0.0;
                    sharedData->JointReference[k][26][0] = 0.0;
                }
                _DEV_MC[25].Joints[0].Reference = 0.0;
                _DEV_MC[26].Joints[0].Reference = 0.0;
                _DEV_MC[25].MoveJoints[0].RefAngleCurrent = 0.0;
                _DEV_MC[26].MoveJoints[0].RefAngleCurrent = 0.0;

                _DEV_MC[25].RBJoint_EnableFETDriver(1, 1);
                _DEV_MC[26].RBJoint_EnableFETDriver(1, 1);
                usleep(100);
                _DEV_MC[25].RBJoint_FindHome(1);
                _DEV_MC[26].RBJoint_FindHome(1);


                _DEV_MC[25].RBJoint_ResetEncoder(2);
                _DEV_MC[26].RBJoint_ResetEncoder(2);
                sharedData->MotionOwner[25][1] = RBCORE_PODO_NO;
                sharedData->MotionOwner[26][1] = RBCORE_PODO_NO;
                for(int k=0; k<_NO_OF_AL; k++){
                    sharedData->JointReference[k][25][1] = 0.0;
                    sharedData->JointReference[k][26][1] = 0.0;
                }
                _DEV_MC[25].Joints[1].Reference = 0.0;
                _DEV_MC[26].Joints[1].Reference = 0.0;
                _DEV_MC[25].MoveJoints[1].RefAngleCurrent = 0.0;
                _DEV_MC[26].MoveJoints[1].RefAngleCurrent = 0.0;

                _DEV_MC[25].RBJoint_EnableFETDriver(2, 1);
                _DEV_MC[26].RBJoint_EnableFETDriver(2, 1);
                usleep(100);
                _DEV_MC[25].RBJoint_FindHome(2);
                _DEV_MC[26].RBJoint_FindHome(2);
            }
        }

        if(((right_left>>1) & 0x01) == 0x01){
            // left
            _DEV_MC[22].RBJoint_ResetEncoder(2);
            sharedData->MotionOwner[22][1] = RBCORE_PODO_NO;
            for(int k=0; k<_NO_OF_AL; k++){
                sharedData->JointReference[k][22][1] = 0.0;
            }
            _DEV_MC[22].Joints[1].Reference = 0.0;
            _DEV_MC[22].MoveJoints[1].RefAngleCurrent = 0.0;

            _DEV_MC[22].RBJoint_EnableFETDriver(2, 1);
            usleep(100);
            _DEV_MC[22].RBJoint_FindHome(2);

            if(__IS_EXF_R){
                _DEV_MC[27].RBJoint_ResetEncoder(1);
                _DEV_MC[28].RBJoint_ResetEncoder(1);
                sharedData->MotionOwner[27][0] = RBCORE_PODO_NO;
                sharedData->MotionOwner[28][0] = RBCORE_PODO_NO;
                for(int k=0; k<_NO_OF_AL; k++){
                    sharedData->JointReference[k][27][0] = 0.0;
                    sharedData->JointReference[k][28][0] = 0.0;
                }
                _DEV_MC[27].Joints[0].Reference = 0.0;
                _DEV_MC[28].Joints[0].Reference = 0.0;
                _DEV_MC[27].MoveJoints[0].RefAngleCurrent = 0.0;
                _DEV_MC[28].MoveJoints[0].RefAngleCurrent = 0.0;

                _DEV_MC[27].RBJoint_EnableFETDriver(1, 1);
                _DEV_MC[28].RBJoint_EnableFETDriver(1, 1);
                usleep(100);
                _DEV_MC[27].RBJoint_FindHome(1);
                _DEV_MC[28].RBJoint_FindHome(1);


                _DEV_MC[27].RBJoint_ResetEncoder(2);
                _DEV_MC[28].RBJoint_ResetEncoder(2);
                sharedData->MotionOwner[27][1] = RBCORE_PODO_NO;
                sharedData->MotionOwner[28][1] = RBCORE_PODO_NO;
                for(int k=0; k<_NO_OF_AL; k++){
                    sharedData->JointReference[k][27][1] = 0.0;
                    sharedData->JointReference[k][28][1] = 0.0;
                }
                _DEV_MC[27].Joints[1].Reference = 0.0;
                _DEV_MC[28].Joints[1].Reference = 0.0;
                _DEV_MC[27].MoveJoints[1].RefAngleCurrent = 0.0;
                _DEV_MC[28].MoveJoints[1].RefAngleCurrent = 0.0;

                _DEV_MC[27].RBJoint_EnableFETDriver(2, 1);
                _DEV_MC[28].RBJoint_EnableFETDriver(2, 1);
                usleep(100);
                _DEV_MC[27].RBJoint_FindHome(2);
                _DEV_MC[28].RBJoint_FindHome(2);
            }
        }
    }else{
        if(id == -1){ // All
            for(int i=0; i<_NO_OF_MC; i++){
                for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
                    _DEV_MC[i].RBJoint_ResetEncoder(j+1);
                    sharedData->MotionOwner[i][j] = RBCORE_PODO_NO;

                    for(int k=0; k<_NO_OF_AL; k++){
                        sharedData->JointReference[k][i][j] = 0.0;
                    }
                    _DEV_MC[i].Joints[j].Reference = 0.0;
                    _DEV_MC[i].MoveJoints[j].RefAngleCurrent = 0.0;

                    _DEV_MC[i].RBJoint_EnableFETDriver(j+1, 1);
                    usleep(100);
                    _DEV_MC[i].RBJoint_FindHome(j+1);
                }
            }
        }else{  // Each
            _DEV_MC[id].RBJoint_ResetEncoder(ch+1);
            sharedData->MotionOwner[id][ch] = RBCORE_PODO_NO;
            for(int k=0; k<_NO_OF_AL; k++){
                sharedData->JointReference[k][id][ch] = 0.0;
            }
            _DEV_MC[id].Joints[ch].Reference = 0.0;
            _DEV_MC[id].MoveJoints[ch].RefAngleCurrent = 0.0;

            _DEV_MC[id].RBJoint_EnableFETDriver(ch+1, 1);
            usleep(100);
            _DEV_MC[id].RBJoint_FindHome(ch+1);
        }
    }
}

void RBCMD_InitFetOnOff(){
    int id = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    int ch = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1];
    int onoff = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2];   // 0->off
    if(onoff != 0) onoff = 1;

    if(id == -1){ // All
        for(int i=0; i<_NO_OF_MC; i++){
            for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
                _DEV_MC[i].RBJoint_EnableFETDriver(j+1, onoff);
            }
        }
    }else{  // Each
        _DEV_MC[id].RBJoint_EnableFETDriver(ch+1, onoff);
    }
}

void RBCMD_InitControlOnOff(){
    int id = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    int ch = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1];
    int onoff = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2];   // 0->off
    if(onoff != 0) onoff = 1;

    if(id == -1){ // All
        for(int i=0; i<_NO_OF_MC; i++){
            for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
                _DEV_MC[i].RBJoint_EnableFeedbackControl(j+1, onoff);
            }
        }
    }else{  // Each
        _DEV_MC[id].RBJoint_EnableFeedbackControl(ch+1, onoff);
    }
}

void RBCMD_InitSetFingerModifier(){
    QSettings settings(settingFile, QSettings::NativeFormat);
    if(sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_INT[0] == 1){
        // just read current value
    }else{
        // set with new values (only available if the Daemon is turned on for extra finger
        for(int i=0; i<5; i++){
            //if(sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[i] && __IS_EXF_R){
            if(__IS_EXF_R){
                settings.setValue(QString().sprintf("exfr%d",i), sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_FLOAT[i]);
            }
            //if(sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[i+5] && __IS_EXF_L){
            if(__IS_EXF_L){
                settings.setValue(QString().sprintf("exfl%d",i), sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_FLOAT[i+5]);
            }
        }
    }

    if(__IS_EXF_R){
        EXF_R_Modifier[0] = settings.value("exfr0", "").toFloat();
        EXF_R_Modifier[1] = settings.value("exfr1", "").toFloat();
        EXF_R_Modifier[2] = settings.value("exfr2", "").toFloat();
        EXF_R_Modifier[3] = settings.value("exfr3", "").toFloat();
        EXF_R_Modifier[4] = settings.value("exfr4", "").toFloat();
    }
    if(__IS_EXF_L){
        EXF_L_Modifier[0] = settings.value("exfl0", "").toFloat();
        EXF_L_Modifier[1] = settings.value("exfl1", "").toFloat();
        EXF_L_Modifier[2] = settings.value("exfl2", "").toFloat();
        EXF_L_Modifier[3] = settings.value("exfl3", "").toFloat();
        EXF_L_Modifier[4] = settings.value("exfl4", "").toFloat();
    }

    for(int i=0; i<5; i++){
        sharedData->EXF_R_Modifier[i] = EXF_R_Modifier[i];
        sharedData->EXF_L_Modifier[i] = EXF_L_Modifier[i];
    }
}

void RBCMD_AttrSwitchingMode(){
    int id = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    int ch = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1];
    int mode = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2];   // 0->complementary 1->non-complementary
    if(mode != 0) mode = 1;

    if(id == -1){ // All
        for(int i=0; i<_NO_OF_MC; i++){
            _DEV_MC[i].RBBoard_SetSwitchingMode(mode);
        }
    }else{  // Each
        _DEV_MC[id].RBBoard_SetSwitchingMode(mode);
    }
}

void RBCMD_AttrFrictionCompensation(){
    int id = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    int ch = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1];
    int onoff = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2];   // 0->off
    if(onoff != 0) onoff = 1;

    if(id == -1){ // All
        for(int i=0; i<_NO_OF_MC; i++){
            for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
                _DEV_MC[i].RBJoint_EnableFrictionCompensation(j+1, onoff);
            }
        }
    }else{  // Each
        _DEV_MC[id].RBJoint_EnableFrictionCompensation(ch+1, onoff);
    }
}

void RBCMD_AttrControlMode(){
    int id = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    int ch = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1];
    int mode = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2];   // 0->pos 2->cur 3->pwm 4->pos+pwm

    if(id == -1){ // All
        for(int i=0; i<_NO_OF_MC; i++){
            for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
                _DEV_MC[i].Joints[j].ControlMode = mode;
            }
        }
    }else{  // Each
        _DEV_MC[id].Joints[ch].ControlMode = mode;
    }
}

void RBCMD_SensorEncoderReset(){
    int id = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    int ch = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1];

    if(id == -1){ // All
        for(int i=0; i<_NO_OF_MC; i++){
            for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
                _DEV_MC[i].RBJoint_ResetEncoder(j+1);
                sharedData->MotionOwner[i][j] = RBCORE_PODO_NO;

                for(int k=0; k<_NO_OF_AL; k++){
                    sharedData->JointReference[k][i][j] = 0.0;
                }
                _DEV_MC[i].Joints[j].Reference = 0.0;
                _DEV_MC[i].MoveJoints[j].RefAngleCurrent = 0.0;
            }
        }
    }else{  // Each
        _DEV_MC[id].RBJoint_ResetEncoder(ch+1);
        sharedData->MotionOwner[id][ch] = RBCORE_PODO_NO;
        for(int k=0; k<_NO_OF_AL; k++){
            sharedData->JointReference[k][id][ch] = 0.0;
        }
        _DEV_MC[id].Joints[ch].Reference = 0.0;
        _DEV_MC[id].MoveJoints[ch].RefAngleCurrent = 0.0;
    }
}

void RBCMD_SensorEncoderOnOff(){
    int onoff = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    if(onoff != 0) onoff = 1;

    if(onoff == 1){ //on
        for(int i=0; i<_NO_OF_MC; i++){
            _DEV_MC[i].RBBoard_RequestEncoder(1);   //continuous
            _ENCODER_ENABLED = true;
        }
    }else{  //off
        for(int i=0; i<_NO_OF_MC; i++){
            _DEV_MC[i].RBBoard_RequestEncoder(0);   //oneshot
            _ENCODER_ENABLED = false;
        }
    }
}

void RBCMD_SensorSensorOnOff(){
    int onoff = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    if(onoff != 0) onoff = 1;

    if(onoff == 1){ //on
        _SENSOR_ENABLED = true;
    }else{  //off
        _SENSOR_ENABLED = false;
    }
}

void RBCMD_SensorFTNull(){
    int id = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];

    if(id == -1){   //All
        for(int i=0; i<_NO_OF_FT; i++){
            _DEV_FT[i].RBFT_Nulling(0);
        }
    }else{  //Each
        _DEV_FT[id].RBFT_Nulling(0);
    }
}

void RBCMD_SensorIMUNull(){
    int id = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];

    if(id == -1){   //All
        for(int i=0; i<_NO_OF_IMU; i++){
            _DEV_IMU[i].RBIMU_RequestNulling();
        }
    }else{  //Each
        _DEV_IMU[id].RBIMU_RequestNulling();
    }
}

void RBCMD_SensorIMUOffsetSet(){
    int imu = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    float roll = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_FLOAT[0];
    float pitch = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_FLOAT[1];

    _DEV_IMU[imu].ROLL_OFFSET = roll;
    _DEV_IMU[imu].PITCH_OFFSET = pitch;
}

void RBCMD_SensorFOGNull(){
    int nullzero = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    if(nullzero != 0) nullzero = 1;

    if(nullzero == 0){  // Null
        _DEV_FOG.FOGNullFlag = true;
    }else{  // Zero

        _DEV_IMU[0].FOG_ROLL_OFFSET  = -_DEV_IMU[0].FOG_ROLL_NULL     + (_DEV_IMU[0].ACC_X);
        _DEV_IMU[0].FOG_PITCH_OFFSET = -_DEV_IMU[0].FOG_PITCH_NULL    + (_DEV_IMU[0].ACC_Y);
        _DEV_IMU[0].FOG_YAW_OFFSET   = -_DEV_IMU[0].FOG_YAW_NULL      + (_DEV_IMU[0].ACC_Z);

        _DEV_FOG.FOGRoll   = _DEV_IMU[0].ACC_X;
        _DEV_FOG.FOGPitch  = _DEV_IMU[0].ACC_Y;
        _DEV_FOG.FOGYaw    = _DEV_IMU[0].ACC_Z;
    }
}

void RBCMD_SensorFOGUSBReset(){
    _DEV_FOG.RBClosePort();
    usleep(50*1000);
    _DEV_FOG.RBOpenPort(B460800);
}

void RBCMD_SensorOFNull(){
    for(int i=0; i<_NO_OF_OF; i++){
        _DEV_OF[i].RBOF_ResetValue();
    }
    usleep(100);
    ODHandler.OD_Zero();
}

void RBCMD_SensorOFLampOnOff(){
    int onoff = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    if(onoff != false) onoff = 1;

    for(int i=0; i<_NO_OF_OF; i++){
        _DEV_OF[i].RBOF_LampOnOff(onoff);
    }
}

void RBCMD_MotionRefOnOff(){
    int onoff = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    _REFERENCE_ENABLED = onoff;

    for(int i=0; i<_NO_OF_MC; i++){
        _DEV_MC[i].RBBoard_ReferenceOutEnable(onoff);
    }
}

void RBCMD_MotionMove(){
    int id = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    int ch = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1];
    int mode = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2];
    float time = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_FLOAT[0];
    float ang = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_FLOAT[1];

    _DEV_MC[id].RBJoint_SetMoveJoint(ch, ang, time, mode);
    sharedData->MotionOwner[id][ch] = RBCORE_PODO_NO;
}

void RBCMD_MotionGainOverride(){
    if(!_CANOUT_ENABLED)
        return;

    int id = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    int ch = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1];
    float time = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_FLOAT[0];
    float gain = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_FLOAT[1];

    _DEV_MC[id].RBJoint_GainOverride(ch+1, gain, time);
}

void RBCMD_MotionErrorClear(){
    int id = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    int ch = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1];
    int mode = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2];
    if(mode != 0) mode = 1;


    ErrorClearStart = true;
    if(mode == 0){  // just error clear
        if(id == -1){ // All
            for(int i=0; i<_NO_OF_MC; i++){
                for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
                    _DEV_MC[i].RBJoint_ClearErrorFlag(j+1);
                }
            }
        }else{  // Each
            _DEV_MC[id].RBJoint_ClearErrorFlag(ch+1);
        }
    }else{          // error clear + joint recovery
        // reference out disable & get motion ownership for all joints
        for(int i=0; i<_NO_OF_MC; i++){
            _DEV_MC[i].RBBoard_ReferenceOutEnable(false);
            for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
                sharedData->MotionOwner[i][j] = RBCORE_PODO_NO;
            }
        }

        // encoder enable
        for(int i=0; i<_NO_OF_MC; i++){
            _DEV_MC[i].RBBoard_RequestEncoder(1);
        }

        // sleep for encoder read
        usleep(30*1000);

        for(int i=0; i<_NO_OF_MC; i++){
            for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
                // update reference with encoder (exception RWH LWH RHAND LHAND)
                if((i == 4 && j == 1) || (i == 10 && j == 1) || (i == 21 && j == 1) || (i == 22 && j == 1)){
                    _DEV_MC[i].MoveJoints[j].RefAngleCurrent = 0.0;
                }else{
                    _DEV_MC[i].MoveJoints[j].RefAngleCurrent = sharedData->ENCODER[i][j].CurrentPosition;
                }
            }
        }

        if(id == -1){ // All
            for(int i=0; i<_NO_OF_MC; i++){
                for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
                    // error clear
                    _DEV_MC[i].RBJoint_ClearErrorFlag(j+1);

                    // FET on & CTRL on
                    _DEV_MC[i].RBJoint_EnableFETDriver(j+1, true);
                    _DEV_MC[i].RBJoint_EnableFeedbackControl(j+1, true);
                }
            }
        }else{  // Each
            // error clear
            _DEV_MC[id].RBJoint_ClearErrorFlag(ch+1);

            // FET on & CTRL on
            _DEV_MC[id].RBJoint_EnableFETDriver(ch+1, true);
            _DEV_MC[id].RBJoint_EnableFeedbackControl(ch+1, true);
        }

        // wait for settling
        usleep(10*1000);

        // reference out enable
        for(int i=0; i<_NO_OF_MC; i++){
            for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
                _DEV_MC[i].RBBoard_ReferenceOutEnable(true);
            }
        }
    }
    ErrorClearStart = false;
}

void RBCMD_CANEnableDisable(){
    _CANOUT_ENABLED = sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
}

//==============================================================================
int RBCore_Initialize(void){
    cout << endl;
    FILE_LOG(logERROR) << "==========Initializing===========";

    IS_WORKING = true;

    // Shared Memory initialize
    if(RBCore_SMInitialize() == false)
        return false;

    // Load RBCore configuration file
    if(RBCore_DBInitialize() == false)
        return false;

    // CAN & LAN Communcation initialize
    if(__IS_GAZEBO){
        if(RBCore_LANInitialize() == false){
            FILE_LOG(logERROR) << "KK";
            return false;
        }
    }else{
        RBCore_CANInitialize();
    }

    // Real-time thread initialize
    if(RBCore_ThreadInitialize() == false)
        return false;

    // Process Manager initialize
    if(RBCore_PMInitialize() == false)
        return false;

    // FOG [RS-232] initialize
    if(!__IS_GAZEBO && __IS_FOG){
        if(_DEV_FOG.RBOpenPort(B460800) != FOG_PORT_SUCCESS){
            IS_RS232_OK = false;
            return false;
        }else{
            IS_RS232_OK = true;
        }
    }

    FILE_LOG(logERROR) << "=================================";
    cout << endl;

    sharedData->PODO_AL_WORKING[RBCORE_PODO_NO] = true;
    IS_WORKING = true;
    return true;
}
//---------------
int RBCore_SMInitialize(){
    shm_unlink(RBCORE_SHM_NAME);

    int shmFD;
    // Core Shared Memory Creation [Reference]==================================
    shmFD = shm_open(RBCORE_SHM_NAME, O_CREAT|O_RDWR, 0666);
    if(shmFD == -1){
        FILE_LOG(logERROR) << "Fail to open core shared memory";
        return -1;
    }else{
        if(ftruncate(shmFD, sizeof(RBCORE_SHM)) == -1){
            FILE_LOG(logERROR) << "Fail to truncate core shared memory";
            return -1;
        }else{
            sharedData = (pRBCORE_SHM)mmap(0, sizeof(RBCORE_SHM), PROT_READ|PROT_WRITE, MAP_SHARED, shmFD, 0);
            if(sharedData == (void*)-1){
                FILE_LOG(logERROR) << "Fail to mapping core shared memory";
                return -1;
            }
        }
    }
    FILE_LOG(logSUCCESS) << "Core shared memory creation = OK";
    // =========================================================================

    shmFD = shm_open("GAZEBO_SHM", O_CREAT|O_RDWR, 0666);
    if(shmFD == -1){
        FILE_LOG(logERROR) << "Fail to open GAZEBO shared memory";
        return -1;
    }else{
        if(ftruncate(shmFD, sizeof(int)) == -1){
            FILE_LOG(logERROR)<< "Fail to truncate GAZEBO shared memory";
            return -1;
        }else{
            GAZEBO_FLAG = (int *)mmap(0, sizeof(int), PROT_READ|PROT_WRITE, MAP_SHARED, shmFD, 0);
            if(GAZEBO_FLAG == (void*)-1){
                FILE_LOG(logERROR) << "Fail to mapping GAZEBO shared memory";
                return -1;
            }
        }
    }
    FILE_LOG(logSUCCESS) << "GAZEBO shared memory creation = OK";
    // =========================================================================

    return true;
}
//---------------
int RBCore_DBInitialize(){
    RBDataBase DB;
    DB.SetFilename("Core_Config.db");
    if(DB.OpenDB() == false){
        FILE_LOG(logERROR) << "Fail to load database file";
        return false;
    }

    _VERSION        = RBDataBase::_DB_GENERAL.VERSION;
    _NO_OF_AL       = RBDataBase::_DB_GENERAL.NO_OF_AL;
    _NO_OF_COMM_CH  = RBDataBase::_DB_GENERAL.NO_OF_COMM_CH;
    _NO_OF_MC       = RBDataBase::_DB_GENERAL.NO_OF_MC;
    _NO_OF_FT       = RBDataBase::_DB_GENERAL.NO_OF_FT;
    _NO_OF_IMU      = RBDataBase::_DB_GENERAL.NO_OF_IMU;
    _NO_OF_SP       = RBDataBase::_DB_GENERAL.NO_OF_SP;
    _NO_OF_OF       = RBDataBase::_DB_GENERAL.NO_OF_OF;

    FILE_LOG(logSUCCESS) << "Core load database = OK";

    std::cout << "----------------------------------" << std::endl;
    std::cout << "     VERSION       : " << _VERSION << std::endl;
    std::cout << "     NO_OF_AL      : " << _NO_OF_AL << std::endl;
    std::cout << "     NO_OF_COMM_CH : " << _NO_OF_COMM_CH << std::endl;
    std::cout << "     NO_OF_MC      : " << _NO_OF_MC << std::endl;
    std::cout << "     NO_OF_FT      : " << _NO_OF_FT << std::endl;
    std::cout << "     NO_OF_IMU     : " << _NO_OF_IMU << std::endl;
    std::cout << "     NO_OF_SP      : " << _NO_OF_SP << std::endl;
    std::cout << "     NO_OF_OF      : " << _NO_OF_OF << std::endl;
    std::cout << "----------------------------------" << std::endl;

    for(int i=0; i<_NO_OF_MC; i++)   _DEV_MC[i].RBBoard_GetDBData(RBDataBase::_DB_MC[i]);
    for(int i=0; i<_NO_OF_FT; i++)   _DEV_FT[i].RBBoard_GetDBData(RBDataBase::_DB_FT[i]);
    for(int i=0; i<_NO_OF_IMU; i++)  _DEV_IMU[i].RBBoard_GetDBData(RBDataBase::_DB_IMU[i]);
    for(int i=0; i<_NO_OF_SP; i++)   _DEV_SP[i].RBBoard_GetDBData(RBDataBase::_DB_SP[i]);
    for(int i=0; i<_NO_OF_OF; i++)   _DEV_OF[i].RBBoard_GetDBData(RBDataBase::_DB_OF[i]);


    return true;
}
//---------------
int RBCore_CANInitialize(){
    canHandler = new RBCAN(_NO_OF_COMM_CH);

    if(canHandler->IsWorking() == false){
        IS_CAN_OK = false;
        return false;
    }else{
        _CANOUT_ENABLED = true;
        for(int i=0; i<_NO_OF_MC; i++)   _DEV_MC[i].RBMC_AddCANMailBox();
        for(int i=0; i<_NO_OF_FT; i++)   _DEV_FT[i].RBFT_AddCANMailBox();
        for(int i=0; i<_NO_OF_IMU; i++)  _DEV_IMU[i].RBIMU_AddCANMailBox();
        for(int i=0; i<_NO_OF_SP; i++)   _DEV_SP[i].RBSP_AddCANMailBox();
        for(int i=0; i<_NO_OF_OF; i++)   _DEV_OF[i].RBOF_AddCANMailBox();
        IS_CAN_OK = true;
        return true;
    }
}
//---------------
int RBCore_LANInitialize(){
    lanHandler = new RBRawLAN();
    if(lanHandler->RBLanOpen(8888) == false){
        FILE_LOG(logSUCCESS) << "qq";
        return false;
    }
    FILE_LOG(logSUCCESS) << "lan success";
    return true;
}
//---------------
int RBCore_ThreadInitialize(){
    if(__IS_GAZEBO){
        int theadID = pthread_create(&nrtTaskCon, NULL, &RBCore_NRTThreadCon, NULL);
        if(theadID < 0){
            FILE_LOG(logERROR) << "Fail to create core non real-time thread";
            return false;
        }
        FILE_LOG(logSUCCESS) << "Core non real-time thread start = OK";
    }else{
        if(rt_task_create(&rtTaskCon, "RBCORE_TASK", 0, 99, 0) == 0){
            cpu_set_t aCPU;
            CPU_ZERO(&aCPU);
            CPU_SET(0, &aCPU);
            if(rt_task_set_affinity(&rtTaskCon, &aCPU) != 0){
                FILE_LOG(logWARNING) << "Core real-time thread set affinity CPU failed..";
            }
            if(rt_task_start(&rtTaskCon, &RBCore_RTThreadCon, NULL) == 0){
                FILE_LOG(logSUCCESS) << "Core real-time thread start = OK";
            }else{
                FILE_LOG(logERROR) << "Core real-time thread start = FAIL";
                return false;
            }
        }else{
            FILE_LOG(logERROR) << "Fail to create core real-time thread";
            return false;
        }
    }

    return true;
}
//---------------
int RBCore_PMInitialize(){
    pmHandler = new RBProcessManager();

    int ret = pmHandler->OpenAL(1);
    if(ret == -99){
        IS_CHILD = true;
        IS_WORKING = false;
        return false;
    }
    return true;
}
//---------------
int RBCore_Termination(){
    if(IS_CHILD)
        return true;

    rt_task_delete(&rtTaskCon);
    shm_unlink(RBCORE_SHM_NAME);
    FILE_LOG(logERROR) << "RBCore will be terminated..";
    return true;
}
//==============================================================================




void THREAD_ReadSensor(){
    if(_SENSOR_ENABLED == false)
        return;

    for(int i=0; i<_NO_OF_IMU; i++){
        _DEV_IMU[i].RBIMU_ReadData();
        sharedData->IMU[i].Roll      = _DEV_IMU[i].ROLL;
        sharedData->IMU[i].Pitch     = _DEV_IMU[i].PITCH;
        sharedData->IMU[i].Yaw       = _DEV_IMU[i].YAW;
        sharedData->IMU[i].RollVel   = _DEV_IMU[i].ROLL_VEL;
        sharedData->IMU[i].PitchVel  = _DEV_IMU[i].PITCH_VEL;
        sharedData->IMU[i].YawVel    = _DEV_IMU[i].YAW_VEL;
        sharedData->IMU[i].AccX      = _DEV_IMU[i].ACC_X;
        sharedData->IMU[i].AccY      = _DEV_IMU[i].ACC_Y;
        sharedData->IMU[i].AccZ      = _DEV_IMU[i].ACC_Z;
    }

    for(int i=0; i<_NO_OF_FT; i++){
        _DEV_FT[i].RBFT_ReadData();
        sharedData->FT[i].Fx     = _DEV_FT[i].FX;
        sharedData->FT[i].Fy     = _DEV_FT[i].FY;
        sharedData->FT[i].Fz     = _DEV_FT[i].FZ;
        sharedData->FT[i].Mx     = _DEV_FT[i].MX;
        sharedData->FT[i].My     = _DEV_FT[i].MY;
        sharedData->FT[i].Mz     = _DEV_FT[i].MZ;

        if(i == 0){
            sharedData->FT[i].Fz = _DEV_FT[i].FZ * 8.0/7.0;
        }

        if(_DEV_FT[i].SENSOR_TYPE == 0){
            sharedData->FT[i].Roll       = _DEV_FT[i].ROLL;
            sharedData->FT[i].RollVel    = _DEV_FT[i].VelRoll;
            sharedData->FT[i].Pitch      = _DEV_FT[i].PITCH;
            sharedData->FT[i].PitchVel   = _DEV_FT[i].VelPitch;
        }
    }

    for(int i=0; i<_NO_OF_OF; i++){
        _DEV_OF[i].RBOF_ReadValue();
        sharedData->OF.AccumX[i] = _DEV_OF[i].AccumX;
        sharedData->OF.AccumY[i] = _DEV_OF[i].AccumY;
    }
    ODHandler.OD_MeasureState();
}

void THREAD_RequestSensor(){
    if(_SENSOR_ENABLED){
        for(int i=0; i<_NO_OF_FT; i++){
            if(_DEV_FT[i].SENSOR_TYPE == 0x00){
                _DEV_FT[i].RBFT_RequestData(0x04);
                _DEV_FT[i].RBFT_RequestData(0x00);
            }else{
                _DEV_FT[i].RBFT_RequestData(0x00);
            }
        }

        for(int i=0; i<_NO_OF_IMU; i++){
            _DEV_IMU[i].RBIMU_RequestData(0x00);
        }

        for(int i=0; i<_NO_OF_OF; i++){
            _DEV_OF[i].RBOF_RequestValue();
        }
    }
}

void THREAD_ReadEncoder(){
    if(_ENCODER_ENABLED == false)
        return;

    for(int i=0; i<_NO_OF_MC; i++){
        _DEV_MC[i].RBBoard_ReadEncoderData();
        for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
            sharedData->ENCODER[i][j].CurrentPosition = _DEV_MC[i].Joints[j].CurrentPosition;
            sharedData->ENCODER[i][j].CurrentVelocity = _DEV_MC[i].Joints[j].CurrentVelocity;
        }
    }
}

void THREAD_RequestEncoder(){
    if(_ENCODER_ENABLED){
        ;
    }
}

void THREAD_ReadTemperature(){
    if(_SENSOR_ENABLED){
        int mc = _ThreadCnt%_NO_OF_MC;
        _DEV_MC[mc].RBBoard_ReadTemperature();
        sharedData->MCTemperature[mc] = _DEV_MC[mc].BoardTemperature;
        for(int i=0; i<_DEV_MC[mc].MOTOR_CHANNEL; i++){
            sharedData->MotorTemperature[mc][i] = _DEV_MC[mc].Joints[i].Temperature;
        }
    }
}

void THREAD_RequestTemperature(){
    int mc = _ThreadCnt%_NO_OF_MC;

    // temperature
    if(_SENSOR_ENABLED){
        _DEV_MC[mc].RBBoard_RequestTemperature();
    }

    // home & error
    if(_DEV_MC[mc].ConnectionStatus == true){
        StatusReadFlag[mc] = false;
        _DEV_MC[mc].RBBoard_RequestStatus();
    }
}


void THREAD_ReadVoltage(){
    if(_SENSOR_ENABLED){
        _DEV_SP[0].RBSP_ReadVoltageCurrent();
        sharedData->SP[0].Voltage = _DEV_SP[0].Voltage;
        sharedData->SP[0].Current = _DEV_SP[0].Current;
    }
}

void THREAD_RequestVoltage(){
    if(_SENSOR_ENABLED){
        _DEV_SP[0].RBSP_RequestVoltageAndCurrent();
    }
}


void THREAD_ReadHomeError(){
    static unsigned int StatusErrorCnt[MAX_MC] = {0,};

    //if(_SENSOR_ENABLED){
    int mc = _ThreadCnt%_NO_OF_MC;

    if(_DEV_MC[mc].ConnectionStatus == true){
        if(_DEV_MC[mc].RBBoard_GetStatus() == true){
            StatusErrorCnt[mc] = 0;
            StatusReadFlag[mc] = true;

            for(int j=0; j<_DEV_MC[mc].MOTOR_CHANNEL; j++){
                sharedData->MCStatus[mc][j] = _DEV_MC[mc].Joints[j].CurrentStatus;
                sharedData->MCStatus[mc][j].b.CUR = 0;
                if(sharedData->MCStatus[mc][j].b.BIG == 1 || sharedData->MCStatus[mc][j].b.INP == 1 ||
                        sharedData->MCStatus[mc][j].b.ENC == 1 || sharedData->MCStatus[mc][j].b.JAM == 1 )
                {
                    if(ErrorClearStart == false){
                        //FILE_LOG(logERROR) << "Need Recover Activated.. Reason: Servo[" << mc << ", " << j << "] Off with Error(" << (int)(sharedData->MCStatus[mc][j].B[1]) << ")";
                        //NeedJointRecover = true;
                    }
                }
            }
        }else if(_SENSOR_ENABLED && StatusReadFlag[mc] == false){
            StatusErrorCnt[mc]++;
            if(StatusErrorCnt[mc] > 3){
                if(ErrorClearStart == false){
                    for(int j=0; j<_DEV_MC[mc].MOTOR_CHANNEL; j++){
                        sharedData->MCStatus[mc][j].b.CUR = 1;
                    }
                    //FILE_LOG(logERROR) << "Need Recover Activated.. Reason: No Status Return";
                    //NeedCANRecover = true;
                }
            }

            if(StatusErrorCnt[mc]%3 == 0)
                FILE_LOG(logERROR) << "Status Get Error from Board " << mc << " Cnt : " << StatusErrorCnt[mc];

        }
    }

    //}
}
