/*
 *	This program is generated from drc-podoal-template.
 *
 *	This AL-Process will activated by "PODO_Daemon" with appropriate "AL-Number".
 *	The AL-Number is determined at Core_config.db file at Share folder.
 *	And the AL-Process needs this AL-Number as an input parameter.
 *	So trying to open the AL-Process without any input parameter will terminate the process.
 *
 *	Please check your PODO_AL_NAME and Core_Config.db file.
 *	Actually the PODO_AL_NAME is used for recognizing the certain process and making it unique.
 *	So the same name of file with different path is allowed if the PODO_AL_NAMEs are different.
 *	But we recommend you that gather all of your build file in one folder (check the Core_Config.db file).
 *
 *	You can change the period of real-time thread by changing "RT_TIMER_PERIOD_MS" as another value in "rt_task_set_periodic".
 *	Please do not change "RT_TIMER_PERIOD_MS" value in "typedef.h".
 *	You can also change the priority of the thread by changing 4th parameter of "rt_task_create".
 *	In this function you need to care about the name of thread (the name should be unique).
 *
 *	Please do not change the "RBInitialize" function and fore code of main().
 *	You may express your idea in while loop of main & real-time thread.
 *
 *	Each AL-Process has its own command structure in Shared Memory.
 *	So, it can have its own command set.
 *	Make sure that the command set of each AL-process start from over than 100.
 *	Under the 100 is reserved for common command set.
 *
 *	Now, you can do everything what you want..!!
 *	If you have any question about PODO, feel free to contact us.
 *	Thank you.
 *
 *
 *
 *	Jungho Lee		: jungho77@rainbow.re.kr
 *	Jeongsoo Lim	: yjs0497@kaist.ac.kr
 *	Okkee Sim		: sim2040@kaist.ac.kr
 *
 *	Copy Right 2014 @ Rainbow Co., HuboLab of KAIST
 *
 */


#include <alchemy/task.h>


#include <QCoreApplication>

#include <iostream>
#include <string>
#include <sys/mman.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>

#include "joint.h"
#include "taskmotion.h"
#include "wmupperbody.h"
#include "ManualCAN.h"


#define PODO_AL_NAME       "WMupperbody_AL"

using namespace std;

// Functions ===========================================
// Signal handler
void CatchSignals(int _signal);
int HasAnyOwnership();
int AskALNumber(char *alname);

// Real-time thread for control
void RBTaskThread(void *);
void RBFlagThread(void *);

// Initialization
int RBInitialize(void);
// =====================================================

// Variables ===========================================
// Shared memory
pRBCORE_SHM     sharedData;
pUSER_SHM       userData;

// RT task handler for control
RT_TASK rtTaskCon;
RT_TASK rtFlagCon;

int isTerminated;
// PODO No.
int PODO_NO;
int PODO_NO_WHEEL = 4;
int PODO_NO_WALK = 2;
int PODO_NO_WALKREADY = 3;
// --------------------------------------------------------------------------------------------- //

// WBIK functins-------------------------------------------------------------------------------- //
JointControlClass *joint;
// CKINE_DRC_HUBO kine_drc_hubo;
int CheckMotionOwned();
// Variables
int             WB_FLAG = false;
int             WB_GLOBAL_FLAG = false;
int             WALK_WST_FLAG = false;
long            LimitedJoint;
long            LimitType;

void TurnOff_WBIK(){
    WB_FLAG = false;
    WB_GLOBAL_FLAG = false;
    usleep(10*1000);
}

// Functions
//int	PushCANMessage(MANUAL_CAN MCData);
//int RBJointGainOverride(unsigned int _canch, unsigned int _bno, unsigned int _override1, unsigned int _override2, unsigned int _duration);
//int RBBoardSetSwitchingMode(unsigned int _canch, unsigned int _bno, int _mode);
//int RBenableFrictionCompensation(unsigned int _canch, unsigned int _bno, unsigned int _enable1, unsigned int _enable2);
//int RBwFTsensorNull(unsigned int _canch, unsigned int _bno);
//int RBindicator(unsigned char number);//0 off n n*100ms on/off -1 on 0x85 0,-1 on off

TaskMotion      *WBmotion;
// WBIK functins-------------------------------------------------------------------------------- //

// Door
// Motion Function------------------------------------------------------------------------------ //
int PullDoor_GrabAndOpen(int _mode);
int PullDoor_OpenDoorAndLHblock(int _mode);
int PullDoor_RHbackAndSmallRotate(void);
int PullDoor_RHblockAndReadyGGu(void);

// EM motion
int EM_ReturnHand();
int EM_RightHandBack();
int EM_LeftHandBack();
int EM_RotateMore(const char rot_dir, const char fb_dir, const double w_radius, const double w_angle);


// Sub motion
void RotateDoorPushHandle(int rot_direction);
void GGuGofunction(double _tar_x, double _tar_y, double _tar_a);
void PreDefinedMovement(double dx, double dy, double da);
// Motion Function------------------------------------------------------------------------------ //



// Door In
// Motion Function------------------------------------------------------------------------------ //
// Basic
int PushDoor_GrabAndOpen_LEFT(int _mode);
int PushDoor_LHblock_RHback_Gothrough_LEFT(int _mode);

// EM motion
int EM_PushDoor_ReturnHand_LEFT(void);
int EM_PushDoor_ReturnLH_LEFT(void);

// sub motion
void PushDoor_RotateHandle_LEFT(int rot_direction);

// Basic
int PushDoor_GrabAndOpen(int _mode);
int PushDoor_LHblock_RHback_Gothrough(int _mode);

// EM motion
int EM_PushDoor_ReturnHand(void);
int EM_PushDoor_ReturnLH(void);

// sub motion
void PushDoor_RotateHandle(int rot_direction);

// Push Saved vector

// Common
vec3 EM_WHEELPUSH_LH_pos_1;
vec3 EM_WHEELPUSH_LH_pos_2;
quat EM_WHEELPUSH_LH_ori_1;
quat EM_WHEELPUSH_LH_ori_2;
// Motion Function------------------------------------------------------------------------------ //

// Walking Push Door In
// Motion Function------------------------------------------------------------------------------ //
// Basic
int WalkPushDoor_GrabAndOpen(int _mode);
int WalkPushDoor_LHblock_RHback_RHblock(int _mode);
int WalkPushDoor_RotateWST_ReadyPass(int _mode);

// EM
int EM_WalkPushDoor_ReturnHand(void);
int EM_WalkPushDoor_ReturnLH(void);

// Basic
int WalkPushDoor_GrabAndOpen_LEFT(int _mode);
int WalkPushDoor_LHblock_RHback_RHblock_LEFT(int _mode);
int WalkPushDoor_RotateWST_ReadyPass_LEFT(int _mode);

// EM
int EM_WalkPushDoor_ReturnHand_LEFT(void);
int EM_WalkPushDoor_ReturnLH_LEFT(void);

vec3 walkpush_save_vec;
vec3 EM_WALKPUSH_LH_pos_1;
vec3 EM_WALKPUSH_LH_pos_2;
quat EM_WALKPUSH_LH_ori_1;
quat EM_WALKPUSH_LH_ori_2;

void WalkPushDoor_RotateHandle(int rot_direction);
// Motion Function------------------------------------------------------------------------------ //


// Hose
// Motion Function------------------------------------------------------------------------------ //
int HOSE_GRAB_HOSE(void);
int HOSE_AFTER_GRAB(const double des_rotate_angle);
int HOSE_PLUG_WYE(void);
int HOSE_RETURN_FROM_WYE(void);

// EM motion
int EM_ReturnHand_From_HOSEGRAB(void);
int EM_ReturnHand_From_WYE(void);
// Motion Function------------------------------------------------------------------------------ //

// PLUG
vec3 pelv_plug_hand[2];
vec3 pelv_align_hand[2];
vec3 pelv_out_hand[2];
vec3 pelv_ap_hand[2];
doubles plug_ds_des_H_ori(4);

double plug_elb[2];
double plug_wst[2];
vec3 plug_offset;
int plug_dis_num = 10;
vec3 _plug_vecX = vec3(1,0,0);
vec3 _plug_vecY = vec3(0,1,0);
vec3 _plug_vecZ = vec3(0,0,1);
// Super freeSleep--------------------------------------------------------------------------------- //
int freeSleep(const int usecond);
// Super freeSleep--------------------------------------------------------------------------------- //

// For rotation--------------------------------------------------------------------------------- //
vec3 handle_rotate_center, handle_rotate_axis;
vec3 hyo_debug;
pos handle_pos_first;
double handle_target_angle;
int handle_dir;

double handle_angle_now=0.;
double current_anglepersec, max_angleAcc = 60;

bool isRotating;
// For rotation--------------------------------------------------------------------------------- //


int     __IS_GAZEBO = false;

void CheckArguments(int argc, char *argv[]){
    int opt = 0;
    int podoNum = -1;
    while((opt = getopt(argc, argv, "g:p:")) != -1){
        switch(opt){
        case 'g':
            if(strcmp(optarg, "true")==0 || strcmp(optarg, "TRUE")==0){
                __IS_GAZEBO = true;
            }else if(strcmp(optarg, "false")==0 || strcmp(optarg, "FALSE")==0){
                __IS_GAZEBO = false;
            }else{
                FILE_LOG(logERROR) << optarg;
                FILE_LOG(logERROR) << "Invalid option for Gazebo";
                FILE_LOG(logERROR) << "Valid options are \"true\", \"TRUE\", \"false\", \"FALSE\"";
                FILE_LOG(logERROR) << "Use default value";
            }
            break;
        case 'p':
            podoNum = atoi(optarg);
            if(podoNum == 0){
                FILE_LOG(logERROR) << optarg;
                FILE_LOG(logERROR) << "Invalid option for AL";
                FILE_LOG(logERROR) << "Valid options are \"true\", \"TRUE\", \"false\", \"FALSE\"";
                FILE_LOG(logERROR) << "Use default value";
            }else{
                PODO_NO = podoNum;
            }
            break;
        case '?':
            if(optopt == 'g'){
                FILE_LOG(logERROR) << "Option for Gazebo";
                FILE_LOG(logERROR) << "Valid options are \"true\", \"TRUE\", \"false\", \"FALSE\"";
            }else if(optopt == 'p'){
                FILE_LOG(logERROR) << "Option for AL";
                FILE_LOG(logERROR) << "Valid options are \"Integer Values\"";
            }
        }
    }


    cout << endl;
    FILE_LOG(logERROR) << "===========AL Setting============";
    FILE_LOG(logWARNING) << argv[0];
    if(__IS_GAZEBO)     FILE_LOG(logWARNING) << "AL for Gazebo";
    else                FILE_LOG(logWARNING) << "AL for Robot";
    FILE_LOG(logWARNING) << "AL Number: " << PODO_NO;
    FILE_LOG(logERROR) << "=================================";
    cout << endl;
}


int main(int argc, char *argv[])
{
    isRotating = false;
    // Termination signal ---------------------------------
    signal(SIGTERM, CatchSignals);   // "kill" from shell
    signal(SIGINT, CatchSignals);    // Ctrl-c
    signal(SIGHUP, CatchSignals);    // shell termination
    signal(SIGKILL, CatchSignals);

    // Block memory swapping ------------------------------
    mlockall(MCL_CURRENT|MCL_FUTURE);

    // Get PODO No. ---------------------------------------
    CheckArguments(argc, argv);
    if(PODO_NO == -1){
        FILE_LOG(logERROR) << "Please check the AL Number";
        FILE_LOG(logERROR) << "Terminate this AL..";
        return 0;
    }
//    if(argc == 1){
//        cout << ">>> No input argument..!!\n";
//        return 0;
//    }
//    else{
//        QString argStr;
//        argStr.sprintf("%s", argv[1]);
//        PODO_NO = argStr.toInt();
//        cout << "======================================================================" << endl;
//        cout << ">>> Process WMupperbody is activated..!!" << endl;
//        cout << ">>> PODO NAME: WMupperbody_AL" << endl;
//        cout << ">>> PODO NO: " << PODO_NO << endl;
//    }

    // Initialize RBCore -----------------------------------
    if( RBInitialize() == -1 ) isTerminated = -1;
    cout << "======================================================================" << endl;


    userData->EmergencyFlag = 5;

    PODO_NO_WHEEL = 4;
    FILE_LOG(logINFO) << "OMNIWHEEL AL number : " << PODO_NO_WHEEL;
    PODO_NO_WALK = 2;
    FILE_LOG(logINFO) << "Walk AL number : " << PODO_NO_WALK;
    PODO_NO_WALKREADY = 3;
    FILE_LOG(logINFO) << "WalkReady AL number : " << PODO_NO_WALKREADY;
    usleep(500*1000);

    // WBIK Initialize--------------------------------------
    WBmotion = new TaskMotion(sharedData, joint);
    // User command cheking --------------------------------
    while(isTerminated == 0){
        usleep(100*1000);

        switch(sharedData->COMMAND[PODO_NO].USER_COMMAND)
        {
        case WMupperbody_AL_PREDEFINED:
            {
            int movenum = sharedData->COMMAND[PODO_NO].USER_PARA_INT[0];
            switch(movenum){
            case 1:// Door -> Hose
            {
                PreDefinedMovement(0.35, 0.7, 0.);
                break;
            }
            case 2:// Hose -> Wye
            {
                PreDefinedMovement(2.65, 0.6, 90.);
                break;
            }
            case 3:// Wye -> Drill
            {
                PreDefinedMovement(-0.5, -0.5, -179);
                break;
            }

            case 4://After Valve
            {
                PreDefinedMovement(0.0, 0.0, 90.);
                PreDefinedMovement(1.4, 0.0, -90.);
                usleep(50*1000);
                sharedData->STATE_COMMAND = TCMD_WHEEL_PRE_DEFINED_FINISH;
                break;
            }
            case 5://After Drill
            {
                PreDefinedMovement(-1.7, 0.0, 90.);
                usleep(50*1000);
                sharedData->STATE_COMMAND = TCMD_WHEEL_PRE_DEFINED_FINISH;
                break;
            }
            case 6://After Surprise
            {
                PreDefinedMovement(-0.6, 0.0, 90.);
                usleep(50*1000);
                sharedData->STATE_COMMAND = TCMD_WHEEL_PRE_DEFINED_FINISH;
                break;
            }

            default :
                break;
            }
            sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
            break;
            }
        case WMupperbody_AL_GENERAL:
            if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0]==0) // WBIK MODE SET
            {
                // Save
                SaveFile();
                printf("Save File Done...!!!\n");
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0]==1) // WBIK MODE SET
            {
                // Reset Global
                printf("WMupper: Reset Global...!!!\n");
                joint->RefreshToCurrentReference();
                joint->SetAllMotionOwner();
                if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[1]==1)
                    StartWBIKmotion(WBmode_RULU);
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[1]==2)
                    StartWBIKmotion(WBmode_RULP);
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[1]==3)
                    StartWBIKmotion(WBmode_RPLU);
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[1]==4)
                    StartWBIKmotion(WBmode_RPLP);
                else
                    StartWBIKmotion(WBmode_RULU);
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0]==2) // WBIK SHOW INFO
            {
                // Show Info
                PrintWBIKinfo();
                GetPosFromENC();
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0]==3)
            {
                sharedData->COMMAND[0].USER_PARA_CHAR[0] = 1;       //enable
                sharedData->COMMAND[0].USER_COMMAND = DAEMON_SENSOR_ENCODER_ONOFF;
                usleep(200*1000);

                // Ready to start
                joint->RefreshToCurrentReference();
                joint->SetAllMotionOwner();

                double des_x = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                double des_y = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
                double des_a = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];
                _Task_WST_ANGLE = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[3];

                TurnOff_WBIK();
                Change_Arm_pos_Task_with_WST_READY(_Task_WST_ANGLE, 1600);
                freeSleep(1610*1000);

                sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[0]=des_x;
                sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[1]=des_y;
                sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[2]=des_a;
                sharedData->COMMAND[PODO_NO_WHEEL].USER_COMMAND= OMNIWHEEL_AL_GOTODES;
                userData->WheelDoneFlag = false;
                whilecnt=0;
                while(1){
                    if(userData->WheelDoneFlag == true){
                        userData->WheelDoneFlag = false;
                        //printf("Wheel is ended...! next step is ready to start\n");
                        break;
                    }
                    whilecnt++;
                    if(whilecnt>=2400){
                        //printf("Wheel is not!!! ended...! time gap out\n");
                        break;
                    }
                    sck = freeSleep(50*1000);
                    if(sck==1) break;
                }
                Change_Arm_pos_Task_with_WST(_Task_WST_ANGLE, 1000);
                freeSleep(1000*1000);
                printf("----------------- Pull Door Motion 0/4 End...!!!--------------------\n");

                sharedData->STATE_COMMAND = TCMD_DOOR_GO_NEARGO;
            }
            sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
            break;
        case WMupperbody_AL_POS:
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();
            if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0]==1)
            {
                // goto Task pos
                printf("WMupper: Task Pos...!!!\n");
                TurnOff_WBIK();
                Change_Arm_pos_Task(3000, 0);
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0]==2)
            {
                // goto Move pos
                printf("WMupper: Move Pos...!!!\n");
                TurnOff_WBIK();
                Change_Arm_pos_Move(3000);
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0]==3)
            {
                // goto Task pos with WST ori
                printf("WMupper: TASK Pos with WST...!!!\n");
                TurnOff_WBIK();
                double temp_WST=sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                Change_Arm_pos_Task_with_WST(temp_WST, 2500);
                freeSleep(2500*1000);
            }
            sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
            break;
        case WMupperbody_AL_HAND:
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();
            switch(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0])
            {
            case RIGHT_HAND:
                if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[1]==1)//Bind
                    joint->SetMoveJoint(RF2, 125, 10, MODE_ABSOLUTE);
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[1]==0)//Un Bind
                    joint->SetMoveJoint(RF2, -125, 10, MODE_ABSOLUTE);
                else
                    joint->SetMoveJoint(RF2, 0, 10, MODE_ABSOLUTE);
                break;
            case LEFT_HAND:
                if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[1]==1)//Bind
                    joint->SetMoveJoint(LF2, 125, 10, MODE_ABSOLUTE);
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[1]==0)//Un Bind
                    joint->SetMoveJoint(LF2, -125, 10, MODE_ABSOLUTE);
                else
                    joint->SetMoveJoint(LF2, 0, 10, MODE_ABSOLUTE);
                break;
            }
            sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
            break;
        case WMupperbody_AL_CONTROL:
            if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0]==1)
            {
                //Gain control;
                if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[1]==1){
                    //reduce
                    MCBoardSetSwitchingMode(2, 15, 1);//RSY REB
                    MCBoardSetSwitchingMode(2, 16, 1);//RWY RWP
                    MCJointGainOverride(2, 15, 1, eq_gain_1, 1000);//RSY
                    MCJointGainOverride(2, 15, 2, eq_gain_1, 1000);//REB
                    MCJointGainOverride(2, 16, 1, eq_gain_0, 1000);//RWY
                    MCJointGainOverride(2, 16, 2, eq_gain_1, 1000);//RWP

                    MCBoardSetSwitchingMode(3, 19, 1);//LSY LEB
                    MCBoardSetSwitchingMode(3, 20, 1);//LWY LWP
                    MCJointGainOverride(3, 19, 1, eq_gain_1, 1000);//LSY
                    MCJointGainOverride(3, 19, 2, eq_gain_1, 1000);//LEB
                    MCJointGainOverride(3, 20, 1, eq_gain_0, 1000);//LWY
                    MCJointGainOverride(3, 20, 2, eq_gain_1, 1000);//LWP

//                    RBBoardSetSwitchingMode(2,15,1);
//                    RBBoardSetSwitchingMode(2,16,1);

//                    unsigned int temp_gain=1;
//                    RBJointGainOverride(2,15,temp_gain,temp_gain,1000);//RSY REB
//                    RBJointGainOverride(2,16,0,temp_gain,1000);//RWY RWP
                }else{
                    //return
                    MCBoardSetSwitchingMode(2, 15, 0);//RSY REB
                    MCBoardSetSwitchingMode(2, 16, 0);//RWY RWP
                    MCJointGainOverride(2, 15, 1, eq_gain_1000, 2000);//RSY
                    MCJointGainOverride(2, 15, 2, eq_gain_1000, 2000);//REB
                    MCJointGainOverride(2, 16, 1, eq_gain_1000, 2000);//RWY
                    MCJointGainOverride(2, 16, 2, eq_gain_1000, 2000);//RWP

                    MCBoardSetSwitchingMode(3, 19, 0);//LSY LEB
                    MCBoardSetSwitchingMode(3, 20, 0);//LWY LWP
                    MCJointGainOverride(3, 19, 1, eq_gain_1000, 2000);//LSY
                    MCJointGainOverride(3, 19, 2, eq_gain_1000, 2000);//LEB
                    MCJointGainOverride(3, 20, 1, eq_gain_1000, 2000);//LWY
                    MCJointGainOverride(3, 20, 2, eq_gain_1000, 2000);//LWP

//                    RBBoardSetSwitchingMode(2,15,0);
//                    RBBoardSetSwitchingMode(2,16,0);

//                    RBJointGainOverride(2,15,1000,1000,2000);//RSY REB
//                    RBJointGainOverride(2,16,1000,1000,2000);//RWY RWP
                }
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0]==2)
            {
                printf("wFT nulling...!!!\n");
                MCWristFTsensorNull(2, 53);
                freeSleep(50*1000);
                MCWristFTsensorNull(3, 54);
                freeSleep(50*1000);
//                RBwFTsensorNull(2, 53);
//                freeSleep(50*1000);
            }
            sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
            break;
        case WMupperbody_AL_GOTOPOINT:
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();
            if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] ==1)
            {
                int MotionChecker_1 = PullDoor_GrabAndOpen(0);
                if(MotionChecker_1 == 1)
                    printf("----------------- Pull Door Motion 1/4 End...!!!--------------------\n");
                else if(MotionChecker_1 == 4)
                    printf("----------------- Pull Door Motion 1/4 E-stop...!!!--------------------\n");
                else
                    printf("----------------- Pull Door Motion 1/4 Fail...!!!--------------------\n");
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 2)
            {
                int MotionChecker_2 = PullDoor_OpenDoorAndLHblock(0);
                if(MotionChecker_2 == 1)
                    printf("----------------- Pull Door Motion 2/4 End...!!!--------------------\n");
                else if(MotionChecker_2 == 4)
                    printf("----------------- Pull Door Motion 2/4 E-stop...!!!--------------------\n");
                else
                    printf("----------------- Pull Door Motion 2/4 Fail...!!!--------------------\n");
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 3)
            {
                int MotionChecker_3 = PullDoor_RHbackAndSmallRotate();
                if(MotionChecker_3 == 1)
                    printf("----------------- Pull Door Motion 3/4 End...!!!--------------------\n");
                else if(MotionChecker_3 == 4)
                    printf("----------------- Pull Door Motion 3/4 E-stop...!!!--------------------\n");
                else
                    printf("----------------- Pull Door Motion 3/4 Fail...!!!--------------------\n");
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 4)
            {
                int MotionChecker_4 = PullDoor_RHblockAndReadyGGu();
                if(MotionChecker_4 == 1)
                    printf("----------------- Pull Door Motion 4/4 End...!!!--------------------\n");
                else if(MotionChecker_4 == 4)
                    printf("----------------- Pull Door Motion 4/4 E-stop...!!!--------------------\n");
                else
                    printf("----------------- Pull Door Motion 4/4 Fail...!!!--------------------\n");
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 5)
            {
                // Do Whole process

                // Step 1
                int MotionChecker_1 = PullDoor_GrabAndOpen(0);
                if(MotionChecker_1 == 1)
                    printf("----------------- Pull Door Motion 1/4 End...!!!--------------------\n");
                else if(MotionChecker_1 == 4){
                    printf("----------------- Pull Door Motion 1/4 E-stop...!!!--------------------\n");
                    sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
                    break;
                }else{
                    printf("----------------- Pull Door Motion 1/4 Fail...!!!--------------------\n");
                    EM_ReturnHand();
                    if(EM_GRAB_INDEX < 1){
                        EM_GRAB_INDEX++;
                        sharedData->STATE_COMMAND = TCMD_DOOR_GO_NEARGO;
                    }else{
                        printf("----------------- Pull Door Motion 1/4 Fail + Try finish...!!!--------------------\n");
                        EM_GRAB_INDEX = 0;
                        sharedData->STATE_COMMAND = TCMD_GENERAL_ESTOP;
                    }
                    sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
                    break;
                }

                freeSleep(200*1000);

                // Step 2
                int MotionChecker_2 = PullDoor_OpenDoorAndLHblock(0);
                if(MotionChecker_2 == 1)
                    printf("----------------- Pull Door Motion 2/4 End...!!!--------------------\n");
                else if(MotionChecker_2 == 4){
                    printf("----------------- Pull Door Motion 2/4 E-stop...!!!--------------------\n");
                    sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
                    break;
                }else
                    printf("----------------- Pull Door Motion 2/4 Fail...!!!--------------------\n");

                freeSleep(200*1000);

                // Step 3
                int MotionChecker_3 = PullDoor_RHbackAndSmallRotate();
                if(MotionChecker_3 == 1)
                    printf("----------------- Pull Door Motion 3/4 End...!!!--------------------\n");
                else if(MotionChecker_3 == 4){
                    printf("----------------- Pull Door Motion 3/4 E-stop...!!!--------------------\n");
                    sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
                    break;
                }else
                    printf("----------------- Pull Door Motion 3/4 Fail...!!!--------------------\n");

                freeSleep(200*1000);

                // Step 4
                int MotionChecker_4 = PullDoor_RHblockAndReadyGGu();
                if(MotionChecker_4 == 1)
                    printf("----------------- Pull Door Motion 4/4 End...!!!--------------------\n");
                else if(MotionChecker_4 == 4){
                    printf("----------------- Pull Door Motion 4/4 E-stop...!!!--------------------\n");
                    sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
                    break;
                }else
                    printf("----------------- Pull Door Motion 4/4 Fail...!!!--------------------\n");

                freeSleep(300*1000);

                sharedData->STATE_COMMAND = TCMD_DOOR_OPEN;
            }

            sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
            break;

        case WMupperbody_AL_GRAB:
            // Unit motion - Grab
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();
            {
            int MotionChecker_1 = PullDoor_GrabAndOpen(1);
            if(MotionChecker_1 == 1)
                printf("----------------- Unit motion : Grab End...!!!--------------------\n");
            else if(MotionChecker_1 == 4){
                printf("----------------- Unit motion : Grab E-stop...!!!--------------------\n");
                sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
                break;
            }
            }
            sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
            break;
        case WMupperbody_AL_HANDLE_ROTATE:
            // Unit motion - Rotate Handle
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();
            {
                vec3 pelv_nobe, cur_vecX;
                for(int k=0; k<3; k++){
                    pelv_nobe[k] = (double)(sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[k]);
                    cur_vecX[k] = (double)(sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[k+3]);
                }
                double nobe_angle = fabs(sharedData->COMMAND[PODO_NO].USER_PARA_FLOAT[0]);
                int move_dir = (int)(sharedData->COMMAND[PODO_NO].USER_PARA_FLOAT[1]);// 1 or -1

                StartWBIKmotion(WBmode_RULU);
                freeSleep(10*1000);

                handle_rotate_center = pelv_nobe;
                handle_rotate_axis = cur_vecX;
                handle_target_angle = nobe_angle;

                RotateDoorPushHandle(move_dir);
                printf("----------------- Unit motion : Handle Rotation End...!!!--------------------\n");
            }
            sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
            break;
        case WMupperbody_AL_WST:
            if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] ==0)
            {
                // EM - Right hand return to ready
                joint->RefreshToCurrentReference();
                joint->SetAllMotionOwner();

                EM_ReturnHand();
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] ==1)
            {
                // EM - Right hand back
                joint->RefreshToCurrentReference();
                joint->SetAllMotionOwner();

                int check = EM_RightHandBack();
                if(check == 4){
                    sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
                    break;
                }
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] ==2)
            {
                // EM - Left hand back
                joint->RefreshToCurrentReference();
                joint->SetAllMotionOwner();

                int check = EM_LeftHandBack();
                if(check == 4){
                    sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
                    break;
                }
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] ==3)
            {
                // EM - Rotate more
                joint->RefreshToCurrentReference();
                joint->SetAllMotionOwner();

                double em_more = sharedData->COMMAND[PODO_NO].USER_PARA_FLOAT[2];

                int check = EM_RotateMore(1, sign(em_more), _door_getin_radius, fabs(em_more));
                if(check == 4){
                    sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
                    break;
                }
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] ==4)
            {
                // Unit - Left hand block
                joint->RefreshToCurrentReference();
                joint->SetAllMotionOwner();
                int MotionChecker_2 = PullDoor_OpenDoorAndLHblock(1);
                if(MotionChecker_2 == 1)
                    printf("----------------- Unit motion : L-block End...!!!--------------------\n");
                else if(MotionChecker_2 == 4){
                    printf("----------------- Unit motion : L-block E-stop...!!!--------------------\n");
                    sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
                    break;
                }
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] ==5)
            {
                // GGu~ go
                double des_x =sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                double des_y =sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
                double des_a =sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];

                TurnOff_WBIK();
                joint->RefreshToCurrentReference();
                joint->SetAllMotionOwner();

                GGuGofunction(des_x, des_y, des_a);
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] ==6)
            {
                // Last pass through
                vec3 cur_vecX;
                double wheel_door_x = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                double wheel_door_y = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
                cur_vecX[0] = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];
                cur_vecX[1] = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[3];
                cur_vecX[2] = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[4];

                printf("Received Data : %lf %lf // vector : %lf %lf %lf\n", wheel_door_x, wheel_door_y, cur_vecX[0], cur_vecX[1], cur_vecX[2]);

                if(cur_vecX[0]<0.95){
                    sharedData->STATE_COMMAND = TCMD_GENERAL_ESTOP;
                    sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
                    break;
                }

                sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[0]=1.15;
                sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[1]=0;
                sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[2]=0;
                sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[3] = 3;//Vel Change Mode
                sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[3] = 200;
                sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[6] = 6;//direct compen
                sharedData->COMMAND[PODO_NO_WHEEL].USER_COMMAND= OMNIWHEEL_AL_GOTODES;
                userData->WheelDoneFlag = false;
                whilecnt=0;
                while(1){
                    if(userData->WheelDoneFlag == true){
                        userData->WheelDoneFlag = false;
                        break;
                    }
                    whilecnt++;
                    if(whilecnt>=2000){
                        break;
                    }
                    freeSleep(50*1000);
                }
                freeSleep(250*1000);

                sharedData->STATE_COMMAND = TCMD_DOOR_PASS;
            }
            sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
            break;
            /******************************************************************************************
             * OPEN DOOR
            *******************************************************************************************/
        case WMupperbody_AL_DOORIN_WHEEL_AND_TASK:
            TurnOff_WBIK();
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();
            if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] ==1)
            {
                sharedData->STATE_COMMAND = TCMD_PUSHDOOR_READY_POSITION_GOSTART;

                double des_x = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                double des_y = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
                double des_a = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];

                sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[0]=des_x;
                sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[1]=des_y;
                sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[2]=des_a;
                sharedData->COMMAND[PODO_NO_WHEEL].USER_COMMAND= OMNIWHEEL_AL_GOTODES;

                postime = 3000.;
                joint->SetMoveJoint(RSP, 10.0, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(RSR, 15.0, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(RSY, 0.0, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(REB, -130, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(RWY, 0.0, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(RWP, 50.0, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(RF1, 0., postime, MOVE_ABSOLUTE);

                joint->SetMoveJoint(LSP, 10.0, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LSR, -15.0, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LEB, -130.0, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LWY, 0.0, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LWP, 50.0, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LF1, 0., postime, MOVE_ABSOLUTE);

                userData->WheelDoneFlag = false;

                whilecnt=0;
                while(1){
                    if(userData->WheelDoneFlag == true){
                        userData->WheelDoneFlag = false;
                        break;
                    }
                    whilecnt++;
                    if(whilecnt>=500){
                        break;
                    }
                    freeSleep(50*1000);
                }
                int timeconsum = whilecnt*50;
                if(timeconsum>postime){
                    ;
                }else{
                    usleep((postime-timeconsum+50)*1000);
                }
                usleep(50*1000);

                sharedData->STATE_COMMAND = TCMD_PUSHDOOR_READY_POSITION_ARRIVE;
            }
            sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
            break;
        case WMupperbody_AL_DOORIN_GRAB:
            printf("WMU - goto Point...!!!\n");
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();
            {
            // Goto Approach point
            double des_x, des_y, des_z;
            double des_x2, des_y2, des_z2;
            doubles des_RH_ori(4);

            des_x=sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            des_y=sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
            des_z=sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];
            des_x2=sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[3];
            des_y2=sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[4];
            des_z2=sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[5];
            des_RH_ori[0]=sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[6];
            des_RH_ori[1]=sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[7];
            des_RH_ori[2]=sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[8];
            des_RH_ori[3]=sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[9];

            StartWBIKmotion(WBmode_RULU);
            freeSleep(10*1000);

            pos_sec=2.5;
            WBmotion->addRHPosInfo(des_x, des_y, des_z, pos_sec);
            WBmotion->addRElbPosInfo(-20, pos_sec);//-70
            WBmotion->addRHOriInfo(des_RH_ori, pos_sec);
            int tc = FingerPositionInput_RIGHT(-14000);

            if(tc>=2500)
                freeSleep(100*1000);
            else
                freeSleep((2500-tc+100)*1000);

            // Insert Hand
            pos_sec = 2.5;
            WBmotion->addRHPosInfo(des_x2, des_y2, des_z2, pos_sec);
            freeSleep(2510*1000);

            // Gain Over start
            MCBoardSetSwitchingMode(2, 15, 1);//RSY REB
            MCBoardSetSwitchingMode(2, 16, 1);//RWY RWP
            MCJointGainOverride(2, 15, 1, eq_gain_1, 1000);//RSY
            MCJointGainOverride(2, 15, 2, eq_gain_1, 1000);//REB
            MCJointGainOverride(2, 16, 1, eq_gain_0, 1000);//RWY
            MCJointGainOverride(2, 16, 2, eq_gain_1, 1000);//RWP
//            RBBoardSetSwitchingMode(2,15,1);
//            RBBoardSetSwitchingMode(2,16,1);

//            unsigned int temp_gain=1;
//            RBJointGainOverride(2,15,temp_gain,temp_gain,1000);//RSY REB
//            RBJointGainOverride(2,16,0,temp_gain,1000);//RWY RWP

            // Grab handle
            freeSleep(800*1000);
            joint->SetMoveJoint(RF2, 125, 10, MODE_ABSOLUTE);
            freeSleep(6000*1000);
            joint->SetMoveJoint(RF2, 0, 10, MODE_ABSOLUTE);

            // Go Slightly foward
            pos_sec = 1.;
            WBmotion->addRHPosInfo(des_x2+0.01, des_y2, des_z2, pos_sec);
            freeSleep(1010*1000);

            }
            sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
            break;
        case WMupperbody_AL_DOORIN_HANDLE_ROTATE:
            printf("WMU - rotate handle start...!!!\n");
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();

            for(int k=0; k<3; k++){
                handle_rotate_center[k] = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[k];
                hyo_debug[k] = handle_rotate_center[k];
                handle_rotate_axis[k] = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[k+3];
            }
            handle_target_angle = fabs(sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[6]);
            handle_dir = sharedData->COMMAND[PODO_NO].USER_PARA_INT[0];//1 is CW -1 is CCW

            StartWBIKmotion(WBmode_RULU);
            freeSleep(10*1000);

            handle_pos_first = pos(vec3(WBmotion->pRH_3x1),quat(WBmotion->qRH_4x1));
            handle_angle_now = 0.;
            _Trap_togo = handle_target_angle;
            _Trap_maxVel = 70;
            _Trap_maxAcc = 50;
            _isTrap = true;
            _Trap_FINISH_FLAG = false;
            _Trap_Motion_Command = TR_HANDLE_ROTATE;

            sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
            break;            
        case WMupperbody_AL_DOORIN_MOTION:
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();
            if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] ==1)
            {
                // Rotate door by grap hand
                StartWBIKmotion(WBmode_RULU);
                freeSleep(10*1000);

                handle_rotate_center[0] = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                handle_rotate_center[1] = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
                handle_rotate_center[2] = WBmotion->pRH_3x1[2];
                handle_rotate_axis[0] = 0.;
                handle_rotate_axis[1] = 0.;
                handle_rotate_axis[2] = 1.;
                handle_target_angle = 8.;
                handle_dir = 1; //

                handle_pos_first = pos(vec3(WBmotion->pRH_3x1),quat(WBmotion->qRH_4x1));
                handle_angle_now = 0.;
                _Trap_togo = handle_target_angle;
                _Trap_maxVel = 10;
                _Trap_maxAcc = 5;
                _isTrap = true;
                _Trap_FINISH_FLAG = false;
                _Trap_Motion_Command = TR_HANDLE_ROTATE;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] ==2)
            {
                // Left Hand Block
                double temp_c = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];

                StartWBIKmotion(WBmode_RULU);
                freeSleep(10*1000);
                GetPosFromENC();
                freeSleep(10*1000);

                vec3 cur_RH_pos = vec3(WBmotion->enc_pRH_3x1);
                vec3 cur_LH_pos = vec3(WBmotion->pLH_3x1);

                vec3 des_LH_pos_1, des_LH_pos_2, des_LH_pos_3;
                quat des_LH_ori;
                doubles ds_des_LH_ori(4);

                des_LH_pos_1 = cur_LH_pos + vec3(0.12, 0., 0.);
                des_LH_pos_1[2] = (cur_RH_pos[2]-0.2);

                des_LH_pos_2 = des_LH_pos_1;
                des_LH_pos_2[0] = (temp_c + cur_RH_pos[0])/2. - 0.06;
                des_LH_pos_2[1] = 0.;

                des_LH_pos_3 = des_LH_pos_2;
                des_LH_pos_3[0] = (temp_c + cur_RH_pos[0])/2. - 0.01;

                quat q_temp1 = quat(vec3(0,1,0), -90*D2R);
                quat q_temp2 = quat(vec3(0,0,1), -90*D2R);
                des_LH_ori = q_temp1*q_temp2;
                for(int k=0; k<4; k++)
                    ds_des_LH_ori[k] = des_LH_ori[k];

                pos_sec = 2.;
                WBmotion->addLHPosInfo(des_LH_pos_1[0], des_LH_pos_1[1], des_LH_pos_1[2], pos_sec);
                WBmotion->addLElbPosInfo(15, pos_sec);

                freeSleep(2010*1000);
                pos_sec=2.4;
                WBmotion->addLHPosInfo(des_LH_pos_2[0], des_LH_pos_2[1], des_LH_pos_2[2], pos_sec);
                WBmotion->addLElbPosInfo(30, pos_sec);
                WBmotion->addLHOriInfo(ds_des_LH_ori, pos_sec);

                freeSleep(2410*1000);
                pos_sec=2.2;
                WBmotion->addLHPosInfo(des_LH_pos_3[0], des_LH_pos_3[1], des_LH_pos_3[2], pos_sec);
                freeSleep(2210*1000);
            }else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] ==3)
            {
                // Right Hand Back
                joint->RefreshToCurrentReference();
                joint->SetAllMotionOwner();                

                // Re-rotate Handle
                StartWBIKmotion(WBmode_RULU);
                freeSleep(10*1000);

                for(int k=0; k<3; k++){
                    handle_rotate_center[k] = hyo_debug[k];
                    handle_rotate_axis[k] = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[k];
                }
                handle_dir = 1; //

                handle_pos_first = pos(vec3(WBmotion->pRH_3x1),quat(WBmotion->qRH_4x1));
                handle_angle_now = 0.;
                _Trap_togo = 30;
                _Trap_maxVel = 50;
                _Trap_maxAcc = 50;
                _isTrap = true;
                _Trap_FINISH_FLAG =false;
                _Trap_Motion_Command = TR_HANDLE_ROTATE;

                freeSleep(10*1000);

                whilecnt=0;
                while(1){
                    if(_Trap_FINISH_FLAG == true)
                        break;
                    whilecnt++;
                    if(whilecnt > 200)
                        break;
                    freeSleep(50*1000);
                }

                // Open Hand
                freeSleep(10*1000);
                FingerPositionInput_RIGHT(-17000);

                // Right Hand Back
                freeSleep(10*1000);
                StartWBIKmotion(WBmode_RULU);
                freeSleep(10*1000);

                vec3 cur_RH_pos = vec3(WBmotion->pRH_3x1);
                pos_sec =3.;
                WBmotion->addRHPosInfo(cur_RH_pos[0]-0.23, cur_RH_pos[1], cur_RH_pos[2]+0.01, pos_sec);
                freeSleep(3010*1000);

                // Gain return
                MCBoardSetSwitchingMode(2, 15, 0);//RSY REB
                MCBoardSetSwitchingMode(2, 16, 0);//RWY RWP
                MCJointGainOverride(2, 15, 1, eq_gain_1000, 3000);//RSY
                MCJointGainOverride(2, 15, 2, eq_gain_1000, 3000);//REB
                MCJointGainOverride(2, 16, 1, eq_gain_1000, 3000);//RWY
                MCJointGainOverride(2, 16, 2, eq_gain_1000, 3000);//RWP
//                RBBoardSetSwitchingMode(2,15,0);
//                RBBoardSetSwitchingMode(2,16,0);
//                RBJointGainOverride(2,15,1000,1000,3000);//RSY REB
//                RBJointGainOverride(2,16,1000,1000,3000);//RWY RWP
                freeSleep(3005*1000);

                // Arm back

                TurnOff_WBIK();

                postime = 3500;
                joint->SetMoveJoint(RSP, -75.0, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(RSR, 15.0, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(RSY, 0.0, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(REB, -130, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(RWY, 0.0, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(RWP, -50.0, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(RF1, 0.0, postime, MOVE_ABSOLUTE);

                freeSleep(1000*1000);
                joint->SetMoveJoint(RF2, 125, 10, MODE_ABSOLUTE);
                freeSleep(4000*1000);
                joint->SetMoveJoint(RF2, 0, 10, MOVE_ABSOLUTE);

                // Go Slightly Foward  37cm
                freeSleep(10*1000);
                sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[3] = 3;// Vel Change Mode
                sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[4] = 4;// No Wait Flag

                sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[3]=100;// des vel

                sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[0]=0.45;
                sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[1]=0.0;
                sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[2]=0.;
                sharedData->COMMAND[PODO_NO_WHEEL].USER_COMMAND= OMNIWHEEL_AL_GOTODES;

            }else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] ==4)
            {
                StartWBIKmotion(WBmode_RULU);
                freeSleep(10*1000);

                vec3 cur_LH_pos = vec3(WBmotion->pLH_3x1);
                vec3 des_LH_pos_1, des_LH_pos_2;

                des_LH_pos_1 = cur_LH_pos + vec3(-0.27, 0., 0.);
                if(des_LH_pos_1[0] <0.38)
                    des_LH_pos_1[0] = 0.38;

                pos_sec = 2.5;
                WBmotion->addLHPosInfo(des_LH_pos_1[0], des_LH_pos_1[1], des_LH_pos_1[2], pos_sec);

                freeSleep(2550*1000);

                // Move Pos
                TurnOff_WBIK();
                joint->RefreshToCurrentReference();
                joint->SetAllMotionOwner();

                postime = 2500;
                joint->SetMoveJoint(LSP, -5.0, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LSR, -15.0, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LSY, -30.0, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LEB, -115.0, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LWY, -68.0, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LWP, -50.0, postime, MOVE_ABSOLUTE);
                joint->SetMoveJoint(LF1, 0.0, postime, MOVE_ABSOLUTE);

                // Go more
                freeSleep(3000*1000);
                sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[3] = 3;// Vel Change Mode
                sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[4] = 4;// No Wait Flag

                sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[3]=100;// des vel

                sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[0]=0.75;
                sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[1]=0.0;
                sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[2]=145.;
                sharedData->COMMAND[PODO_NO_WHEEL].USER_COMMAND= OMNIWHEEL_AL_GOTODES;

                // Back
                userData->WheelDoneFlag = false;
                whilecnt=0;
                while(1){
                    if(userData->WheelDoneFlag == true){
                        userData->WheelDoneFlag = false;
                        printf("Wheel is ended...! next step is ready to start\n");
                        break;
                    }
                    whilecnt++;
                    if(whilecnt>=500){
                        printf("Wheel is not!!! ended...! time gap out\n");
                        break;
                    }
                    freeSleep(50*1000);
                }

                freeSleep(10*1000);
                sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[3] = 3;// Vel Change Mode
                sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[4] = 4;// No Wait Flag

                sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[3]=200;// des vel

                sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[0]=-0.7;
                sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[1]=0.0;
                sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[2]=0.0;
                sharedData->COMMAND[PODO_NO_WHEEL].USER_COMMAND= OMNIWHEEL_AL_GOTODES;
                userData->WheelDoneFlag = false;
                whilecnt=0;
                while(1){
                    if(userData->WheelDoneFlag == true){
                        userData->WheelDoneFlag = false;
                        printf("Wheel is ended...! next step is ready to start\n");
                        break;
                    }
                    whilecnt++;
                    if(whilecnt>=500){
                        printf("Wheel is not!!! ended...! time gap out\n");
                        break;
                    }
                    freeSleep(50*1000);
                }


            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 5)
            {
                // Whole Step 1;
                int pushdoor_check_1;
                if(DOOR_MODE == LEFT_DOOR_MODE)
                    pushdoor_check_1 = PushDoor_GrabAndOpen_LEFT(0);
                else if(DOOR_MODE == RIGHT_DOOR_MODE)
                    pushdoor_check_1 = PushDoor_GrabAndOpen(0);

                if(pushdoor_check_1 == 1)
                    printf("----------------- Pull Push Motion 1/2 End...!!!--------------------\n");
                else if(pushdoor_check_1 == 4)
                    printf("----------------- Pull Push Motion 1/2 E-stop...!!!--------------------\n");
                else
                    printf("----------------- Pull Push Motion 1/2 Fail...!!!--------------------\n");
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 6)
            {
                // Whole Step 2;
                int pushdoor_check_2;
                if(DOOR_MODE == LEFT_DOOR_MODE)
                    pushdoor_check_2 = PushDoor_LHblock_RHback_Gothrough_LEFT(0);
                else if(DOOR_MODE == RIGHT_DOOR_MODE)
                    pushdoor_check_2 = PushDoor_LHblock_RHback_Gothrough(0);

                if(pushdoor_check_2 == 1)
                    printf("----------------- Pull Push Motion 2/2 End...!!!--------------------\n");
                else if(pushdoor_check_2 == 4)
                    printf("----------------- Pull Push Motion 2/2 E-stop...!!!--------------------\n");
                else
                    printf("----------------- Pull Push Motion 2/2 Fail...!!!--------------------\n");
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 7)
            {
                // Whole Step 1 & 2
                // Step 1
                int pushdoor_check_1;
                if(DOOR_MODE == LEFT_DOOR_MODE)
                    pushdoor_check_1 = PushDoor_GrabAndOpen_LEFT(0);
                else if(DOOR_MODE == RIGHT_DOOR_MODE)
                    pushdoor_check_1 = PushDoor_GrabAndOpen(0);

                if(pushdoor_check_1 == 1){
                    printf("----------------- Pull Push Motion 1/2 End...!!!--------------------\n");
                }else if(pushdoor_check_1 == 4){
                    printf("----------------- Pull Push Motion 1/2 E-stop...!!!--------------------\n");
                    sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
                    break;
                }else{
                    printf("----------------- Pull Push Motion 1/2 Fail...!!!--------------------\n");
                    EM_PushDoor_ReturnHand();
                    sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
                    break;
                }

                freeSleep(200*1000);

                // Step 2
                int pushdoor_check_2;
                if(DOOR_MODE == LEFT_DOOR_MODE)
                    pushdoor_check_2 = PushDoor_LHblock_RHback_Gothrough_LEFT(0);
                else if(DOOR_MODE == RIGHT_DOOR_MODE)
                    pushdoor_check_2 = PushDoor_LHblock_RHback_Gothrough(0);

                if(pushdoor_check_2 == 1)
                    printf("----------------- Pull Push Motion 1/2 End...!!!--------------------\n");
                else if(pushdoor_check_2 == 4){
                    printf("----------------- Pull Push Motion 1/2 E-stop...!!!--------------------\n");
                    sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
                    break;
                }else
                    printf("----------------- Pull Push Motion 2/2 Fail...!!!--------------------\n");

                freeSleep(200*1000);
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 8)
            {
                printf("Wheel Push Door - EM 1 Return RH...!!!\n");
                if(DOOR_MODE == LEFT_DOOR_MODE)
                    EM_PushDoor_ReturnHand_LEFT();
                else if(DOOR_MODE == RIGHT_DOOR_MODE)
                    EM_PushDoor_ReturnHand();
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 9)
            {
                printf("Wheel Push Door - EM 2 Return LH...!!!\n");
                if(DOOR_MODE == LEFT_DOOR_MODE)
                    EM_PushDoor_ReturnLH_LEFT();
                else if(DOOR_MODE == RIGHT_DOOR_MODE)
                    EM_PushDoor_ReturnLH();
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 10)
            {
                printf("Wheel Push Door - EM 2 Keep Going...!!!\n");
                if(DOOR_MODE == LEFT_DOOR_MODE)
                    PushDoor_LHblock_RHback_Gothrough_LEFT(1);
                else if(DOOR_MODE == RIGHT_DOOR_MODE)
                    PushDoor_LHblock_RHback_Gothrough(1);
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 20)
            {
                char door_mode = sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[1];
                // 0 : right door
                // 1 : left door
                if(door_mode == 0){
                    printf("Door Mode - Now mode is RIGHT ...!!!\n");
                    DOOR_MODE = RIGHT_DOOR_MODE;
                }else if(door_mode ==1){
                    printf("Door Mode - Now mode is LEFT ...!!!\n");
                    DOOR_MODE = LEFT_DOOR_MODE;
                }
                sharedData->STATE_COMMAND = TCMD_PUSHDOOR_MODE_CHANGED;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] ==21)
            {
                printf("Door Nobe Parameter Changed ...!!!\n");
                LEFT_NOBE_PUSH_X = sharedData->COMMAND[PODO_NO].USER_PARA_FLOAT[0];
                LEFT_NOBE_PUSH_Y = sharedData->COMMAND[PODO_NO].USER_PARA_FLOAT[1];
                RIGHT_NOBE_PUSH_X = sharedData->COMMAND[PODO_NO].USER_PARA_FLOAT[2];
                RIGHT_NOBE_PUSH_Y = sharedData->COMMAND[PODO_NO].USER_PARA_FLOAT[3];

                DOOR_NOBE_LENGTH = sharedData->COMMAND[PODO_NO].USER_PARA_FLOAT[4];

                LEFT_NOBE_ROT_ANGLE = sharedData->COMMAND[PODO_NO].USER_PARA_FLOAT[5];
                RIGHT_NOBE_ROT_ANGLE = sharedData->COMMAND[PODO_NO].USER_PARA_FLOAT[6];


                if(LEFT_NOBE_PUSH_X < 0 || LEFT_NOBE_PUSH_Y <0 || RIGHT_NOBE_PUSH_X <0 || RIGHT_NOBE_PUSH_Y<0 || LEFT_NOBE_PUSH_X > 0.12 || LEFT_NOBE_PUSH_Y > 0.12 || RIGHT_NOBE_PUSH_X > 0.12 || RIGHT_NOBE_PUSH_Y > 0.12){
                    LEFT_NOBE_PUSH_X = 0.075;
                    LEFT_NOBE_PUSH_Y = 0.035;
                    RIGHT_NOBE_PUSH_X = 0.075;
                    RIGHT_NOBE_PUSH_Y = 0.055;
                }
                printf("Door Nobe Paramter : %lf %lf %lf %lf\n", LEFT_NOBE_PUSH_X, LEFT_NOBE_PUSH_Y, RIGHT_NOBE_PUSH_X, RIGHT_NOBE_PUSH_Y);

                if(DOOR_NOBE_LENGTH<0 || DOOR_NOBE_LENGTH >0.3){
                    DOOR_NOBE_LENGTH = 0.13;
                }
                printf("Door Nobe Length : %lf\n", DOOR_NOBE_LENGTH);

                if(LEFT_NOBE_ROT_ANGLE < 0 || RIGHT_NOBE_ROT_ANGLE <0 || LEFT_NOBE_ROT_ANGLE > 90 || RIGHT_NOBE_ROT_ANGLE > 90){
                    LEFT_NOBE_ROT_ANGLE = 60.;
                    RIGHT_NOBE_ROT_ANGLE = 55.;
                }
                printf("Door Nobe Rot Angle : %lf %lf\n", LEFT_NOBE_ROT_ANGLE, RIGHT_NOBE_ROT_ANGLE);

                sharedData->STATE_COMMAND = TCMD_PUSHDOOR_NOBE_PARA_CHANGED;
            }
            else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] ==30)
            {
                char select = sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[1];
                if(select ==0){
                    // goto terminate pos
                    if(DOOR_MODE == LEFT_DOOR_MODE){
                        postime = 3000.;
                        joint->SetMoveJoint(LSP, -75.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LSR, -15.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LEB, -130, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LWY, 0.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LWP, -50.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LF1, 0.0, postime, MOVE_ABSOLUTE);

                        joint->SetMoveJoint(RSP, -5.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(RSR, 15.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(RSY, 30.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(REB, -115.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(RWY, 68.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(RWP, -50.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(RF1, 0.0, postime, MOVE_ABSOLUTE);
                    }else if(DOOR_MODE == RIGHT_DOOR_MODE){
                        postime = 3000.;
                        joint->SetMoveJoint(RSP, -75.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(RSR, 15.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(RSY, 0.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(REB, -130, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(RWY, 0.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(RWP, -50.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(RF1, 0.0, postime, MOVE_ABSOLUTE);

                        joint->SetMoveJoint(LSP, -5.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LSR, -15.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LSY, -30.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LEB, -115.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LWY, -68.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LWP, -50.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LF1, 0.0, postime, MOVE_ABSOLUTE);
                    }
                }else if(select ==1){
                    // goto finish pos
                    postime = 3500.;
                    joint->SetMoveJoint(WST, 0.0, postime, MOVE_ABSOLUTE);

                    joint->SetMoveJoint(RF1, 0.0, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RF2, 0.0, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LF1, 0.0, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(LF2, 0.0, postime, MOVE_ABSOLUTE);

                    if(DOOR_MODE == LEFT_DOOR_MODE){
                        joint->SetMoveJoint(LSP, 40.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LSR, -17.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LEB, -130.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LWY, 10.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LWP, 40.0, postime, MOVE_ABSOLUTE);

                        joint->SetMoveJoint(RSP, 40.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(RSR, -5.0, 1500, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(RSY, 0.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(REB, -130, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(RWY, -10.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(RWP, 40.0, postime, MOVE_ABSOLUTE);

                        usleep(1500*1000);
                        joint->SetMoveJoint(RSR, 17.0, 2000, MOVE_ABSOLUTE);
                        usleep(2000*1000);
                    }else if(DOOR_MODE == RIGHT_DOOR_MODE){
                        joint->SetMoveJoint(RSP, 40.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(RSR, 17.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(RSY, 0.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(REB, -130, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(RWY, -10.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(RWP, 40.0, postime, MOVE_ABSOLUTE);

                        joint->SetMoveJoint(LSP, 40.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LSR, 5.0, 1500, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LEB, -130.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LWY, 10.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LWP, 40.0, postime, MOVE_ABSOLUTE);

                        usleep(1500*1000);
                        joint->SetMoveJoint(LSR, -17.0, 2000, MOVE_ABSOLUTE);
                        usleep(2000*1000);
                    }
                }
            }
            sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
            break;
            /******************************************************************************************
             * Hose Task
            *******************************************************************************************/
        case WMupperbody_AL_HOSE_APPROACH:
            {
            sharedData->STATE_COMMAND = TCMD_HOSE_APPROACH_START;
            double temp_x = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            double temp_y = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
            double temp_a = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];

            sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[0]= temp_x;
            sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[1]= temp_y;
            sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[2]= temp_a;
            sharedData->COMMAND[PODO_NO_WHEEL].USER_COMMAND= OMNIWHEEL_AL_GOTODES;
            userData->WheelDoneFlag = false;
            whilecnt=0;
            while(1){
                if(userData->WheelDoneFlag == true){
                    userData->WheelDoneFlag = false;
                    break;
                }
                whilecnt++;
                if(whilecnt>=1200){
                    break;
                }
                freeSleep(50*1000);
            }
            usleep(200*1000);
            sharedData->STATE_COMMAND = TCMD_HOSE_APPROACH_DONE;

            }
            sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
            break;
        case WMupperbody_AL_HOSE_APPROACH2:
            {
            sharedData->STATE_COMMAND = TCMD_WYE_APPROACH_START;
            double temp_x = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            double temp_y = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
            double temp_a = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];

            sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[0]= temp_x;
            sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[1]= temp_y;
            sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[2]= temp_a;
            sharedData->COMMAND[PODO_NO_WHEEL].USER_COMMAND= OMNIWHEEL_AL_GOTODES;
            userData->WheelDoneFlag = false;
            whilecnt=0;
            while(1){
                if(userData->WheelDoneFlag == true){
                    userData->WheelDoneFlag = false;
                    break;
                }
                whilecnt++;
                if(whilecnt>=1200){
                    break;
                }
                freeSleep(50*1000);
            }
            usleep(200*1000);
            sharedData->STATE_COMMAND = TCMD_WYE_APPROACH_DONE;

            }
            sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
            break;
        case WMupperbody_AL_HOSE_GET:
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();
            if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 1){
                // Hose Grab - Approach and grab hose
                int HOSE_MOTION_CHECK_GRAB_1 = HOSE_GRAB_HOSE();
                if(HOSE_MOTION_CHECK_GRAB_1 == 1)
                    printf("----------------- Hose Grab Motion 1/2 End...!!!--------------------\n");
                else if(HOSE_MOTION_CHECK_GRAB_1 == 4)
                    printf("----------------- Hose Grab Motion 1/2 E-stop...!!!--------------------\n");
                else
                    printf("----------------- Hose Grab Motion 1/2 Fail...!!!--------------------\n");

            }else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] ==2){
                // Hose Grab - Return Hand and Rotate body
                double des_rotate_angle = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[3];
                int HOSE_MOTION_CHECK_GRAB_2 = HOSE_AFTER_GRAB(des_rotate_angle);
                if(HOSE_MOTION_CHECK_GRAB_2 == 1)
                    printf("----------------- Hose Grab Motion 2/2 End...!!!--------------------\n");
                else if(HOSE_MOTION_CHECK_GRAB_2 == 4)
                    printf("----------------- Hose Grab Motion 2/2 E-stop...!!!--------------------\n");
                else
                    printf("----------------- Hose Grab Motion 2/2 Fail...!!!--------------------\n");
            }else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] ==3){
                // Hose Grab - Grab Whole Process
                sharedData->STATE_COMMAND = TCMD_HOSE_GRAB_START;
                double des_rotate_angle = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[3];
                int HOSE_MOTION_CHECK_GRAB_1 = HOSE_GRAB_HOSE();
                if(HOSE_MOTION_CHECK_GRAB_1 == 1)
                    printf("----------------- Hose Grab Motion 1/2 End...!!!--------------------\n");
                else if(HOSE_MOTION_CHECK_GRAB_1 == 4){
                    printf("----------------- Hose Grab Motion 1/2 E-stop...!!!--------------------\n");
                    sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
                    break;
                }else{
                    printf("----------------- Hose Grab Motion 1/2 Fail...!!!--------------------\n");
                    EM_ReturnHand_From_HOSEGRAB();
                    sharedData->STATE_COMMAND = TCMD_HOSE_APPROACH_DONE;
                    sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
                    break;
                }
                freeSleep(200*1000);

                int HOSE_MOTION_CHECK_GRAB_2 = HOSE_AFTER_GRAB(des_rotate_angle);
                if(HOSE_MOTION_CHECK_GRAB_2 == 1)
                    printf("----------------- Hose Grab Motion 2/2 End...!!!--------------------\n");
                else if(HOSE_MOTION_CHECK_GRAB_2 == 4){
                    printf("----------------- Hose Grab Motion 2/2 E-stop...!!!--------------------\n");
                    sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
                    break;
                }else{
                    printf("----------------- Hose Grab Motion 2/2 Fail...!!!--------------------\n");
                    sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
                    break;
                }
                freeSleep(200*1000);
                sharedData->STATE_COMMAND = TCMD_HOSE_GRAB_DONE;
            }else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] ==4){
                EM_ReturnHand_From_HOSEGRAB();
            }
            sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
            break;
        case WMupperbody_AL_HOSE_PLUG:
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();
            if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 1){
                // Align hose and wye way
                int HOSE_MOTION_CHECK_WYE_1 = HOSE_PLUG_WYE();
                if(HOSE_MOTION_CHECK_WYE_1 == 1)
                    printf("----------------- Hose Wye Motion 1/2 End...!!!--------------------\n");
                else if(HOSE_MOTION_CHECK_WYE_1 == 4)
                    printf("----------------- Hose Wye Motion 1/2 E-stop...!!!--------------------\n");
                else
                    printf("----------------- Hose Wye Motion 1/2 Fail...!!!--------------------\n");
            }else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 2){
                // Return hand from wye position
                int HOSE_MOTION_CHECK_WYE_2 = HOSE_RETURN_FROM_WYE();
                if(HOSE_MOTION_CHECK_WYE_2 == 1)
                    printf("----------------- Hose Wye Motion 2/2 End...!!!--------------------\n");
                else if(HOSE_MOTION_CHECK_WYE_2 == 4)
                    printf("----------------- Hose Wye Motion 2/2 E-stop...!!!--------------------\n");
                else
                    printf("----------------- Hose Wye Motion 2/2 Fail...!!!--------------------\n");
            }else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 3){
                // HOSE WYE - Whole
                sharedData->STATE_COMMAND = TCMD_WYE_ALIGN_START;
                int HOSE_MOTION_CHECK_WYE_1 = HOSE_PLUG_WYE();
                if(HOSE_MOTION_CHECK_WYE_1 == 1)
                    printf("----------------- Hose Wye Motion 1/2 End...!!!--------------------\n");
                else if(HOSE_MOTION_CHECK_WYE_1 == 4){
                    printf("----------------- Hose Wye Motion 1/2 E-stop...!!!--------------------\n");
                    sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
                    break;
                }else{
                    printf("----------------- Hose Wye Motion 1/2 Fail...!!!--------------------\n");
                    EM_ReturnHand_From_WYE();
                    sharedData->STATE_COMMAND = TCMD_WYE_APPROACH_DONE;
                    sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
                    break;
                }

                freeSleep(200*1000);

                int HOSE_MOTION_CHECK_WYE_2 = HOSE_RETURN_FROM_WYE();
                if(HOSE_MOTION_CHECK_WYE_2 == 1)
                    printf("----------------- Hose Wye Motion 2/2 End...!!!--------------------\n");
                else if(HOSE_MOTION_CHECK_WYE_2 == 4){
                    printf("----------------- Hose Wye Motion 2/2 E-stop...!!!--------------------\n");
                    sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
                    break;
                }else{
                    printf("----------------- Hose Wye Motion 2/2 Fail...!!!--------------------\n");
                    sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
                    break;
                }
                sharedData->STATE_COMMAND = TCMD_WYE_ALIGN_DONE;
            }else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 4){
                // EM return from wye
                EM_ReturnHand_From_WYE();
            }

            // -----PLUG----

            else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 10)
            {
                char type = sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[1];
                if(type ==0){
                    printf("PLUG DATA RECEIVED...!!!\n");

                    // Set constant variables
                    double tube_length = 0.05;
                    double finger_length = 0.15;//m
                    double hand_ori = 30.;//deg
                    if(PLUG_POSTURE == PLUG_STAND_MODE)
                        hand_ori = 30;
                    else if(PLUG_POSTURE == PLUG_WHEEL_MODE)
                        hand_ori = 30.;
                    vec3 plug_align_offset = vec3(-tube_length, 0., 0.);
                    vec3 plug_out_offset = vec3(0., finger_length*cos(hand_ori*D2R), -finger_length*sin(hand_ori*D2R));
                    vec3 plug_ap_offset = plug_align_offset + plug_out_offset;


                    double grap_point_position_length = 0.095;//0.1;//0.083;
                    double grap_point_radius = 0.022;
                    vec3 tip2hand = vec3(-grap_point_position_length, grap_point_radius*cos(hand_ori*D2R), -grap_point_radius*sin(hand_ori*D2R));


                    plug_dis_num = (int)(tube_length/0.005)+1;
                    // Tip point calculation
                    vec3 up_plug_left, up_plug_right, up_cur_vecX;
                    for(int k=0; k<3; k++){
                        up_plug_left[k] = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[k];
                        up_plug_right[k] = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[k+3];
                        up_cur_vecX[k] = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[k+6];
                    }
                    vec3 up_cur_vecZ = vec3(0, 0, 1);
                    vec3 up_cur_vecY = cross(up_cur_vecZ, up_cur_vecX);

                    _plug_vecX = up_cur_vecX;
                    _plug_vecY = up_cur_vecY;
                    _plug_vecZ = up_cur_vecZ;

                    vec3 up_align_left = up_plug_left + up_cur_vecX*plug_align_offset[0] + up_cur_vecY*plug_align_offset[1] + up_cur_vecZ*plug_align_offset[2];
                    vec3 up_align_right = up_plug_right + up_cur_vecX*plug_align_offset[0] + up_cur_vecY*plug_align_offset[1] + up_cur_vecZ*plug_align_offset[2];
                    vec3 up_out_left = up_plug_left + up_cur_vecX*plug_out_offset[0] + up_cur_vecY*plug_out_offset[1] + up_cur_vecZ*plug_out_offset[2];
                    vec3 up_out_right = up_plug_right + up_cur_vecX*plug_out_offset[0] + up_cur_vecY*plug_out_offset[1] + up_cur_vecZ*plug_out_offset[2];
                    vec3 up_ap_left = up_plug_left + up_cur_vecX*plug_ap_offset[0] + up_cur_vecY*plug_ap_offset[1] + up_cur_vecZ*plug_ap_offset[2];
                    vec3 up_ap_right = up_plug_right + up_cur_vecX*plug_ap_offset[0] + up_cur_vecY*plug_ap_offset[1] + up_cur_vecZ*plug_ap_offset[2];

                    // Hand point calculation
                    vec3 real_tip2hand_offset = up_cur_vecX*tip2hand[0] + up_cur_vecY*tip2hand[1] + up_cur_vecZ*tip2hand[2];
                    vec3 up_plug_hand_left = up_plug_left + real_tip2hand_offset;
                    vec3 up_plug_hand_right = up_plug_right + real_tip2hand_offset;
                    vec3 up_align_hand_left = up_align_left + real_tip2hand_offset;
                    vec3 up_align_hand_right = up_align_right + real_tip2hand_offset;
                    vec3 up_out_hand_left = up_out_left + real_tip2hand_offset;
                    vec3 up_out_hand_right = up_out_right + real_tip2hand_offset;
                    vec3 up_ap_hand_left = up_ap_left + real_tip2hand_offset;
                    vec3 up_ap_hand_right = up_ap_right + real_tip2hand_offset;

                    // Convert to Pelvis coordinate
                    double rot_angle;
                    if(PLUG_POSTURE == PLUG_STAND_MODE){
                        rot_angle = 0;
                    }else if(PLUG_POSTURE == PLUG_WHEEL_MODE){
                        rot_angle = 180;
                    }
                    mat3 convert(vec3(0,0,1),rot_angle*D2R);
                    pelv_plug_hand[0] = convert*up_plug_hand_right;
                    pelv_plug_hand[1] = convert*up_plug_hand_left;
                    pelv_align_hand[0] = convert*up_align_hand_right;
                    pelv_align_hand[1] = convert*up_align_hand_left;
                    pelv_out_hand[0] = convert*up_out_hand_right;
                    pelv_out_hand[1] = convert*up_out_hand_left;
                    pelv_ap_hand[0] = convert*up_ap_hand_right;
                    pelv_ap_hand[1] = convert*up_ap_hand_left;

                    if(PLUG_POSTURE == PLUG_STAND_MODE){
                        plug_elb[0] = 5;//deg
                        plug_elb[1] = -5;//deg//LEFT
                        plug_wst[0] = -48;//deg
                        plug_wst[1] = -20;//deg//LEFT
                    }else if(PLUG_POSTURE == PLUG_WHEEL_MODE){
                        plug_elb[0] = 20;//deg
                        plug_elb[1] = 20;//deg//LEFT
                        plug_wst[0] = 130;//deg
                        plug_wst[1] = 170;//deg//LEFT
                    }

                    quat des_H_ori = quat(vec3(0,0,1), rot_angle*D2R)*quat(vec3(0,1,0),-90*D2R) * quat(vec3(1,0,0), -90*D2R) * quat(vec3(0,1,0),  -hand_ori*D2R);
                    for(int k=0; k<4; k++)
                        plug_ds_des_H_ori[k] = des_H_ori[k];

                    sharedData->STATE_COMMAND = TCMD_PLUG_DATA_RECEIVED;

                }else if(type ==1){
                    printf("LEFT PLUG FIRST MODE...!!!\n");
                    PLUG_FIRST = LEFT_FIRST_PLUG;
                    PLUG_SECOND = RIGHT_FIRST_PLUG;

                    sharedData->STATE_COMMAND = TCMD_PLUG_MODE_LEFT_FIRST;
                }else if(type ==2){
                    printf("RIGHT PLUG FIRST MODE...!!!\n");
                    PLUG_FIRST = RIGHT_FIRST_PLUG;
                    PLUG_SECOND = LEFT_FIRST_PLUG;

                    sharedData->STATE_COMMAND = TCMD_PLUG_MODE_RIGHT_FIRST;
                }else if(type ==3){
                    printf("PLUG WHEEL APPROACH...!!!\n");
                    double des_x = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                    double des_y = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
                    double des_a = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];

                    // To omniwheel
                }else if(type ==4){
                    printf("PLUG Walking APPROACH...!!!\n");

                    sharedData->STATE_COMMAND = TCMD_PLUG_WALKING_START;

                    double des_x = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                    double des_y = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
                    double des_a = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];

                    userData->G2M.GoalPosX = des_x;
                    userData->G2M.GoalPosY = des_y;
                    userData->G2M.GoalAngle = des_a;
                    userData->G2M.WalkingGoalGoModeCommand = GOALGO_NORMAL;
                    sharedData->COMMAND[PODO_NO_WALK].USER_PARA_INT[9] = GOAL_WALKING;
                    sharedData->COMMAND[PODO_NO_WALK].USER_PARA_INT[4] = INSIDE_WALKING;
                    sharedData->COMMAND[PODO_NO_WALK].USER_COMMAND = FREEWALK_WALK;

                    userData->WalkDoneFlag = false;
                    int whilecnt=0;
                    while(1){
                        if(userData->WalkDoneFlag == true){
                            userData->WalkDoneFlag = false;
                            break;
                        }
                        whilecnt++;
                        if(whilecnt>=800){
                            break;
                        }
                        usleep(50*1000);
                    }
                    usleep(200*1000);
                    printf("Plug Walking Finished...!!!\n");
                    sharedData->STATE_COMMAND = TCMD_PLUG_WALKING_DONE;

                    // To freewalking
                }
            }

            else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 11)
            {
                char plug_step = sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[1];
                joint->RefreshToCurrentReference();
                joint->SetAllMotionOwner();

                switch(plug_step){
                case 0:
                {
                    TurnOff_WBIK();

                    if(PLUG_POSTURE == PLUG_STAND_MODE){
                        Change_Arm_pos_Move(3000);
                        usleep(3000*1000);
                    }else if(PLUG_POSTURE == PLUG_WHEEL_MODE){
                        postime = 4500;
                        joint->SetMoveJoint(RSP, 70.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(RSR, 15.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(RSY, 0.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(REB, -130, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(RWY, 0.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(RWP, 40.0, postime, MOVE_ABSOLUTE);

                        joint->SetMoveJoint(LSP, 30.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LSR, -15.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LEB, -130, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LWY, 0.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LWP, 40.0, postime, MOVE_ABSOLUTE);

                        joint->SetMoveJoint(WST, 180.0, postime, MOVE_ABSOLUTE);

                        joint->SetMoveJoint(RF1, 0.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(RF2, 0.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LF1, 0.0, postime, MOVE_ABSOLUTE);
                        joint->SetMoveJoint(LF2, 0.0, postime, MOVE_ABSOLUTE);
                        usleep(postime*1000);
                    }

                    printf("Plug :: pos change done ...!!!\n");

                    break;
                }
                case 1:
                {
                    sharedData->STATE_COMMAND = TCMD_PLUG_MOTION_GRAB_START;

                    // Plug grab
                    StartWBIKmotion(WBmode_RULP);
                    usleep(10*1000);

                    plug_offset = vec3(0,0,0);

                    pos_sec = 3.;
                    WBmotion->addLHPosInfo(pelv_ap_hand[PLUG_FIRST][0], pelv_ap_hand[PLUG_FIRST][1], pelv_ap_hand[PLUG_FIRST][2], pos_sec);
                    WBmotion->addLHOriInfo(plug_ds_des_H_ori, pos_sec);
                    WBmotion->addLElbPosInfo(plug_elb[PLUG_FIRST], pos_sec);
                    WBmotion->addWSTPosInfo(plug_wst[PLUG_FIRST], pos_sec);
                    usleep(3010*1000);

                    joint->SetMoveJoint(LF2, -125, 10, MODE_ABSOLUTE);
                    usleep(4500*1000);
                    joint->SetMoveJoint(LF2, 0, 10, MODE_ABSOLUTE);
                    usleep(10*1000);

                    pos_sec = 1.8;
                    WBmotion->addLHPosInfo(pelv_out_hand[PLUG_FIRST][0], pelv_out_hand[PLUG_FIRST][1], pelv_out_hand[PLUG_FIRST][2], pos_sec);
                    WBmotion->addLHOriInfo(plug_ds_des_H_ori, pos_sec);
                    WBmotion->addLElbPosInfo(plug_elb[PLUG_FIRST], pos_sec);
                    usleep(1810*1000);

                    pos_sec = 2.5;
                    WBmotion->addLHPosInfo(pelv_plug_hand[PLUG_FIRST][0], pelv_plug_hand[PLUG_FIRST][1], pelv_plug_hand[PLUG_FIRST][2], pos_sec);
                    WBmotion->addLHOriInfo(plug_ds_des_H_ori, pos_sec);
                    WBmotion->addLElbPosInfo(plug_elb[PLUG_FIRST], pos_sec);
                    usleep(2510*1000);

                    joint->SetMoveJoint(LF2, 125, 10, MODE_ABSOLUTE);
                    usleep(4500*1000);

                    double finger_enc = finger_scale*(sharedData->ENCODER[MC_ID_CH_Pairs[LF2].id][MC_ID_CH_Pairs[LF2].ch].CurrentPosition);
                    if(finger_enc < -3700){
                        // Success
                        sharedData->STATE_COMMAND = TCMD_PLUG_MOTION_GRAB_DONE;
                    }else{
                        // Fail
                        sharedData->STATE_COMMAND = TCMD_PLUG_MOTION_GRAB_FAIL;
                    }

                    printf("Plug :: grab process done ...!!!\n");



                    break;
                }
                case 2:
                {
                    sharedData->STATE_COMMAND = TCMD_PLUG_MOTION_PLUGOUTMOVE_START;

                    // Plug out and move
                    StartWBIKmotion(WBmode_RULP);
                    usleep(10*1000);

                    pos_sec = 1.5;
                    WBmotion->addLHPosInfo(pelv_align_hand[PLUG_FIRST][0], pelv_align_hand[PLUG_FIRST][1], pelv_align_hand[PLUG_FIRST][2], pos_sec);
                    WBmotion->addLHOriInfo(plug_ds_des_H_ori, pos_sec);
                    WBmotion->addLElbPosInfo(plug_elb[PLUG_FIRST], pos_sec);
                    usleep(1510*1000);

                    pos_sec = 5.;
                    WBmotion->addLHPosInfo(pelv_align_hand[PLUG_SECOND][0], pelv_align_hand[PLUG_SECOND][1], pelv_align_hand[PLUG_SECOND][2], pos_sec);
                    WBmotion->addLHOriInfo(plug_ds_des_H_ori, pos_sec);
                    WBmotion->addLElbPosInfo(plug_elb[PLUG_SECOND], pos_sec);
                    WBmotion->addWSTPosInfo(plug_wst[PLUG_SECOND], pos_sec);
                    usleep(5010*1000);

                    printf("Plug :: plug out and move done ...!!!\n");

                    sharedData->STATE_COMMAND = TCMD_PLUG_MOTION_PLUGOUTMOVE_DONE;
                    break;
                }
                case 3:
                {
                    sharedData->STATE_COMMAND = TCMD_PLUG_MOTION_PLUG_IN_START;

                    // Plug In
//                    StartWBIKmotion(WBmode_RULP);
//                    usleep(10*1000);
//                    vec3 cur_target = pelv_plug_hand[PLUG_SECOND] + plug_offset + vec3(0.005, 0., 0.);
//                    pos_sec = 1.5;
//                    WBmotion->addLHPosInfo(cur_target[0], cur_target[1], cur_target[2], pos_sec);
//                    WBmotion->addLHOriInfo(plug_ds_des_H_ori, pos_sec);
//                    WBmotion->addLElbPosInfo(plug_elb[PLUG_SECOND], pos_sec);
//                    WBmotion->addWSTPosInfo(plug_wst[PLUG_SECOND], pos_sec);
//                    usleep(1510*1000);

//                    joint->SetMoveJoint(LF2, 0, 10, MODE_ABSOLUTE);
//                    usleep(10*1000);

                    double x_distort = 0.005;
                    double y_distort = 0.005;
                    double z_distort = 0.005;
                    double distort_time_s = 0.3;

                    StartWBIKmotion(WBmode_RULP);
                    usleep(10*1000);

                    vec3 ori_H_pos = vec3(WBmotion->pLH_3x1);

                    for(int k=0; k<plug_dis_num; k++){
                        double delta_x = (k+1)*x_distort;
                        for(int j=0; j<5; j++){
                            double delta_y = 0.;
                            double delta_z = 0.;
                            if(j!=0){
                                delta_y = y_distort*cos((j-1)*90*D2R);
                                delta_z = z_distort*sin((j-1)*90*D2R);
                            }

                            vec3 cur_delta = delta_x*_plug_vecX + delta_y*_plug_vecY + delta_z*_plug_vecZ;
                            vec3 cur_target = ori_H_pos + cur_delta;

                            WBmotion->addLHPosInfo(cur_target[0], cur_target[1], cur_target[2], distort_time_s);
                            WBmotion->addLHOriInfo(plug_ds_des_H_ori, distort_time_s);
                            WBmotion->addLElbPosInfo(plug_elb[PLUG_SECOND], distort_time_s);
                            WBmotion->addWSTPosInfo(plug_wst[PLUG_SECOND], distort_time_s);
                            usleep((distort_time_s*1000+10)*1000);
                        }
                    }

                    printf("Plug :: plug in done ...!!!\n");

                    sharedData->STATE_COMMAND = TCMD_PLUG_MOTION_PLUG_IN_DONE;
                    break;
                }
                case 4:
                {
                    sharedData->STATE_COMMAND = TCMD_PLUG_MOTION_ESCAPE_START;

                    // Escape
                    StartWBIKmotion(WBmode_RULP);
                    usleep(10*1000);

                    joint->SetMoveJoint(LF2, -125, 10, MODE_ABSOLUTE);
                    usleep(4500*1000);
                    joint->SetMoveJoint(LF2, 0, 10, MODE_ABSOLUTE);
                    usleep(10*1000);

                    // Check open??

                    vec3 cur_target = pelv_out_hand[PLUG_SECOND] + plug_offset;
                    pos_sec = 3.;
                    WBmotion->addLHPosInfo(cur_target[0],cur_target[1], cur_target[2], pos_sec);
                    WBmotion->addLHOriInfo(plug_ds_des_H_ori, pos_sec);
                    WBmotion->addLElbPosInfo(plug_elb[PLUG_SECOND], pos_sec);
                    WBmotion->addWSTPosInfo(plug_wst[PLUG_SECOND], pos_sec);
                    usleep(3010*1000);

                    if(PLUG_FIRST == LEFT_FIRST_PLUG){
                        cur_target = ((pelv_ap_hand[PLUG_SECOND] + plug_offset)+pelv_ap_hand[PLUG_FIRST])/2.;
                        WBmotion->addLHPosInfo(cur_target[0], cur_target[1], cur_target[2], pos_sec);
                        WBmotion->addLHOriInfo(plug_ds_des_H_ori, pos_sec);
                        WBmotion->addLElbPosInfo(((plug_elb[PLUG_SECOND]+plug_elb[PLUG_FIRST])/2.), pos_sec);
                        WBmotion->addWSTPosInfo(((plug_wst[PLUG_SECOND]+plug_wst[PLUG_FIRST])/2.), pos_sec);
                        usleep(3010*1000);
                    }else if(PLUG_FIRST == RIGHT_FIRST_PLUG){
                        ;
                    }
                    joint->SetMoveJoint(LF2, 125, 10, MODE_ABSOLUTE);
                    usleep(4500*1000);
                    joint->SetMoveJoint(LF2, 0, 10, MODE_ABSOLUTE);
                    usleep(10*1000);

                    TurnOff_WBIK();
                    Change_Arm_pos_Move(3000);
                    usleep(3000*1000);

                    printf("Plug :: escape ...!!!\n");

                    sharedData->STATE_COMMAND = TCMD_PLUG_MOTION_ESCAPE_DONE;
                    break;
                }
                case 5:
                {
                    // Plug offset move
                    for(int k=0; k<3; k++)
                        plug_offset[k] = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[k]/100.;

                    double check = plug_offset.norm();
                    if(check >0.1)
                        plug_offset = vec3(0,0,0);

                    pos_sec = plug_offset.norm()/0.01;
                    printf("Plug Manual : %lf %lf %lf time %lf\n", plug_offset[0], plug_offset[1], plug_offset[2], pos_sec);
                    vec3 original_pos = pelv_align_hand[PLUG_SECOND];
                    vec3 new_pos = original_pos + plug_offset;

                    StartWBIKmotion(WBmode_RULP);
                    usleep(10*1000);

                    WBmotion->addLHPosInfo(new_pos[0], new_pos[1], new_pos[2], pos_sec);
                    WBmotion->addLHOriInfo(plug_ds_des_H_ori, pos_sec);
                    WBmotion->addLElbPosInfo(plug_elb[PLUG_SECOND], pos_sec);
                    WBmotion->addWSTPosInfo(plug_wst[PLUG_SECOND], pos_sec);
                    usleep(pos_sec*1000*1000);

                    sharedData->STATE_COMMAND = TCMD_PLUG_MOTION_MODIFY_DONE;
                    break;
                }
                case 6:
                {
                    TurnOff_WBIK();

                    postime = 4000;
                    joint->SetMoveJoint(RSP, -50.0, postime, MOVE_ABSOLUTE);
                    usleep(1000*1000);
                    postime = 3000;
                    joint->SetMoveJoint(RSR, 10.0, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RSY, 43.0, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(REB, -45.0, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RWY, 21.0, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RWP, 45.0, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RF1, 0.0, postime, MOVE_ABSOLUTE);
                    usleep(3000*1000);

                    sharedData->STATE_COMMAND = TCMD_PLUG_MOTION_CAM_POS_DONE;
                    break;
                }
                case 7:
                {
                    TurnOff_WBIK();

                    postime = 4000;
                    joint->SetMoveJoint(RSP, 40.0, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(REB, -130, postime, MOVE_ABSOLUTE);
                    usleep(1000*1000);
                    postime = 3000;
                    joint->SetMoveJoint(RSR, 17.0, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RSY, 0.0, postime, MOVE_ABSOLUTE);

                    joint->SetMoveJoint(RWY, -10.0, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RWP, 40.0, postime, MOVE_ABSOLUTE);
                    joint->SetMoveJoint(RF1, 0.0, postime, MOVE_ABSOLUTE);
                    usleep(3000*1000);

                    sharedData->STATE_COMMAND = TCMD_PLUG_MOTION_RETURN_NORMAL_DONE;
                    break;
                }
                case 8:
                {
                    // EM of grab
                    StartWBIKmotion(WBmode_RULP);
                    usleep(10*1000);

                    joint->SetMoveJoint(LF2, -125, 10, MODE_ABSOLUTE);
                    usleep(4500*1000);

                    pos_sec = 2.5;
                    WBmotion->addLHPosInfo(pelv_out_hand[PLUG_FIRST][0], pelv_out_hand[PLUG_FIRST][1], pelv_out_hand[PLUG_FIRST][2], pos_sec);
                    WBmotion->addLHOriInfo(plug_ds_des_H_ori, pos_sec);
                    WBmotion->addLElbPosInfo(plug_elb[PLUG_FIRST], pos_sec);
                    usleep(2510*1000);

                    joint->SetMoveJoint(LF2, 125, 10, MODE_ABSOLUTE);
                    usleep(4500*1000);
                    joint->SetMoveJoint(LF2, 0, 10, MODE_ABSOLUTE);
                    usleep(10*1000);

                    pos_sec = 1.8;
                    WBmotion->addLHPosInfo(pelv_ap_hand[PLUG_FIRST][0], pelv_ap_hand[PLUG_FIRST][1], pelv_ap_hand[PLUG_FIRST][2], pos_sec);
                    WBmotion->addLHOriInfo(plug_ds_des_H_ori, pos_sec);
                    WBmotion->addLElbPosInfo(plug_elb[PLUG_FIRST], pos_sec);
                    WBmotion->addWSTPosInfo(plug_wst[PLUG_FIRST], pos_sec);
                    usleep(1810*1000);

                    TurnOff_WBIK();

                    Change_Arm_pos_Move(3000);
                    usleep(3000*1000);

                    break;
                }
                case 11:
                {
//                    StartWBIKmotion(WBmode_RULP);
//                    usleep(10*1000);
                    double cur_WST = WBmotion->des_rWST*R2D;
                    double tar_WST = cur_WST +5;
                    pos_sec = 2;
                    WBmotion->addWSTPosInfo(tar_WST, pos_sec);
                    break;
                }
                case 12:
                {
//                    StartWBIKmotion(WBmode_RULP);
//                    usleep(10*1000);
                    double cur_WST = WBmotion->des_rWST*R2D;
                    double tar_WST = cur_WST -5;
                    pos_sec = 2;
                    WBmotion->addWSTPosInfo(tar_WST, pos_sec);
                    break;
                }
                default :
                    break;
                }
            }
            sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
            break;
            /******************************************************************************************
             * MANUAL movement
            *******************************************************************************************/
        case WMupperbody_AL_MANUAL:
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();

            StartWBIKmotion(WBmode_RULU);
            freeSleep(10*1000);
            {
            int des_des = sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0];
            int des_hand = sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[1];
            double tik_distance = fabs(sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[0]);
            pos_sec =1.*(tik_distance/0.01);

            if(pos_sec < 0.3)
                pos_sec = 0.3;
            if(pos_sec > 5.)
                pos_sec = 5.;

            vec3 delta;

            if(des_des == 1){
                delta = vec3(0, 0, tik_distance);
            }else if(des_des == 2){
                delta = vec3(0, tik_distance, tik_distance);
            }else if(des_des == 3){
                delta = vec3(0, tik_distance, 0);
            }else if(des_des == 4){
                delta = vec3(0, tik_distance, -tik_distance);
            }else if(des_des == 5){
                delta = vec3(0, 0, -tik_distance);
            }else if(des_des == 6){
                delta = vec3(0, -tik_distance, -tik_distance);
            }else if(des_des == 7){
                delta = vec3(0, -tik_distance, 0);
            }else if(des_des == 8){
                delta = vec3(0, -tik_distance, tik_distance);
            }else if(des_des == 9){
                delta = vec3(tik_distance, 0, 0);
            }else if(des_des == 10){
                delta = vec3(-tik_distance, 0, 0);
            }

            if(des_hand ==0){
                vec3 cur_pos = vec3(WBmotion->pLH_3x1);
                vec3 des_pos = cur_pos + delta;
                WBmotion->addLHPosInfo(des_pos[0], des_pos[1], des_pos[2], pos_sec);
            }else{
                vec3 cur_pos = vec3(WBmotion->pRH_3x1);
                vec3 des_pos = cur_pos + delta;
                WBmotion->addRHPosInfo(des_pos[0], des_pos[1], des_pos[2], pos_sec);
            }
            freeSleep((pos_sec*1000+10)*1000);

            }
            sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
            break;
        case WMupperbody_AL_MANUAL_ORI:
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();

            StartWBIKmotion(WBmode_RULU);
            freeSleep(10*1000);
            {
            int des_des = sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0];
            int des_hand = sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[1];
            double tik_angle = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            pos_sec =1.*(fabs(tik_angle)/5.);

            if(pos_sec < 0.3)
                pos_sec = 0.3;
            if(pos_sec > 5.)
                pos_sec = 5.;

            quat cur_ori;
            vec3 des_axis;
            if(des_hand ==0)
                cur_ori = quat(WBmotion->qLH_4x1);
            else
                cur_ori = quat(WBmotion->qRH_4x1);

            if(des_des == 1){
                des_axis = vec3(1,0,0);
            }else if(des_des ==2){
                des_axis = vec3(0,1,0);
            }else if(des_des == 3){
                des_axis = vec3(0,0,1);
            }

            quat des_delta = quat(des_axis, tik_angle*D2R);
            quat des_ori = des_delta*cur_ori;
            doubles ds_des_ori;
            for(int k=0; k<4; k++)
                ds_des_ori[k] = des_ori[k];

            if(des_hand ==0)
                WBmotion->addLHOriInfo(ds_des_ori, pos_sec);
            else
                WBmotion->addRHOriInfo(ds_des_ori, pos_sec);

            freeSleep((pos_sec*1000+10)*1000);
            }
            sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
            break;
        case WMupperbody_AL_MANUAL_ELB:
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();

            StartWBIKmotion(WBmode_RULU);
            freeSleep(10*1000);
            {
            int des_hand = sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[1];
            double tik_angle = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];

            pos_sec =1.5*(fabs(tik_angle)/10.);

            if(pos_sec < 0.3)
                pos_sec = 0.3;
            if(pos_sec > 5.)
                pos_sec = 5.;

            double cur_elb, des_elb;
            if(des_hand ==0){
                cur_elb = WBmotion->LElb_ang*R2D;
                des_elb = cur_elb + tik_angle;
            }else{
                cur_elb = WBmotion->RElb_ang*R2D;
                des_elb = cur_elb - tik_angle;
            }

            if(des_hand ==0)
                WBmotion->addLElbPosInfo(des_elb, pos_sec);
            else
                WBmotion->addRElbPosInfo(des_elb, pos_sec);

            freeSleep((pos_sec*1000+10)*1000);
            }
            sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
            break;
        /******************************************************************************************
         * Walking Push Door
        *******************************************************************************************/
        case WMupperbody_AL_WALKPUSH_ETC:
        {
            char etc_num = sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0];
            if(etc_num == 1){
                // Push Door Walking Functions;

                sharedData->STATE_COMMAND = TCMD_WALKPUSHDOOR_WALKTHROUGH_START;

                double ap_x = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                double ap_y = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
                double ap_a = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];

                userData->G2M.GoalPosX = ap_x;
                userData->G2M.GoalPosY = ap_y;
                userData->G2M.GoalAngle = ap_a;
                userData->G2M.WalkingGoalGoModeCommand = GOALGO_NORMAL;
                sharedData->COMMAND[PODO_NO_WALK].USER_PARA_INT[9] = GOAL_WALKING;
                sharedData->COMMAND[PODO_NO_WALK].USER_PARA_INT[4] = INSIDE_WALKING;
                sharedData->COMMAND[PODO_NO_WALK].USER_COMMAND = FREEWALK_WALK;

                userData->WalkDoneFlag = false;
                int whilecnt=0;
                while(1){
                    if(userData->WalkDoneFlag == true){
                        userData->WalkDoneFlag = false;
                        break;
                    }
                    whilecnt++;
                    if(whilecnt>=800){
                        break;
                    }
                    usleep(50*1000);
                }
                usleep(200*1000);
                printf("Push Door Walking Finished...!!!\n");
                sharedData->STATE_COMMAND = TCMD_WALKPUSHDOOR_WALKTHROUGH_DONE;
            }else if(etc_num == 2){
                double step_length = fabs(sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[0]);
                if(step_length > 0.4)
                    step_length = 0.4;
                printf("Step Length : %lf\n", step_length);


                userData->G2M.WalkingStopModeCommand = COMPLETE_STOP_WALKING;
                userData->G2M.WalkingModeCommand = FORWARD_WALKING;

                userData->G2M.StepTime = 1.1;
                userData->G2M.StepNum = 1;
                userData->G2M.StepLength = step_length;
                userData->G2M.StepAngle = 0.;

                sharedData->COMMAND[PODO_NO_WALK].USER_PARA_INT[9] = NORMAL_WALKING;
                sharedData->COMMAND[PODO_NO_WALK].USER_COMMAND = FREEWALK_DSP_HOLD_WALK;

                userData->WalkDoneFlag = false;
                int whilecnt=0;
                while(1){
                    if(userData->WalkDoneFlag == true){
                        userData->WalkDoneFlag = false;
                        break;
                    }
                    whilecnt++;
                    if(whilecnt>=800){
                        break;
                    }
                    usleep(50*1000);
                }
                usleep(200*1000);
                printf("Push Door Walking Finished...!!!\n");
            }
            sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
            break;
        }
        case WMupperbody_AL_WALKPUSH_STEP:
        {
            char motion_num = sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0];
            if(motion_num == 1){
                int walk_push_door_check_1;
                if(DOOR_MODE == LEFT_DOOR_MODE)
                    walk_push_door_check_1 = WalkPushDoor_GrabAndOpen_LEFT(0);
                else if(DOOR_MODE == RIGHT_DOOR_MODE)
                    walk_push_door_check_1 = WalkPushDoor_GrabAndOpen(0);

                if(walk_push_door_check_1 == 1)
                    printf("----------------- Walking Push Motion 1/3 End...!!!--------------------\n");
                else if(walk_push_door_check_1 == 4)
                    printf("----------------- Walking Push Motion 1/3 E-stop...!!!--------------------\n");
                else
                    printf("----------------- Walking Push Motion 1/3 Fail...!!!--------------------\n");

            }else if(motion_num ==2){
                int walk_push_door_check_2;
                if(DOOR_MODE == LEFT_DOOR_MODE)
                    walk_push_door_check_2 = WalkPushDoor_LHblock_RHback_RHblock_LEFT(0);
                else if(DOOR_MODE == RIGHT_DOOR_MODE)
                    walk_push_door_check_2 = WalkPushDoor_LHblock_RHback_RHblock(0);

                if(walk_push_door_check_2 == 1)
                    printf("----------------- Walking Push Motion 2/3 End...!!!--------------------\n");
                else if(walk_push_door_check_2 == 4)
                    printf("----------------- Walking Push Motion 2/3 E-stop...!!!--------------------\n");
                else
                    printf("----------------- Walking Push Motion 2/3 Fail...!!!--------------------\n");

            }else if(motion_num ==3){
                int walk_push_door_check_3;
                if(DOOR_MODE == LEFT_DOOR_MODE)
                    walk_push_door_check_3 = WalkPushDoor_RotateWST_ReadyPass_LEFT(0);
                else if(DOOR_MODE == RIGHT_DOOR_MODE)
                    walk_push_door_check_3 = WalkPushDoor_RotateWST_ReadyPass(0);

                if(walk_push_door_check_3 == 1)
                    printf("----------------- Walking Push Motion 3/3 End...!!!--------------------\n");
                else if(walk_push_door_check_3 == 4)
                    printf("----------------- Walking Push Motion 3/3 E-stop...!!!--------------------\n");
                else
                    printf("----------------- Walking Push Motion 3/3 Fail...!!!--------------------\n");
            }
            sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
            break;
        }
        case WMupperbody_AL_WALKPUSH_EM:
        {
            char em_category = sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0];
            if(em_category == 1){
                // Walk Push step 1 EM
                printf("Walkg Push EM 1 - Return R Hand...!!!\n");
                if(DOOR_MODE == LEFT_DOOR_MODE)
                    EM_WalkPushDoor_ReturnHand_LEFT();
                else if(DOOR_MODE == RIGHT_DOOR_MODE)
                    EM_WalkPushDoor_ReturnHand();
            }else if(em_category == 2){
                // Walk Push step 2 EM
                printf("Walkg Push EM 2 - Return L Hand...!!!\n");
                if(DOOR_MODE == LEFT_DOOR_MODE)
                    EM_WalkPushDoor_ReturnLH_LEFT();
                else if(DOOR_MODE == RIGHT_DOOR_MODE)
                    EM_WalkPushDoor_ReturnLH();
            }else if(em_category == 3){
                // Walk Push step 2 EM Keep
                printf("Walkg Push EM 2 - Keep going...!!!\n");
                if(DOOR_MODE == LEFT_DOOR_MODE)
                    WalkPushDoor_LHblock_RHback_RHblock_LEFT(1);
                else if(DOOR_MODE == RIGHT_DOOR_MODE)
                    WalkPushDoor_LHblock_RHback_RHblock(1);
            }
            sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
            break;
        }
        default:
            sharedData->COMMAND[PODO_NO].USER_COMMAND = WMupperbody_AL_NO_ACT;
            break;
        }
    }
    cout << ">>> Process WMupperbody is terminated..!!" << endl;
    return 0;
}
// --------------------------------------------------------------------------------------------- //



// --------------------------------------------------------------------------------------------- //
void RBTaskThread(void *)
{

    while(isTerminated == 0){

        // update the joint values and push it to the shared memory

//        {
//        double temp_d = sharedData->IMUAccZ[0];
//        printf("oh nul debug : %lf\n", temp_d);
//        }

        // Rotate Handle
        if(_isTrap == true)
        {
            _Trap_indicator = TrapFunction();
            if(_Trap_indicator==0)
                ;
            else if(_Trap_indicator==1)
            {
                _TrapCount = 0;
                _Trap_togo = 0;
                _Trap_maxVel = 0;
                _Trap_maxAcc = 0;
                _isTrap = false;
            }
            else
                _isTrap = false;
            _TrapCount++;
            if(_TrapCount>=1000000)
                _TrapCount =0;
        }
        // FT averaging
        if(FTavgFlag == true)
        {
            vec3 RWFTvalue;
            vec3 LWFTvalue;
            RWFTvalue[0] = sharedData->FT[2].Fx;
            RWFTvalue[1] = sharedData->FT[2].Fy;
            RWFTvalue[2] = sharedData->FT[2].Fz;
            LWFTvalue[0] = sharedData->FT[3].Fx;
            LWFTvalue[1] = sharedData->FT[3].Fy;
            LWFTvalue[2] = sharedData->FT[3].Fz;

            double cur_RWFT = RWFTvalue.norm();
            double cur_LWFT = LWFTvalue.norm();
            FTavgSUM += cur_RWFT;
            FTavgSUM_2 += cur_LWFT;

            FTavgCount++;
            if(FTavgCount ==200){
                FTavg = FTavgSUM/200.;
                FTavg_2 = FTavgSUM_2/200.;
                FTavgSUM = 0.;
                FTavgSUM_2 = 0.;
                FTavgCount =0;
                FTavgFlag = false;
            }
            if(FTavgCount>=1000000)
                FTavgCount =0;
        }

        if(WB_FLAG == true)
        {
            WBmotion->updateAll();
            WBmotion->WBIK_UB();

//            for(int i=RHY; i<=LAR; i++) joint->SetJointRefAngle(i, WBmotion->Q_filt_34x1[idRHY+i-RHY]*R2D);

            joint->SetJointRefAngle(WST, WBmotion->Q_filt_34x1[idWST]*R2D);

            joint->SetJointRefAngle(RSP, WBmotion->Q_filt_34x1[idRSP]*R2D);
            joint->SetJointRefAngle(RSR, WBmotion->Q_filt_34x1[idRSR]*R2D - OFFSET_RSR);
            joint->SetJointRefAngle(RSY, WBmotion->Q_filt_34x1[idRSY]*R2D);
            joint->SetJointRefAngle(REB, WBmotion->Q_filt_34x1[idREB]*R2D - OFFSET_ELB);
            joint->SetJointRefAngle(RWY, WBmotion->Q_filt_34x1[idRWY]*R2D);
            joint->SetJointRefAngle(RWP, WBmotion->Q_filt_34x1[idRWP]*R2D);
            joint->SetJointRefAngle(RF1, WBmotion->Q_filt_34x1[idRWY2]*R2D);

            joint->SetJointRefAngle(LSP, WBmotion->Q_filt_34x1[idLSP]*R2D);
            joint->SetJointRefAngle(LSR, WBmotion->Q_filt_34x1[idLSR]*R2D - OFFSET_LSR);
            joint->SetJointRefAngle(LSY, WBmotion->Q_filt_34x1[idLSY]*R2D);
            joint->SetJointRefAngle(LEB, WBmotion->Q_filt_34x1[idLEB]*R2D - OFFSET_ELB);
            joint->SetJointRefAngle(LWY, WBmotion->Q_filt_34x1[idLWY]*R2D);
            joint->SetJointRefAngle(LWP, WBmotion->Q_filt_34x1[idLWP]*R2D);
            joint->SetJointRefAngle(LF1, WBmotion->Q_filt_34x1[idLWY2]*R2D);

            if(!CheckMotionOwned())
                WB_FLAG = false;
        }

        if(WB_GLOBAL_FLAG == true)
        {
            WBmotion->updateAll();
            WBmotion->WBIK();

            for(int i=RHY; i<=LAR; i++) joint->SetJointRefAngle(i, WBmotion->Q_filt_34x1[idRHY+i-RHY]*R2D);

            joint->SetJointRefAngle(WST, WBmotion->Q_filt_34x1[idWST]*R2D);

            joint->SetJointRefAngle(RSP, WBmotion->Q_filt_34x1[idRSP]*R2D);
            joint->SetJointRefAngle(RSR, WBmotion->Q_filt_34x1[idRSR]*R2D - OFFSET_RSR);
            joint->SetJointRefAngle(RSY, WBmotion->Q_filt_34x1[idRSY]*R2D);
            joint->SetJointRefAngle(REB, WBmotion->Q_filt_34x1[idREB]*R2D - OFFSET_ELB);
            joint->SetJointRefAngle(RWY, WBmotion->Q_filt_34x1[idRWY]*R2D);
            joint->SetJointRefAngle(RWP, WBmotion->Q_filt_34x1[idRWP]*R2D);
            joint->SetJointRefAngle(RF1, WBmotion->Q_filt_34x1[idRWY2]*R2D);

            joint->SetJointRefAngle(LSP, WBmotion->Q_filt_34x1[idLSP]*R2D);
            joint->SetJointRefAngle(LSR, WBmotion->Q_filt_34x1[idLSR]*R2D - OFFSET_LSR);
            joint->SetJointRefAngle(LSY, WBmotion->Q_filt_34x1[idLSY]*R2D);
            joint->SetJointRefAngle(LEB, WBmotion->Q_filt_34x1[idLEB]*R2D - OFFSET_ELB);
            joint->SetJointRefAngle(LWY, WBmotion->Q_filt_34x1[idLWY]*R2D);
            joint->SetJointRefAngle(LWP, WBmotion->Q_filt_34x1[idLWP]*R2D);
            joint->SetJointRefAngle(LF1, WBmotion->Q_filt_34x1[idLWY2]*R2D);

            if(!CheckMotionOwned())
                WB_GLOBAL_FLAG = false;
        }

//        if(saveFlag==1)
//        {
//            DataBuf[0][saveIndex]=sharedData->FTFx[2];
//            DataBuf[1][saveIndex]=sharedData->FTFy[2];
//            DataBuf[2][saveIndex]=sharedData->FTFz[2];
//            saveIndex++;
//        }

        joint->MoveAllJoint();
        rt_task_suspend(&rtTaskCon);
    }
}
void RBFlagThread(void *)
{
    rt_task_set_periodic(NULL, TM_NOW, 300*1000);        // 300 usec

    while(isTerminated == 0)
    {
        rt_task_wait_period(NULL);

        if(HasAnyOwnership()){
            if(sharedData->SYNC_SIGNAL[PODO_NO] == true){
                joint->JointUpdate();
                rt_task_resume(&rtTaskCon);
            }
        }
    }
}

/**************************************************************************************************
 * Motion Functions
 *************************************************************************************************/
int PullDoor_GrabAndOpen(int _mode)
{
    // Grab handle and rotate handle

    // ----------------------------------------------------------------------
    // Goto Approach point
    // ----------------------------------------------------------------------

    double des_x, des_y, des_z;
    double des_x2, des_y2, des_z2;
    doubles des_RH_ori(4);

    vec3 pelv_nobe, cur_vecX, cur_vecY, cur_vecZ;
    for(int k=0; k<3; k++){
        pelv_nobe[k] = (double)(sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[k]);
        cur_vecX[k] = (double)(sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[k+3]);
    }
    cur_vecZ[0] = 0.;
    cur_vecZ[1] = 0.;
    cur_vecZ[2] = 1.;

    cur_vecY = cross(cur_vecZ, cur_vecX);

    float door_hand_ap_margin[3];
    // Approach
    door_hand_ap_margin[0]=0.155;
    door_hand_ap_margin[1]=0.073;//depend // 0.06 ~ 0.7
    door_hand_ap_margin[2]=0.;
    des_x = pelv_nobe[0]-door_hand_ap_margin[0]*cur_vecX[0]-door_hand_ap_margin[1]*cur_vecY[0]-door_hand_ap_margin[2]*cur_vecZ[0];
    des_y = pelv_nobe[1]-door_hand_ap_margin[0]*cur_vecX[1]-door_hand_ap_margin[1]*cur_vecY[1]-door_hand_ap_margin[2]*cur_vecZ[1];
    des_z = pelv_nobe[2]-door_hand_ap_margin[0]*cur_vecX[2]-door_hand_ap_margin[1]*cur_vecY[2]-door_hand_ap_margin[2]*cur_vecZ[2];
    // Insert
    door_hand_ap_margin[0]=0.085;//Depend 0.07~0.09
    door_hand_ap_margin[1]=0.073;//depend
    door_hand_ap_margin[2]=0.;
    des_x2 = pelv_nobe[0]-door_hand_ap_margin[0]*cur_vecX[0]-door_hand_ap_margin[1]*cur_vecY[0]-door_hand_ap_margin[2]*cur_vecZ[0];
    des_y2=pelv_nobe[1]-door_hand_ap_margin[0]*cur_vecX[1]-door_hand_ap_margin[1]*cur_vecY[1]-door_hand_ap_margin[2]*cur_vecZ[1];
    des_z2=pelv_nobe[2]-door_hand_ap_margin[0]*cur_vecX[2]-door_hand_ap_margin[1]*cur_vecY[2]-door_hand_ap_margin[2]*cur_vecZ[2];

    mat3 _wall_rotaion_matrix;
    _wall_rotaion_matrix[0][0]=cur_vecX[0];  _wall_rotaion_matrix[0][1]=cur_vecY[0];  _wall_rotaion_matrix[0][2]=cur_vecZ[0];
    _wall_rotaion_matrix[1][0]=cur_vecX[1];  _wall_rotaion_matrix[1][1]=cur_vecY[1];  _wall_rotaion_matrix[1][2]=cur_vecZ[1];
    _wall_rotaion_matrix[2][0]=cur_vecX[2];  _wall_rotaion_matrix[2][1]=cur_vecY[2];  _wall_rotaion_matrix[2][2]=cur_vecZ[2];

    quat q_wall = quat(_wall_rotaion_matrix);
    quat q_temp = quat(vec3(0,1,0),-90*D2R);
    quat q_des_RH=q_wall*q_temp;
    for(int k=0; k<4; k++){
        des_RH_ori[k] = q_des_RH[k];
        _EM_AP_ORI[k] = q_des_RH[k];
    }
    _EM_AP_POS[0] =des_x;
    _EM_AP_POS[1] =des_y;
    _EM_AP_POS[2] =des_z;

    StartWBIKmotion(WBmode_RULU);
    freeSleep(10*1000);

    pos_sec=3.5;
    WBmotion->addRHPosInfo(des_x, des_y, des_z, pos_sec);
    WBmotion->addRElbPosInfo(-70, pos_sec);
    WBmotion->addRHOriInfo(des_RH_ori, pos_sec);
    int tc = FingerPositionInput_RIGHT(-14000);

    if(tc>=3500){
        sck = freeSleep(50*1000);
        if(sck==1) return 4;
    }else{
        sck = freeSleep((3500-tc+50)*1000);
        if(sck==1) return 4;
    }

    // ----------------------------------------------------------------------
    // FT sensor null
    // ----------------------------------------------------------------------
    MCWristFTsensorNull(2, 53);
    //RBwFTsensorNull(2, 53);
    sck = freeSleep(600*1000);
    if(sck==1) return 4;

    // ----------------------------------------------------------------------
    // Insert Hand
    // ----------------------------------------------------------------------
    pos_sec = 1.8;
    WBmotion->addRHPosInfo(des_x2, des_y2, des_z2, pos_sec);
    sck = freeSleep(1810*1000);
    if(sck==1) return 4;

    // ----------------------------------------------------------------------
    // Gain Over start
    // ----------------------------------------------------------------------
    MCBoardSetSwitchingMode(2, 15, 1);//RSY REB
    MCBoardSetSwitchingMode(2, 16, 1);//RWY RWP
    MCJointGainOverride(2, 15, 1, eq_gain_1, 500);//RSY
    MCJointGainOverride(2, 15, 2, eq_gain_1, 500);//REB
    MCJointGainOverride(2, 16, 1, eq_gain_0, 500);//RWY
    MCJointGainOverride(2, 16, 2, eq_gain_1, 500);//RWP
//    RBBoardSetSwitchingMode(2,15,1);
//    RBBoardSetSwitchingMode(2,16,1);

//    unsigned int temp_gain=1;
//    RBJointGainOverride(2,15,temp_gain,temp_gain,500);//RSY REB
//    RBJointGainOverride(2,16,0,temp_gain,500);//RWY RWP
    sck = freeSleep(500*1000);
    if(sck==1) return 4;

    // ----------------------------------------------------------------------
    // Grab handle
    // ----------------------------------------------------------------------

    joint->SetMoveJoint(RF2, 125, 10, MODE_ABSOLUTE);
    sck = freeSleep(2700*1000);
    if(sck==1) return 4;
    joint->SetMoveJoint(RF2, 0, 10, MODE_ABSOLUTE);
    pos_sec = 0.8;
    WBmotion->addRHPosInfo((des_x2+0.03), des_y2, des_z2, pos_sec);
    sck = freeSleep(600*1000);
    if(sck==1) return 4;
    joint->SetMoveJoint(RF2, 125, 10, MODE_ABSOLUTE);
    sck = freeSleep(4000*1000);
    if(sck==1) return 4;
    //joint->SetMoveJoint(RF2, 0, 10, MODE_ABSOLUTE);//hyoin test
    WBmotion->addRHPosInfo(des_x2, des_y2, des_z2, pos_sec);
    sck = freeSleep(820*1000);
    if(sck==1) return 4;


    GetPosFromENC();
    freeSleep(20*1000);
    vec3 befor_rot = vec3(WBmotion->enc_pRH_3x1);

    FTavgFlag = true;
    freeSleep(1010*1000);
    befor_FTavg = FTavg;
    freeSleep(50*1000);

    if(_mode == 1)//unit motion case
        return 1;

    // ----------------------------------------------------------------------
    // Rotate handle
    // ----------------------------------------------------------------------
    handle_rotate_center = pelv_nobe;
    handle_rotate_axis = cur_vecX;
    handle_target_angle = 55;//depend

    RotateDoorPushHandle(1);

    // ----------------------------------------------------------------------
    // Check OK
    // ----------------------------------------------------------------------
    GetPosFromENC();
    freeSleep(20*1000);
    vec3 after_rot = vec3(WBmotion->enc_pRH_3x1);

    FTavgFlag = true;
    freeSleep(1010*1000);
    after_FTavg = FTavg;

    vec3 hand_pos_err = after_rot - befor_rot;
    printf("Motion Checker 1::Hand pos change:: %lf .....!!!\n", hand_pos_err[0]);
    double FTratio = befor_FTavg / after_FTavg;
    printf("Motion Checker 1::FT sensor change:: %lf %lf %lf\n", befor_FTavg, after_FTavg, FTratio);

    int ENCODER_TEST = false;
    int FTSENSOR_TEST = false;

    if(hand_pos_err[0] < -0.011)
        ENCODER_TEST = true;
    if((FTratio >2.) || isnan(FTratio))
        FTSENSOR_TEST = true;

    if((ENCODER_TEST==true) && (FTSENSOR_TEST==true)){
        return 1;//success
    }else{
        return 0;//fail
    }

    // ----------------------------------------------------------------------
    // END
    // ----------------------------------------------------------------------
}
int PullDoor_OpenDoorAndLHblock(int _mode)
{
    // Open door with wheel and L-block
    if(_mode==0){
        // ----------------------------------------------------------------------
        // Open Door
        // ----------------------------------------------------------------------
        double Door_WHEEL_radius = fabs(sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[6]);

        sck = freeSleep(200*1000);
        if(sck==1) return 4;

        sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[0] =1;//right  0 left
        sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[1] =-1;//1 straight -1 back
        sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[0] = Door_WHEEL_radius;//wheel radius
        sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[1] =60;//movaing angle

        sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[3] = 3;//vel change falg
        sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[3] = 195;
        sharedData->COMMAND[PODO_NO_WHEEL].USER_COMMAND= OMNIWHEEL_AL_RADIUS;
        userData->WheelDoneFlag = false;
        sck = freeSleep(1000*1000);
        if(sck==1) return 4;

        // ----------------------------------------------------------------------
        // Re-rotate handle
        // ----------------------------------------------------------------------
        RotateDoorPushHandle(-1);

        // ----------------------------------------------------------------------
        // Rotate WST
        // ----------------------------------------------------------------------
        whilecnt=0;
        while(1){
            if(userData->WheelDoneFlag == true){
                userData->WheelDoneFlag = false;
                printf("Wheel is ended...! next step is ready to start\n");
                break;
            }
            whilecnt++;
            if(whilecnt>=1200){
                printf("Wheel is not!!! ended...! time gap out\n");
                break;
            }
            sck = freeSleep(50*1000);
            if(sck==1) break;
        }
        sck = freeSleep(100*1000);
        if(sck==1) return 4;
    }

    StartWBIKmotion(WBmode_RPLU);
    freeSleep(10*1000);

    double des_WST=-55;
    WBmotion->addWSTPosInfo(des_WST, 2.7);

    sck = freeSleep(2730*1000);
    if(sck==1) return 4;

    // ----------------------------------------------------------------------
    // Open door with arm motion
    // ----------------------------------------------------------------------
    StartWBIKmotion(WBmode_RULU);
    freeSleep(20*1000);

    GetPosFromENC();
    freeSleep(20*1000);

    vec3 temp_RH_pos = vec3(WBmotion->enc_pRH_3x1);//WBmotion->pRH_3x1
    vec3 temp_RH_pos_2 = vec3(WBmotion->pRH_3x1);
    quat temp_RH_ori = quat(WBmotion->qRH_4x1);//WBmotion->qRH_4x1

    mat3 temp_RH_mat=mat3(temp_RH_ori);
    vec3 door_hin = temp_RH_pos +temp_RH_mat*vec3(0.0, -0.72, -0.07);

    handle_rotate_center[0] = door_hin[0];
    handle_rotate_center[1] = door_hin[1];
    handle_rotate_center[2] = temp_RH_pos[2];
    handle_rotate_axis[0] = 0.;
    handle_rotate_axis[1] = 0.;
    handle_rotate_axis[2] = 1.;
    handle_target_angle = 15;
    handle_dir = 1;

    handle_pos_first = pos(temp_RH_pos_2, temp_RH_ori);
    handle_angle_now = 0.;
    _Trap_togo = handle_target_angle;
    _Trap_maxVel = 10;
    _Trap_maxAcc = 10;
    _isTrap = true;
    _Trap_FINISH_FLAG = false;
    _Trap_Motion_Command = TR_HANDLE_ROTATE;

    Handle_rotate_done_Flag =false;

    whilecnt =0;
    while(1){
        if(Handle_rotate_done_Flag == true){
            Handle_rotate_done_Flag = false;
            break;
        }
        whilecnt ++;
        if(whilecnt > 300)
            break;
        sck = freeSleep(50*1000);
        if(sck==1) break;
    }
    // ----------------------------------------------------------------------
    // Left arm blocking
    // ----------------------------------------------------------------------

    StartWBIKmotion(WBmode_RULU);
    freeSleep(10*1000);

    GetPosFromENC();
    freeSleep(10*1000);

    vec3 cur_RH_pos = vec3(WBmotion->enc_pRH_3x1);//WBmotion->pRH_3x1
    quat cur_RH_ori = quat(WBmotion->qRH_4x1);//WBmotion->qRH_4x1
    vec3 des_LH_pos, des_LH_pos2;    quat des_LH_ori;

    mat3 cur_RH_mat=mat3(cur_RH_ori);
    // depend
    des_LH_pos = cur_RH_pos +cur_RH_mat*vec3(-0.28, 0.3, -0.29);// global z --> -0.165 for original door // -0.28 for original door(3cm)
    des_LH_pos2 = cur_RH_pos +cur_RH_mat*vec3(-0.28, 0.015, -0.24);// global z --> -0.165 // -0.23 for original door(3cm)

    for(int k=0; k<3; k++)
        _EM_LH_POS[k] = des_LH_pos[k];

    quat q_temp = quat(vec3(0,0,1),-90*D2R);
    quat q_temp_2 = quat(vec3(0,0,1), 90*D2R);
    quat q_temp_3 = quat(vec3(0,1,0), 30*D2R);
    des_LH_ori = q_temp*cur_RH_ori*q_temp_2*q_temp_3;
    doubles ds_des_LH_ori(4);
    for(int k=0; k<4; k++){
        ds_des_LH_ori[k] = des_LH_ori[k];
        _EM_LH_ORI[k] = des_LH_ori[k];
    }

    pos_sec=2.3;
    WBmotion->addLHPosInfo(des_LH_pos[0],des_LH_pos[1],des_LH_pos[2], pos_sec);
    WBmotion->addLHOriInfo(ds_des_LH_ori, pos_sec);
    WBmotion->addLElbPosInfo(55, pos_sec);
    sck = freeSleep(2310*1000);
    if(sck==1) return 4;

    WBmotion->addLHPosInfo(des_LH_pos2[0],des_LH_pos2[1],des_LH_pos2[2], 2.3);
    sck = freeSleep(2310*1000);
    if(sck==1) return 4;

    joint->SetMoveJoint(RF2, 0, 10, MODE_ABSOLUTE);//hyoin test
    usleep(20*1000);

    // ----------------------------------------------------------------------
    // Check OK
    // ----------------------------------------------------------------------
    return 1;
    // ----------------------------------------------------------------------
    // END
    // ----------------------------------------------------------------------
}
int PullDoor_RHbackAndSmallRotate(void)
{
    // Right hand back and rotate with L-Hand
    // ----------------------------------------------------------------------
    // Right Hand back and Gain back
    // ----------------------------------------------------------------------
    int check = EM_RightHandBack();
    if(check == 4) return 4;

    // ----------------------------------------------------------------------
    // Rotate WST & shorten moment arm
    // ----------------------------------------------------------------------
    // Rotate WST
    double des_wheel_radius_rotate_angle = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[7];
    double back_length = 0.15;

    StartWBIKmotion(WBmode_RULU);
    freeSleep(10*1000);

    double des_WST =0.;
    double off = fabs(WH2PEL_x);
    double temp_err = 1000000;
    double px = WBmotion->pLH_3x1[0]-back_length;
    double py = WBmotion->pLH_3x1[1];
    for(int k=0; k<135; k++)
    {
        double th = -((double)k)*D2R;
        double cur_err = fabs(off - (px*cos(th)-py*sin(th)));
        if(cur_err < temp_err)
        {
            temp_err = cur_err;
            des_WST = th*R2D;
        }
    }

    double dw = des_WST*D2R;
    double temp_1 = -off + ((px-0)*cos(dw) - py*sin(dw));
    double temp_2 = (px-0)*sin(dw) + py*cos(dw);
    _door_getin_radius = sqrtp(temp_1*temp_1 + temp_2*temp_2);

    WBmotion->addWSTPosInfo(des_WST, 2.8);
    sck = freeSleep(2810*1000);
    if(sck==1) return 4;

    // Shorten L Arm
    vec3 cur_LH_pos = vec3(WBmotion->pLH_3x1);
    vec3 des_LH_pos = cur_LH_pos - vec3(back_length, 0., 0.);
    pos_sec = 2.;
    WBmotion->addLHPosInfo(des_LH_pos[0], des_LH_pos[1], des_LH_pos[2], pos_sec);
    sck = freeSleep(2010*1000);
    if(sck==1) return 4;
    TurnOff_WBIK();
    postime = 1300;
    joint->SetMoveJoint(RSP, 95.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(REB, -140.0, postime, MOVE_ABSOLUTE);
    sck = freeSleep(1310*1000);
    if(sck==1) return 4;

    // ----------------------------------------------------------------------
    // radius rotate
    // ----------------------------------------------------------------------
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[0] =1;//right  0 left
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[1] =1;//1 straight -1 back
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[0] =_door_getin_radius;//wheel radius
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[1] =des_wheel_radius_rotate_angle+6;//movaing angle

    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[3] = 3;//vel change falg
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[3] = 170;
    sharedData->COMMAND[PODO_NO_WHEEL].USER_COMMAND= OMNIWHEEL_AL_RADIUS;

    userData->WheelDoneFlag = false;
    whilecnt=0;
    while(1){
        if(userData->WheelDoneFlag == true){
            userData->WheelDoneFlag = false;
            printf("Wheel is ended...! next step is ready to start\n");
            break;
        }
        whilecnt++;
        if(whilecnt>=600){
            printf("Wheel is not!!! ended...! time gap out\n");
            break;
        }
        sck = freeSleep(50*1000);
        if(sck==1) break;
    }

    // ----------------------------------------------------------------------
    // Check OK
    // ----------------------------------------------------------------------
    return 1;
    // ----------------------------------------------------------------------
    // END
    // ----------------------------------------------------------------------
}
int PullDoor_RHblockAndReadyGGu(void)
{
    // R-block and ready to ggu mode

    // ----------------------------------------------------------------------
    // Right block
    // ----------------------------------------------------------------------
    double stretch_length = 0.195;

    StartWBIKmotion(WBmode_RULU);
    freeSleep(10*1000);

    vec3 cur_LH_pos = vec3(WBmotion->pLH_3x1);
    vec3 cur_RH_pos = vec3(WBmotion->pRH_3x1);

    vec3 des_LH_pos, des_LH_pos2, des_RH_pos, des_RH_pos2, des_RH_pos3;
    quat des_RH_ori;

    des_LH_pos = cur_LH_pos + vec3(stretch_length, 0., 0.);

    des_RH_pos = cur_RH_pos + vec3(0.4, 0.0, 0.1);
    des_RH_pos3 = des_LH_pos+ vec3(-0.03, -0.03, -0.15);
    des_RH_pos2 = des_RH_pos3 + vec3(-0.09, 0, 0);
    printf("Debug data : %lf \n", des_RH_pos2[0]);
    double temp_compen = 0.;
    if(des_RH_pos2[0]<0.4){
        temp_compen = 0.42 - des_RH_pos2[0];
        des_RH_pos2[0] = 0.42;
    }
    des_RH_pos3[0] += temp_compen;

    quat q_temp = quat(vec3(0,0,1), -90*D2R);
    des_RH_ori = quat(vec3(0,1,0),-90*D2R)*q_temp;
    doubles ds_des_RH_ori;
    for(int k=0; k<4; k++)
        ds_des_RH_ori[k] = des_RH_ori[k];

    // Stretch LH and RH
    pos_sec = 2.5;
    WBmotion->addLHPosInfo(des_LH_pos[0], des_LH_pos[1], des_LH_pos[2], pos_sec);

    WBmotion->addRHPosInfo(des_RH_pos[0],des_RH_pos[1],des_RH_pos[2], pos_sec);
    WBmotion->addRHOriInfo(ds_des_RH_ori, pos_sec);
    WBmotion->addRElbPosInfo(-55, pos_sec);
    sck = freeSleep(2510*1000);
    if(sck==1) return 4;

    pos_sec = 2.5;
    WBmotion->addRHPosInfo(des_RH_pos2[0],des_RH_pos2[1],des_RH_pos2[2], pos_sec);
    WBmotion->addRElbPosInfo(-65, pos_sec);
    sck = freeSleep(2510*1000);
    if(sck==1) return 4;

    pos_sec = 2.;
    WBmotion->addRHPosInfo(des_RH_pos3[0],des_RH_pos3[1],des_RH_pos3[2], pos_sec);
    sck = freeSleep(2010*1000);
    if(sck==1) return 4;

    // Stretch RH more
    cur_RH_pos = vec3(WBmotion->pRH_3x1);
    des_RH_pos = cur_RH_pos + vec3(0.06, 0., 0.);

    pos_sec = 1.2;
    WBmotion->addRHPosInfo(des_RH_pos[0],des_RH_pos[1],des_RH_pos[2], pos_sec);
    sck = freeSleep(1210*1000);
    if(sck==1) return 4;

    // ----------------------------------------------------------------------
    // Left Hand back and rotate WST to 0
    // ----------------------------------------------------------------------
    // back LH
    doubles ds_des_LH_ori(4);
    ds_des_LH_ori[0]=0.828992;  ds_des_LH_ori[1]=-0.359506;    ds_des_LH_ori[2]=-0.391188;  ds_des_LH_ori[3]=0.174642;
    double norm_temp = sqrtp(ds_des_LH_ori[0]*ds_des_LH_ori[0]+ds_des_LH_ori[1]*ds_des_LH_ori[1]+ds_des_LH_ori[2]*ds_des_LH_ori[2]+ds_des_LH_ori[3]*ds_des_LH_ori[3]);
    for(int k=0; k<4; k++)
        ds_des_LH_ori[k]=ds_des_LH_ori[k]/norm_temp;

    cur_LH_pos = vec3(WBmotion->pLH_3x1);
    des_LH_pos = cur_LH_pos - vec3(stretch_length, -0.18, 0.);

    pos_sec = 2.3;
    WBmotion->addLHPosInfo(des_LH_pos[0], des_LH_pos[1], des_LH_pos[2], pos_sec);
    sck = freeSleep(2310*1000);
    if(sck==1) return 4;

    pos_sec = 2.3;
    WBmotion->addLHPosInfo(0.25, 0.25, 0.15, pos_sec);
    WBmotion->addLHOriInfo(ds_des_LH_ori, pos_sec);
    WBmotion->addLElbPosInfo(12, pos_sec);
    sck = freeSleep(2310*1000);
    if(sck==1) return 4;

    // Make WST to 0
    StartWBIKmotion(WBmode_RPLU);
    freeSleep(10*1000);
    pos_sec = 3.3;
    WBmotion->addWSTPosInfo(0., pos_sec);
    WBmotion->addRElbPosInfo(-12, pos_sec);
    sck = freeSleep(3310*1000);
    if(sck==1) return 4;

    // ----------------------------------------------------------------------
    // Go more and change pos
    // ----------------------------------------------------------------------
    // Go slight
    TurnOff_WBIK();
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[0]=0.20;
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[1]=0.;
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[2]=0.;
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[3] = 3;//Vel Change Mode
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[3] = 200;
    sharedData->COMMAND[PODO_NO_WHEEL].USER_COMMAND= OMNIWHEEL_AL_GOTODES;

    userData->WheelDoneFlag = false;
    whilecnt=0;
    while(1){
        if(userData->WheelDoneFlag == true){
            userData->WheelDoneFlag = false;
            printf("Wheel is ended...! next step is ready to start\n");
            break;
        }
        whilecnt++;
        if(whilecnt>=600){
            printf("Wheel is not!!! ended...! time gap out\n");
            break;
        }
        sck = freeSleep(50*1000);
        if(sck==1) break;
    }

    // change pos
    freeSleep(200*1000);
    postime = 3100;
    joint->SetMoveJoint(RSP, -20, 2200, MOVE_RELATIVE);
    joint->SetMoveJoint(RF1, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWP, -70., postime, MOVE_ABSOLUTE);    

    joint->SetMoveJoint(LSP, 50.0, postime, MOVE_ABSOLUTE);//40
    joint->SetMoveJoint(LSR, -15.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LEB, -135.0, postime, MOVE_ABSOLUTE);//-130
    joint->SetMoveJoint(LWY, 60.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWP, 60.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LF1, 0.0, postime, MOVE_ABSOLUTE);
    sck = freeSleep(3100*1000);
    if(sck==1) return 4;

    postime = 2000;
    joint->SetMoveJoint(RSP, -25, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSR, -37, postime, MOVE_ABSOLUTE);
    sck = freeSleep(2010*1000);
    if(sck==1) return 4;

    // ----------------------------------------------------------------------
    // Check OK
    // ----------------------------------------------------------------------
    return 1;
    // ----------------------------------------------------------------------
    // END
    // ----------------------------------------------------------------------
}

/**************************************************************************************************
 * sub-Motion Functions
 *************************************************************************************************/
int EM_ReturnHand()
{
    joint->RefreshToCurrentReference();
    joint->SetAllMotionOwner();

    StartWBIKmotion(WBmode_RULU);
    freeSleep(10*1000);

    // Re rotate handle
    RotateDoorPushHandle(-1);
    freeSleep(100*1000);

    // Open Hand
    joint->SetMoveJoint(RF2, -125, 10, MODE_ABSOLUTE);
    freeSleep(4500*1000);
    joint->SetMoveJoint(RF2, 0, 10, MODE_ABSOLUTE);
    freeSleep(30*1000);

    // Hand back
    doubles original_quat(4);
    for(int k=0; k<4; k++)
        original_quat[k] = _EM_AP_ORI[k];

    pos_sec=2.;
    WBmotion->addRHPosInfo(_EM_AP_POS[0]-0.01, _EM_AP_POS[1], _EM_AP_POS[2], pos_sec);
    WBmotion->addRElbPosInfo(-70, pos_sec);
    WBmotion->addRHOriInfo(original_quat, pos_sec);
    freeSleep(2000*1000);

    // gain return
    MCBoardSetSwitchingMode(2, 15, 0);//RSY REB
    MCBoardSetSwitchingMode(2, 16, 0);//RWY RWP
    MCJointGainOverride(2, 15, 1, eq_gain_1000, 3000);//RSY
    MCJointGainOverride(2, 15, 2, eq_gain_1000, 3000);//REB
    MCJointGainOverride(2, 16, 1, eq_gain_1000, 3000);//RWY
    MCJointGainOverride(2, 16, 2, eq_gain_1000, 3000);//RWP
//    RBBoardSetSwitchingMode(2,15,0);
//    RBBoardSetSwitchingMode(2,16,0);
//    RBJointGainOverride(2,15,1000,1000,3000);//RSY REB
//    RBJointGainOverride(2,16,1000,1000,3000);//RWY RWP
    freeSleep(3000*1000);

    // Ready pos
    TurnOff_WBIK();
    Change_Arm_pos_Task_with_WST(_Task_WST_ANGLE, 4000);
    joint->SetMoveJoint(RF2, 125, 10, MODE_ABSOLUTE);
    freeSleep(4000*1000);
    joint->SetMoveJoint(RF2, 0, 10, MODE_ABSOLUTE);

    return 0;
}
int EM_RightHandBack()
{
    // Right Hand Open
    freeSleep(10*1000);
    FingerPositionInput_RIGHT(-17000);

    // Right Hand Back-1
    freeSleep(10*1000);
    StartWBIKmotion(WBmode_RULU);
    freeSleep(10*1000);

    vec3 cur_RH_pos = vec3(WBmotion->pRH_3x1);
    quat cur_RH_ori = quat(WBmotion->qRH_4x1);

    vec3 des_RH_pos;
    mat3 cur_RH_mat=mat3(cur_RH_ori);
    des_RH_pos = cur_RH_pos +cur_RH_mat*vec3(0.05,0,0.12);

    pos_sec=2.2;
    WBmotion->addRHPosInfo(des_RH_pos[0],des_RH_pos[1],des_RH_pos[2], pos_sec);
    WBmotion->addRElbPosInfo(-40, pos_sec);
    sck = freeSleep(2300*1000);
    if(sck==1) return 4;

    // Right Hand Back-2
    doubles ds_des_RH_ori(4);
    ds_des_RH_ori[0]=0.939693;  ds_des_RH_ori[1]=0.;    ds_des_RH_ori[2]=-0.34202;  ds_des_RH_ori[3]=0.;
    double normalize = sqrtp(ds_des_RH_ori[0]*ds_des_RH_ori[0]+ds_des_RH_ori[1]*ds_des_RH_ori[1]+ds_des_RH_ori[2]*ds_des_RH_ori[2]+ds_des_RH_ori[3]*ds_des_RH_ori[3]);
    for(int k=0; k<4; k++)
        ds_des_RH_ori[k]=ds_des_RH_ori[k]/normalize;

    pos_sec =3.2;
    WBmotion->addRHPosInfo(0.27,-0.33,0.2, pos_sec);
    WBmotion->addRHOriInfo(ds_des_RH_ori, pos_sec);
    WBmotion->addRElbPosInfo(-22,pos_sec);
    joint->SetMoveJoint(RF2, 125, 10, MODE_ABSOLUTE);
    sck = freeSleep(3300*1000);
    if(sck==1) return 4;

    //Right Hand Back-3
    MCBoardSetSwitchingMode(2, 15, 0);//RSY REB
    MCBoardSetSwitchingMode(2, 16, 0);//RWY RWP
    MCJointGainOverride(2, 15, 1, eq_gain_1000, 3000);//RSY
    MCJointGainOverride(2, 15, 2, eq_gain_1000, 3000);//REB
    MCJointGainOverride(2, 16, 1, eq_gain_1000, 3000);//RWY
    MCJointGainOverride(2, 16, 2, eq_gain_1000, 3000);//RWP
//    RBBoardSetSwitchingMode(2,15,0);
//    RBBoardSetSwitchingMode(2,16,0);
//    RBJointGainOverride(2,15,1000,1000,3000);//RSY REB
//    RBJointGainOverride(2,16,1000,1000,3000);//RWY RWP
    usleep(3000*1000);

    pos_sec = 2.;
    WBmotion->addRHPosInfo(0.16,-0.3,0.1, pos_sec);
    sck = freeSleep(2000*1000);
    if(sck==1) return 4;
    joint->SetMoveJoint(RF2, 0, 10, MODE_ABSOLUTE);
    sck = freeSleep(20*1000);
    if(sck==1) return 4;

    return 0;
}
int EM_LeftHandBack()
{
    StartWBIKmotion(WBmode_RULU);
    freeSleep(10*1000);

    doubles temp_ori(4);
    for(int k=0; k<4; k++)
        temp_ori[k] = _EM_LH_ORI[k];

    pos_sec=2.5;
    WBmotion->addLHPosInfo(_EM_LH_POS[0],_EM_LH_POS[1],_EM_LH_POS[2], pos_sec);
    WBmotion->addLHOriInfo(temp_ori, pos_sec);
    WBmotion->addLElbPosInfo(55, pos_sec);
    sck = freeSleep(2550*1000);
    if(sck==1) return 4;

    TurnOff_WBIK();

    postime = 3200;
    joint->SetMoveJoint(LSP, 40.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSR, -17.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LEB, -130.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, 10.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWP, 40.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LF1, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LF2, 0.0, postime, MOVE_ABSOLUTE);
    sck = freeSleep(3210*1000);
    if(sck==1) return 4;

    return 0;
}
int EM_RotateMore(const char rot_dir, const char fb_dir, const double w_radius, const double w_angle)
{
    int result = 0;
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[0] = rot_dir;//right  0 left
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[1] = fb_dir;//1 straight -1 back
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[0] = w_radius;//wheel radius
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[1] = w_angle;//movaing angle
    sharedData->COMMAND[PODO_NO_WHEEL].USER_COMMAND= OMNIWHEEL_AL_RADIUS;

    userData->WheelDoneFlag = false;
    whilecnt=0;
    while(1){
        if(userData->WheelDoneFlag == true){
            userData->WheelDoneFlag = false;
            printf("Wheel is ended...! next step is ready to start\n");
            break;
        }
        whilecnt++;
        if(whilecnt>=1200){
            printf("Wheel is not!!! ended...! time gap out\n");
            break;
        }
        sck = freeSleep(50*1000);
        if(sck==1){
            result = 4;
            break;
        }
    }

    return result;
}

void RotateDoorPushHandle(int rot_direction)
{
    handle_dir = rot_direction;
    handle_pos_first = pos(vec3(WBmotion->pRH_3x1),quat(WBmotion->qRH_4x1));
    handle_angle_now = 0.;
    _Trap_togo = handle_target_angle;
    _Trap_maxVel = 75;
    _Trap_maxAcc = 75;
    _isTrap = true;
    _Trap_FINISH_FLAG = false;
    _Trap_Motion_Command = TR_HANDLE_ROTATE;

    Handle_rotate_done_Flag =false;

    whilecnt =0;
    while(1){
        if(Handle_rotate_done_Flag == true){
            Handle_rotate_done_Flag = false;
            break;
        }
        whilecnt ++;
        if(whilecnt > 400)
            break;
        freeSleep(50*1000);
    }
}

void GGuGofunction(double _tar_x, double _tar_y, double _tar_a)
{
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[0] = _tar_x;
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[1] = _tar_y;
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[2] = _tar_a;
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[5] = 5;//Tank Mode
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[3] = 3;//Vel Change Mode
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[6] = 6;//direct compen
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[3] = 120;
    sharedData->COMMAND[PODO_NO_WHEEL].USER_COMMAND= OMNIWHEEL_AL_GOTODES;

    userData->WheelDoneFlag = false;
    whilecnt=0;
    while(1){
        if(userData->WheelDoneFlag == true){
            userData->WheelDoneFlag = false;
            printf("Wheel is ended...! next step is ready to start\n");
            break;
        }
        whilecnt++;
        if(whilecnt>=400){
            printf("Wheel is not!!! ended...! time gap out\n");
            break;
        }
        freeSleep(50*1000);
    }

    joint->RefreshToCurrentReference();
    joint->SetAllMotionOwner();

    double checkangle = joint->GetJointRefAngle(RWP);
    double checkerror = fabs(checkangle - (-70));
    if(checkerror < 1.){// if it is first ggu position
        Change_Arm_pos_Pass(2700);
        freeSleep(2700*1000);
    }else{
        joint->SetMoveJoint(WST, 0.0, 1500, MOVE_ABSOLUTE);
        freeSleep(1510*1000);
    }

    sharedData->STATE_COMMAND = TCMD_DOOR_CROSS;
}


// --------------------------------------------------------------------------------------------- //
// --------------------------------------------------------------------------------------------- //
/**************************************************************************************************
 * Hose Motion Function
 *************************************************************************************************/
void PreDefinedMovement(double dx, double dy, double da){
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[0]=dx;
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[1]=dy;
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[2]=da;
    sharedData->COMMAND[PODO_NO_WHEEL].USER_COMMAND= OMNIWHEEL_AL_GOTODES;
    userData->WheelDoneFlag = false;

    whilecnt=0;
    while(1){
        if(userData->WheelDoneFlag == true){
            userData->WheelDoneFlag = false;
            break;
        }
        whilecnt++;
        if(whilecnt>=1000){
            break;
        }
        freeSleep(50*1000);
    };
    usleep(50*1000);
}

int HOSE_GRAB_HOSE(void){
    double ap_angle =fabs(45.);
    StartWBIKmotion(WBmode_RULU);
    freeSleep(10*1000);
    vec3 cur_H_pos;
    vec3 des_H_pos_1, des_H_pos_2, des_H_pos_3;
    quat des_H_ori;

    int selected_hand = sharedData->COMMAND[PODO_NO].USER_PARA_INT[0]; // 1 is left -1 is right

    if(selected_hand ==1)//left
        cur_H_pos = vec3(WBmotion->pLH_3x1);
    else//right
        cur_H_pos = vec3(WBmotion->pRH_3x1);

    for(int k=0; k<3; k++)
        des_H_pos_3[k] = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[k];

    des_H_pos_3 = des_H_pos_3 + vec3(-0.005*cos(ap_angle*D2R), 0.005*((double)selected_hand)*sin(ap_angle*D2R), -0.025);
    des_H_pos_2 = des_H_pos_3 + vec3(-0.15*cos(ap_angle*D2R), 0.15*((double)selected_hand)*sin(ap_angle*D2R), -0.03);
    for(int k=0; k<3; k++)
        _EM_HOSE_RETURN_GRAB_POS[k] = des_H_pos_2[k];

    des_H_ori = quat(vec3(0,1,0), -90*D2R) * quat(vec3(0,0,1), 90*((double)selected_hand)*D2R)* quat(vec3(0,1,0), ap_angle*D2R);

    doubles ds_des_H_ori;
    for(int k=0; k<4; k++)
        ds_des_H_ori[k] = des_H_ori[k];

    if(des_H_pos_2[0]<0.3)
        des_H_pos_2[0] =0.3;

    // Move Hand

    pos_sec =2.5;
    if(selected_hand ==1){//left
        WBmotion->addLHPosInfo(des_H_pos_2[0], des_H_pos_2[1], des_H_pos_2[2], pos_sec);
        WBmotion->addLHOriInfo(ds_des_H_ori, pos_sec);
        WBmotion->addLElbPosInfo(40, pos_sec);
        joint->SetMoveJoint(LF2, -125, 5, MODE_ABSOLUTE);

        sck = freeSleep(2500*1000);
        if(sck==1) return 4;
        sck = freeSleep(2000*1000);//because of hand open
        if(sck==1) return 4;
        WBmotion->addLHPosInfo(des_H_pos_3[0], des_H_pos_3[1], des_H_pos_3[2], pos_sec);
        joint->SetMoveJoint(LF2, 0., 5, MODE_ABSOLUTE);

        sck = freeSleep(2500*1000);
        if(sck==1) return 4;
    }else{//right
        WBmotion->addRHPosInfo(des_H_pos_2[0], des_H_pos_2[1], des_H_pos_2[2], pos_sec);
        WBmotion->addRHOriInfo(ds_des_H_ori, pos_sec);
        WBmotion->addRElbPosInfo(-40, pos_sec);
        joint->SetMoveJoint(RF2, -125, 5, MODE_ABSOLUTE);

        sck = freeSleep(2500*1000);
        if(sck==1) return 4;
        sck = freeSleep(2000*1000);//because of hand open
        if(sck==1) return 4;
        WBmotion->addRHPosInfo(des_H_pos_3[0], des_H_pos_3[1], des_H_pos_3[2], pos_sec);
        joint->SetMoveJoint(RF2, 0., 5, MODE_ABSOLUTE);

        sck = freeSleep(2500*1000);
        if(sck==1) return 4;
    }

    // Grab hand
    if(selected_hand == 1){
        joint->SetMoveJoint(LF2, 125, 5, MODE_ABSOLUTE);
        usleep(4500*1000);
        joint->SetMoveJoint(LF2, 0, 5, MODE_ABSOLUTE);
    }else{
        joint->SetMoveJoint(RF2, 125, 5, MODE_ABSOLUTE);
        usleep(4500*1000);
        joint->SetMoveJoint(RF2, 0, 5, MODE_ABSOLUTE);
    }

    // Check
    double finger_enc = finger_scale*(sharedData->ENCODER[MC_ID_CH_Pairs[LF2].id][MC_ID_CH_Pairs[LF2].ch].CurrentPosition);
    if(finger_enc < -3500)
        return 1;
    else
        return 0;
}
int HOSE_AFTER_GRAB(const double des_rotate_angle){

    int selected_hand = sharedData->COMMAND[PODO_NO].USER_PARA_INT[0]; // 1 is left -1 is right
    TurnOff_WBIK();
    postime = 2500;
    if(selected_hand == 1){//left
        joint->SetMoveJoint(LSP, -10.0, postime, MOVE_ABSOLUTE);
        joint->SetMoveJoint(LSR, -2.0, postime, MOVE_ABSOLUTE);
        joint->SetMoveJoint(LSY, -20.0, postime, MOVE_ABSOLUTE);
        joint->SetMoveJoint(LEB, -120.0, postime, MOVE_ABSOLUTE);
        joint->SetMoveJoint(LWY, 20.0, postime, MOVE_ABSOLUTE);
        joint->SetMoveJoint(LWP, 70.0, postime, MOVE_ABSOLUTE);
        joint->SetMoveJoint(LF1, 50.0, postime, MOVE_ABSOLUTE);
    }else{
        joint->SetMoveJoint(RSP, -10.0, postime, MOVE_ABSOLUTE);
        joint->SetMoveJoint(RSR, 2.0, postime, MOVE_ABSOLUTE);
        joint->SetMoveJoint(RSY, 20.0, postime, MOVE_ABSOLUTE);
        joint->SetMoveJoint(REB, -120.0, postime, MOVE_ABSOLUTE);
        joint->SetMoveJoint(RWY, -20.0, postime, MOVE_ABSOLUTE);
        joint->SetMoveJoint(RWP, 70.0, postime, MOVE_ABSOLUTE);
        joint->SetMoveJoint(RF1, -50.0, postime, MOVE_ABSOLUTE);
    }

    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[4] = 4;// No Wait Flag
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[0]=-0.2;//-0.5
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[1]=0.0;
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[2]=des_rotate_angle;
    sharedData->COMMAND[PODO_NO_WHEEL].USER_COMMAND= OMNIWHEEL_AL_GOTODES;
    userData->WheelDoneFlag = false;

    whilecnt=0;
    while(1){
        if(userData->WheelDoneFlag == true){
            userData->WheelDoneFlag = false;
            printf("Wheel is ended...! next step is ready to start\n");
            break;
        }
        whilecnt++;
        if(whilecnt>=500){
            printf("Wheel is not!!! ended...! time gap out\n");
            break;
        }
        freeSleep(50*1000);
    }
    int timeconsum = whilecnt*50;
    if(timeconsum>postime){
        ;
    }else{
        usleep((postime-timeconsum+50)*1000);
    }
    usleep(50*1000);

    // Check
    double finger_enc = finger_scale*(sharedData->ENCODER[MC_ID_CH_Pairs[LF2].id][MC_ID_CH_Pairs[LF2].ch].CurrentPosition);
    if(finger_enc < -3500)
        return 1;
    else
        return 0;
}
int HOSE_PLUG_WYE(void){
    vec3 des_H_pos_1, des_H_pos_2, des_H_pos_3;
    vec3 cur_vecX, cur_vecY, cur_vecZ;

    for(int k=0; k<3; k++){
        des_H_pos_3[k] = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[k];
        cur_vecX[k] = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[k+3];
    }
    //des_H_pos_3[1] += 0.25;
    int selected_hand = sharedData->COMMAND[PODO_NO].USER_PARA_INT[0]; // 1 is left -1 is right
    cur_vecX[2]=0.;

    double temp = sqrtp(cur_vecX[0]*cur_vecX[0] + cur_vecX[1]*cur_vecX[1] + cur_vecX[2]*cur_vecX[2]);

    if(cur_vecX[0]<0)
        for(int k=0; k<3; k++)
            cur_vecX[k] = -cur_vecX[k];
    for(int k=0; k<3; k++)
        cur_vecX[k] = cur_vecX[k] / temp;

    cur_vecZ = vec3(0,0,1);
    cur_vecY = cross(cur_vecZ, cur_vecX);

    mat3 rot_mat;
    rot_mat[0][0] = cur_vecX[0];        rot_mat[0][1] = cur_vecY[0];        rot_mat[0][2] = cur_vecZ[0];
    rot_mat[1][0] = cur_vecX[1];        rot_mat[1][1] = cur_vecY[1];        rot_mat[1][2] = cur_vecZ[1];
    rot_mat[2][0] = cur_vecX[2];        rot_mat[2][1] = cur_vecY[2];        rot_mat[2][2] = cur_vecZ[2];

    quat des_ori = quat(rot_mat)*quat(vec3(0, 1, 0),-90*D2R)*quat(vec3(1,0,0),-90*((double)selected_hand)*D2R);
    doubles ds_des_ori;
    for(int k=0; k<4; k++)
        ds_des_ori[k] = des_ori[k];

    double hose_radius = 0.0265;
    des_H_pos_3 = des_H_pos_3 + rot_mat*vec3(-0.06, (((double)selected_hand)*hose_radius), 0.003);
    des_H_pos_2 = des_H_pos_3 + rot_mat*vec3(-0.08, 0, 0);

    for(int k=0; k<3; k++)
        _EM_WYE_RETURN_POS[k] = des_H_pos_2[k];
    for(int k=0; k<4; k++)
        _EM_WYE_RETURN_ORI[k] = des_ori[k];

    StartWBIKmotion(WBmode_RULP);
    freeSleep(10*1000);

    pos_sec =3.;
    if(selected_hand == 1){//left
        WBmotion->addLHPosInfo(des_H_pos_2[0], des_H_pos_2[1], des_H_pos_2[2], pos_sec);
        WBmotion->addLHOriInfo(ds_des_ori, pos_sec);
        WBmotion->addLElbPosInfo(45, pos_sec);//25
        WBmotion->addWSTPosInfo(-15, pos_sec);
        sck = freeSleep(3010*1000);
        if(sck==1) return 4;



        // FT sensor null
        MCWristFTsensorNull(3,54);
        //RBwFTsensorNull(3, 54);
        sck = freeSleep(600*1000);
        if(sck==1) return 4;

        FTavgFlag = true;
        freeSleep(1010*1000);
        befor_FTavg = FTavg_2;
        freeSleep(50*1000);
        printf("Befor insert : %lf \n", befor_FTavg);

        WBmotion->addLHPosInfo(des_H_pos_3[0], des_H_pos_3[1], des_H_pos_3[2], pos_sec);
        WBmotion->addLElbPosInfo(60, pos_sec);//45
        sck = freeSleep(3010*1000);
        if(sck==1) return 4;

        MCBoardSetSwitchingMode(3, 20, 1);//LWY LWP
        MCJointGainOverride(3, 20, 1, eq_gain_1, 400);//LWY
        MCJointGainOverride(3, 20, 2, eq_gain_1, 400);//LWP
//        RBBoardSetSwitchingMode(3,20,1);
//        RBJointGainOverride(3,20,1,1,400);//LWY LWP
        usleep(400*1000);

        vec3 original_H = vec3(WBmotion->pLH_3x1);
        quat original_Q = quat(WBmotion->qLH_4x1);
        int yz_flag = false;
        double zang = 0.;
        double yang = 0.;
        double pur_ang = 0.9;
        for(int k=0; k<5; k++){//depend
            //  X dir
            for(int j=0; j<7; j++){
                // Y-Z plane
                double delta_x = 0.0055*k;
                double delta_y, delta_z;
                if(j==6){
                    delta_y = 0.;
                    delta_z = 0.;
                    yang = 0.;
                    zang = 0.;
                }else{
                    delta_y = 0.014*cos(j*60*D2R);
                    delta_z = 0.014*sin(j*60*D2R);
                    if(delta_y >0.)
                        yang = -pur_ang;
                    else
                        yang = pur_ang;
                    if(delta_z >0.)
                        zang =pur_ang;
                    else
                        zang =-pur_ang;
                }

                vec3 delta_H = vec3(delta_x, delta_y, delta_z);
                quat delta_Q = quat(vec3(0, 0, 1), zang*D2R)*quat(vec3(0, 1, 0), yang*D2R);
                vec3 current_tart_H = original_H + delta_H;
                quat current_tar_ori_H = delta_Q*original_Q;
                doubles ds_current_tar_ori_H;
                for(int i=0; i<4; i++)
                    ds_current_tar_ori_H[i] = current_tar_ori_H[i];
                WBmotion->addLHPosInfo(current_tart_H[0], current_tart_H[1], current_tart_H[2], 0.56);
                WBmotion->addLHOriInfo(ds_current_tar_ori_H, 0.56);
                usleep(570*1000);
                vec3 LWFT;
                LWFT[0] = sharedData->FT[3].Fx;
                LWFT[1] = sharedData->FT[3].Fy;
                LWFT[2] = sharedData->FT[3].Fz;
                //printf("CUR delta : %lf %lf %lf FT: %lf %lf %lf\n", delta_x, delta_y, delta_z, LWFT[0],LWFT[1], LWFT[2]);
                if(fabs(LWFT[1]) > 30.){
                    printf("Cur searching index : %d %d\n", k, j);
                    yz_flag = true;
                    break;
                }
            }
            if(yz_flag == true)
                break;
        }
    }else{
        //right
    }

    return 1;
}
int HOSE_RETURN_FROM_WYE(void){
    StartWBIKmotion(WBmode_RULP);
    freeSleep(10*1000);

    // get data
    vec3 cur_H_pos;
    vec3 des_H_pos_1, des_H_pos_2, des_H_pos_3;
    vec3 cur_vecX, cur_vecY, cur_vecZ;

    for(int k=0; k<3; k++)
        cur_vecX[k] = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[k+3];

    int selected_hand = sharedData->COMMAND[PODO_NO].USER_PARA_INT[0]; // 1 is left -1 is right
    cur_vecX[2]=0.;

    double temp = sqrtp(cur_vecX[0]*cur_vecX[0] + cur_vecX[1]*cur_vecX[1] + cur_vecX[2]*cur_vecX[2]);

    if(cur_vecX[0]<0)
        for(int k=0; k<3; k++)
            cur_vecX[k] = -cur_vecX[k];
    for(int k=0; k<3; k++)
        cur_vecX[k] = cur_vecX[k] / temp;

    cur_vecZ = vec3(0,0,1);
    cur_vecY = cross(cur_vecZ, cur_vecX);

    mat3 rot_mat;
    rot_mat[0][0] = cur_vecX[0];        rot_mat[0][1] = cur_vecY[0];        rot_mat[0][2] = cur_vecZ[0];
    rot_mat[1][0] = cur_vecX[1];        rot_mat[1][1] = cur_vecY[1];        rot_mat[1][2] = cur_vecZ[1];
    rot_mat[2][0] = cur_vecX[2];        rot_mat[2][1] = cur_vecY[2];        rot_mat[2][2] = cur_vecZ[2];

    // open hand
    if(selected_hand == 1){
        cur_H_pos = vec3(WBmotion->pLH_3x1);
        joint->SetMoveJoint(LF2, -125, 10, MODE_ABSOLUTE);
        freeSleep(5000*1000);
        joint->SetMoveJoint(LF2, 0, 10, MODE_ABSOLUTE);
        freeSleep(20*1000);
    }else{
        cur_H_pos = vec3(WBmotion->pRH_3x1);
        joint->SetMoveJoint(RF2, -125, 10, MODE_ABSOLUTE);
        freeSleep(5000*1000);
        joint->SetMoveJoint(RF2, 0, 10, MODE_ABSOLUTE);
        freeSleep(20*1000);
    }

    MCBoardSetSwitchingMode(3, 20, 0);//LWY LWP
    MCJointGainOverride(3, 20, 1, eq_gain_1000, 400);//LWY
    MCJointGainOverride(3, 20, 2, eq_gain_1000, 400);//LWP
//    RBBoardSetSwitchingMode(3,20,0);
//    RBJointGainOverride(3,20,1000,1000,400);//LWY LWP
    usleep(400*1000);

    // go back pos
    des_H_pos_1 = cur_H_pos + rot_mat*vec3(-0.05, (((double)selected_hand)*0.18), 0.);
    des_H_pos_2 = des_H_pos_1 + rot_mat*vec3(-0.01, 0, -0.4);
    if(des_H_pos_2[0]<0.3)
        des_H_pos_2[0] =0.3;

    pos_sec =2.5;
    postime = 2500.;
    if(selected_hand == 1){
        WBmotion->addLHPosInfo(des_H_pos_1[0], des_H_pos_1[1], des_H_pos_1[2], pos_sec);
        WBmotion->addLElbPosInfo(60, pos_sec);
        sck = freeSleep(2510*1000);
        if(sck==1) return 4;

        WBmotion->addLHPosInfo(des_H_pos_2[0], des_H_pos_2[1], des_H_pos_2[2], pos_sec);
        sck = freeSleep(2510*1000);
        if(sck==1) return 4;

        joint->SetMoveJoint(LF2, 125, 10, MODE_ABSOLUTE);
        freeSleep(5000*1000);
        joint->SetMoveJoint(LF2, 0, 10, MODE_ABSOLUTE);
        freeSleep(20*1000);

        TurnOff_WBIK();

        postime = 2500;
        joint->SetMoveJoint(LWY, -100.0, postime, MOVE_RELATIVE);
        sck = freeSleep(2510*1000);
        if(sck==1) return 4;

        sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[0] = -0.3;
        sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[1] = 0.;
        sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[2] = 0.;
        sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[5] = 5;//Tank Mode
        sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[3] = 3;//Vel Change Mode
        sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[6] = 6;//direct compen
        sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[3] = 120;
        sharedData->COMMAND[PODO_NO_WHEEL].USER_COMMAND= OMNIWHEEL_AL_GOTODES;

        userData->WheelDoneFlag = false;
        whilecnt=0;
        while(1){
            if(userData->WheelDoneFlag == true){
                userData->WheelDoneFlag = false;
                printf("Wheel is ended...! next step is ready to start\n");
                break;
            }
            whilecnt++;
            if(whilecnt>=400){
                printf("Wheel is not!!! ended...! time gap out\n");
                break;
            }
            freeSleep(50*1000);
        }

        joint->RefreshToCurrentReference();
        joint->SetAllMotionOwner();

        Change_Arm_pos_Move(3500);
        usleep(3500*1000);

    }else{
        //right
    }

    return 1;
}
int EM_ReturnHand_From_HOSEGRAB(void){
    vec3 back_pos;
    for(int k=0; k<3; k++)
        back_pos[k] = _EM_HOSE_RETURN_GRAB_POS[k];

    TurnOff_WBIK();
    joint->SetMoveJoint(LF2, -125, 5, MODE_ABSOLUTE);
    usleep(4500*1000);
    joint->SetMoveJoint(LF2, 0, 5, MODE_ABSOLUTE);
    usleep(10*1000);

    StartWBIKmotion(WBmode_RULU);
    freeSleep(10*1000);

    pos_sec = 3;
    WBmotion->addLHPosInfo(back_pos[0], back_pos[1], back_pos[2], pos_sec);
    usleep(3010*1000);

    TurnOff_WBIK();

    postime = 4000;
    joint->SetMoveJoint(RSP, 40.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSR, 17.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(REB, -130, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWY, -10.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWP, 40.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LSP, 40.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSR, -17.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LEB, -130.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, 10.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWP, 40.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(WST, 0.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(RF1, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LF1, 0.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LF2, 125, 5, MODE_ABSOLUTE);
    usleep(4010*1000);
    joint->SetMoveJoint(LF2, 0, 5, MODE_ABSOLUTE);
    usleep(10*1000);

    return 1;
}
int EM_ReturnHand_From_WYE(void){
    int selected_hand = sharedData->COMMAND[PODO_NO].USER_PARA_INT[0]; // 1 is left -1 is right
    doubles em_back_temp(4);
    for(int k =0; k<4; k++)
        em_back_temp[k] = _EM_WYE_RETURN_ORI[k];

    usleep(10*1000);
    StartWBIKmotion(WBmode_RULP);
    freeSleep(10*1000);

    pos_sec = 3.;
    WBmotion->addLHPosInfo(_EM_WYE_RETURN_POS[0], _EM_WYE_RETURN_POS[1], _EM_WYE_RETURN_POS[2], pos_sec);
    WBmotion->addLHOriInfo(em_back_temp, pos_sec);
    WBmotion->addLElbPosInfo(45, pos_sec);//25

    usleep(3010*1000);

    TurnOff_WBIK();
    postime = 2500;
    if(selected_hand == 1){//left
        joint->SetMoveJoint(LSP, -10.0, postime, MOVE_ABSOLUTE);
        joint->SetMoveJoint(LSR, -2.0, postime, MOVE_ABSOLUTE);
        joint->SetMoveJoint(LSY, -20.0, postime, MOVE_ABSOLUTE);
        joint->SetMoveJoint(LEB, -120.0, postime, MOVE_ABSOLUTE);
        joint->SetMoveJoint(LWY, 20.0, postime, MOVE_ABSOLUTE);
        joint->SetMoveJoint(LWP, 70.0, postime, MOVE_ABSOLUTE);
        joint->SetMoveJoint(LF1, 50.0, postime, MOVE_ABSOLUTE);

        joint->SetMoveJoint(WST, 0.0, postime, MOVE_ABSOLUTE);
    }else{
        joint->SetMoveJoint(RSP, -10.0, postime, MOVE_ABSOLUTE);
        joint->SetMoveJoint(RSR, 2.0, postime, MOVE_ABSOLUTE);
        joint->SetMoveJoint(RSY, 20.0, postime, MOVE_ABSOLUTE);
        joint->SetMoveJoint(REB, -120.0, postime, MOVE_ABSOLUTE);
        joint->SetMoveJoint(RWY, -20.0, postime, MOVE_ABSOLUTE);
        joint->SetMoveJoint(RWP, 70.0, postime, MOVE_ABSOLUTE);
        joint->SetMoveJoint(RF1, -50.0, postime, MOVE_ABSOLUTE);

        joint->SetMoveJoint(WST, 0.0, postime, MOVE_ABSOLUTE);
    }
    usleep(2500*1000);

    return 1;
}
// --------------------------------------------------------------------------------------------- //
// --------------------------------------------------------------------------------------------- //
/**************************************************************************************************
 * Door In-Push Motion Function
 *************************************************************************************************/
int PushDoor_GrabAndOpen_LEFT(int _mode){

    sharedData->STATE_COMMAND = TCMD_PUSHDOOR_MOTION_1_START;

    // Grab handle - rotate handle
    // ----------------------------------------------------------------------
    // Data Get
    // ----------------------------------------------------------------------
    vec3 cur_vecX, cur_vecY,  cur_vecZ;
    vec3 pelv_nobe;

    double des_x, des_y, des_z;
    double des_x2, des_y2, des_z2;
    doubles des_LH_ori(4);

    unsigned char data_ok_flag = true;
    unsigned char norm_ok_flag = true;
    pelv_nobe[0] = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
    pelv_nobe[1] = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
    pelv_nobe[2] = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];
    cur_vecX[0] = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[3];
    cur_vecX[1] = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[4];
    cur_vecX[2] = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[5];

    if((pelv_nobe[0] < 0.25) || (pelv_nobe[1]<0.1)){
        data_ok_flag = false;
    }
    if(fabs(cur_vecX[0])<0.85){
        norm_ok_flag = false;
    }
    if((data_ok_flag==false) || (norm_ok_flag==false)){
        return 4;
    }
    if(cur_vecX[0]<0){
        cur_vecX = -cur_vecX;
    }
    for(int k=0; k<3; k++)
        saved_normal[k] = cur_vecX[k];

    cur_vecZ[0]=0.;
    cur_vecZ[1]=0.;
    cur_vecZ[2]=1.;
    cur_vecY=cross(cur_vecZ, cur_vecX);

    float door_hand_ap_margin[3];
    door_hand_ap_margin[0]=0.15;
    door_hand_ap_margin[1]=-LEFT_NOBE_PUSH_Y;//depend push door case
    door_hand_ap_margin[2]=0.;
    des_x = pelv_nobe[0]-door_hand_ap_margin[0]*cur_vecX[0]-door_hand_ap_margin[1]*cur_vecY[0]-door_hand_ap_margin[2]*cur_vecZ[0];
    des_y = pelv_nobe[1]-door_hand_ap_margin[0]*cur_vecX[1]-door_hand_ap_margin[1]*cur_vecY[1]-door_hand_ap_margin[2]*cur_vecZ[1];
    des_z = pelv_nobe[2]-door_hand_ap_margin[0]*cur_vecX[2]-door_hand_ap_margin[1]*cur_vecY[2]-door_hand_ap_margin[2]*cur_vecZ[2];
    // Insert
    door_hand_ap_margin[0]= LEFT_NOBE_PUSH_X;//depend
    door_hand_ap_margin[1]=-LEFT_NOBE_PUSH_Y;
    door_hand_ap_margin[2]=0.;
    des_x2 = pelv_nobe[0]-door_hand_ap_margin[0]*cur_vecX[0]-door_hand_ap_margin[1]*cur_vecY[0]-door_hand_ap_margin[2]*cur_vecZ[0];
    des_y2 = pelv_nobe[1]-door_hand_ap_margin[0]*cur_vecX[1]-door_hand_ap_margin[1]*cur_vecY[1]-door_hand_ap_margin[2]*cur_vecZ[1];
    des_z2 = pelv_nobe[2]-door_hand_ap_margin[0]*cur_vecX[2]-door_hand_ap_margin[1]*cur_vecY[2]-door_hand_ap_margin[2]*cur_vecZ[2];

    mat3 _wall_rotaion_matrix;
    _wall_rotaion_matrix[0][0]=cur_vecX[0];  _wall_rotaion_matrix[0][1]=cur_vecY[0];  _wall_rotaion_matrix[0][2]=cur_vecZ[0];
    _wall_rotaion_matrix[1][0]=cur_vecX[1];  _wall_rotaion_matrix[1][1]=cur_vecY[1];  _wall_rotaion_matrix[1][2]=cur_vecZ[1];
    _wall_rotaion_matrix[2][0]=cur_vecX[2];  _wall_rotaion_matrix[2][1]=cur_vecY[2];  _wall_rotaion_matrix[2][2]=cur_vecZ[2];

    quat q_wall = quat(_wall_rotaion_matrix);
    quat q_temp = quat(vec3(0,1,0),-90*D2R);
    quat q_des_LH=q_wall*q_temp;

    for(int k=0;k<4;k++){
        des_LH_ori[k] = q_des_LH[k];
        _EM_PUSHDOOR_AP_ORI[k] = q_des_LH[k];
    }
    _EM_PUSHDOOR_AP_POS[0] = des_x;
    _EM_PUSHDOOR_AP_POS[1] = des_y;
    _EM_PUSHDOOR_AP_POS[2] = des_z;
    // ----------------------------------------------------------------------
    // Goto Approach point
    // ----------------------------------------------------------------------
    StartWBIKmotion(WBmode_RULU);
    freeSleep(10*1000);

    vec3 cur_RH_pos = vec3(WBmotion->pRH_3x1);//
    vec3 des_RH_pos_1 = cur_RH_pos + vec3(0.12, 0., 0.);//
    des_RH_pos_1[2] = 0.43;//

    pos_sec=2.5;
    WBmotion->addLHPosInfo(des_x, des_y, des_z, pos_sec);
    WBmotion->addLElbPosInfo(20, pos_sec);
    WBmotion->addLHOriInfo(des_LH_ori, pos_sec);

    WBmotion->addRHPosInfo(des_RH_pos_1[0], des_RH_pos_1[1], des_RH_pos_1[2], pos_sec);//
    WBmotion->addRElbPosInfo(-15, pos_sec);//

    int tc = FingerPositionInput_LEFT(-14000);

    if(tc>=2500)
        freeSleep(20*1000);
    else
        freeSleep((2500-tc+20)*1000);
    // ----------------------------------------------------------------------
    // FT sensor null
    // ----------------------------------------------------------------------
//    MCWristFTsensorNull(3, 54);
//    //RBwFTsensorNull(3, 54);
//    sck = freeSleep(600*1000);
//    if(sck==1) return 4;

    // ----------------------------------------------------------------------
    // Insert Hand
    // ----------------------------------------------------------------------
    vec3 des_RH_pos_2 = des_RH_pos_1;
    des_RH_pos_2[0] = 0.49;
    des_RH_pos_2[1] = 0.;
    doubles ds_des_RH_ori(4);
    quat q_temp1 = quat(vec3(0,1,0), -90*D2R);
    quat q_temp2 = quat(vec3(0,0,1), 90*D2R);
    quat des_RH_ori = q_temp1*q_temp2;
    for(int k=0; k<4; k++)
        ds_des_RH_ori[k] = des_RH_ori[k];

    pos_sec = 1.8;
    WBmotion->addLHPosInfo(des_x2, des_y2, des_z2, pos_sec);
    WBmotion->addLElbPosInfo(15, pos_sec);//unlv

    WBmotion->addRHPosInfo(des_RH_pos_2[0], des_RH_pos_2[1], des_RH_pos_2[2], pos_sec);
    WBmotion->addRHOriInfo(ds_des_RH_ori, pos_sec);
    WBmotion->addRElbPosInfo(-20, pos_sec);

    freeSleep(1810*1000);
    if(sck==1) return 4;

    // ----------------------------------------------------------------------
    // Gain Over and Grap hnad
    // ----------------------------------------------------------------------
    MCBoardSetSwitchingMode(3, 19, 1);//LSY LEB
    MCBoardSetSwitchingMode(3, 20, 1);//LWY LWP
    MCJointGainOverride(3, 19, 1, eq_gain_1, 300);//LSY
    MCJointGainOverride(3, 19, 2, eq_gain_1, 300);//LEB
    MCJointGainOverride(3, 20, 1, eq_gain_0, 300);//LWY
    MCJointGainOverride(3, 20, 2, eq_gain_1, 300);//LWP
    sck = freeSleep(300*1000);
    if(sck==1) return 4;

    joint->SetMoveJoint(LF2, 125, 10, MODE_ABSOLUTE);
    sck = freeSleep(2700*1000);
    if(sck==1) return 4;
    joint->SetMoveJoint(LF2, 0, 10, MODE_ABSOLUTE);
    pos_sec = 0.8;
    WBmotion->addLHPosInfo((des_x2+0.05), des_y2, des_z2, pos_sec);
    sck = freeSleep(600*1000);
    if(sck==1) return 4;
    joint->SetMoveJoint(LF2, 125, 10, MODE_ABSOLUTE);
    sck = freeSleep(2000*1000);//3800
    if(sck==1) return 4;
    //joint->SetMoveJoint(RF2, 0, 10, MODE_ABSOLUTE);
    //usleep(10*1000);
    WBmotion->addLHPosInfo((des_x2+0.02), des_y2, des_z2, 0.6);
    freeSleep(620*1000);

    GetPosFromENC();
    freeSleep(20*1000);
    vec3 befor_rot = vec3(WBmotion->enc_pLH_3x1);

//    FTavgFlag = true;
//    freeSleep(1010*1000);
//    befor_FTavg = FTavg_2;
//    freeSleep(50*1000);
    // ----------------------------------------------------------------------
    // Rotate Handle
    // ----------------------------------------------------------------------
    StartWBIKmotion(WBmode_RULU);
    freeSleep(10*1000);

    for(int k=0; k<3; k++){
        handle_rotate_center[k] = pelv_nobe[k] + cur_vecY[k]*(DOOR_NOBE_LENGTH);//depend
        hyo_debug[k] = handle_rotate_center[k];
        handle_rotate_axis[k] = cur_vecX[k];
    }
    handle_target_angle = LEFT_NOBE_ROT_ANGLE;

    PushDoor_RotateHandle_LEFT(1);

    // ----------------------------------------------------------------------
    // Check OK
    // ----------------------------------------------------------------------
    GetPosFromENC();
    freeSleep(20*1000);
    vec3 after_rot = vec3(WBmotion->enc_pLH_3x1);

//    FTavgFlag = true;
//    freeSleep(1010*1000);
//    after_FTavg = FTavg_2;

    vec3 hand_pos_err = after_rot - befor_rot;
//    printf("Motion Checker 1::Hand pos change:: %lf .....!!!\n", hand_pos_err[0]);
    double FTratio = befor_FTavg / after_FTavg;
//    printf("Motion Checker 1::FT sensor change:: %lf %lf %lf\n", befor_FTavg, after_FTavg, FTratio);

    int ENCODER_TEST = true;
    int FTSENSOR_TEST = true;

    if(hand_pos_err[0] > 0.011)
        ENCODER_TEST = true;
    if(fabs(after_FTavg)>40)
        FTSENSOR_TEST = true;

    if((ENCODER_TEST==true) && (FTSENSOR_TEST==true)){
        sharedData->STATE_COMMAND = TCMD_PUSHDOOR_MOTION_1_END;
        return 1;//success
    }else{
        sharedData->STATE_COMMAND = TCMD_PUSHDOOR_MOTION_1_FAIL;
        return 0;//fail        
    }
    // ----------------------------------------------------------------------
    // END
    // ----------------------------------------------------------------------
}

int PushDoor_LHblock_RHback_Gothrough_LEFT(int _mode){
    if(_mode ==1){
        sharedData->STATE_COMMAND = TCMD_PUSHDOOR_MOTION_2_EM_2_START;
        // Pass
    }else{
        sharedData->STATE_COMMAND = TCMD_PUSHDOOR_MOTION_2_START;
        // ----------------------------------------------------------------------
        // Rotate Door to Open
        // ---------------------------------------------------------------------
        StartWBIKmotion(WBmode_RULU);
        freeSleep(10*1000);

        vec3 temp_RH = vec3(WBmotion->pRH_3x1);

        handle_rotate_center[0] = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[6];
        handle_rotate_center[1] = -sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[7];
        handle_rotate_center[2] = WBmotion->pLH_3x1[2];
        handle_rotate_axis[0] = 0.;
        handle_rotate_axis[1] = 0.;
        handle_rotate_axis[2] = 1.;
        handle_target_angle = door_open_angle;
        handle_dir = -1; //

        handle_pos_first = pos(vec3(WBmotion->pLH_3x1),quat(WBmotion->qLH_4x1));
        handle_angle_now = 0.;
        _Trap_togo = handle_target_angle;
        _Trap_maxVel = 10;
        _Trap_maxAcc = 5;
        _isTrap = true;
        _Trap_FINISH_FLAG = false;
        _Trap_Motion_Command = TR_HANDLE_ROTATE;

        pos_sec=1.5;
        WBmotion->addRHPosInfo((temp_RH[0]+0.08), temp_RH[1], temp_RH[2], pos_sec);
        WBmotion->addRElbPosInfo(-25, pos_sec);
        double mot_time = pos_sec*1000;

        whilecnt=0;
        while(1){
            if(_Trap_FINISH_FLAG == true)
                break;
            whilecnt++;
            if(whilecnt > 300)
                break;
            freeSleep(50*1000);
        }
        double timeelapse = whilecnt*50;
        if(timeelapse > mot_time)
            usleep(5*1000);
        else if(timeelapse <= mot_time){
            double diff_time = mot_time - timeelapse+5;
            usleep(diff_time*1000);
        }
        // ----------------------------------------------------------------------
        // Left Hand Blocking
        // ----------------------------------------------------------------------
        double temp_c = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[6];
        if((temp_c < 0.6) || (temp_c > 0.85)){
            temp_c = 0.7208;
        }

        GetPosFromENC();
        freeSleep(10*1000);

        vec3 cur_LH_pos = vec3(WBmotion->enc_pLH_3x1);
        vec3 cur_RH_pos = vec3(WBmotion->pRH_3x1);

        vec3 des_RH_pos;

        des_RH_pos[0] = (temp_c + cur_LH_pos[0])/2. - 0.03;
        des_RH_pos[1] = 0.;
        des_RH_pos[2] = cur_RH_pos[2];

        pos_sec=1.8;
        WBmotion->addRHPosInfo(des_RH_pos[0], des_RH_pos[1], des_RH_pos[2], pos_sec);
        WBmotion->addRElbPosInfo(-30, pos_sec);
        sck = freeSleep(1810*1000);
        if(sck==1) return 4;

        // hyoin test
        joint->SetMoveJoint(LF2, 0, 10, MODE_ABSOLUTE);
        usleep(20*1000);

        // ----------------------------------------------------------------------
        // Right Hand Back
        // ----------------------------------------------------------------------

        vec3 ori_normal = vec3(saved_normal);
        mat3 rotmatrix(vec3(0,0,1),door_open_angle*D2R);
        vec3 new_normal = rotmatrix*ori_normal;

        for(int k=0; k<3; k++){
            handle_rotate_center[k] = hyo_debug[k];
            handle_rotate_axis[k] = new_normal[k];
        }
        handle_dir = -1; //

        handle_pos_first = pos(vec3(WBmotion->pLH_3x1),quat(WBmotion->qLH_4x1));
        handle_angle_now = 0.;
        _Trap_togo = LEFT_NOBE_ROT_ANGLE;
        _Trap_maxVel = 50;
        _Trap_maxAcc = 50;
        _isTrap = true;
        _Trap_FINISH_FLAG =false;
        _Trap_Motion_Command = TR_HANDLE_ROTATE;

        freeSleep(10*1000);

        whilecnt=0;
        while(1){
            if(_Trap_FINISH_FLAG == true)
                break;
            whilecnt++;
            if(whilecnt > 400)
                break;
            freeSleep(50*1000);
        }
        // Open Hand
        freeSleep(10*1000);
        joint->SetMoveJoint(LF2, -125, 5, MODE_ABSOLUTE);
        usleep(6000*1000);
        joint->SetMoveJoint(LF2, 0, 5, MODE_ABSOLUTE);
        usleep(10*1000);

        WBmotion->addRHPosInfo((des_RH_pos[0]+0.01), des_RH_pos[1], des_RH_pos[2], 0.5);
        usleep(510*1000);

        // Check Hand if Open
        long temp_finger_enc=finger_scale*(sharedData->ENCODER[MC_ID_CH_Pairs[LF2].id][MC_ID_CH_Pairs[LF2].ch].CurrentPosition);
        usleep(10*1000);
        if(temp_finger_enc < -16000){
            sharedData->STATE_COMMAND = TCMD_PUSHDOOR_MOTION_2_NOPROBLEM;
        }else{
            sharedData->STATE_COMMAND = TCMD_PUSHDOOR_MOTION_2_ESTOP;
            return 4;
        }

    }

    // Right Hand Back
    freeSleep(10*1000);
    StartWBIKmotion(WBmode_RULU);
    freeSleep(10*1000);

    vec3 cur_LH_pos = vec3(WBmotion->pLH_3x1);
    pos_sec =2.5;
    WBmotion->addLHPosInfo(cur_LH_pos[0]-0.23, cur_LH_pos[1], cur_LH_pos[2]+0.02, pos_sec);
    sck = freeSleep(2510*1000);
    if(sck==1) return 4;

    // Gain return
    MCBoardSetSwitchingMode(3, 19, 0);//LSY LEB
    MCBoardSetSwitchingMode(3, 20, 0);//LWY LWP
    MCJointGainOverride(3, 19, 1, eq_gain_1000, 1300);//LSY
    MCJointGainOverride(3, 19, 2, eq_gain_1000, 1300);//LEB
    MCJointGainOverride(3, 20, 1, eq_gain_1000, 1300);//LWY
    MCJointGainOverride(3, 20, 2, eq_gain_1000, 1300);//LWP
    sck = freeSleep(1310*1000);
    if(sck==1) return 4;

    // Arm back
    TurnOff_WBIK();
    postime = 3000;//3500
    joint->SetMoveJoint(LSP, -75.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSR, -15.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LEB, -130, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWP, -50.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LF1, 0.0, postime, MOVE_ABSOLUTE);
    usleep(1000*1000);

    joint->SetMoveJoint(LF2, 125, 10, MODE_ABSOLUTE);
    usleep(1000*1000);

    // ----------------------------------------------------------------------
    // Go slightly foward
    // ----------------------------------------------------------------------
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[3] = 3;// Vel Change Mode
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[4] = 4;// No Wait Flag
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[6] = 6;// Direct compen mode

    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[3]=240;// des vel

    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[0]=0.45;
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[1]=0.0;
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[2]=0.;
    sharedData->COMMAND[PODO_NO_WHEEL].USER_COMMAND= OMNIWHEEL_AL_GOTODES;
    userData->WheelDoneFlag = false;
    whilecnt=0;
    while(1){
        if(userData->WheelDoneFlag == true){
            userData->WheelDoneFlag = false;
            break;
        }
        whilecnt++;
        if(whilecnt>=500){
            break;
        }
        freeSleep(50*1000);
    }
    double elapsed_time = whilecnt*50;//ms
    if(elapsed_time > 2500){
        joint->SetMoveJoint(LF2, 0, 10, MOVE_ABSOLUTE);
        usleep(10*1000);
    }else{
        double remain = 2500-elapsed_time;
        usleep(remain*1000);
        joint->SetMoveJoint(LF2, 0, 10, MOVE_ABSOLUTE);
        usleep(10*1000);
    }

    // ----------------------------------------------------------------------
    // Change Pos and To Pass
    // ----------------------------------------------------------------------
    StartWBIKmotion(WBmode_RULU);
    freeSleep(10*1000);

    vec3 des_RH_pos_1 = vec3(WBmotion->pRH_3x1) + vec3(-0.27, 0., 0.);

    if(des_RH_pos_1[0] <0.38)
        des_RH_pos_1[0] = 0.38;

    pos_sec = 1.8;
    WBmotion->addRHPosInfo(des_RH_pos_1[0], des_RH_pos_1[1], des_RH_pos_1[2], pos_sec);
    sck = freeSleep(1850*1000);
    if(sck==1) return 4;

    // Move Pos
    TurnOff_WBIK();
    joint->RefreshToCurrentReference();
    joint->SetAllMotionOwner();

    postime = 1600;
    joint->SetMoveJoint(RSP, -5.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSR, 15.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSY, 30.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(REB, -115.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWY, 68.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWP, -50.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RF1, 0.0, postime, MOVE_ABSOLUTE);

    // ----------------------------------------------------------------------
    // Go more
    // ----------------------------------------------------------------------
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[3] = 3;// Vel Change Mode
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[4] = 4;// No Wait Flag
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[6] = 6;// Direct compen mode

    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[3]=280;// des vel

    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[0]=2.;//0.75
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[1]=0.01;//0
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[2]=-30.;//-145
    sharedData->COMMAND[PODO_NO_WHEEL].USER_COMMAND= OMNIWHEEL_AL_GOTODES;

    // Back
    userData->WheelDoneFlag = false;
    whilecnt=0;
    while(1){
        if(userData->WheelDoneFlag == true){
            userData->WheelDoneFlag = false;
            break;
        }
        whilecnt++;
        if(whilecnt>=500){
            break;
        }
        freeSleep(50*1000);
    }
    double time_elapse = whilecnt*50;//ms
    if(time_elapse >= 1610){
        usleep(10*1000);
    }else if(time_elapse < 1610){
        double surplus = fabs(1610-time_elapse)+10;
        usleep(surplus*1000);
    }

    // ----------------------------------------------------------------------
    // Go back to get out from door
    // ----------------------------------------------------------------------
//    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[3] = 3;// Vel Change Mode
//    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[4] = 4;// No Wait Flag

//    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[3]=250;// des vel

//    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[0]=-0.4;
//    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[1]=0.0;
//    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[2]=0.0;
//    sharedData->COMMAND[PODO_NO_WHEEL].USER_COMMAND= OMNIWHEEL_AL_GOTODES;
//    userData->WheelDoneFlag = false;
//    whilecnt=0;
//    while(1){
//        if(userData->WheelDoneFlag == true){
//            userData->WheelDoneFlag = false;
//            break;
//        }
//        whilecnt++;
//        if(whilecnt>=500){
//            break;
//        }
//        freeSleep(50*1000);
//    }


    postime = 3200.;
    joint->SetMoveJoint(WST, 0.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(RF1, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RF2, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LF1, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LF2, 0.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LSP, 40.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSR, -17.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LEB, -130.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, 10.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWP, 40.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(RSP, 40.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSR, -5.0, 1500, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(REB, -130, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWY, -10.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWP, 40.0, postime, MOVE_ABSOLUTE);

    usleep(1500*1000);
    joint->SetMoveJoint(RSR, 17.0, 2000, MOVE_ABSOLUTE);
    usleep(1700*1000);



    if(_mode == 1){
        sharedData->STATE_COMMAND = TCMD_PUSHDOOR_MOTION_2_EM_2_STOP;
    }else{
        sharedData->STATE_COMMAND = TCMD_PUSHDOOR_MOTION_2_END;
    }
    return 1;
    // ----------------------------------------------------------------------
    // END
    // ----------------------------------------------------------------------
}

int EM_PushDoor_ReturnHand_LEFT(void){
    sharedData->STATE_COMMAND = TCMD_PUSHDOOR_MOTION_1_EM_START;
    // Release Hand
    joint->SetMoveJoint(LF2, 0, 10, MODE_ABSOLUTE);
    usleep(15*1000);

    StartWBIKmotion(WBmode_RULU);
    freeSleep(10*1000);

    // RE-ROTATE HANDLE
    PushDoor_RotateHandle_LEFT(-1);
    usleep(100*1000);

    // HAND OPEN
    joint->SetMoveJoint(LF2, -125, 10, MODE_ABSOLUTE);
    usleep(4500*1000);
    joint->SetMoveJoint(LF2, 0, 10, MODE_ABSOLUTE);

    // BACK HAND
    doubles temp_return(4);
    for(int k=0; k<4; k++)
        temp_return[k] = _EM_PUSHDOOR_AP_ORI[k];
    pos_sec = 3.;
    WBmotion->addLHPosInfo(_EM_PUSHDOOR_AP_POS[0], _EM_PUSHDOOR_AP_POS[1], (_EM_PUSHDOOR_AP_POS[2]+0.03), pos_sec);
    WBmotion->addLElbPosInfo(20, pos_sec);//-70
    WBmotion->addLHOriInfo(temp_return, pos_sec);

    vec3 cur_RH_pos = vec3(WBmotion->pRH_3x1);
    WBmotion->addRHPosInfo(cur_RH_pos[0], (cur_RH_pos[1]-0.22), cur_RH_pos[2], pos_sec);
    WBmotion->addRElbPosInfo(-10, pos_sec);
    usleep(3010*1000);

    // GAIN BACK
    MCBoardSetSwitchingMode(3, 19, 0);//LSY LEB
    MCBoardSetSwitchingMode(3, 20, 0);//LWY LWP
    MCJointGainOverride(3, 19, 1, eq_gain_1000, 1600);//LSY
    MCJointGainOverride(3, 19, 2, eq_gain_1000, 1600);//LEB
    MCJointGainOverride(3, 20, 1, eq_gain_1000, 1600);//LWY
    MCJointGainOverride(3, 20, 2, eq_gain_1000, 1600);//LWP
    usleep(1610*1000);

    // RETURN POS
    TurnOff_WBIK();
    usleep(10*1000);
    joint->SetMoveJoint(LF2, 125, 10, MODE_ABSOLUTE);
    usleep(4000*1000);
    joint->SetMoveJoint(LF2, 0, 10, MODE_ABSOLUTE);

    postime = 3000.;
    joint->SetMoveJoint(RSP, 10.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSR, 15.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(REB, -130, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWP, 50.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RF1, 0., postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LSP, 10.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSR, -15.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LEB, -130.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWP, 50.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LF1, 0., postime, MOVE_ABSOLUTE);
    freeSleep(3010*1000);

    sharedData->STATE_COMMAND = TCMD_PUSHDOOR_MOTION_1_EM_STOP;

    return 1;
}
int EM_PushDoor_ReturnLH_LEFT(void){
//    sharedData->STATE_COMMAND = TCMD_PUSHDOOR_MOTION_2_EM_1_START;
//    // WBIK Set
//    StartWBIKmotion(WBmode_RULU);
//    freeSleep(10*1000);

//    // Back LH - middle
//    doubles temp_ds_H_ori_1(4), temp_ds_H_ori_2(4);
//    for(int k=0; k<4; k++){
//        temp_ds_H_ori_1[k] = EM_WHEELPUSH_LH_ori_1[k];
//        temp_ds_H_ori_2[k] = EM_WHEELPUSH_LH_ori_2[k];
//    }
//    pos_sec = 2.;
//    WBmotion->addRHPosInfo(EM_WHEELPUSH_LH_pos_2[0], EM_WHEELPUSH_LH_pos_2[1], EM_WHEELPUSH_LH_pos_2[2], pos_sec);
//    WBmotion->addRElbPosInfo(-30, pos_sec);
//    WBmotion->addRHOriInfo(temp_ds_H_ori_2, pos_sec);
//    usleep(2010*1000);

//    // Back LH - side
//    pos_sec = 2.2;
//    WBmotion->addRHPosInfo(EM_WHEELPUSH_LH_pos_1[0], EM_WHEELPUSH_LH_pos_1[1], EM_WHEELPUSH_LH_pos_1[2], pos_sec);
//    WBmotion->addRElbPosInfo(-15, pos_sec);
//    WBmotion->addRHOriInfo(temp_ds_H_ori_1, pos_sec);

//    // LH move pos
//    TurnOff_WBIK();

//    postime = 3000.;
//    joint->SetMoveJoint(RSP, 10.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RSR, 15.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RSY, 0.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(REB, -130.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RWY, 0.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RWP, 50.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RF1, 0., postime, MOVE_ABSOLUTE);
//    usleep(3010*1000);

//    sharedData->STATE_COMMAND = TCMD_PUSHDOOR_MOTION_2_EM_1_STOP;
    return 1;
}

void PushDoor_RotateHandle_LEFT(int rot_direction){
    handle_dir = rot_direction;
    handle_pos_first = pos(vec3(WBmotion->pLH_3x1),quat(WBmotion->qLH_4x1));
    handle_angle_now = 0.;
    _Trap_togo = handle_target_angle;
    _Trap_maxVel = 75;
    _Trap_maxAcc = 75;
    _isTrap = true;
    _Trap_FINISH_FLAG = false;
    _Trap_Motion_Command = TR_HANDLE_ROTATE;

    Handle_rotate_done_Flag =false;

    whilecnt =0;
    while(1){
        if(Handle_rotate_done_Flag == true){
            Handle_rotate_done_Flag = false;
            break;
        }
        whilecnt ++;
        if(whilecnt > 400)
            break;
        freeSleep(50*1000);
    }
}




int PushDoor_GrabAndOpen(int _mode){

    sharedData->STATE_COMMAND = TCMD_PUSHDOOR_MOTION_1_START;

    // Grab handle - rotate handle
    // ----------------------------------------------------------------------
    // Data Get
    // ----------------------------------------------------------------------
    vec3 cur_vecX, cur_vecY,  cur_vecZ;
    vec3 pelv_nobe;

    double des_x, des_y, des_z;
    double des_x2, des_y2, des_z2;
    doubles des_RH_ori(4);

    unsigned char data_ok_flag = true;
    unsigned char norm_ok_flag = true;
    pelv_nobe[0] = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
    pelv_nobe[1] = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
    pelv_nobe[2] = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];
    cur_vecX[0] = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[3];
    cur_vecX[1] = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[4];
    cur_vecX[2] = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[5];

    if((pelv_nobe[0] < 0.25) || (pelv_nobe[1]>-0.1)){
        data_ok_flag = false;
    }
    if(fabs(cur_vecX[0])<0.85){
        norm_ok_flag = false;
    }
    if((data_ok_flag==false) || (norm_ok_flag==false)){
        return 4;
    }
    if(cur_vecX[0]<0){
        cur_vecX = -cur_vecX;
    }
    for(int k=0; k<3; k++)
        saved_normal[k] = cur_vecX[k];

    cur_vecZ[0]=0.;
    cur_vecZ[1]=0.;
    cur_vecZ[2]=1.;
    cur_vecY=cross(cur_vecZ, cur_vecX);

    float door_hand_ap_margin[3];
    door_hand_ap_margin[0]=0.15;
    door_hand_ap_margin[1]= RIGHT_NOBE_PUSH_Y;//depend push door case
    door_hand_ap_margin[2]=0.;
    des_x = pelv_nobe[0]-door_hand_ap_margin[0]*cur_vecX[0]-door_hand_ap_margin[1]*cur_vecY[0]-door_hand_ap_margin[2]*cur_vecZ[0];
    des_y = pelv_nobe[1]-door_hand_ap_margin[0]*cur_vecX[1]-door_hand_ap_margin[1]*cur_vecY[1]-door_hand_ap_margin[2]*cur_vecZ[1];
    des_z = pelv_nobe[2]-door_hand_ap_margin[0]*cur_vecX[2]-door_hand_ap_margin[1]*cur_vecY[2]-door_hand_ap_margin[2]*cur_vecZ[2];
    // Insert
    door_hand_ap_margin[0]= RIGHT_NOBE_PUSH_X;//depend
    door_hand_ap_margin[1]= RIGHT_NOBE_PUSH_Y;
    door_hand_ap_margin[2]=0.;
    des_x2 = pelv_nobe[0]-door_hand_ap_margin[0]*cur_vecX[0]-door_hand_ap_margin[1]*cur_vecY[0]-door_hand_ap_margin[2]*cur_vecZ[0];
    des_y2 = pelv_nobe[1]-door_hand_ap_margin[0]*cur_vecX[1]-door_hand_ap_margin[1]*cur_vecY[1]-door_hand_ap_margin[2]*cur_vecZ[1];
    des_z2 = pelv_nobe[2]-door_hand_ap_margin[0]*cur_vecX[2]-door_hand_ap_margin[1]*cur_vecY[2]-door_hand_ap_margin[2]*cur_vecZ[2];

    mat3 _wall_rotaion_matrix;
    _wall_rotaion_matrix[0][0]=cur_vecX[0];  _wall_rotaion_matrix[0][1]=cur_vecY[0];  _wall_rotaion_matrix[0][2]=cur_vecZ[0];
    _wall_rotaion_matrix[1][0]=cur_vecX[1];  _wall_rotaion_matrix[1][1]=cur_vecY[1];  _wall_rotaion_matrix[1][2]=cur_vecZ[1];
    _wall_rotaion_matrix[2][0]=cur_vecX[2];  _wall_rotaion_matrix[2][1]=cur_vecY[2];  _wall_rotaion_matrix[2][2]=cur_vecZ[2];

    quat q_wall = quat(_wall_rotaion_matrix);
    quat q_temp = quat(vec3(0,1,0),-90*D2R);
    quat q_des_RH=q_wall*q_temp;

    for(int k=0;k<4;k++){
        des_RH_ori[k] = q_des_RH[k];
        _EM_PUSHDOOR_AP_ORI[k] = q_des_RH[k];
    }
    _EM_PUSHDOOR_AP_POS[0] = des_x;
    _EM_PUSHDOOR_AP_POS[1] = des_y;
    _EM_PUSHDOOR_AP_POS[2] = des_z;
    // ----------------------------------------------------------------------
    // Goto Approach point
    // ----------------------------------------------------------------------
    StartWBIKmotion(WBmode_RULU);
    freeSleep(10*1000);

    vec3 cur_LH_pos = vec3(WBmotion->pLH_3x1);//
    vec3 des_LH_pos_1 = cur_LH_pos + vec3(0.12, 0., 0.);//
    des_LH_pos_1[2] = 0.43;//

    pos_sec=2.5;
    WBmotion->addRHPosInfo(des_x, des_y, des_z, pos_sec);
    WBmotion->addRElbPosInfo(-20, pos_sec);//-70
    WBmotion->addRHOriInfo(des_RH_ori, pos_sec);

    WBmotion->addLHPosInfo(des_LH_pos_1[0], des_LH_pos_1[1], des_LH_pos_1[2], pos_sec);//
    WBmotion->addLElbPosInfo(15, pos_sec);//

    int tc = FingerPositionInput_RIGHT(-14000);

    if(tc>=2500)
        freeSleep(20*1000);
    else
        freeSleep((2500-tc+20)*1000);
//    // ----------------------------------------------------------------------
//    // FT sensor null
//    // ----------------------------------------------------------------------
//    MCWristFTsensorNull(2, 53);
//    //RBwFTsensorNull(2, 53);
//    sck = freeSleep(600*1000);
//    if(sck==1) return 4;

    // ----------------------------------------------------------------------
    // Insert Hand
    // ----------------------------------------------------------------------
    vec3 des_LH_pos_2 = des_LH_pos_1;
    des_LH_pos_2[0] = 0.49;
    des_LH_pos_2[1] = 0.;
    doubles ds_des_LH_ori(4);
    quat q_temp1 = quat(vec3(0,1,0), -90*D2R);
    quat q_temp2 = quat(vec3(0,0,1), -90*D2R);
    quat des_LH_ori = q_temp1*q_temp2;
    for(int k=0; k<4; k++)
        ds_des_LH_ori[k] = des_LH_ori[k];

    pos_sec = 1.8;
    WBmotion->addRHPosInfo(des_x2, des_y2, des_z2, pos_sec);
    WBmotion->addRElbPosInfo(-15, pos_sec);//unlv

    WBmotion->addLHPosInfo(des_LH_pos_2[0], des_LH_pos_2[1], des_LH_pos_2[2], pos_sec);//
    WBmotion->addLHOriInfo(ds_des_LH_ori, pos_sec);
    WBmotion->addLElbPosInfo(20, pos_sec);//

    freeSleep(1810*1000);
    if(sck==1) return 4;

    // ----------------------------------------------------------------------
    // Gain Over and Grap hnad
    // ----------------------------------------------------------------------
    MCBoardSetSwitchingMode(2, 15, 1);//RSY REB
    MCBoardSetSwitchingMode(2, 16, 1);//RWY RWP
    MCJointGainOverride(2, 15, 1, eq_gain_1, 300);//RSY
    MCJointGainOverride(2, 15, 2, eq_gain_1, 300);//REB
    MCJointGainOverride(2, 16, 1, eq_gain_0, 300);//RWY
    MCJointGainOverride(2, 16, 2, eq_gain_1, 300);//RWP
    sck = freeSleep(300*1000);
    if(sck==1) return 4;

    joint->SetMoveJoint(RF2, 125, 10, MODE_ABSOLUTE);
    sck = freeSleep(2700*1000);
    if(sck==1) return 4;
    joint->SetMoveJoint(RF2, 0, 10, MODE_ABSOLUTE);
    pos_sec = 0.8;
    WBmotion->addRHPosInfo((des_x2+0.05), des_y2, des_z2, pos_sec);
    sck = freeSleep(600*1000);
    if(sck==1) return 4;
    joint->SetMoveJoint(RF2, 125, 10, MODE_ABSOLUTE);
    sck = freeSleep(2000*1000);//3800
    if(sck==1) return 4;
    //joint->SetMoveJoint(RF2, 0, 10, MODE_ABSOLUTE);
    //usleep(10*1000);
    WBmotion->addRHPosInfo((des_x2+0.02), des_y2, des_z2, 0.6);
    freeSleep(620*1000);

    GetPosFromENC();
    freeSleep(20*1000);
    vec3 befor_rot = vec3(WBmotion->enc_pRH_3x1);

//    FTavgFlag = true;
//    freeSleep(1010*1000);
//    befor_FTavg = FTavg;
//    freeSleep(50*1000);
    // ----------------------------------------------------------------------
    // Rotate Handle
    // ----------------------------------------------------------------------
    StartWBIKmotion(WBmode_RULU);
    freeSleep(10*1000);

    for(int k=0; k<3; k++){
        handle_rotate_center[k] = pelv_nobe[k] + cur_vecY[k]*(- DOOR_NOBE_LENGTH);//depend
        hyo_debug[k] = handle_rotate_center[k];
        handle_rotate_axis[k] = cur_vecX[k];
    }
    handle_target_angle = RIGHT_NOBE_ROT_ANGLE;

    PushDoor_RotateHandle(-1);

    // ----------------------------------------------------------------------
    // Check OK
    // ----------------------------------------------------------------------
    GetPosFromENC();
    freeSleep(20*1000);
    vec3 after_rot = vec3(WBmotion->enc_pRH_3x1);

//    FTavgFlag = true;
//    freeSleep(1010*1000);
//    after_FTavg = FTavg;

    vec3 hand_pos_err = after_rot - befor_rot;
//    printf("Motion Checker 1::Hand pos change:: %lf .....!!!\n", hand_pos_err[0]);
    double FTratio = befor_FTavg / after_FTavg;
//    printf("Motion Checker 1::FT sensor change:: %lf %lf %lf\n", befor_FTavg, after_FTavg, FTratio);

    int ENCODER_TEST = true;
    int FTSENSOR_TEST = true;

    if(hand_pos_err[0] > 0.011)
        ENCODER_TEST = true;
    if(fabs(after_FTavg)>40)
        FTSENSOR_TEST = true;

    if((ENCODER_TEST==true) && (FTSENSOR_TEST==true)){
        sharedData->STATE_COMMAND = TCMD_PUSHDOOR_MOTION_1_END;
        return 1;//success
    }else{
        sharedData->STATE_COMMAND = TCMD_PUSHDOOR_MOTION_1_FAIL;
        return 0;//fail
    }
    // ----------------------------------------------------------------------
    // END
    // ----------------------------------------------------------------------
}

int PushDoor_LHblock_RHback_Gothrough(int _mode){
    if(_mode ==1){
        sharedData->STATE_COMMAND = TCMD_PUSHDOOR_MOTION_2_EM_2_START;
        // Pass
    }else{
        sharedData->STATE_COMMAND = TCMD_PUSHDOOR_MOTION_2_START;
        // ----------------------------------------------------------------------
        // Rotate Door to Open
        // ---------------------------------------------------------------------
        StartWBIKmotion(WBmode_RULU);
        freeSleep(10*1000);

        vec3 temp_LH = vec3(WBmotion->pLH_3x1);

        handle_rotate_center[0] = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[6];
        handle_rotate_center[1] = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[7];
        handle_rotate_center[2] = WBmotion->pRH_3x1[2];
        handle_rotate_axis[0] = 0.;
        handle_rotate_axis[1] = 0.;
        handle_rotate_axis[2] = 1.;
        handle_target_angle = door_open_angle;
        handle_dir = 1; //

        handle_pos_first = pos(vec3(WBmotion->pRH_3x1),quat(WBmotion->qRH_4x1));
        handle_angle_now = 0.;
        _Trap_togo = handle_target_angle;
        _Trap_maxVel = 10;
        _Trap_maxAcc = 5;
        _isTrap = true;
        _Trap_FINISH_FLAG = false;
        _Trap_Motion_Command = TR_HANDLE_ROTATE;

        pos_sec=1.5;
        WBmotion->addLHPosInfo((temp_LH[0]+0.08), temp_LH[1], temp_LH[2], pos_sec);
        WBmotion->addLElbPosInfo(25, pos_sec);
        double mot_time = pos_sec*1000;

        whilecnt=0;
        while(1){
            if(_Trap_FINISH_FLAG == true)
                break;
            whilecnt++;
            if(whilecnt > 300)
                break;
            freeSleep(50*1000);
        }
        double timeelapse = whilecnt*50;
        if(timeelapse > mot_time)
            usleep(5*1000);
        else if(timeelapse <= mot_time){
            double diff_time = mot_time - timeelapse+5;
            usleep(diff_time*1000);
        }
        // ----------------------------------------------------------------------
        // Left Hand Blocking
        // ----------------------------------------------------------------------
        double temp_c = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[6];
        if((temp_c < 0.6) || (temp_c > 0.85)){
            temp_c = 0.7208;
        }

        GetPosFromENC();
        freeSleep(10*1000);

        vec3 cur_RH_pos = vec3(WBmotion->enc_pRH_3x1);
        vec3 cur_LH_pos = vec3(WBmotion->pLH_3x1);

        vec3 des_LH_pos;

        des_LH_pos[0] = (temp_c + cur_RH_pos[0])/2. - 0.03;
        des_LH_pos[1] = 0.;
        des_LH_pos[2] = cur_LH_pos[2];

        pos_sec=1.8;
        WBmotion->addLHPosInfo(des_LH_pos[0], des_LH_pos[1], des_LH_pos[2], pos_sec);
        WBmotion->addLElbPosInfo(30, pos_sec);
        sck = freeSleep(1810*1000);
        if(sck==1) return 4;

        // hyoin test
        joint->SetMoveJoint(RF2, 0, 10, MODE_ABSOLUTE);
        usleep(20*1000);

        // ----------------------------------------------------------------------
        // Right Hand Back
        // ----------------------------------------------------------------------

        vec3 ori_normal = vec3(saved_normal);
        mat3 rotmatrix(vec3(0,0,-1),door_open_angle*D2R);
        vec3 new_normal = rotmatrix*ori_normal;

        for(int k=0; k<3; k++){
            handle_rotate_center[k] = hyo_debug[k];
            handle_rotate_axis[k] = new_normal[k];
        }
        handle_dir = 1; //

        handle_pos_first = pos(vec3(WBmotion->pRH_3x1),quat(WBmotion->qRH_4x1));
        handle_angle_now = 0.;
        _Trap_togo = RIGHT_NOBE_ROT_ANGLE;
        _Trap_maxVel = 50;
        _Trap_maxAcc = 50;
        _isTrap = true;
        _Trap_FINISH_FLAG =false;
        _Trap_Motion_Command = TR_HANDLE_ROTATE;

        freeSleep(10*1000);

        whilecnt=0;
        while(1){
            if(_Trap_FINISH_FLAG == true)
                break;
            whilecnt++;
            if(whilecnt > 400)
                break;
            freeSleep(50*1000);
        }
        // Open Hand
        freeSleep(10*1000);
        joint->SetMoveJoint(RF2, -125, 5, MODE_ABSOLUTE);
        usleep(4500*1000);
        joint->SetMoveJoint(RF2, 0, 5, MODE_ABSOLUTE);
        usleep(10*1000);

        WBmotion->addLHPosInfo((des_LH_pos[0]+0.01), des_LH_pos[1], des_LH_pos[2], 0.5);
        usleep(510*1000);

        // Check Hand if Open
        long temp_finger_enc=finger_scale*(sharedData->ENCODER[MC_ID_CH_Pairs[RF2].id][MC_ID_CH_Pairs[RF2].ch].CurrentPosition);
        usleep(10*1000);
        if(temp_finger_enc < -16000){
            sharedData->STATE_COMMAND = TCMD_PUSHDOOR_MOTION_2_NOPROBLEM;
        }else{
            sharedData->STATE_COMMAND = TCMD_PUSHDOOR_MOTION_2_ESTOP;
            return 4;
        }

    }

    // Right Hand Back
    freeSleep(10*1000);
    StartWBIKmotion(WBmode_RULU);
    freeSleep(10*1000);

    vec3 cur_RH_pos = vec3(WBmotion->pRH_3x1);
    pos_sec =2.5;
    WBmotion->addRHPosInfo(cur_RH_pos[0]-0.23, cur_RH_pos[1], cur_RH_pos[2]+0.01, pos_sec);
    sck = freeSleep(2510*1000);
    if(sck==1) return 4;

    // Gain return
    MCBoardSetSwitchingMode(2, 15, 0);//RSY REB
    MCBoardSetSwitchingMode(2, 16, 0);//RWY RWP
    MCJointGainOverride(2, 15, 1, eq_gain_1000, 1300);//RSY
    MCJointGainOverride(2, 15, 2, eq_gain_1000, 1300);//REB
    MCJointGainOverride(2, 16, 1, eq_gain_1000, 1300);//RWY
    MCJointGainOverride(2, 16, 2, eq_gain_1000, 1300);//RWP
    sck = freeSleep(1310*1000);
    if(sck==1) return 4;

    // Arm back
    TurnOff_WBIK();
    postime = 3000;//3500
    joint->SetMoveJoint(RSP, -75.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSR, 15.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(REB, -130, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWP, -50.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RF1, 0.0, postime, MOVE_ABSOLUTE);
    usleep(1000*1000);

    joint->SetMoveJoint(RF2, 125, 10, MODE_ABSOLUTE);
    usleep(1000*1000);

    // ----------------------------------------------------------------------
    // Go slightly foward
    // ----------------------------------------------------------------------
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[3] = 3;// Vel Change Mode
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[4] = 4;// No Wait Flag
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[6] = 6;// Direct compen mode

    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[3]=240;// des vel

    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[0]=0.45;
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[1]=0.0;
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[2]=0.;
    sharedData->COMMAND[PODO_NO_WHEEL].USER_COMMAND= OMNIWHEEL_AL_GOTODES;
    userData->WheelDoneFlag = false;
    whilecnt=0;
    while(1){
        if(userData->WheelDoneFlag == true){
            userData->WheelDoneFlag = false;
            break;
        }
        whilecnt++;
        if(whilecnt>=500){
            break;
        }
        freeSleep(50*1000);
    }
    double elapsed_time = whilecnt*50;//ms
    if(elapsed_time > 2500){
        joint->SetMoveJoint(RF2, 0, 10, MOVE_ABSOLUTE);
        usleep(10*1000);
    }else{
        double remain = 2500-elapsed_time;
        usleep(remain*1000);
        joint->SetMoveJoint(RF2, 0, 10, MOVE_ABSOLUTE);
        usleep(10*1000);
    }

    // ----------------------------------------------------------------------
    // Change Pos and To Pass
    // ----------------------------------------------------------------------
    StartWBIKmotion(WBmode_RULU);
    freeSleep(10*1000);

    vec3 des_LH_pos_1 = vec3(WBmotion->pLH_3x1) + vec3(-0.27, 0., 0.);

    if(des_LH_pos_1[0] <0.38)
        des_LH_pos_1[0] = 0.38;

    pos_sec = 1.8;
    WBmotion->addLHPosInfo(des_LH_pos_1[0], des_LH_pos_1[1], des_LH_pos_1[2], pos_sec);
    sck = freeSleep(1850*1000);
    if(sck==1) return 4;

    // Move Pos
    TurnOff_WBIK();
    joint->RefreshToCurrentReference();
    joint->SetAllMotionOwner();

    postime = 1600;
    joint->SetMoveJoint(LSP, -5.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSR, -15.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSY, -30.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LEB, -115.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, -68.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWP, -50.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LF1, 0.0, postime, MOVE_ABSOLUTE);

    // ----------------------------------------------------------------------
    // Go more
    // ----------------------------------------------------------------------
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[3] = 3;// Vel Change Mode
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[4] = 4;// No Wait Flag
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[6] = 6;// Direct compen mode

    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[3]=250;// des vel

    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[0]=1.5;//0.75
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[1]=-0.01;//0
    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[2]=0.;//145
    sharedData->COMMAND[PODO_NO_WHEEL].USER_COMMAND= OMNIWHEEL_AL_GOTODES;

    // Back
    userData->WheelDoneFlag = false;
    whilecnt=0;
    while(1){
        if(userData->WheelDoneFlag == true){
            userData->WheelDoneFlag = false;
            break;
        }
        whilecnt++;
        if(whilecnt>=500){
            break;
        }
        freeSleep(50*1000);
    }
    double time_elapse = whilecnt*50;//ms
    if(time_elapse >= 1610){
        usleep(10*1000);
    }else if(time_elapse < 1610){
        double surplus = fabs(1610-time_elapse)+10;
        usleep(surplus*1000);
    }

//    // ----------------------------------------------------------------------
//    // Go back to get out from door
//    // ----------------------------------------------------------------------
//    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[3] = 3;// Vel Change Mode
//    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[4] = 4;// No Wait Flag

//    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[3]=250;// des vel

//    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[0]=-0.4;
//    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[1]=0.0;
//    sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_DOUBLE[2]=0.0;
//    sharedData->COMMAND[PODO_NO_WHEEL].USER_COMMAND= OMNIWHEEL_AL_GOTODES;
//    userData->WheelDoneFlag = false;
//    whilecnt=0;
//    while(1){
//        if(userData->WheelDoneFlag == true){
//            userData->WheelDoneFlag = false;
//            break;
//        }
//        whilecnt++;
//        if(whilecnt>=500){
//            break;
//        }
//        freeSleep(50*1000);
//    }


    postime = 3200.;
    joint->SetMoveJoint(WST, 0.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(RF1, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RF2, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LF1, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LF2, 0.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(RSP, 40.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSR, 17.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(REB, -130, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWY, -10.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWP, 40.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LSP, 40.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSR, 5.0, 1500, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LEB, -130.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, 10.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWP, 40.0, postime, MOVE_ABSOLUTE);

    usleep(1500*1000);
    joint->SetMoveJoint(LSR, -17.0, 2000, MOVE_ABSOLUTE);
    usleep(1700*1000);



    if(_mode == 1){
        sharedData->STATE_COMMAND = TCMD_PUSHDOOR_MOTION_2_EM_2_STOP;
    }else{
        sharedData->STATE_COMMAND = TCMD_PUSHDOOR_MOTION_2_END;
    }
    return 1;
    // ----------------------------------------------------------------------
    // END
    // ----------------------------------------------------------------------
}

int EM_PushDoor_ReturnHand(void){
    sharedData->STATE_COMMAND = TCMD_PUSHDOOR_MOTION_1_EM_START;
    // Release Hand
    joint->SetMoveJoint(RF2, 0, 10, MODE_ABSOLUTE);
    usleep(15*1000);

    StartWBIKmotion(WBmode_RULU);
    freeSleep(10*1000);

    // RE-ROTATE HANDLE
    PushDoor_RotateHandle(1);
    usleep(100*1000);

    // HAND OPEN
    joint->SetMoveJoint(RF2, -125, 10, MODE_ABSOLUTE);
    usleep(4500*1000);
    joint->SetMoveJoint(RF2, 0, 10, MODE_ABSOLUTE);

    // BACK HAND
    doubles temp_return(4);
    for(int k=0; k<4; k++)
        temp_return[k] = _EM_PUSHDOOR_AP_ORI[k];
    pos_sec = 3.;
    WBmotion->addRHPosInfo(_EM_PUSHDOOR_AP_POS[0], _EM_PUSHDOOR_AP_POS[1], (_EM_PUSHDOOR_AP_POS[2]+0.03), pos_sec);
    WBmotion->addRElbPosInfo(-20, pos_sec);//-70
    WBmotion->addRHOriInfo(temp_return, pos_sec);

    vec3 cur_LH_pos = vec3(WBmotion->pLH_3x1);
    WBmotion->addLHPosInfo(cur_LH_pos[0], (cur_LH_pos[1]+0.22), cur_LH_pos[2], pos_sec);
    WBmotion->addLElbPosInfo(10, pos_sec);
    usleep(3010*1000);

    // GAIN BACK
    MCBoardSetSwitchingMode(2, 15, 0);//RSY REB
    MCBoardSetSwitchingMode(2, 16, 0);//RWY RWP
    MCJointGainOverride(2, 15, 1, eq_gain_1000, 1600);//RSY
    MCJointGainOverride(2, 15, 2, eq_gain_1000, 1600);//REB
    MCJointGainOverride(2, 16, 1, eq_gain_1000, 1600);//RWY
    MCJointGainOverride(2, 16, 2, eq_gain_1000, 1600);//RWP
    usleep(1610*1000);

    // RETURN POS
    TurnOff_WBIK();
    usleep(10*1000);
    joint->SetMoveJoint(RF2, 125, 10, MODE_ABSOLUTE);
    usleep(4000*1000);
    joint->SetMoveJoint(RF2, 0, 10, MODE_ABSOLUTE);

    postime = 3000.;
    joint->SetMoveJoint(RSP, 10.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSR, 15.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(REB, -130, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWP, 50.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RF1, 0., postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LSP, 10.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSR, -15.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LEB, -130.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWP, 50.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LF1, 0., postime, MOVE_ABSOLUTE);
    freeSleep(3010*1000);

    sharedData->STATE_COMMAND = TCMD_PUSHDOOR_MOTION_1_EM_STOP;

    return 1;
}
int EM_PushDoor_ReturnLH(void){
//    sharedData->STATE_COMMAND = TCMD_PUSHDOOR_MOTION_2_EM_1_START;
//    // WBIK Set
//    StartWBIKmotion(WBmode_RULU);
//    freeSleep(10*1000);

//    // Back LH - middle
//    doubles temp_ds_H_ori_1(4), temp_ds_H_ori_2(4);
//    for(int k=0; k<4; k++){
//        temp_ds_H_ori_1[k] = EM_WHEELPUSH_LH_ori_1[k];
//        temp_ds_H_ori_2[k] = EM_WHEELPUSH_LH_ori_2[k];
//    }
//    pos_sec = 2.;
//    WBmotion->addLHPosInfo(EM_WHEELPUSH_LH_pos_2[0], EM_WHEELPUSH_LH_pos_2[1], EM_WHEELPUSH_LH_pos_2[2], pos_sec);
//    WBmotion->addLElbPosInfo(30, pos_sec);
//    WBmotion->addLHOriInfo(temp_ds_H_ori_2, pos_sec);
//    usleep(2010*1000);

//    // Back LH - side
//    pos_sec = 2.2;
//    WBmotion->addLHPosInfo(EM_WHEELPUSH_LH_pos_1[0], EM_WHEELPUSH_LH_pos_1[1], EM_WHEELPUSH_LH_pos_1[2], pos_sec);
//    WBmotion->addLElbPosInfo(15, pos_sec);
//    WBmotion->addLHOriInfo(temp_ds_H_ori_1, pos_sec);

//    // LH move pos
//    TurnOff_WBIK();

//    postime = 3000.;
//    joint->SetMoveJoint(LSP, 10.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LSR, -15.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LEB, -130.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LWY, 0.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LWP, 50.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LF1, 0., postime, MOVE_ABSOLUTE);
//    usleep(3010*1000);

//    sharedData->STATE_COMMAND = TCMD_PUSHDOOR_MOTION_2_EM_1_STOP;
    return 1;
}

void PushDoor_RotateHandle(int rot_direction){
    handle_dir = rot_direction;
    handle_pos_first = pos(vec3(WBmotion->pRH_3x1),quat(WBmotion->qRH_4x1));
    handle_angle_now = 0.;
    _Trap_togo = handle_target_angle;
    _Trap_maxVel = 75;
    _Trap_maxAcc = 75;
    _isTrap = true;
    _Trap_FINISH_FLAG = false;
    _Trap_Motion_Command = TR_HANDLE_ROTATE;

    Handle_rotate_done_Flag =false;

    whilecnt =0;
    while(1){
        if(Handle_rotate_done_Flag == true){
            Handle_rotate_done_Flag = false;
            break;
        }
        whilecnt ++;
        if(whilecnt > 400)
            break;
        freeSleep(50*1000);
    }
}
/**************************************************************************************************
 * Walking Push Door In Motion Function
 *************************************************************************************************/
int WalkPushDoor_GrabAndOpen(int _mode){
    // ----------------------------------------------------------------------
    // Data Get
    // ----------------------------------------------------------------------
    StartWBIK_Globalmotion(WBmode_GLOBAL);
    freeSleep(10*1000);
    mat4 temp_FT2PEL;
    temp_FT2PEL = mat4::eye();
    temp_FT2PEL[0][3]= -WBmotion->pRF_3x1[0];
    temp_FT2PEL[1][3]= 0.;
    temp_FT2PEL[2][3]= WBmotion->pPelZ;
    temp_FT2PEL[3][3]=1.;


    vec4 pelv_pushHandle_4, foot_pushHandle_4;
    vec3 cur_vecX, cur_vecY, cur_vecZ;
    for(int k=0; k<3; k++){
        pelv_pushHandle_4[k] = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[k];
        cur_vecX[k] = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[k+3];
    }
    pelv_pushHandle_4[3] = 1.;
    foot_pushHandle_4 = temp_FT2PEL*pelv_pushHandle_4;

    vec3 foot_pushHandle;
    for(int k=0; k<3; k++)
        foot_pushHandle[k] = foot_pushHandle_4[k];

    if(cur_vecX[0]<0){
        for(int k=0; k<3; k++)
            cur_vecX[k]= -cur_vecX[k];
    }
    for(int k=0; k<3; k++)
        saved_normal[k] = cur_vecX[k];
    cur_vecZ = vec3(0, 0, 1);
    cur_vecY = cross(cur_vecZ, cur_vecX);

    vec3 door_ap_margin;
    door_ap_margin = vec3(0.14, 0.05, 0);
    vec3 des_pos_1 = foot_pushHandle - door_ap_margin[0]*cur_vecX - door_ap_margin[1]*cur_vecY - door_ap_margin[2]*cur_vecZ;
    door_ap_margin = vec3(0.065, 0.05, 0.);
    vec3 des_pos_2 = foot_pushHandle - door_ap_margin[0]*cur_vecX - door_ap_margin[1]*cur_vecY - door_ap_margin[2]*cur_vecZ;

    mat3 _wall_rotaion_matrix;
    _wall_rotaion_matrix[0][0]=cur_vecX[0];  _wall_rotaion_matrix[0][1]=cur_vecY[0];  _wall_rotaion_matrix[0][2]=cur_vecZ[0];
    _wall_rotaion_matrix[1][0]=cur_vecX[1];  _wall_rotaion_matrix[1][1]=cur_vecY[1];  _wall_rotaion_matrix[1][2]=cur_vecZ[1];
    _wall_rotaion_matrix[2][0]=cur_vecX[2];  _wall_rotaion_matrix[2][1]=cur_vecY[2];  _wall_rotaion_matrix[2][2]=cur_vecZ[2];

    quat q_wall = quat(_wall_rotaion_matrix);
    quat q_temp = quat(vec3(0,1,0),-90*D2R);
    quat q_des_ori=q_wall*q_temp;

    doubles ds_des_ori(4);
    for(int k=0; k<4; k++){
        ds_des_ori[k] = q_des_ori[k];
        _EM_WALKPUSHDOOR_AP_ORI[k] = q_des_ori[k];
    }
    _EM_WALKPUSHDOOR_AP_POS[0] = des_pos_1[0];
    _EM_WALKPUSHDOOR_AP_POS[1] = des_pos_1[1];
    _EM_WALKPUSHDOOR_AP_POS[2] = des_pos_1[2];
    // ----------------------------------------------------------------------
    // Approach to handle
    // ----------------------------------------------------------------------
    PEL2FOOT = WBmotion->pRF_3x1[0];

    pos_sec=1.5;
    WBmotion->addRHPosInfo(des_pos_1[0], des_pos_1[1], des_pos_1[2], pos_sec);
    WBmotion->addRElbPosInfo(-5, pos_sec);
    WBmotion->addRHOriInfo(ds_des_ori, pos_sec);
    int tc = FingerPositionInput_RIGHT(-14000);

    if(tc>=1500)
        freeSleep(20*1000);
    else
        freeSleep((1500-tc+20)*1000);
    // ----------------------------------------------------------------------
    // FT null
    // ----------------------------------------------------------------------
    MCWristFTsensorNull(2, 53);
    //RBwFTsensorNull(2, 53);
    freeSleep(600*1000);
    // ----------------------------------------------------------------------
    // Insert Hand to handle
    // ----------------------------------------------------------------------
    pos_sec = 1.2;
    WBmotion->addRHPosInfo(des_pos_2[0]+0.02, des_pos_2[1], des_pos_2[2], pos_sec);
    sck = freeSleep(1210*1000);
    if(sck==1) return 4;
    // ----------------------------------------------------------------------
    // Gain override
    // ----------------------------------------------------------------------
    MCBoardSetSwitchingMode(2, 15, 1);//RSY REB
    MCBoardSetSwitchingMode(2, 16, 1);//RWY RWP
    MCJointGainOverride(2, 15, 1, eq_gain_1, 500);//RSY
    MCJointGainOverride(2, 15, 2, eq_gain_1, 500);//REB
    MCJointGainOverride(2, 16, 1, eq_gain_0, 500);//RWY
    MCJointGainOverride(2, 16, 2, eq_gain_1, 500);//RWP
//    RBBoardSetSwitchingMode(2,15,1);
//    RBBoardSetSwitchingMode(2,16,1);
//    unsigned int temp_gain=1;
//    RBJointGainOverride(2,15,temp_gain,temp_gain,500);//RSY REB
//    RBJointGainOverride(2,16,0,temp_gain,500);//RWY RWP
    freeSleep(500*1000);
    // ----------------------------------------------------------------------
    // Grab handle
    // ----------------------------------------------------------------------
    joint->SetMoveJoint(RF2, 125, 10, MODE_ABSOLUTE);
    sck = freeSleep(2700*1000);
    if(sck==1) return 4;
    joint->SetMoveJoint(RF2, 0, 10, MODE_ABSOLUTE);
    pos_sec = 0.8;
    WBmotion->addRHPosInfo((des_pos_2[0]+0.045), des_pos_2[1], des_pos_2[2], pos_sec);
    sck = freeSleep(600*1000);
    if(sck==1) return 4;
    joint->SetMoveJoint(RF2, 125, 10, MODE_ABSOLUTE);
    sck = freeSleep(4000*1000);
    if(sck==1) return 4;
    //joint->SetMoveJoint(RF2, 0, 10, MODE_ABSOLUTE); //hyoin test
    //usleep(10*1000);
//    WBmotion->addRHPosInfo((des_pos_2[0]+0.045), des_pos_2[1], des_pos_2[2], 0.8);
//    sck = freeSleep(810*1000);
//    if(sck==1) return 4;
    // ----------------------------------------------------------------------
    // Rotate Handle
    // ----------------------------------------------------------------------
    for(int k=0; k<3; k++){
        handle_rotate_center[k] = foot_pushHandle[k] + cur_vecY[k]*(- DOOR_NOBE_LENGTH);//depend
        hyo_debug[k] = handle_rotate_center[k];
        handle_rotate_axis[k] = cur_vecX[k];
    }
    handle_target_angle = 55.;
    PushDoor_RotateHandle(-1);

    return 1;
}

int WalkPushDoor_LHblock_RHback_RHblock(int _mode){
    if(_mode ==1){
        // Pass
    }else{
        // ----------------------------------------------------------------------
        // Rotate Door
        // ----------------------------------------------------------------------
        handle_rotate_center[0] = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[6];
        handle_rotate_center[1] = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[7];
        handle_rotate_center[2] = WBmotion->pRH_3x1[2];
        handle_rotate_axis[0] = 0.;
        handle_rotate_axis[1] = 0.;
        handle_rotate_axis[2] = 1.;
        handle_target_angle = walk_door_open_angle;
        handle_dir = 1; //

        handle_pos_first = pos(vec3(WBmotion->pRH_3x1),quat(WBmotion->qRH_4x1));
        handle_angle_now = 0.;
        _Trap_togo = handle_target_angle;
        _Trap_maxVel = 10;
        _Trap_maxAcc = 5;
        _isTrap = true;
        _Trap_FINISH_FLAG = false;
        _Trap_Motion_Command = TR_HANDLE_ROTATE;

        whilecnt=0;
        while(1){
            if(_Trap_FINISH_FLAG == true)
                break;
            whilecnt++;
            if(whilecnt > 300)
                break;
            freeSleep(50*1000);
        }
        // ----------------------------------------------------------------------
        // LH block
        // ----------------------------------------------------------------------
        double temp_c = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[6];
        //GetPosFromENC();
        //freeSleep(10*1000);

        vec3 cur_RH_pos = vec3(WBmotion->pRH_3x1);//vec3(WBmotion->enc_pRH_3x1);
        vec3 cur_LH_pos = vec3(WBmotion->pLH_3x1);

        vec3 des_LH_pos_1, des_LH_pos_2, des_LH_pos_3;
        quat des_LH_ori;
        doubles ds_des_LH_ori(4);

        des_LH_pos_1 = cur_LH_pos + vec3(0.12, 0., 0.);
        des_LH_pos_1[2] = (cur_RH_pos[2]+0.1);

        des_LH_pos_2 = des_LH_pos_1;
        des_LH_pos_2[0] = (temp_c + cur_RH_pos[0])/2. - 0.05;
        des_LH_pos_2[1] = 0.;

        des_LH_pos_3 = des_LH_pos_2;
        des_LH_pos_3[0] = (temp_c + cur_RH_pos[0])/2. - 0.02;
        walkpush_save_vec = des_LH_pos_3;

        quat q_temp1 = quat(vec3(0,1,0), -90*D2R);
        quat q_temp2 = quat(vec3(0,0,1), -90*D2R);
        des_LH_ori = q_temp1*q_temp2;
        for(int k=0; k<4; k++)
            ds_des_LH_ori[k] = des_LH_ori[k];

        EM_WALKPUSH_LH_pos_1 = des_LH_pos_1;
        EM_WALKPUSH_LH_pos_2 = des_LH_pos_2;
        EM_WALKPUSH_LH_ori_1 = quat(WBmotion->qLH_4x1);
        EM_WALKPUSH_LH_ori_2 = des_LH_ori;

        // real moving

        pos_sec = 1.5;
        WBmotion->addLHPosInfo(des_LH_pos_1[0], des_LH_pos_1[1], des_LH_pos_1[2], pos_sec);
        WBmotion->addLElbPosInfo(15, pos_sec);
        sck = freeSleep(1510*1000);
        if(sck==1) return 4;

        pos_sec=2.;
        WBmotion->addLHPosInfo(des_LH_pos_2[0], des_LH_pos_2[1], des_LH_pos_2[2], pos_sec);
        WBmotion->addLElbPosInfo(30, pos_sec);
        WBmotion->addLHOriInfo(ds_des_LH_ori, pos_sec);
        sck = freeSleep(2010*1000);
        if(sck==1) return 4;

        pos_sec=1.5;
        WBmotion->addLHPosInfo(des_LH_pos_3[0], des_LH_pos_3[1], des_LH_pos_3[2], pos_sec);
        WBmotion->addLElbPosInfo(35, pos_sec);
        sck = freeSleep(1510*1000);
        if(sck==1) return 4;
        // ----------------------------------------------------------------------
        // RH re-rotate
        // ----------------------------------------------------------------------
        joint->SetMoveJoint(RF2, 0, 10, MODE_ABSOLUTE);// hyoin test
        usleep(10*1000);

        vec3 ori_normal = vec3(saved_normal);
        mat3 rotmatrix(vec3(0,0,-1),door_open_angle*D2R);
        vec3 new_normal = rotmatrix*ori_normal;

        for(int k=0; k<3; k++){
            handle_rotate_center[k] = hyo_debug[k];
            handle_rotate_axis[k] = new_normal[k];
        }
        handle_dir = 1; //

        handle_pos_first = pos(vec3(WBmotion->pRH_3x1),quat(WBmotion->qRH_4x1));
        handle_angle_now = 0.;
        _Trap_togo = 55;
        _Trap_maxVel = 50;
        _Trap_maxAcc = 50;
        _isTrap = true;
        _Trap_FINISH_FLAG =false;
        _Trap_Motion_Command = TR_HANDLE_ROTATE;

        freeSleep(10*1000);

        whilecnt=0;
        while(1){
            if(_Trap_FINISH_FLAG == true)
                break;
            whilecnt++;
            if(whilecnt > 400)
                break;
            freeSleep(50*1000);
        }
        // ----------------------------------------------------------------------
        // Open RH
        // ----------------------------------------------------------------------
        freeSleep(10*1000);
        joint->SetMoveJoint(RF2, -125, 5, MODE_ABSOLUTE);
        sck = freeSleep(4500*1000);
        if(sck==1) return 4;
        joint->SetMoveJoint(RF2, 0, 5, MODE_ABSOLUTE);

        // Check Open
        usleep(10*1000);
        long temp_finger_enc=finger_scale*(sharedData->ENCODER[MC_ID_CH_Pairs[RF2].id][MC_ID_CH_Pairs[RF2].ch].CurrentPosition);
        usleep(10*1000);
        if(temp_finger_enc < -16000)
            ;
        else
            return 4;
    }
    // ----------------------------------------------------------------------
    // RH return
    // ----------------------------------------------------------------------
    WBmotion->addLHPosInfo((walkpush_save_vec[0]+0.01), walkpush_save_vec[1], walkpush_save_vec[2], 1.);
    sck = freeSleep(1010*1000);
    if(sck==1) return 4;

    vec3 cur_RH_pos = vec3(WBmotion->pRH_3x1);
    pos_sec =3.;
    WBmotion->addRHPosInfo(cur_RH_pos[0]-0.23, cur_RH_pos[1], cur_RH_pos[2]+0.02, pos_sec);
    sck = freeSleep(3010*1000);
    if(sck==1) return 4;

    // Gain return
    MCBoardSetSwitchingMode(2, 15, 0);//RSY REB
    MCBoardSetSwitchingMode(2, 16, 0);//RWY RWP
    MCJointGainOverride(2, 15, 1, eq_gain_1000, 1600);//RSY
    MCJointGainOverride(2, 15, 2, eq_gain_1000, 1600);//REB
    MCJointGainOverride(2, 16, 1, eq_gain_1000, 1600);//RWY
    MCJointGainOverride(2, 16, 2, eq_gain_1000, 1600);//RWP
//    RBBoardSetSwitchingMode(2,15,0);
//    RBBoardSetSwitchingMode(2,16,0);
//    RBJointGainOverride(2,15,1000,1000,1600);//RSY REB
//    RBJointGainOverride(2,16,1000,1000,1600);//RWY RWP
    freeSleep(1610*1000);
    // ----------------------------------------------------------------------
    // RH block and fold LH
    // ----------------------------------------------------------------------
    joint->SetMoveJoint(RF2, 125, 5, MODE_ABSOLUTE);
    sck = freeSleep(4000*1000);
    if(sck==1) return 4;
    joint->SetMoveJoint(RF2, 0, 5, MODE_ABSOLUTE);
    usleep(10*1000);
    quat q_R_temp1 = quat(vec3(0,1,0), -90*D2R);
    quat q_R_temp2 = quat(vec3(0,0,1), -90*D2R);
    quat des_RH_ori = q_R_temp1*q_R_temp2;
    doubles ds_des_RH_ori(4);
    for(int k=0; k<4; k++)
        ds_des_RH_ori[k] = des_RH_ori[k];

    cur_RH_pos = vec3(WBmotion->pRH_3x1);
    pos_sec =2.8;
    WBmotion->addRHPosInfo(cur_RH_pos[0]+0.23, cur_RH_pos[1]+0.05, cur_RH_pos[2]-0.15, pos_sec);
    WBmotion->addRHOriInfo(ds_des_RH_ori, pos_sec);
    WBmotion->addRElbPosInfo(-45, pos_sec);
    sck = freeSleep(2810*1000);
    if(sck==1) return 4;

    TurnOff_WBIK();

    postime = 2200;
    joint->SetMoveJoint(LSP, -5.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSR, -12.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSY, -30.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LEB, -115.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, -68.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWP, -50.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LF1, 0.0, postime, MOVE_ABSOLUTE);
    sck = freeSleep(2210*1000);
    if(sck==1) return 4;

    StartWBIK_Globalmotion(WBmode_GLOBAL);

    pos_sec =3.;
    WBmotion->addRHPosInfo(cur_RH_pos[0]+0.4, cur_RH_pos[1]+0.12, cur_RH_pos[2]-0.15, pos_sec);
    sck = freeSleep(3010*1000);
    if(sck==1) return 4;

    return 1;
}

int WalkPushDoor_RotateWST_ReadyPass(int _mode){
    // ----------------------------------------------------------------------
    // Rotate WST
    // ----------------------------------------------------------------------
    WALK_WST_FLAG = true;
    StartWBIK_Globalmotion(WBmode_RPLU);
    freeSleep(10*1000);
    WBmotion->addWSTPosInfo(90, 5);
    WBmotion->addRElbPosInfo(65,5);
    sck = freeSleep(5010*1000);
    if(sck==1) return 4;

    StartWBIKmotion(WBmode_RULU);
    freeSleep(10*1000);
    pos_sec = 2.5;
    WBmotion->addWSTPosInfo(120, pos_sec);
    sck = freeSleep(2510*1000);
    if(sck==1) return 4;
    WBmotion->addWSTPosInfo(90, pos_sec);
    sck = freeSleep(2510*1000);
    if(sck==1) return 4;
    // ----------------------------------------------------------------------
    // chage to Ready Pass pos
    // ----------------------------------------------------------------------
    TurnOff_WBIK();
    postime = 2500;
    joint->SetMoveJoint(RSP, 13.54, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSR, 3.32, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSY, -68.74, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(REB, -70.8, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWY, 58.79, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWP, 24.53, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RF1, -164.38, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LSP, 35.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSR, -12.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LEB, -140.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWP, -40.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LF1, 0.0, postime, MOVE_ABSOLUTE);
    sck = freeSleep(2510*1000);
    if(sck==1) return 4;

    return 1;
}

int EM_WalkPushDoor_ReturnHand(void){
    // Release Hand
    joint->SetMoveJoint(RF2, 0, 10, MODE_ABSOLUTE);
    usleep(15*1000);

    StartWBIK_Globalmotion(WBmode_GLOBAL);
    freeSleep(10*1000);

    // RE-ROTATE HANDLE
    PushDoor_RotateHandle(1);
    usleep(100*1000);

    // HAND OPEN
    joint->SetMoveJoint(RF2, -125, 10, MODE_ABSOLUTE);
    usleep(4500*1000);
    joint->SetMoveJoint(RF2, 0, 10, MODE_ABSOLUTE);

    // BACK HAND
    doubles temp_return(4);
    for(int k=0; k<4; k++)
        temp_return[k] = _EM_WALKPUSHDOOR_AP_ORI[k];
    pos_sec = 3.;
    WBmotion->addRHPosInfo(_EM_WALKPUSHDOOR_AP_POS[0], _EM_WALKPUSHDOOR_AP_POS[1], (_EM_WALKPUSHDOOR_AP_POS[2]+0.03), pos_sec);
    WBmotion->addRElbPosInfo(-5, pos_sec);
    WBmotion->addRHOriInfo(temp_return, pos_sec);
    usleep(3010*1000);

    // GAIN BACK
    joint->SetMoveJoint(RF2, 125, 10, MODE_ABSOLUTE);

    MCBoardSetSwitchingMode(2, 15, 0);//RSY REB
    MCBoardSetSwitchingMode(2, 16, 0);//RWY RWP
    MCJointGainOverride(2, 15, 1, eq_gain_1000, 2000);//RSY
    MCJointGainOverride(2, 15, 2, eq_gain_1000, 2000);//REB
    MCJointGainOverride(2, 16, 1, eq_gain_1000, 2000);//RWY
    MCJointGainOverride(2, 16, 2, eq_gain_1000, 2000);//RWP
//    RBBoardSetSwitchingMode(2,15,0);
//    RBBoardSetSwitchingMode(2,16,0);
//    RBJointGainOverride(2,15,1000,1000,2000);//RSY REB
//    RBJointGainOverride(2,16,1000,1000,2000);//RWY RWP
    usleep(4000*1000);
    joint->SetMoveJoint(RF2, 0, 10, MODE_ABSOLUTE);

    // RETURN POS
    TurnOff_WBIK();

    // BACK to WALKREADY
    sharedData->COMMAND[PODO_NO_WALKREADY].USER_PARA_INT[0]=3;
    sharedData->COMMAND[PODO_NO_WALKREADY].USER_COMMAND=WALKREADY_GO_WALKREADYPOS;

    return 1;
}

int EM_WalkPushDoor_ReturnLH(void){
    // WBIK set
    StartWBIK_Globalmotion(WBmode_GLOBAL);
    freeSleep(10*1000);

    // Back LH - middle
    doubles temp_ds_H_ori_1(4), temp_ds_H_ori_2(4);
    for(int k=0; k<4; k++){
        temp_ds_H_ori_1[k] = EM_WALKPUSH_LH_ori_1[k];
        temp_ds_H_ori_2[k] = EM_WALKPUSH_LH_ori_2[k];
    }
    pos_sec = 2.;
    WBmotion->addLHPosInfo(EM_WALKPUSH_LH_pos_2[0], EM_WALKPUSH_LH_pos_2[1], EM_WALKPUSH_LH_pos_2[2], pos_sec);
    WBmotion->addLElbPosInfo(30, pos_sec);
    WBmotion->addLHOriInfo(temp_ds_H_ori_2, pos_sec);
    usleep(2010*1000);

    // Back LH - side
    pos_sec = 2.2;
    WBmotion->addLHPosInfo(EM_WALKPUSH_LH_pos_1[0], EM_WALKPUSH_LH_pos_1[1], EM_WALKPUSH_LH_pos_1[2], pos_sec);
    WBmotion->addLElbPosInfo(15, pos_sec);
    WBmotion->addLHOriInfo(temp_ds_H_ori_1, pos_sec);

    // LH move pos
    TurnOff_WBIK();

    postime = 3000.;
    joint->SetMoveJoint(LSP, 40.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSR, -10.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LEB, -130.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWP, 20.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LF1, 0., postime, MOVE_ABSOLUTE);
    usleep(3010*1000);
    return 1;
}

void WalkPushDoor_RotateHandle(int rot_direction){
    ;//Pass
}

int WalkPushDoor_GrabAndOpen_LEFT(int _mode){
    // ----------------------------------------------------------------------
    // Data Get
    // ----------------------------------------------------------------------
    StartWBIK_Globalmotion(WBmode_GLOBAL);
    freeSleep(10*1000);
    mat4 temp_FT2PEL;
    temp_FT2PEL = mat4::eye();
    temp_FT2PEL[0][3]= -WBmotion->pRF_3x1[0];
    temp_FT2PEL[1][3]= 0.;
    temp_FT2PEL[2][3]= WBmotion->pPelZ;
    temp_FT2PEL[3][3]=1.;


    vec4 pelv_pushHandle_4, foot_pushHandle_4;
    vec3 cur_vecX, cur_vecY, cur_vecZ;
    for(int k=0; k<3; k++){
        pelv_pushHandle_4[k] = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[k];
        cur_vecX[k] = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[k+3];
    }
    pelv_pushHandle_4[3] = 1.;
    foot_pushHandle_4 = temp_FT2PEL*pelv_pushHandle_4;

    vec3 foot_pushHandle;
    for(int k=0; k<3; k++)
        foot_pushHandle[k] = foot_pushHandle_4[k];

    if(cur_vecX[0]<0){
        for(int k=0; k<3; k++)
            cur_vecX[k]= -cur_vecX[k];
    }
    for(int k=0; k<3; k++)
        saved_normal[k] = cur_vecX[k];
    cur_vecZ = vec3(0, 0, 1);
    cur_vecY = cross(cur_vecZ, cur_vecX);

    vec3 door_ap_margin;
    door_ap_margin = vec3(0.14, -0.05, 0);
    vec3 des_pos_1 = foot_pushHandle - door_ap_margin[0]*cur_vecX - door_ap_margin[1]*cur_vecY - door_ap_margin[2]*cur_vecZ;
    door_ap_margin = vec3(0.065, -0.05, 0.);
    vec3 des_pos_2 = foot_pushHandle - door_ap_margin[0]*cur_vecX - door_ap_margin[1]*cur_vecY - door_ap_margin[2]*cur_vecZ;

    mat3 _wall_rotaion_matrix;
    _wall_rotaion_matrix[0][0]=cur_vecX[0];  _wall_rotaion_matrix[0][1]=cur_vecY[0];  _wall_rotaion_matrix[0][2]=cur_vecZ[0];
    _wall_rotaion_matrix[1][0]=cur_vecX[1];  _wall_rotaion_matrix[1][1]=cur_vecY[1];  _wall_rotaion_matrix[1][2]=cur_vecZ[1];
    _wall_rotaion_matrix[2][0]=cur_vecX[2];  _wall_rotaion_matrix[2][1]=cur_vecY[2];  _wall_rotaion_matrix[2][2]=cur_vecZ[2];

    quat q_wall = quat(_wall_rotaion_matrix);
    quat q_temp = quat(vec3(0,1,0),-90*D2R);
    quat q_des_ori=q_wall*q_temp;

    doubles ds_des_ori(4);
    for(int k=0; k<4; k++){
        ds_des_ori[k] = q_des_ori[k];
        _EM_WALKPUSHDOOR_AP_ORI[k] = q_des_ori[k];
    }
    _EM_WALKPUSHDOOR_AP_POS[0] = des_pos_1[0];
    _EM_WALKPUSHDOOR_AP_POS[1] = des_pos_1[1];
    _EM_WALKPUSHDOOR_AP_POS[2] = des_pos_1[2];
    // ----------------------------------------------------------------------
    // Approach to handle
    // ----------------------------------------------------------------------
    PEL2FOOT = WBmotion->pRF_3x1[0];

    pos_sec=1.5;
    WBmotion->addLHPosInfo(des_pos_1[0], des_pos_1[1], des_pos_1[2], pos_sec);
    WBmotion->addLElbPosInfo(5, pos_sec);
    WBmotion->addLHOriInfo(ds_des_ori, pos_sec);
    int tc = FingerPositionInput_LEFT(-14000);

    if(tc>=1500)
        freeSleep(20*1000);
    else
        freeSleep((1500-tc+20)*1000);
    // ----------------------------------------------------------------------
    // FT null
    // ----------------------------------------------------------------------
    MCWristFTsensorNull(3,54);
    //RBwFTsensorNull(3, 54);
    freeSleep(600*1000);
    // ----------------------------------------------------------------------
    // Insert Hand to handle
    // ----------------------------------------------------------------------
    pos_sec = 1.2;
    WBmotion->addLHPosInfo(des_pos_2[0]+0.02, des_pos_2[1], des_pos_2[2], pos_sec);
    sck = freeSleep(1210*1000);
    if(sck==1) return 4;
    // ----------------------------------------------------------------------
    // Gain override
    // ----------------------------------------------------------------------
    MCBoardSetSwitchingMode(3, 19, 1);//LSY LEB
    MCBoardSetSwitchingMode(3, 20, 1);//LWY LWP
    MCJointGainOverride(3, 19, 1, eq_gain_1, 500);//LSY
    MCJointGainOverride(3, 19, 2, eq_gain_1, 500);//LEB
    MCJointGainOverride(3, 20, 1, eq_gain_0, 500);//LWY
    MCJointGainOverride(3, 20, 2, eq_gain_1, 500);//LWP
//    RBBoardSetSwitchingMode(3,19,1);
//    RBBoardSetSwitchingMode(3,20,1);
//    unsigned int temp_gain=1;
//    RBJointGainOverride(3,19,temp_gain,temp_gain,500);//LSY LEB
//    RBJointGainOverride(3,20,0,temp_gain,500);//LWY LWP
    freeSleep(500*1000);
    // ----------------------------------------------------------------------
    // Grab handle
    // ----------------------------------------------------------------------
    joint->SetMoveJoint(LF2, 125, 10, MODE_ABSOLUTE);
    sck = freeSleep(2700*1000);
    if(sck==1) return 4;
    joint->SetMoveJoint(LF2, 0, 10, MODE_ABSOLUTE);
    pos_sec = 0.8;
    WBmotion->addLHPosInfo((des_pos_2[0]+0.045), des_pos_2[1], des_pos_2[2], pos_sec);
    sck = freeSleep(600*1000);
    if(sck==1) return 4;
    joint->SetMoveJoint(LF2, 125, 10, MODE_ABSOLUTE);
    sck = freeSleep(4000*1000);
    if(sck==1) return 4;
    //joint->SetMoveJoint(RF2, 0, 10, MODE_ABSOLUTE); //hyoin test
    //usleep(10*1000);
//    WBmotion->addRHPosInfo((des_pos_2[0]+0.045), des_pos_2[1], des_pos_2[2], 0.8);
//    sck = freeSleep(810*1000);
//    if(sck==1) return 4;
    // ----------------------------------------------------------------------
    // Rotate Handle
    // ----------------------------------------------------------------------
    for(int k=0; k<3; k++){
        handle_rotate_center[k] = foot_pushHandle[k] + cur_vecY[k]*(DOOR_NOBE_LENGTH);//depend
        hyo_debug[k] = handle_rotate_center[k];
        handle_rotate_axis[k] = cur_vecX[k];
    }
    handle_target_angle = 55.;
    PushDoor_RotateHandle_LEFT(1);

    return 1;
}

int WalkPushDoor_LHblock_RHback_RHblock_LEFT(int _mode){
    if(_mode ==1){
        // Pass
    }else{
        // ----------------------------------------------------------------------
        // Rotate Door
        // ----------------------------------------------------------------------
        handle_rotate_center[0] = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[6];
        handle_rotate_center[1] = -sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[7];
        handle_rotate_center[2] = WBmotion->pLH_3x1[2];
        handle_rotate_axis[0] = 0.;
        handle_rotate_axis[1] = 0.;
        handle_rotate_axis[2] = 1.;
        handle_target_angle = walk_door_open_angle;
        handle_dir = -1; //

        handle_pos_first = pos(vec3(WBmotion->pLH_3x1),quat(WBmotion->qLH_4x1));
        handle_angle_now = 0.;
        _Trap_togo = handle_target_angle;
        _Trap_maxVel = 10;
        _Trap_maxAcc = 5;
        _isTrap = true;
        _Trap_FINISH_FLAG = false;
        _Trap_Motion_Command = TR_HANDLE_ROTATE;

        whilecnt=0;
        while(1){
            if(_Trap_FINISH_FLAG == true)
                break;
            whilecnt++;
            if(whilecnt > 300)
                break;
            freeSleep(50*1000);
        }
        // ----------------------------------------------------------------------
        // LH block
        // ----------------------------------------------------------------------
        double temp_c = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[6];
        //GetPosFromENC();
        //freeSleep(10*1000);

        vec3 cur_LH_pos = vec3(WBmotion->pLH_3x1);//vec3(WBmotion->enc_pRH_3x1);
        vec3 cur_RH_pos = vec3(WBmotion->pRH_3x1);

        vec3 des_RH_pos_1, des_RH_pos_2, des_RH_pos_3;
        quat des_RH_ori;
        doubles ds_des_RH_ori(4);

        des_RH_pos_1 = cur_RH_pos + vec3(0.12, 0., 0.);
        des_RH_pos_1[2] = (cur_LH_pos[2]+0.1);

        des_RH_pos_2 = des_RH_pos_1;
        des_RH_pos_2[0] = (temp_c + cur_LH_pos[0])/2. - 0.05;
        des_RH_pos_2[1] = 0.;

        des_RH_pos_3 = des_RH_pos_2;
        des_RH_pos_3[0] = (temp_c + cur_LH_pos[0])/2. - 0.02;
        walkpush_save_vec = des_RH_pos_3;

        quat q_temp1 = quat(vec3(0,1,0), -90*D2R);
        quat q_temp2 = quat(vec3(0,0,1), 90*D2R);
        des_RH_ori = q_temp1*q_temp2;
        for(int k=0; k<4; k++)
            ds_des_RH_ori[k] = des_RH_ori[k];

        EM_WALKPUSH_LH_pos_1 = des_RH_pos_1;
        EM_WALKPUSH_LH_pos_2 = des_RH_pos_2;
        EM_WALKPUSH_LH_ori_1 = quat(WBmotion->qRH_4x1);
        EM_WALKPUSH_LH_ori_2 = des_RH_ori;

        // real moving

        pos_sec = 1.5;
        WBmotion->addRHPosInfo(des_RH_pos_1[0], des_RH_pos_1[1], des_RH_pos_1[2], pos_sec);
        WBmotion->addRElbPosInfo(-15, pos_sec);
        sck = freeSleep(1510*1000);
        if(sck==1) return 4;

        pos_sec=2.;
        WBmotion->addRHPosInfo(des_RH_pos_2[0], des_RH_pos_2[1], des_RH_pos_2[2], pos_sec);
        WBmotion->addRElbPosInfo(-30, pos_sec);
        WBmotion->addRHOriInfo(ds_des_RH_ori, pos_sec);
        sck = freeSleep(2010*1000);
        if(sck==1) return 4;

        pos_sec=1.5;
        WBmotion->addRHPosInfo(des_RH_pos_3[0], des_RH_pos_3[1], des_RH_pos_3[2], pos_sec);
        WBmotion->addRElbPosInfo(-35, pos_sec);
        sck = freeSleep(1510*1000);
        if(sck==1) return 4;
        // ----------------------------------------------------------------------
        // RH re-rotate
        // ----------------------------------------------------------------------
        joint->SetMoveJoint(LF2, 0, 10, MODE_ABSOLUTE);// hyoin test
        usleep(10*1000);

        vec3 ori_normal = vec3(saved_normal);
        mat3 rotmatrix(vec3(0,0,1),door_open_angle*D2R);
        vec3 new_normal = rotmatrix*ori_normal;

        for(int k=0; k<3; k++){
            handle_rotate_center[k] = hyo_debug[k];
            handle_rotate_axis[k] = new_normal[k];
        }
        handle_dir = -1; //

        handle_pos_first = pos(vec3(WBmotion->pLH_3x1),quat(WBmotion->qLH_4x1));
        handle_angle_now = 0.;
        _Trap_togo = 55;
        _Trap_maxVel = 50;
        _Trap_maxAcc = 50;
        _isTrap = true;
        _Trap_FINISH_FLAG =false;
        _Trap_Motion_Command = TR_HANDLE_ROTATE;

        freeSleep(10*1000);

        whilecnt=0;
        while(1){
            if(_Trap_FINISH_FLAG == true)
                break;
            whilecnt++;
            if(whilecnt > 400)
                break;
            freeSleep(50*1000);
        }
        // ----------------------------------------------------------------------
        // Open LH
        // ----------------------------------------------------------------------
        freeSleep(10*1000);
        joint->SetMoveJoint(LF2, -125, 5, MODE_ABSOLUTE);
        sck = freeSleep(4500*1000);
        if(sck==1) return 4;
        joint->SetMoveJoint(LF2, 0, 5, MODE_ABSOLUTE);

        // Check Open
        usleep(10*1000);
        long temp_finger_enc=finger_scale*(sharedData->ENCODER[MC_ID_CH_Pairs[LF2].id][MC_ID_CH_Pairs[LF2].ch].CurrentPosition);
        usleep(10*1000);
        if(temp_finger_enc < -16000)
            ;
        else
            return 4;
    }
    // ----------------------------------------------------------------------
    // RH return
    // ----------------------------------------------------------------------
    WBmotion->addRHPosInfo((walkpush_save_vec[0]+0.01), walkpush_save_vec[1], walkpush_save_vec[2], 1.);
    sck = freeSleep(1010*1000);
    if(sck==1) return 4;

    vec3 cur_LH_pos = vec3(WBmotion->pLH_3x1);
    pos_sec =3.;
    WBmotion->addLHPosInfo(cur_LH_pos[0]-0.23, cur_LH_pos[1], cur_LH_pos[2]+0.02, pos_sec);
    sck = freeSleep(3010*1000);
    if(sck==1) return 4;

    // Gain return
    MCBoardSetSwitchingMode(3, 19, 0);//LSY LEB
    MCBoardSetSwitchingMode(3, 20, 0);//LWY LWP
    MCJointGainOverride(3, 19, 1, eq_gain_1000, 1600);//LSY
    MCJointGainOverride(3, 19, 2, eq_gain_1000, 1600);//LEB
    MCJointGainOverride(3, 20, 1, eq_gain_1000, 1600);//LWY
    MCJointGainOverride(3, 20, 2, eq_gain_1000, 1600);//LWP
//    RBBoardSetSwitchingMode(3,19,0);
//    RBBoardSetSwitchingMode(3,20,0);
//    RBJointGainOverride(3,19,1000,1000,1600);//RSY REB
//    RBJointGainOverride(3,20,1000,1000,1600);//RWY RWP
    freeSleep(1610*1000);
    // ----------------------------------------------------------------------
    // RH block and fold LH
    // ----------------------------------------------------------------------
    joint->SetMoveJoint(LF2, 125, 5, MODE_ABSOLUTE);
    sck = freeSleep(4000*1000);
    if(sck==1) return 4;
    joint->SetMoveJoint(LF2, 0, 5, MODE_ABSOLUTE);
    usleep(10*1000);
    quat q_L_temp1 = quat(vec3(0,1,0), -90*D2R);
    quat q_L_temp2 = quat(vec3(0,0,1), 90*D2R);
    quat des_LH_ori = q_L_temp1*q_L_temp2;
    doubles ds_des_LH_ori(4);
    for(int k=0; k<4; k++)
        ds_des_LH_ori[k] = des_LH_ori[k];

    cur_LH_pos = vec3(WBmotion->pLH_3x1);
    pos_sec =2.8;
    WBmotion->addLHPosInfo(cur_LH_pos[0]+0.23, cur_LH_pos[1]-0.05, cur_LH_pos[2]-0.15, pos_sec);
    WBmotion->addLHOriInfo(ds_des_LH_ori, pos_sec);
    WBmotion->addLElbPosInfo(45, pos_sec);
    sck = freeSleep(2810*1000);
    if(sck==1) return 4;

    TurnOff_WBIK();

    postime = 2200;
    joint->SetMoveJoint(RSP, -5.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSR, 12.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSY, 30.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(REB, -115.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWY, 68.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWP, -50.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RF1, 0.0, postime, MOVE_ABSOLUTE);
    sck = freeSleep(2210*1000);
    if(sck==1) return 4;

    StartWBIK_Globalmotion(WBmode_GLOBAL);

    pos_sec =3.;
    WBmotion->addLHPosInfo(cur_LH_pos[0]+0.4, cur_LH_pos[1]-0.12, cur_LH_pos[2]-0.15, pos_sec);
    sck = freeSleep(3010*1000);
    if(sck==1) return 4;

    return 1;
}

int WalkPushDoor_RotateWST_ReadyPass_LEFT(int _mode){
    // ----------------------------------------------------------------------
    // Rotate WST
    // ----------------------------------------------------------------------
    WALK_WST_FLAG = true;
    StartWBIK_Globalmotion(WBmode_RULP);
    freeSleep(10*1000);
    WBmotion->addWSTPosInfo(-90, 5);
    WBmotion->addLElbPosInfo(-65,5);
    sck = freeSleep(5010*1000);
    if(sck==1) return 4;

    StartWBIKmotion(WBmode_RULU);
    freeSleep(10*1000);
    pos_sec = 2.5;
    WBmotion->addWSTPosInfo(-120, pos_sec);
    sck = freeSleep(2510*1000);
    if(sck==1) return 4;
    WBmotion->addWSTPosInfo(-90, pos_sec);
    sck = freeSleep(2510*1000);
    if(sck==1) return 4;
    // ----------------------------------------------------------------------
    // chage to Ready Pass pos
    // ----------------------------------------------------------------------
    TurnOff_WBIK();
    postime = 2500;
    joint->SetMoveJoint(RSP, 35.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSR, 12.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(REB, -140.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWP, -40.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RF1, 0.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LSP, 13.54, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSR, -3.32, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSY, 68.74, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LEB, -70.8, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, -58.79, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWP, 24.53, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LF1, 164.38, postime, MOVE_ABSOLUTE);

//    joint->SetMoveJoint(RSP, 13.54, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RSR, 3.32, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RSY, -68.74, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(REB, -70.8, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RWY, 58.79, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RWP, 24.53, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RF1, -164.38, postime, MOVE_ABSOLUTE);

//    joint->SetMoveJoint(LSP, 35.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LSR, -12.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LEB, -140.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LWY, 0.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LWP, -40.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LF1, 0.0, postime, MOVE_ABSOLUTE);
    sck = freeSleep(2510*1000);
    if(sck==1) return 4;

    return 1;
}

int EM_WalkPushDoor_ReturnHand_LEFT(void){
    // Release Hand
    joint->SetMoveJoint(LF2, 0, 10, MODE_ABSOLUTE);
    usleep(15*1000);

    StartWBIK_Globalmotion(WBmode_GLOBAL);
    freeSleep(10*1000);

    // RE-ROTATE HANDLE
    PushDoor_RotateHandle_LEFT(-1);
    usleep(100*1000);

    // HAND OPEN
    joint->SetMoveJoint(LF2, -125, 10, MODE_ABSOLUTE);
    usleep(4500*1000);
    joint->SetMoveJoint(LF2, 0, 10, MODE_ABSOLUTE);

    // BACK HAND
    doubles temp_return(4);
    for(int k=0; k<4; k++)
        temp_return[k] = _EM_WALKPUSHDOOR_AP_ORI[k];
    pos_sec = 3.;
    WBmotion->addLHPosInfo(_EM_WALKPUSHDOOR_AP_POS[0], _EM_WALKPUSHDOOR_AP_POS[1], (_EM_WALKPUSHDOOR_AP_POS[2]+0.03), pos_sec);
    WBmotion->addLElbPosInfo(5, pos_sec);
    WBmotion->addLHOriInfo(temp_return, pos_sec);
    usleep(3010*1000);

    // GAIN BACK
    joint->SetMoveJoint(LF2, 125, 10, MODE_ABSOLUTE);

    MCBoardSetSwitchingMode(3, 19, 0);//LSY LEB
    MCBoardSetSwitchingMode(3, 20, 0);//LWY LWP
    MCJointGainOverride(3, 19, 1, eq_gain_1000, 2000);//LSY
    MCJointGainOverride(3, 19, 2, eq_gain_1000, 2000);//LEB
    MCJointGainOverride(3, 20, 1, eq_gain_1000, 2000);//LWY
    MCJointGainOverride(3, 20, 2, eq_gain_1000, 2000);//LWP
//    RBBoardSetSwitchingMode(3,19,0);
//    RBBoardSetSwitchingMode(3,20,0);
//    RBJointGainOverride(3,19,1000,1000,2000);//RSY REB
//    RBJointGainOverride(3,20,1000,1000,2000);//RWY RWP
    usleep(4000*1000);
    joint->SetMoveJoint(LF2, 10, 10, MODE_ABSOLUTE);

    // RETURN POS
    TurnOff_WBIK();

    // BACK to WALKREADY
    sharedData->COMMAND[PODO_NO_WALKREADY].USER_PARA_INT[0]=3;
    sharedData->COMMAND[PODO_NO_WALKREADY].USER_COMMAND=WALKREADY_GO_WALKREADYPOS;

    return 1;
}

int EM_WalkPushDoor_ReturnLH_LEFT(void){
    // WBIK set
    StartWBIK_Globalmotion(WBmode_GLOBAL);
    freeSleep(10*1000);

    // Back LH - middle
    doubles temp_ds_H_ori_1(4), temp_ds_H_ori_2(4);
    for(int k=0; k<4; k++){
        temp_ds_H_ori_1[k] = EM_WALKPUSH_LH_ori_1[k];
        temp_ds_H_ori_2[k] = EM_WALKPUSH_LH_ori_2[k];
    }
    pos_sec = 2.;
    WBmotion->addRHPosInfo(EM_WALKPUSH_LH_pos_2[0], EM_WALKPUSH_LH_pos_2[1], EM_WALKPUSH_LH_pos_2[2], pos_sec);
    WBmotion->addRElbPosInfo(-30, pos_sec);
    WBmotion->addRHOriInfo(temp_ds_H_ori_2, pos_sec);
    usleep(2010*1000);

    // Back LH - side
    pos_sec = 2.2;
    WBmotion->addRHPosInfo(EM_WALKPUSH_LH_pos_1[0], EM_WALKPUSH_LH_pos_1[1], EM_WALKPUSH_LH_pos_1[2], pos_sec);
    WBmotion->addRElbPosInfo(-15, pos_sec);
    WBmotion->addRHOriInfo(temp_ds_H_ori_1, pos_sec);

    // LH move pos
    TurnOff_WBIK();

    postime = 3000.;
    joint->SetMoveJoint(RSP, 40.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSR, 10.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(REB, -130.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWP, 20.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RF1, 0., postime, MOVE_ABSOLUTE);
    usleep(3010*1000);
    return 1;
}




// --------------------------------------------------------------------------------------------- //
// --------------------------------------------------------------------------------------------- //
// --------------------------------------------------------------------------------------------- //
int TrapFunction(void)
{
    // Made by Hyoin
    int result =0;
    if(_is_Trap_First==1)
    {
        _Trap_T1=_Trap_T3=_Trap_maxVel/_Trap_maxAcc;
        double tempTriangle=_Trap_T1*_Trap_maxVel;
        if(tempTriangle > _Trap_togo){
            _Trap_T1 = _Trap_T3= sqrtp(_Trap_togo/_Trap_maxAcc);
            _Trap_T2 = 0;
        }else{
            double tempRemain = _Trap_togo - tempTriangle;
            _Trap_T2 = tempRemain / _Trap_maxVel;
        }
        _Trap_T1_count=(unsigned long)(_Trap_T1*200);
        _Trap_T2_count=(unsigned long)(_Trap_T2*200);
        _Trap_T3_count=(unsigned long)(_Trap_T3*200);
        _Trap_gap1=_Trap_T1_count;
        _Trap_gap2=_Trap_T1_count+_Trap_T2_count;
        _Trap_gap3=_Trap_T1_count+_Trap_T2_count+_Trap_T3_count;

        _TrapCount=0;
        _is_Trap_First=0;
    }
    else
    {
        if(_TrapCount<= _Trap_gap1)
            _Trap_GOAL_DELTA = _Trap_GOAL_DELTA+_Trap_maxAcc/200.;
        else if((_TrapCount >_Trap_gap1) && (_TrapCount<=_Trap_gap2))
            _Trap_GOAL_DELTA = _Trap_maxVel;
        else if((_TrapCount >_Trap_gap2) && (_TrapCount<=_Trap_gap3))
            _Trap_GOAL_DELTA = _Trap_GOAL_DELTA-_Trap_maxAcc/200.;
        if(_TrapCount == _Trap_gap3){
            Handle_rotate_done_Flag =true;

            _Trap_GOAL_DELTA =0.;
            _is_Trap_First =1;
            result=1;
            //printf("Trap trajectory end...!!!\n");
            _Trap_FINISH_FLAG = true;
        }
    }

    switch(_Trap_Motion_Command)
    {
    case TR_IDLE:
        break;
    case TR_HANDLE_ROTATE:
        {
        handle_angle_now=handle_angle_now+((double)handle_dir)*_Trap_GOAL_DELTA/200.;

        vec3 vec3_first(handle_pos_first.x, handle_pos_first.y, handle_pos_first.z);
        quat quat_first(handle_pos_first.qw, handle_pos_first.qx, handle_pos_first.qy,handle_pos_first.qz);

        mat3 rotmatrix(handle_rotate_axis,-handle_angle_now*D2R);
        vec3 posoff = rotmatrix*(vec3_first-handle_rotate_center);
        vec3 vec3_now = handle_rotate_center+posoff;
        quat quat_now = quat(rotmatrix)*quat_first;

        doubles temp_q(4);
        for(int k=0; k<4; k++)
            temp_q[k] = quat_now[k];

        if(DOOR_MODE == LEFT_DOOR_MODE){
            WBmotion->addLHPosInfo(vec3_now.x, vec3_now.y, vec3_now.z,0.005);
            WBmotion->addLHOriInfo(temp_q, 0.005);
        }else if(DOOR_MODE == RIGHT_DOOR_MODE){
            WBmotion->addRHPosInfo(vec3_now.x, vec3_now.y, vec3_now.z,0.005);
            WBmotion->addRHOriInfo(temp_q, 0.005);
        }
        if( handle_angle_now == _Trap_togo)
            _Trap_Motion_Command=TR_IDLE;
        }
        break;
    default:
        break;
    }

    return result;
}

void StartWBIKmotion(int _mode)
{
    WB_GLOBAL_FLAG = false;
    usleep(10*1000);
    WB_FLAG = false;
    usleep(10*1000);

    joint->RefreshToCurrentReference();

    WBmotion->myWBIKmode = _mode;

    WBmotion->ResetGlobalCoord(0);//Coord

    WBmotion->StopAll();// Trajectory making ...

    WBmotion->RefreshToCurrentReferenceUB();

    joint->SetAllMotionOwner();

    WB_FLAG = true;
}
void StartWBIK_Globalmotion(int _mode)
{
    WB_FLAG = false;
    usleep(10*1000);
    WB_GLOBAL_FLAG = false;
    usleep(10*1000);

    int reset_coor = -1;
    if(WALK_WST_FLAG == true)
        reset_coor = 0;
    else
        reset_coor = -1;
    WALK_WST_FLAG = false;

    joint->RefreshToCurrentReference();

    WBmotion->myWBIKmode = _mode;

    WBmotion->ResetGlobalCoord(reset_coor);//Coord

    WBmotion->StopAll();// Trajectory making ...

    WBmotion->RefreshToCurrentReference();

    joint->SetAllMotionOwner();

    WB_GLOBAL_FLAG = true;



//    WB_FLAG = false;
//    usleep(10*1000);
//    WB_GLOBAL_FLAG = false;
//    usleep(10*1000);

//    joint->RefreshToCurrentReference();

//    WBmotion->myWBIKmode = _mode;

//    WBmotion->ResetGlobalCoord(0);//Coord

//    WBmotion->StopAll();// Trajectory making ...

//    WBmotion->RefreshToCurrentReference();

//    joint->SetAllMotionOwner();

//    WB_GLOBAL_FLAG = true;
}

void PrintWBIKinfo(void)
{
    printf("--------------------POS INFO------------------------------\n");
    printf("RH pos: %f %f %f\n",WBmotion->pRH_3x1[0], WBmotion->pRH_3x1[1], WBmotion->pRH_3x1[2]);
    printf("LH pos: %f %f %f\n",WBmotion->pLH_3x1[0], WBmotion->pLH_3x1[1], WBmotion->pLH_3x1[2]);
    printf("--------------------ROT INFO--------------------\n");
    printf("RH rot: %f %f %f %f\n",WBmotion->qRH_4x1[0], WBmotion->qRH_4x1[1], WBmotion->qRH_4x1[2], WBmotion->qRH_4x1[3]);
    printf("LH rot: %f %f %f %f\n",WBmotion->qLH_4x1[0], WBmotion->qLH_4x1[1], WBmotion->qLH_4x1[2], WBmotion->qLH_4x1[3]);
    printf("--------------------POS  FOOT-------------------\n");
    printf("Foot R: %f %f %f\n",WBmotion->pRF_3x1[0], WBmotion->pRF_3x1[1], WBmotion->pRF_3x1[2]);
    printf("Foot L: %f %f %f\n",WBmotion->pLF_3x1[0], WBmotion->pLF_3x1[1], WBmotion->pLF_3x1[2]);
    printf("--------------------RELB INFO-------------------\n");
    printf("RI ELB: %f\n",WBmotion->RElb_ang*R2D);
    printf("LE ELB: %f\n",WBmotion->LElb_ang*R2D);
    printf("--------------------COMP INFO-------------------\n");
    printf("COM X : %f\n",WBmotion->pCOM_2x1[0]);
    printf("COM Y : %f\n",WBmotion->pCOM_2x1[1]);
    printf("--------------------PEL XY POS------------------\n");
    printf("PEL Z : %f\n",WBmotion->pPelZ);
}
void GetPosFromENC(void)
{
    WBmotion->Q_enc_34x1[idX] = 0.;
    WBmotion->Q_enc_34x1[idY] = 0.;
    WBmotion->Q_enc_34x1[idZ] = 0.;
    WBmotion->Q_enc_34x1[idQ0] = 0.;
    WBmotion->Q_enc_34x1[idQ1] = 1.;
    WBmotion->Q_enc_34x1[idQ2] = 0.;
    WBmotion->Q_enc_34x1[idQ3] = 0.;

    if(sharedData->ENCODER[MC_ID_CH_Pairs[RKN].id][MC_ID_CH_Pairs[RKN].ch].CurrentPosition != 0)
    {
        for(int i=RHY; i<=LAR; i++){
            WBmotion->Q_enc_34x1[idRHY+i] = sharedData->ENCODER[MC_ID_CH_Pairs[i].id][MC_ID_CH_Pairs[i].ch].CurrentPosition*D2R;
        }
        WBmotion->Q_enc_34x1[idWST] = sharedData->ENCODER[MC_ID_CH_Pairs[WST].id][MC_ID_CH_Pairs[WST].ch].CurrentPosition*D2R;

        WBmotion->Q_enc_34x1[idRSP] = sharedData->ENCODER[MC_ID_CH_Pairs[RSP].id][MC_ID_CH_Pairs[RSP].ch].CurrentPosition*D2R;
        WBmotion->Q_enc_34x1[idRSR] = (sharedData->ENCODER[MC_ID_CH_Pairs[RSR].id][MC_ID_CH_Pairs[RSR].ch].CurrentPosition+OFFSET_RSR)*D2R;
        WBmotion->Q_enc_34x1[idRSY] = sharedData->ENCODER[MC_ID_CH_Pairs[RSY].id][MC_ID_CH_Pairs[RSY].ch].CurrentPosition*D2R;
        WBmotion->Q_enc_34x1[idREB] = (sharedData->ENCODER[MC_ID_CH_Pairs[REB].id][MC_ID_CH_Pairs[REB].ch].CurrentPosition+OFFSET_ELB)*D2R;
        WBmotion->Q_enc_34x1[idRWY] = sharedData->ENCODER[MC_ID_CH_Pairs[RWY].id][MC_ID_CH_Pairs[RWY].ch].CurrentPosition*D2R;
        WBmotion->Q_enc_34x1[idRWP] = sharedData->ENCODER[MC_ID_CH_Pairs[RWP].id][MC_ID_CH_Pairs[RWP].ch].CurrentPosition*D2R;
        WBmotion->Q_enc_34x1[idRWY2] = sharedData->ENCODER[MC_ID_CH_Pairs[RF1].id][MC_ID_CH_Pairs[RF1].ch].CurrentPosition*D2R;

        WBmotion->Q_enc_34x1[idLSP] = sharedData->ENCODER[MC_ID_CH_Pairs[LSP].id][MC_ID_CH_Pairs[LSP].ch].CurrentPosition*D2R;
        WBmotion->Q_enc_34x1[idLSR] = (sharedData->ENCODER[MC_ID_CH_Pairs[LSR].id][MC_ID_CH_Pairs[LSR].ch].CurrentPosition+OFFSET_LSR)*D2R;
        WBmotion->Q_enc_34x1[idLSY] = sharedData->ENCODER[MC_ID_CH_Pairs[LSY].id][MC_ID_CH_Pairs[LSY].ch].CurrentPosition*D2R;
        WBmotion->Q_enc_34x1[idLEB] = (sharedData->ENCODER[MC_ID_CH_Pairs[LEB].id][MC_ID_CH_Pairs[LEB].ch].CurrentPosition+OFFSET_ELB)*D2R;
        WBmotion->Q_enc_34x1[idLWY] = sharedData->ENCODER[MC_ID_CH_Pairs[LWY].id][MC_ID_CH_Pairs[LWY].ch].CurrentPosition*D2R;
        WBmotion->Q_enc_34x1[idLWP] = sharedData->ENCODER[MC_ID_CH_Pairs[LWP].id][MC_ID_CH_Pairs[LWP].ch].CurrentPosition*D2R;
        WBmotion->Q_enc_34x1[idLWY2] = sharedData->ENCODER[MC_ID_CH_Pairs[LF1].id][MC_ID_CH_Pairs[LF1].ch].CurrentPosition*D2R;
    }
    else
    {
//        WBmotion->Q_enc_34x1[0] = 0;
//        WBmotion->Q_enc_34x1[1] = 0;
//        WBmotion->Q_enc_34x1[2] = 0.8;
//        WBmotion->Q_enc_34x1[3] = 1;
//        WBmotion->Q_enc_34x1[4] = 0;
//        WBmotion->Q_enc_34x1[5] = 0;
//        WBmotion->Q_enc_34x1[6] = 0;

        for(int i=RHY; i<=LAR; i++){
            WBmotion->Q_enc_34x1[idRHY+i] = joint->GetJointRefAngle(i)*D2R;
        }
        WBmotion->Q_enc_34x1[idWST] = joint->GetJointRefAngle(WST)*D2R;

        WBmotion->Q_enc_34x1[idRSP] = joint->GetJointRefAngle(RSP)*D2R;
        WBmotion->Q_enc_34x1[idRSR] = (joint->GetJointRefAngle(RSR)+OFFSET_RSR)*D2R;
        WBmotion->Q_enc_34x1[idRSY] = joint->GetJointRefAngle(RSY)*D2R;
        WBmotion->Q_enc_34x1[idREB] = (joint->GetJointRefAngle(REB)+OFFSET_ELB)*D2R;
        WBmotion->Q_enc_34x1[idRWY] = joint->GetJointRefAngle(RWY)*D2R;
        WBmotion->Q_enc_34x1[idRWP] = joint->GetJointRefAngle(RWP)*D2R;
        WBmotion->Q_enc_34x1[idRWY2] = joint->GetJointRefAngle(RF1)*D2R;

        WBmotion->Q_enc_34x1[idLSP] = joint->GetJointRefAngle(LSP)*D2R;
        WBmotion->Q_enc_34x1[idLSR] = (joint->GetJointRefAngle(LSR)+OFFSET_LSR)*D2R;
        WBmotion->Q_enc_34x1[idLSY] = joint->GetJointRefAngle(LSY)*D2R;
        WBmotion->Q_enc_34x1[idLEB] = (joint->GetJointRefAngle(LEB)+OFFSET_ELB)*D2R;
        WBmotion->Q_enc_34x1[idLWY] = joint->GetJointRefAngle(LWY)*D2R;
        WBmotion->Q_enc_34x1[idLWP] = joint->GetJointRefAngle(LWP)*D2R;
        WBmotion->Q_enc_34x1[idLWY2] = joint->GetJointRefAngle(LF1)*D2R;
    }

    WBmotion->enc_FK();
}
int FingerPositionInput_RIGHT(const long des_right)
{
    long cur_enc_right=finger_scale*(sharedData->ENCODER[MC_ID_CH_Pairs[RF2].id][MC_ID_CH_Pairs[RF2].ch].CurrentPosition);

    int move_dir_right;
    if(cur_enc_right<=des_right){
        joint->SetMoveJoint(RF2, 125, 5, MODE_ABSOLUTE);
        move_dir_right=1;//bind
    }else{
        joint->SetMoveJoint(RF2, -125, 5, MODE_ABSOLUTE);
        move_dir_right=-1;//un-bind
    }
    freeSleep(5*1000);
    int wc = 0;
    while(1)
    {
        cur_enc_right=finger_scale*(sharedData->ENCODER[MC_ID_CH_Pairs[RF2].id][MC_ID_CH_Pairs[RF2].ch].CurrentPosition);
        if(move_dir_right>0){
            if(cur_enc_right>=des_right){
                joint->SetMoveJoint(RF2, 0, 5, MODE_ABSOLUTE);
                break;
            }
        }else{
            if(cur_enc_right<=des_right){
                joint->SetMoveJoint(RF2, 0, 5, MODE_ABSOLUTE);
                break;
            }
        }
        freeSleep(50*1000);
        wc++;
        if(wc>120) return wc*50;

    }
    return wc*50;
}
int FingerPositionInput_LEFT(const long des_left)
{
    long cur_enc_left=finger_scale*(sharedData->ENCODER[MC_ID_CH_Pairs[LF2].id][MC_ID_CH_Pairs[LF2].ch].CurrentPosition);

    int move_dir_left;
    if(cur_enc_left<=des_left){
        joint->SetMoveJoint(LF2, 125, 5, MODE_ABSOLUTE);
        move_dir_left=1;//bind
    }else{
        joint->SetMoveJoint(LF2, -125, 5, MODE_ABSOLUTE);
        move_dir_left=-1;//un-bind
    }
    freeSleep(5*1000);
    int wc = 0;
    while(1)
    {
        cur_enc_left=finger_scale*(sharedData->ENCODER[MC_ID_CH_Pairs[LF2].id][MC_ID_CH_Pairs[LF2].ch].CurrentPosition);
        if(move_dir_left>0){
            if(cur_enc_left>=des_left){
                joint->SetMoveJoint(LF2, 0, 5, MODE_ABSOLUTE);
                break;
            }
        }else{
            if(cur_enc_left<=des_left){
                joint->SetMoveJoint(LF2, 0, 5, MODE_ABSOLUTE);
                break;
            }
        }
        freeSleep(50*1000);
        wc++;
        if(wc>120) return wc*50;

    }
    return wc*50;
}
// --------------------------------------------------------------------------------------------- //
void Change_Arm_pos_Task(const double postime, const double dWST)
{
    joint->SetMoveJoint(RSP, -30.50, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSR, -15, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSY, 16.54, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(REB, -113.39, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWY, 4.88, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWP, 72.14, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LSP, -30.50, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSR, 15, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSY, -16.54, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LEB, -113.39, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, -4.88, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWP, 72.14, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(WST, dWST, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(RF1, -26.24, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RF2, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LF1, 26.24, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LF2, 0.0, postime, MOVE_ABSOLUTE);
}
void Change_Arm_pos_Task_with_WST_READY(const double temp_WST, const double tasktime)
{
    joint->SetMoveJoint(RSP, 40, tasktime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSR, -10, tasktime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSY, 16, tasktime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(REB, -135, tasktime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWY, 5, tasktime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWP, 50, tasktime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LSP, 40.0, tasktime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSR, -10.0, tasktime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSY, 0.0, tasktime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LEB, -130.0, tasktime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, 0.0, tasktime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWP, 20.0, tasktime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(WST, temp_WST, tasktime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(RF1, -26.24, tasktime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RF2, 0.0, tasktime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LF1, 0.0, tasktime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LF2, 0.0, tasktime, MOVE_ABSOLUTE);
}
void Change_Arm_pos_Task_with_WST(const double temp_WST, const double tasktime)
{
    joint->SetMoveJoint(RSP, 20, tasktime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSR, -30, tasktime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSY, 16, tasktime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(REB, -130, tasktime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWY, 5, tasktime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWP, 50, tasktime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LSP, 40.0, tasktime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSR, -10.0, tasktime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSY, 0.0, tasktime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LEB, -130.0, tasktime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, 0.0, tasktime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWP, 20.0, tasktime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(WST, temp_WST, tasktime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(RF1, -26.24, tasktime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RF2, 0.0, tasktime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LF1, 0.0, tasktime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LF2, 0.0, tasktime, MOVE_ABSOLUTE);
}
void Change_Arm_pos_Move(const double postime)
{
    joint->SetMoveJoint(RSP, 40.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSR, 17.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(REB, -130, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWY, -10.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWP, 40.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LSP, 40.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSR, -17.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LEB, -130.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, 10.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWP, 40.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(WST, 0.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(RF1, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RF2, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LF1, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LF2, 0.0, postime, MOVE_ABSOLUTE);
}

void Change_Arm_pos_Pass(const double postime)
{
    joint->SetMoveJoint(RSP, 40.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSR, 17.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(REB, -130, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWY, -10.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWP, 40.0, postime, MOVE_ABSOLUTE);

//    joint->SetMoveJoint(LSP, 40.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LSR, -17.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LEB, -130.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LWY, 10.0, postime, MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LWP, 40.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSP, 50.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSR, -15.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LEB, -135.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, 60.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWP, 60.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(WST, 0.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(RF1, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RF2, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LF1, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LF2, 0.0, postime, MOVE_ABSOLUTE);
}

// --------------------------------------------------------------------------------------------- //
int CheckMotionOwned()
{
    for(int i=0;i<(NO_OF_JOINTS-2);i++)
    {
        if(sharedData->MotionOwner[MC_ID_CH_Pairs[i].id][MC_ID_CH_Pairs[i].ch]!=PODO_NO)	return 0;
    }
    return 1;
}
// --------------------------------------------------------------------------------------------- //
void CatchSignals(int _signal)
{
    switch(_signal){
    case SIGHUP:
    case SIGINT:     // Ctrl-c
    case SIGTERM:    // "kill" from shell
    case SIGKILL:
        isTerminated = -1;
        break;
    }
    freeSleep(1000*500);
}
// --------------------------------------------------------------------------------------------- //

int RBInitialize(void)
{
    isTerminated = 0;

    // Core Shared Memory Creation ============================================
    int shmFD = shm_open(RBCORE_SHM_NAME, O_RDWR, 0666);
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
    }FILE_LOG(logSUCCESS) << "Core shared memory creation = OK";
    // =========================================================================

    // User Shared Memory Creation ============================================
    shmFD = shm_open(USER_SHM_NAME, O_RDWR, 0666);
    if(shmFD == -1){
        FILE_LOG(logERROR) << "Fail to open user shared memory";
        return -1;
    }else{
        if(ftruncate(shmFD, sizeof(USER_SHM)) == -1){
            FILE_LOG(logERROR) << "Fail to truncate user shared memory";
            return -1;
        }else{
            userData = (pUSER_SHM)mmap(0, sizeof(USER_SHM), PROT_READ|PROT_WRITE, MAP_SHARED, shmFD, 0);
            if(userData == (void*)-1){
                FILE_LOG(logERROR) << "Fail to mapping user shared memory";
                return -1;
            }
        }
    }
    FILE_LOG(logSUCCESS) << "User shared memory creation = OK";
    // =========================================================================


    // Initialize internal joint classes =======================================
    joint = new JointControlClass(sharedData, PODO_NO);
    joint->RefreshToCurrentReference();
    // =========================================================================


    // Create and start real-time thread =======================================
    if(rt_task_create(&rtFlagCon, "WMupper_AL_FLAG", 0, 95, 0) == 0){
        cpu_set_t aCPU;
        CPU_ZERO(&aCPU);
        CPU_SET(3, &aCPU);
        if(rt_task_set_affinity(&rtFlagCon, &aCPU) != 0){
            FILE_LOG(logWARNING) << "Flag real-time thread set affinity CPU failed..";
        }
        if(rt_task_start(&rtFlagCon, &RBFlagThread, NULL) == 0 ){
            FILE_LOG(logSUCCESS) << "Flag real-time thread start = OK";
        }else{
            FILE_LOG(logERROR) << "Flag real-time thread start = FAIL";
            return -1;
        }
    }else{
        FILE_LOG(logERROR) << "Fail to create Flag real-time thread";
        return -1;
    }

    if(rt_task_create(&rtTaskCon, "WMupper_AL_TASK", 0, 90, 0) == 0){
        cpu_set_t aCPU;
        CPU_ZERO(&aCPU);
        CPU_SET(2, &aCPU);
        if(rt_task_set_affinity(&rtTaskCon, &aCPU) != 0){
            FILE_LOG(logWARNING) << "Task real-time thread set affinity CPU failed..";
        }
        if(rt_task_start(&rtTaskCon, &RBTaskThread, NULL) == 0 ){
            FILE_LOG(logSUCCESS) << "Task real-time thread start = OK";
        }else{
            FILE_LOG(logERROR) << "Task real-time thread start = FAIL";
            return -1;
        }
    }else{
        FILE_LOG(logERROR) << "Fail to create Task real-time thread";
        return -1;
    }
    // =========================================================================

    return 0;
}
//int RBJointGainOverride(unsigned int _canch, unsigned int _bno, unsigned int _override1, unsigned int _override2, unsigned int _duration){
//    // MsgID                Byte0	Byte1	Byte2	Byte3	Byte4		Byte5
//    // CANID_SEND_CMD		BNO		0x6F	OVER1	OVER2	DURATION	DURATION
//    MANUAL_CAN	MCData;

//    MCData.channel = _canch;
//    MCData.id = COMMAND_CANID;
//    MCData.dlc = 6;
//    MCData.data[0] = _bno;
//    MCData.data[1] = 0x6F;
//    MCData.data[2] = _override1;
//    MCData.data[3] = _override2;
//    MCData.data[4] = (_duration & 0xFF);
//    MCData.data[5] = ((_duration>>8) & (0xFF));

//    return PushCANMessage(MCData);
//}

//int RBBoardSetSwitchingMode(unsigned int _canch, unsigned int _bno, int _mode){
//    // MsgID		Byte0	Byte1	Byte2
//    // CMD_TXDF		BNO		0x13	_mode
//    MANUAL_CAN	MCData;

//    MCData.channel = _canch;
//    MCData.id = COMMAND_CANID;
//    MCData.dlc = 3;
//    MCData.data[0] = _bno;
//    MCData.data[1] = 0x13;
//    MCData.data[2] = _mode;

//    return PushCANMessage(MCData);
//}

//int RBenableFrictionCompensation(unsigned int _canch, unsigned int _bno, unsigned int _enable1, unsigned int _enable2){
//    // MsgID		Byte0	Byte1	Byte2
//    // CMD_TXDF		BNO		0xB1	ENABLE
//    MANUAL_CAN	MCData;

//    MCData.channel = _canch;
//    MCData.id = COMMAND_CANID;
//    MCData.dlc = 3;
//    MCData.data[0] = _bno;
//    MCData.data[1] = 0xB1;
//    MCData.data[2] = (_enable1&0x01) | ((_enable2&0x01)<<1);

//    return PushCANMessage(MCData);
//}
//int RBwFTsensorNull(unsigned int _canch, unsigned int _bno){
//    // hand -> 0= Right 1=left 2=both
//    // MsgID		Byte0	Byte1	Byte2
//    // CMD_TXDF		BNO		0x81	_mode
//    MANUAL_CAN	MCData;

//    MCData.channel = _canch;
//    MCData.id = COMMAND_CANID;
//    MCData.dlc = 3;
//    MCData.data[0] = _bno;
//    MCData.data[1] = 0x81;
//    MCData.data[2] = 0x00;//_mode
//    // _mode = 0x00 : FT sensor
//    // _mode = 0x04 : Inclinometers in FT sensor

//    return PushCANMessage(MCData);
//}
//int RBindicator(unsigned char number)
//{
//    MANUAL_CAN	MCData;

//    MCData.channel = 1;
//    MCData.id = COMMAND_CANID;
//    MCData.dlc = 3;
//    MCData.data[0] = 24;
//    MCData.data[1] = 0x84;
//    MCData.data[2] = (unsigned char)number;

//    return PushCANMessage(MCData);
//}

// --------------------------------------------------------------------------------------------- //
//int	PushCANMessage(MANUAL_CAN MCData){
//    for(int i=0; i<MAX_MANUAL_CAN; i++){
//        if(sharedData->ManualCAN[i].status == MANUALCAN_EMPTY){
//            sharedData->ManualCAN[i].status = MANUALCAN_WRITING;
//            sharedData->ManualCAN[i].channel = MCData.channel;
//            sharedData->ManualCAN[i].id = MCData.id;
//            sharedData->ManualCAN[i].dlc = MCData.dlc;
//            for(int j=0; j<MCData.dlc; j++){
//                sharedData->ManualCAN[i].data[j] = MCData.data[j];
//            }
//            sharedData->ManualCAN[i].status = MANUALCAN_NEW;
//            return RB_SUCCESS;
//        }
//    }
//    cout << "Fail to send Manual CAN..!!" << endl;
//    return RB_FAIL;
//}
int HasAnyOwnership(){
    for(int i=0; i<NO_OF_JOINTS; i++){
        if(sharedData->MotionOwner[MC_ID_CH_Pairs[i].id][MC_ID_CH_Pairs[i].ch] == PODO_NO)
            return true;
    }
    return false;
}
int AskALNumber(char *alname){
    memset(&(sharedData->COMMAND[0].USER_PARA_CHAR[0]), 0, 20);
    memcpy(&(sharedData->COMMAND[0].USER_PARA_CHAR[0]), alname, strlen(alname));
    sharedData->COMMAND[0].USER_COMMAND = REQUEST_AL_NUM;
    freeSleep(200*1000);
    if(sharedData->COMMAND[PODO_NO].USER_COMMAND == REQUEST_AL_NUM){
        sharedData->COMMAND[PODO_NO].USER_COMMAND = 100;        // AL NO ACT
        return sharedData->COMMAND[PODO_NO].USER_PARA_INT[0];
    }
    return -1;
}
/**************************************************************************************************
 *File Save
 *************************************************************************************************/
void SaveFile(void)
{
    FILE* fp;
    unsigned int i, j;
    fp = fopen("DoorPULL.txt", "w");

    for(i=0 ; i<saveIndex ; i++)
    {
        for(j=0 ; j<3 ; j++)
            fprintf(fp, "%f\t", DataBuf[j][i]);
        fprintf(fp, "\n");
    }
    fclose(fp);

    saveIndex=0;
    saveFlag=0;
}
int freeSleep(const int usecond)
{
    // EmergencyFlag :: 0=pause 1=resume 2=stop
    // 0 : temporary stop the motion
    // 1 : keep going motion
    // 2 : quit/stop motion

    int sleepCnt = usecond/(50*1000);
    int sleepRm = usecond%(50*1000);
    int result = 0;

    usleep(sleepRm);

    for(int k=0; k<sleepCnt; k++){
        usleep(50*1000);
        if(sharedData->COMMAND[PODO_NO].USER_COMMAND == WMupperbody_AL_ESTOP){
            // stop wheel and finger
            joint->SetMoveJoint(RF2, 0., 10, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LF2, 0., 10, MOVE_ABSOLUTE);
            //usleep(12*1000);
            sharedData->COMMAND[PODO_NO_WHEEL].USER_PARA_CHAR[0] = 0;
            sharedData->COMMAND[PODO_NO_WHEEL].USER_COMMAND = OMNIWHEEL_AL_VELMODE;
            usleep(200*1000);
            sharedData->COMMAND[PODO_NO_WHEEL].USER_COMMAND = OMNIWHEEL_AL_NO_ACT;
            usleep(10*1000);

            // pass motion owner to Daemon
//            sharedData->DaemonTurnOffALNum = PODO_NO;
//            sharedData->DaemonTurnOffFlag = DAEMON_TURNOFF_WORKING;
//            usleep(100*1000);
//            sharedData->DaemonTurnOffALNum = PODO_NO_WHEEL;
//            sharedData->DaemonTurnOffFlag = DAEMON_TURNOFF_WORKING;

//            result = 1;
            break;
        }
    }
    return result;
}
