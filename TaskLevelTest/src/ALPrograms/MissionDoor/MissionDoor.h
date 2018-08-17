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
#ifndef MISSIONDOOR_H
#define MISSIONDOOR_H
//#include <QCoreApplication>

#include "../../../share/Headers/commandlist.h"
#include "joint.h"
#include "taskmotion.h"
#include "ManualCAN.h"
#include "UserSharedMemory.h"

#include <iostream>
#include <libpcan.h>
#include <iostream>
#include <sys/mman.h>
#include <signal.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>

#include <alchemy/task.h>

#define PODO_AL_NAME       "APPROACHBOX_AL"

using namespace std;

inline void pushData(doubles &tar, double var){
    tar.push_back(var);
    tar.pop_front();
}

/***************************** 1. Structs ************************************/
/* Shared memory */
pRBCORE_SHM     sharedData;
pUSER_SHM       userData;

/* RT task handler for control */
TaskMotion      *WBmotion;
RT_TASK rtTaskCon;
RT_TASK rtFlagCon;

enum{
    MODE_NO_ACT = 0, MODE_SITDOWN_START, MODE_SITDOWN_DONE,
     MODE_HOLDBOX_START, MODE_HOLDBOX_DONE, MODE_LIFTBOX_START, MODE_LIFTBOX_DONE,
     MODE_STANDUP_START, MODE_DONE
};

/***************************** 2. Flags **************************************/
int     isTerminated;
int     __IS_WORKING;
int     __IS_GAZEBO;

/* Command */
double     OnOff_compliance = false;
double     OnOff_zmpcontrol = false;
double     FLAG_pushdoor = false;
double     FLAG_leaned = false;

/***************************** 3. Variables **********************************/
/* Basic */
int PODO_NO;
int PODO_NO_DAEMON = 0;
int PODO_NO_WALKREADY;
char __AL_NAME[30];

/* WBIK */
int             WB_FLAG = false;
long            LimitedJoint;
long            LimitType;
double _FOOT_CENT_X;

/* Sensor */
float LPF_RH_Fz = 0, LPF_RH_Fx = 0, LPF_RH_Fy = 0;
float LPF_LH_Fz = 0, LPF_LH_Fx = 0, LPF_LH_Fy = 0;
float Before_RH_Fz = 0, Before_RH_Fx = 0, Before_RH_Fy = 0;
float Before_LH_Fz = 0, Before_LH_Fx = 0, Before_LH_Fy = 0;

float LPFLPF_RH_Fz = 0, LPFLPF_LH_Fz = 0, Before_RH_Fz2 = 0, Before_LH_Fz2 = 0;

float Desired_Force_X[2]={0,};
float Desired_Force_Y[2]={0,};
float Desired_Force_Z[2]={0,};
float Measured_Force_X[2]={0,};
float Measured_Force_Y[2]={0,};
float Measured_Force_Z[2]={0,};
float VelX[2],VelY[2],VelZ[2];
float PosX[2], PosY[2], PosZ[2];
double ZMPControlX;
double ZMPControlY;

/* Motion */
int     LeanedFrontCOUNT=0;
int     LeanedBackCOUNT=0;
double COMpos, InitCOMpos;
double RHpos[3], LHpos[3];

/* Kirk variables */
double  X_ZMP_Local,Y_ZMP_Local,X_ZMP_Global,Y_ZMP_Global,X_ZMP_REF_Local,Y_ZMP_REF_Local,X_ZMP_REF_Global,Y_ZMP_REF_Global;
double  X_ZMP_IMU = 0., Y_ZMP_IMU = 0.;
double  X_ZMP_IMU_n = 0., Y_ZMP_IMU_n = 0.;
double  X_ZMP = 0., Y_ZMP = 0., X_ZMP_LF = 0., Y_ZMP_LF = 0., Old_X_ZMP_LF = 0., Old_Y_ZMP_LF = 0.;
double  X_ZMP_n = 0., Y_ZMP_n = 0.;
double  X_ZMP_LPF = 0., Y_ZMP_LPF = 0.;
double  final_gain_DSP_ZMP_CON = 0., final_gain_SSP_ZMP_CON = 0.;
double  Del_PC_X_DSP_XZMP_CON = 0., Del_PC_Y_DSP_YZMP_CON = 0., Old_Del_PC_X_DSP_XZMP_CON = 0., Old_Del_PC_Y_DSP_YZMP_CON = 0.,
        Del_PC_X_SSP_XZMP_CON = 0., Del_PC_Y_SSP_YZMP_CON = 0., Old_Del_PC_X_SSP_XZMP_CON = 0., Old_Del_PC_Y_SSP_YZMP_CON = 0.;
double  LPF_Del_PC_X_DSP_XZMP_CON = 0., LPF_Del_PC_Y_DSP_YZMP_CON = 0.;
double  Del_PC_X_DSP_XZMP_CON_n = 0.,Del_PC_Y_DSP_YZMP_CON_n = 0.;
double  LPF_Del_PC_X_SSP_XZMP_CON = 0., LPF_Del_PC_Y_SSP_YZMP_CON = 0.;
double  Old_Del_PC_X_DSP_XZMP_CON2 = 0;
double  Old_Del_PC_Y_DSP_YZMP_CON2 = 0;
double  Old_Del_PC_X_SSP_XZMP_CON_2 = 0., Old_Del_PC_Y_SSP_YZMP_CON_2 = 0.;
unsigned int CNT_final_gain_DSP_ZMP_CON = 0,  CNT_final_gain_SSP_ZMP_CON = 0;
unsigned int CNT_SSP_ZMP_CON = 0;

double I_ZMP_CON_X=0.f,I_ZMP_CON_Y=0.f;
double I_ZMP_CON_X_last=0.f,I_ZMP_CON_Y_last=0.f;
double Old_I_ZMP_CON_X=0.f,Old_I_ZMP_CON_Y=0.f;
double Global[3],Local[3];
const double DEL_T = 0.005;

// ZMP 읽기 //
double M_LF[3],M_RF[3],M_LF_Global[3],M_RF_Global[3],F_LF[3],F_RF[3],F_LF_Global[3],F_RF_Global[3];
double pCenter[3],qCenter[4],qCenter_bar[4];
double zmp[3],zmp_local[3],zmp_ref_local[3];

/* Data Save Variable */
#define ROW 20000
#define COL 30
int     Save_Index;
double  Save_Data[COL][ROW];
FILE *fp;
FILE *fp2;
FILE *fp3;
FILE *fp4;

/***************************** 4. Functions **********************************/
/* Basic */
int HasAnyOwnership();
int CheckMotionOwned();
void RBTaskThread(void *);
void RBFlagThread(void *);
void CatchSignals(int _signal);
void SendDatatoGUI();
void save();

/* Initialization */
int RBInitialize(void);
void ShutDownAllFlag();
void StartWBIKmotion(int _mode);

/* Motion */
void StartPushDoor();
void LeanedFORWARD();
void LeanedFORWARD(int cnt);
void LeanedBACKWARD(int cnt);

/* Control */
void StartComplianceControl();
void ZMPControl();

/* Kirk Controllers */
void Kirk_Control();
double  kirkZMPCon_XP2(double u, double ZMP, int zero);
double  kirkZMPCon_YP2(double u, double ZMP, int zero);
void get_zmp2();
void ZMP_intergral_control();
void Local2Global(double _Local[],double _Global[]);
#endif // MISSIONDOOR_H
