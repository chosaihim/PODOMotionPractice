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
#ifndef LIFTBOX_H
#define LIFTBOX_H
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
int     Command_LIFTBOX = MODE_NO_ACT;


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

/* Motion */
double SITDOWN_TIME;
double HOLDBOX_TIME;
double LIFTBOX_TIME;
double STANDUP_TIME;

double HandFTz;

double HandPos_x;
double HandPos_y;
double HandPos_z;
/* WalkReady Save */
double WalkReady_LHPos[3];
double WalkReady_RHPos[3];

double WalkReady_LHOri[4];
double WalkReady_RHOri[4];

double WalkReady_PelPos[3];
double WalkReady_PelOri[4];

double HoldBox_LHPos;

/***************************** 4. Functions **********************************/
/* Basic */
int HasAnyOwnership();
int CheckMotionOwned();
void RBTaskThread(void *);
void RBFlagThread(void *);
void CatchSignals(int _signal);
void ShowWBInfos();

/* Initialization */
int RBInitialize(void);
void ShutDownAllFlag();
void SaveWalkReadyPos();
void StartWBIKmotion(int _mode);

/* Motion */
void LiftBox_Supervisor();
void SitDown();
void HoldBox();
void LiftBox();
void StandUp();


#endif // LIFTBOX_H
