//#include "liftbox.h"
#include "handsup.h"
// --------------------------------------------------------------------------------------------- //

JointControlClass *joint;
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
    // Termination signal ---------------------------------
    signal(SIGTERM, CatchSignals);   // "kill" from shell
    signal(SIGINT, CatchSignals);    // Ctrl-c
    signal(SIGHUP, CatchSignals);    // shell termination
    signal(SIGKILL, CatchSignals);
    signal(SIGSEGV, CatchSignals);

    // Block memory swapping ------------------------------
    mlockall(MCL_CURRENT|MCL_FUTURE);

    sprintf(__AL_NAME, "HandsUp");
    CheckArguments(argc, argv);

    if(PODO_NO == -1){
        FILE_LOG(logERROR) << "Please check the AL Number";
        FILE_LOG(logERROR) << "Terminate this AL..";
        return 0;
    }

    // Initialize RBCore -----------------------------------
    if(RBInitialize() == -1 )
        isTerminated = -1;


    // WBIK Initialize--------------------------------------
    WBmotion = new TaskMotion(sharedData, joint);

    // User command cheking --------------------------------
    while(isTerminated == 0){
        usleep(100*1000);
        switch(sharedData->COMMAND[PODO_NO].USER_COMMAND)
        {
        case HANDSUP_LEFT:
        {
            FILE_LOG(logSUCCESS) << "1st command HANDSUP_LEFT received..\n";

            ShutDownAllFlag();
            joint->RefreshToCurrentReference();
//            StartWBIKmotion(-1);
            StartWBIKmotion(-1);
            joint->SetAllMotionOwner();

            FILE_LOG(logSUCCESS) << "2nd command HANDSUP_LEFT received..\n";
            Command_LIFTBOX = MODE_SITDOWN_START;

            LeftHand();
            sharedData->COMMAND[PODO_NO].USER_COMMAND = LIFTBOX_NO_ACT;
            break;
        }
        case HANDSUP_WHOLEBODY:
        {
            FILE_LOG(logSUCCESS) << "1st command HANDSUP_WHOLEBODY received..\n";

            ShutDownAllFlag();
            joint->RefreshToCurrentReference();
            StartWBIKmotion(-1);
            joint->SetAllMotionOwner();

            FILE_LOG(logSUCCESS) << "2nd command HANDSUP_WHOLEBODY received..\n";
            Command_LIFTBOX = MODE_SITDOWN_START;

            WholeBody();
            sharedData->COMMAND[PODO_NO].USER_COMMAND = LIFTBOX_NO_ACT;
            break;
        }
        case HANDSUP_HOW:
        {
            FILE_LOG(logSUCCESS) << "1st command HANDSUP_HOW received..\n";

            ShutDownAllFlag();
            joint->RefreshToCurrentReference();
            StartWBIKmotion(-1);
            joint->SetAllMotionOwner();

            FILE_LOG(logSUCCESS) << "2nd command HANDSUP_HOW received..\n";
            Command_LIFTBOX = MODE_SITDOWN_START;

            HowMotion();
            sharedData->COMMAND[PODO_NO].USER_COMMAND = LIFTBOX_NO_ACT;
            break;
        }
        case HANDSUP_WALKREADY:
        {
            FILE_LOG(logSUCCESS) << "1st command HANDSUP_WALKREADY received..\n";

            ShutDownAllFlag();
            joint->RefreshToCurrentReference();
            StartWBIKmotion(-1);
            joint->SetAllMotionOwner();

            FILE_LOG(logSUCCESS) << "2nd command HANDSUP_WALKREADY received..\n";
            Command_LIFTBOX = MODE_SITDOWN_START;

            WalkReady();
            sharedData->COMMAND[PODO_NO].USER_COMMAND = LIFTBOX_NO_ACT;
            break;
        }
        case LIFTBOX_SIT_DOWN:
        {
            FILE_LOG(logSUCCESS) << "11Command LIFTBOX_SIT_DOWN received..\n";

            ShutDownAllFlag();
            joint->RefreshToCurrentReference();
            StartWBIKmotion(-1);
            joint->SetAllMotionOwner();
            FILE_LOG(logSUCCESS) << "22Command LIFTBOX_SIT_DOWN received..\n";
            Command_LIFTBOX = MODE_SITDOWN_START;
            SitDown();
            sharedData->COMMAND[PODO_NO].USER_COMMAND = LIFTBOX_NO_ACT;
            break;
        }
        case LIFTBOX_HOLD_BOX:
        {
            FILE_LOG(logSUCCESS) << "11Command LIFTBOX_HOLD_BOX received..\n";

            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();
            Command_LIFTBOX = MODE_HOLDBOX_START;
            HoldBox();
            sharedData->COMMAND[PODO_NO].USER_COMMAND = LIFTBOX_NO_ACT;
            break;
        }
        case LIFTBOX_LIFT_BOX:
        {
            FILE_LOG(logSUCCESS) << "11Command LIFTBOX_LIFT_BOX received..\n";
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();
            Command_LIFTBOX = MODE_LIFTBOX_START;
            LiftBox();
            FILE_LOG(logSUCCESS) << "22Command LIFTBOX_LIFT_BOX received..\n";
            sharedData->COMMAND[PODO_NO].USER_COMMAND = LIFTBOX_NO_ACT;
            break;
        }
        case LIFTBOX_STAND_UP:
        {
            FILE_LOG(logSUCCESS) << "11Command LIFTBOX_STAND_UP received..\n";
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();
            StandUp();
            FILE_LOG(logSUCCESS) << "22Command LIFTBOX_STAND_UP received..\n";
            Command_LIFTBOX = MODE_STANDUP_START;
            sharedData->COMMAND[PODO_NO].USER_COMMAND = LIFTBOX_NO_ACT;
            break;
        }
        default:{
            sharedData->COMMAND[PODO_NO].USER_COMMAND = LIFTBOX_NO_ACT;
            break;}
        }
    }
    cout << ">>> Process LiftBox is terminated..!!" << endl;
    return 0;
}

void RBTaskThread(void *)
{
     while(isTerminated == 0)
    {

        LiftBox_Supervisor();
        if(WB_FLAG == true)
        {
            // Global whole body model
            WBmotion->updateAll();
            WBmotion->WBIK();

            for(int i=RHY; i<=LAR; i++)
            {
                joint->SetJointRefAngle(i, WBmotion->Q_filt_34x1[idRHY+i-RHY]*R2D);
            }
            joint->SetJointRefAngle(WST, WBmotion->Q_filt_34x1[idWST]*R2D);

            joint->SetJointRefAngle(RSP, WBmotion->Q_filt_34x1[idRSP]*R2D);
            joint->SetJointRefAngle(RSR, WBmotion->Q_filt_34x1[idRSR]*R2D - OFFSET_RSR);
            joint->SetJointRefAngle(RSY, WBmotion->Q_filt_34x1[idRSY]*R2D);
            joint->SetJointRefAngle(REB, WBmotion->Q_filt_34x1[idREB]*R2D - OFFSET_ELB);

            joint->SetJointRefAngle(LSP, WBmotion->Q_filt_34x1[idLSP]*R2D);
            joint->SetJointRefAngle(LSR, WBmotion->Q_filt_34x1[idLSR]*R2D - OFFSET_LSR);
            joint->SetJointRefAngle(LSY, WBmotion->Q_filt_34x1[idLSY]*R2D);
            joint->SetJointRefAngle(LEB, WBmotion->Q_filt_34x1[idLEB]*R2D - OFFSET_ELB);

            if(Command_LIFTBOX == LIFTBOX_NO_ACT)
            {
                joint->SetJointRefAngle(RWY, WBmotion->Q_filt_34x1[idRWY]*R2D);
                joint->SetJointRefAngle(RWP, WBmotion->Q_filt_34x1[idRWP]*R2D);
                joint->SetJointRefAngle(RWY2, WBmotion->Q_filt_34x1[idRWY2]*R2D);
                joint->SetJointRefAngle(LWY, WBmotion->Q_filt_34x1[idLWY]*R2D);
                joint->SetJointRefAngle(LWP, WBmotion->Q_filt_34x1[idLWP]*R2D);
                joint->SetJointRefAngle(LWY2, WBmotion->Q_filt_34x1[idLWY2]*R2D);
            } else
            {
//                joint->SetJointRefAngle(RWY, WBmotion->Q_filt_34x1[idRWY]*R2D);
//                joint->SetJointRefAngle(RWP, WBmotion->Q_filt_34x1[idRWP]*R2D);
//                joint->SetJointRefAngle(RWY2, WBmotion->Q_filt_34x1[idRWY2]*R2D);
//                joint->SetJointRefAngle(LWY, WBmotion->Q_filt_34x1[idLWY]*R2D);
//                joint->SetJointRefAngle(LWP, WBmotion->Q_filt_34x1[idLWP]*R2D);
//                joint->SetJointRefAngle(LWY2, WBmotion->Q_filt_34x1[idLWY2]*R2D);

            }

            if(!CheckMotionOwned())
                WB_FLAG = false;
        }

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


        if(sharedData->SYNC_SIGNAL[PODO_NO] == true){
            joint->JointUpdate();
            rt_task_resume(&rtTaskCon);
        }

    }
}

// --------------------------------------------------------------------------------------------- //
int CheckMotionOwned()
{
    for(int i=0;i<NO_OF_JOINTS;i++)
    {
        if(sharedData->MotionOwner[MC_ID_CH_Pairs[i].id][MC_ID_CH_Pairs[i].ch]!=PODO_NO)	return 0;
    }
    return 1;
}
int HasAnyOwnership(){
    for(int i=0; i<NO_OF_JOINTS; i++){
        if(sharedData->MotionOwner[MC_ID_CH_Pairs[i].id][MC_ID_CH_Pairs[i].ch] == PODO_NO)
            return true;
    }
    return false;
}
// --------------------------------------------------------------------------------------------- //
void CatchSignals(int _signal)
{
    switch(_signal){
    case SIGHUP:
    case SIGINT:     // Ctrl-c
    case SIGTERM:    // "kill" from shell
    case SIGKILL:
    case SIGSEGV:
        isTerminated = -1;
        break;
    }
    usleep(1000*500);
}
// --------------------------------------------------------------------------------------------- //

// --------------------------------------------------------------------------------------------- //
int RBInitialize(void)
{
    // Block program termination
    isTerminated = 0;

    char task_thread_name[30];
    char flag_thread_name[30];
    sprintf(task_thread_name, "%s_TASK", __AL_NAME);
    sprintf(flag_thread_name, "%s_FLAG", __AL_NAME);

    int shmFD;
    // Core Shared Memory Creation [Reference]==================================
    shmFD = shm_open(RBCORE_SHM_NAME, O_CREAT|O_RDWR, 0666);
    if(shmFD == -1){
        FILE_LOG(logERROR) << "Fail to open core shared memory";
        return false;
    }else{
        if(ftruncate(shmFD, sizeof(RBCORE_SHM)) == -1){
            FILE_LOG(logERROR) << "Fail to truncate core shared memory";
            return false;
        }else{
            sharedData = (pRBCORE_SHM)mmap(0, sizeof(RBCORE_SHM), PROT_READ|PROT_WRITE, MAP_SHARED, shmFD, 0);
            if(sharedData == (void*)-1){
                FILE_LOG(logERROR) << "Fail to mapping core shared memory";
                return false;
            }
        }
    }
    FILE_LOG(logSUCCESS) << "Core shared memory creation = OK";
    // =========================================================================


    // User Shared Memory Creation ============================================
    shmFD = shm_open(USER_SHM_NAME, O_CREAT|O_RDWR, 0666);
    if(shmFD == -1){
        FILE_LOG(logERROR) << "Fail to open user shared memory";
        return false;
    }else{
        if(ftruncate(shmFD, sizeof(USER_SHM)) == -1){
            FILE_LOG(logERROR) << "Fail to truncate user shared memory";
            return false;
        }else{
            userData = (pUSER_SHM)mmap(0, sizeof(USER_SHM), PROT_READ|PROT_WRITE, MAP_SHARED, shmFD, 0);
            if(userData == (void*)-1){
                FILE_LOG(logERROR) << "Fail to mapping user shared memory";
                return false;
            }
        }
        memset(userData, 0, sizeof(USER_SHM));
    }
    FILE_LOG(logSUCCESS) << "User shared memory creation = OK";
    // =========================================================================


    // Initialize internal joint classes =======================================
    joint = new JointControlClass(sharedData, PODO_NO);
    joint->RefreshToCurrentReference();
    // =========================================================================


    // Create and start real-time thread =======================================
    if(rt_task_create(&rtFlagCon, flag_thread_name, 0, 95, 0) == 0){
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

    if(rt_task_create(&rtTaskCon, task_thread_name, 0, 90, 0) == 0){
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
void StartWBIKmotion(int _mode)
{
    WB_FLAG = false;
    usleep(10*1000);

    joint->RefreshToCurrentReference();

    // RF_or_LF: 1=LF, -1=RF, 0=PC
    WBmotion->ResetGlobalCoord(_mode);

    WBmotion->StopAll();

    WBmotion->RefreshToCurrentReference();

    joint->SetAllMotionOwner();

    WB_FLAG = true;
}

void ShutDownAllFlag()
{
    Command_LIFTBOX = MODE_NO_ACT;
    WB_FLAG = false;
}

void ShowWBInfos()
{
    printf("======================= WBInfos =======================\n");
    printf("LHPos = (%f, %f, %f)\n",WBmotion->pLH_3x1[0],WBmotion->pLH_3x1[1],WBmotion->pLH_3x1[2]);
    printf("LHOri = (%f, %f, %f, %f)\n",WBmotion->qLH_4x1[0],WBmotion->qLH_4x1[1],WBmotion->qLH_4x1[2],WBmotion->qLH_4x1[3]);
    printf("RHPos = (%f, %f, %f)\n",WBmotion->pRH_3x1[0],WBmotion->pRH_3x1[1],WBmotion->pRH_3x1[2]);
    printf("RHOri = (%f, %f, %f, %f)\n",WBmotion->qRH_4x1[0],WBmotion->qRH_4x1[1],WBmotion->qRH_4x1[2],WBmotion->qRH_4x1[3]);
    printf("PelPos = (%f, %f, %f)\n",WBmotion->pCOM_2x1[0],WBmotion->pCOM_2x1[1],WBmotion->pPelZ);
    printf("PelOri = (%f, %f, %f, %f)\n",WBmotion->qPEL_4x1[0],WBmotion->qPEL_4x1[1],WBmotion->qPEL_4x1[2],WBmotion->qPEL_4x1[3]);
    printf("=======================================================\n\n");
}

void LiftBox_Supervisor()
{
    if(Command_LIFTBOX == MODE_SITDOWN_START)
    {
//        SitDown();
        TRInfo tempInfo;

    }
//    if(Command_LIFTBOX == MODE_SITDOWN_DONE)
//    {
//        static int sleepcnt = 0;
//        sleepcnt++;

//        if(sleepcnt > SITDOWN_TIME*200)
//        {
//            sleepcnt = 0;
//            Command_LIFTBOX = MODE_HOLDBOX_START;
//        }
//    }
    if(Command_LIFTBOX == MODE_HOLDBOX_START)
    {
        HoldBox();
    }

    else if(Command_LIFTBOX == MODE_LIFTBOX_START)
        LiftBox();
//    else if(Command_LIFTBOX == MODE_LIFTBOX_DONE)
//    {
//        static int sleepcnt = 0;
//        sleepcnt++;

//        if(sleepcnt > LIFTBOX_TIME*200)
//        {
//            sleepcnt = 0;
//            Command_LIFTBOX = MODE_STANDUP_START;
//        }
//    }
    else if(Command_LIFTBOX == MODE_STANDUP_START)
        StandUp();
}

void SitDown()
{
    /*
     * This function implement a motion in which the robot sit down
     * and extend the arm after approaches the box.
     * Keeping the COM at the center of the foot,
     * lower the height of the pelvis and move the hand to the position to hold the box.
     */

    printf("SitDown start..\n");
    ShowWBInfos();
    /*
     * StartWBIK (WB_FLAG = true)
     * Global = RF (RF = -1, LF = 1, PC = 0)
     */
    //StartWBIKmotion(-1);
    SaveWalkReadyPos();

    double _FOOT_CENT_X = (WBmotion->pRF_3x1[0] + WBmotion->pLF_3x1[0])/2.;
    //WBmotion->addCOMInfo(_FOOT_CENT_X+0.02, 0., 3);


    quat qt_q_pelv=quat(vec3(0,1,0),60*D2R);
    doubles ds_q_pelv(4);
    for(int k=0;k<4;k++)
        ds_q_pelv[k]=qt_q_pelv[k];

    SITDOWN_TIME = 3.;
    HandPos_x = 0.45;
    HandPos_y = 0.4;
    HandPos_z = 0.4;
    usleep(100);
    WBmotion->addPELPosInfo(0.8, SITDOWN_TIME);
    printf("SHOULDER2WST = %f\n",WBmotion->kine_drc.L_WST2SHOULDER);
    printf("Arm = %f\n",WBmotion->kine_drc.L_UPPER_ARM+WBmotion->kine_drc.L_LOWER_ARM);
    WBmotion->addPELOriInfo(ds_q_pelv, SITDOWN_TIME);

    WBmotion->addLHPosInfo(HandPos_x,HandPos_y,HandPos_z,SITDOWN_TIME);
    WBmotion->addRHPosInfo(HandPos_x,-HandPos_y,HandPos_z,SITDOWN_TIME);

    quat qt_q_hand = quat(vec3(0,1,0),-40*D2R);
    doubles ds_q_rhand(4), ds_q_lhand(4);
    ds_q_lhand[0] = qt_q_hand[0];
    ds_q_lhand[1] = qt_q_hand[1];
    ds_q_lhand[2] = qt_q_hand[2];
    ds_q_lhand[3] = qt_q_hand[3];

    ds_q_rhand[0] = qt_q_hand[0];
    ds_q_rhand[1] = -qt_q_hand[1]; // Right hand rotates in the opposite direction to left hand.
    ds_q_rhand[2] = qt_q_hand[2];
    ds_q_rhand[3] = qt_q_hand[3];


    WBmotion->addLHOriInfo(ds_q_lhand,SITDOWN_TIME);
    WBmotion->addRHOriInfo(ds_q_rhand,SITDOWN_TIME);

    joint->SetMoveJoint(RWY, 180, 1000,MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, 180, 1000,MOVE_ABSOLUTE);
    printf("Sit down done .. \n");
    Command_LIFTBOX = MODE_SITDOWN_DONE;
}

void HoldBox()
{
    /*
     * This function implements a motion in which the robot
     * holds the box in consideration of friction and force.
     */

    //ShowWBInfos();
    printf("HoldBox ");

    static double holdbox_lhand = 0.4;
    static double holdbox_rhand = -0.4;
    static int cnt = 0;

    holdbox_lhand -= 0.0001;
    holdbox_rhand += 0.0001;


    WBmotion->addLHPosInfo(HandPos_x,holdbox_lhand, HandPos_z, 0.005);
    WBmotion->addRHPosInfo(HandPos_x,holdbox_rhand, HandPos_z, 0.005);
    cnt++;

    HandFTz = 0.2; //Any number(?)
    if(sharedData->FT[2].Fz > 80)
    {
        HOLDBOX_TIME = cnt*0.005;
        cnt = 0;
        HoldBox_LHPos = holdbox_lhand;
        printf("HoldBox done .. t = %f\n",HOLDBOX_TIME);
        Command_LIFTBOX = MODE_HOLDBOX_DONE;
        holdbox_lhand = 0.3;
        holdbox_rhand = -0.3;

    }
    else if(cnt > 800)
    {
        HOLDBOX_TIME = cnt*0.005;
        cnt = 0;
        printf("HoldBox can't done!!!!!!!!!\n");
        HoldBox_LHPos = holdbox_lhand;
        Command_LIFTBOX = MODE_HOLDBOX_DONE;
        holdbox_lhand = 0.3;
        holdbox_rhand = -0.3;
    }
}

void LiftBox()
{
    /*
     * This function implements a motion in which the robot
     * lift the box after holding box.
     */

    printf("LiftBox start..\n");

    LIFTBOX_TIME = 3.;
    WBmotion->addLHPosInfo(HandPos_x,HoldBox_LHPos,HandPos_z+0.3, LIFTBOX_TIME);
    WBmotion->addRHPosInfo(HandPos_x,-HoldBox_LHPos,HandPos_z+0.3, LIFTBOX_TIME);

    printf("LiftBox done...\n");
    Command_LIFTBOX = MODE_LIFTBOX_DONE;

}

void StandUp()
{
    /*
     * This function implements a motion in which the robot
     * stand up to walk.
     */
    printf("Stand up..\n");

    double _FOOT_CENT_X = (WBmotion->pRF_3x1[0] + WBmotion->pLF_3x1[0]/2.);
    WBmotion->addCOMInfo(_FOOT_CENT_X+0.02, 0., 3);

    quat qt_q_pelv=quat(vec3(0,1,0),0*D2R);
    doubles ds_q_pelv(4);
    for(int k=0;k<4;k++)
        ds_q_pelv[k]=qt_q_pelv[k];

    usleep(100);
    WBmotion->addPELPosInfo(WalkReady_PelPos[2], 3);
    WBmotion->addPELOriInfo(ds_q_pelv, 3);

    WBmotion->addLHPosInfo(WalkReady_LHPos[0]+0.1,HoldBox_LHPos,WalkReady_LHPos[2],3);
    WBmotion->addRHPosInfo(WalkReady_RHPos[0]+0.1,-HoldBox_LHPos,WalkReady_RHPos[2],3);

    printf("StandUp done...\n");
    Command_LIFTBOX = MODE_DONE;
}

void SaveWalkReadyPos()
{
    /*
     * This Function save the data of WBmotion at WalkReady Pos..
     * We can use this when we need to go back to WalkReady Pos
     */
    for(int i=0;i<3;i++)
    {
        /* HandPos Save */
        WalkReady_LHPos[i] = WBmotion->pLH_3x1[i];
        WalkReady_RHPos[i] = WBmotion->pRH_3x1[i];

    }
    for(int i=0;i<4;i++)
    {
        /* HandOri Save */
//        WBmotion->qLH_4x1[i] = WBmotion->qLH_4x1[i];
        WalkReady_LHOri[i] = WBmotion->qLH_4x1[i];
        WalkReady_RHOri[i] = WBmotion->qRH_4x1[i];
        /* PelOri Save */
        WalkReady_PelOri[i] = WBmotion->qPEL_4x1[i];
    }

    /* PelPos Save */
    WalkReady_PelPos[0] = WBmotion->pCOM_2x1[0];
    WalkReady_PelPos[1] = WBmotion->pCOM_2x1[1];
    WalkReady_PelPos[2] = WBmotion->pPelZ;

    printf("==================WalkReady Save ====================\n");
    printf("LHPos = (%f, %f, %f)\n",WalkReady_LHPos[0],WalkReady_LHPos[1],WalkReady_LHPos[2]);
    printf("LHOri = (%f, %f, %f, %f)\n",WBmotion->qLH_4x1[0],WBmotion->qLH_4x1[1],WBmotion->qLH_4x1[2]);
    printf("RHPos = (%f, %f, %f)\n",WalkReady_RHPos[0],WalkReady_RHPos[1],WalkReady_RHPos[2]);
    printf("RHOri = (%f, %f, %f, %f)\n",WalkReady_RHOri[0],WalkReady_RHOri[1],WalkReady_RHOri[2]);
    printf("PelPos = (%f, %f, %f)\n",WalkReady_PelPos[0],WalkReady_PelPos[1],WalkReady_PelPos[2]);
    printf("PelOri = (%f, %f, %f, %f)\n",WalkReady_PelOri[0],WalkReady_PelOri[1],WalkReady_PelOri[2]);
    printf("======================================================\n\n");

}

void LeftHand()
{
    printf("LeftHand start..\n");
    ShowWBInfos();

    /*
     * StartWBIK (WB_FLAG = true)
     * Global = RF (RF = -1, LF = 1, PC = 0)
     */
    //StartWBIKmotion(-1);
    SaveWalkReadyPos();

    double _FOOT_CENT_X = (WBmotion->pRF_3x1[0] + WBmotion->pLF_3x1[0])/2.;

//    quat qt_q_pelv=quat(vec3(0,1,0),60*D2R);
    quat qt_q_pelv=quat(vec3(0,0,0),0*D2R);
    doubles ds_q_pelv(4);
    for(int k=0;k<4;k++)
        ds_q_pelv[k]=qt_q_pelv[k];

//    SITDOWN_TIME = 3.;
//    HandPos_x = 0.45;
//    HandPos_y = 0.4;
//    HandPos_z = 0.4;

    SITDOWN_TIME = 5.;
    //variables for hand position when both hands are symetrically positioned
    HandPos_x = 0.4;
    HandPos_y = 0.4;
    HandPos_z = 1;
    //variables for each hand
    double HandPos_lx = 0.5, HandPos_rx=0.1;
    double HandPos_ly = 0.3, HandPos_ry=-0.3;
    double HandPos_lz = 1.5, HandPos_rz=0.1;

    usleep(100);

    WBmotion->addPELPosInfo(1, SITDOWN_TIME);
    WBmotion->addLHPosInfo(HandPos_lx,HandPos_ly,HandPos_lz,SITDOWN_TIME);
    WBmotion->addRHPosInfo(HandPos_rx,HandPos_ry,HandPos_rz,SITDOWN_TIME);
//    WBmotion->addLHPosInfo(HandPos_x,HandPos_y,HandPos_z,SITDOWN_TIME);
//    WBmotion->addRHPosInfo(HandPos_x,-HandPos_y,HandPos_z,SITDOWN_TIME);
    printf("SHOULDER2WST = %f\n",WBmotion->kine_drc.L_WST2SHOULDER);
    printf("Arm = %f\n",WBmotion->kine_drc.L_UPPER_ARM + WBmotion->kine_drc.L_LOWER_ARM);


    quat qt_q_hand = quat(vec3(0,1,0),-60*D2R);
    doubles ds_q_rhand(4), ds_q_lhand(4);
    ds_q_lhand[0] = qt_q_hand[0];
    ds_q_lhand[1] = qt_q_hand[1];
    ds_q_lhand[2] = qt_q_hand[2];
    ds_q_lhand[3] = qt_q_hand[3];

    ds_q_rhand[0] = qt_q_hand[0];
    ds_q_rhand[1] =-qt_q_hand[1]; // Right hand rotates in the opposite direction to left hand.
    ds_q_rhand[2] = qt_q_hand[2];
    ds_q_rhand[3] = qt_q_hand[3];


    WBmotion->addPELOriInfo(ds_q_pelv, SITDOWN_TIME);
    WBmotion->addLHOriInfo(ds_q_lhand,SITDOWN_TIME);
    WBmotion->addRHOriInfo(ds_q_rhand,SITDOWN_TIME);

//    joint->SetMoveJoint(RWY, 180, 1000,MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LWY, 180, 1000,MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWY, 0, 1000,MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, 180, 1000,MOVE_ABSOLUTE);

    printf("Sit down done .. \n");

    Command_LIFTBOX = MODE_SITDOWN_DONE;
}

void WholeBody()
{
    printf("WholeBody start..\n");
    ShowWBInfos();

    /*
     * StartWBIK (WB_FLAG = true)
     * Global = RF (RF = -1, LF = 1, PC = 0)
     */
    //StartWBIKmotion(-1);
    //SaveWalkReadyPos();

    //double _FOOT_CENT_X = (WBmotion->pRF_3x1[0] + WBmotion->pLF_3x1[0])/2.;

    //SITDOWN_TIME = 5.;
    MOTION_TIME = 5;
    //variables for each hand
    double HandPos_lx = 0.5, HandPos_rx=-0.5;
    double HandPos_ly = 0.3, HandPos_ry=-0.3;
    double HandPos_lz = 1, HandPos_rz=0.7;

    double Leg_lx=0.0   ,Leg_rx=0.3;
    double Leg_ly=0.105 ,Leg_ry=-0.105;
    double Leg_lz=0.0   ,Leg_rz=0.3;

    usleep(100);

    WBmotion->addPELPosInfo(0.7, MOTION_TIME);
    WBmotion->addLHPosInfo(HandPos_lx,HandPos_ly,HandPos_lz,MOTION_TIME);
    WBmotion->addRHPosInfo(HandPos_rx,HandPos_ry,HandPos_rz,MOTION_TIME);
//    WBmotion->addLFPosInfo(Leg_lx, Leg_ly, Leg_lz, SITDOWN_TIME);
//    WBmotion->addRFPosInfo(Leg_rx, Leg_ry, Leg_rz, SITDOWN_TIME);

//    WBmotion->addLFPosInfo(double _xLeg, double _yLeg, double _zleg, SITDOWN_TIME);
//    WBmotion->addLHPosInfo(HandPos_x,HandPos_y,HandPos_z,SITDOWN_TIME);
//    WBmotion->addRHPosInfo(HandPos_x,-HandPos_y,HandPos_z,SITDOWN_TIME);


    printf("SHOULDER2WST = %f\n",WBmotion->kine_drc.L_WST2SHOULDER);
    printf("Arm = %f\n",WBmotion->kine_drc.L_UPPER_ARM + WBmotion->kine_drc.L_LOWER_ARM);


    quat qt_q_pelv=quat(vec3(0,1,0),20*D2R);
    doubles ds_q_pelv(4);
    for(int k=0;k<4;k++)
        ds_q_pelv[k]=qt_q_pelv[k];

    quat qt_q_hand = quat(vec3(0,1,0),0*D2R);
    doubles ds_q_rhand(4), ds_q_lhand(4);
    for(int k=0;k<4;k++){
        ds_q_lhand[k]=qt_q_hand[k];
        ds_q_rhand[k]=qt_q_hand[k];
    }
    ds_q_rhand[1] =-qt_q_hand[1];

//    ds_q_lhand[0] = qt_q_hand[0];
//    ds_q_lhand[1] = qt_q_hand[1];
//    ds_q_lhand[2] = qt_q_hand[2];
//    ds_q_lhand[3] = qt_q_hand[3];

//    ds_q_rhand[0] = qt_q_hand[0];
//    ds_q_rhand[1] =-qt_q_hand[1]; // Right hand rotates in the opposite direction to left hand.
//    ds_q_rhand[2] = qt_q_hand[2];
//    ds_q_rhand[3] = qt_q_hand[3];


    WBmotion->addPELOriInfo(ds_q_pelv, SITDOWN_TIME);
    WBmotion->addLHOriInfo(ds_q_lhand,SITDOWN_TIME);
    WBmotion->addRHOriInfo(ds_q_rhand,SITDOWN_TIME);


    joint->SetMoveJoint(RWY, 180, 1000,MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, 180, 1000,MOVE_ABSOLUTE);
//    joint->SetMoveJoint(RWY, 0, 1000,MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LWY, 0, 1000,MOVE_ABSOLUTE);

    printf("WholeBody done .. \n");

    Command_LIFTBOX = MODE_SITDOWN_DONE;
}

void HowMotion()
{
    printf("HowMotion start..\n");
    ShowWBInfos();


    MOTION_TIME = 1;
    //variables for each hand
    double HandPos_lx = 0.1, HandPos_rx= 0.3;
    double HandPos_ly = 0.3, HandPos_ry= -0.15;
    double HandPos_lz = 0.9, HandPos_rz= 1.1;

    double COM_x =0.0, COM_y = 0.0, COM_z = 0.8;

    usleep(100);

    WBmotion->addPELPosInfo(COM_z, MOTION_TIME);
//    WBmotion->addCOMInfo(COM_x,COM_y,MOTION_TIME);
    WBmotion->addLHPosInfo(HandPos_lx,HandPos_ly,HandPos_lz,MOTION_TIME);
    WBmotion->addRHPosInfo(HandPos_rx,HandPos_ry,HandPos_rz,MOTION_TIME);
    WBmotion->addRElbPosInfo(-30,MOTION_TIME);
    WBmotion->addLElbPosInfo(20,MOTION_TIME);


//    printf("SHOULDER2WST = %f\n",WBmotion->kine_drc.L_WST2SHOULDER);
//    printf("Arm = %f\n",WBmotion->kine_drc.L_UPPER_ARM + WBmotion->kine_drc.L_LOWER_ARM);


    quat qt_q_pelv=quat(vec3(1,0,0),5*D2R);
    doubles ds_q_pelv(4);
    for(int k=0;k<4;k++)
        ds_q_pelv[k]=qt_q_pelv[k];

//    quat qt_q_hand = quat(vec3(0,1,0),0*D2R);
//    doubles ds_q_rhand(4), ds_q_lhand(4);
//    for(int k=0;k<4;k++){
//        ds_q_lhand[k]=qt_q_hand[k];
//        ds_q_rhand[k]=qt_q_hand[k];
//    }
//    ds_q_rhand[1] =-qt_q_hand[1];


    WBmotion->addPELOriInfo(ds_q_pelv, MOTION_TIME);
//    WBmotion->addLHOriInfo(ds_q_lhand,MOTION_TIME);
//    WBmotion->addRHOriInfo(ds_q_rhand,MOTION_TIME);


//    joint->SetMoveJoint(RWY, 0, 1000,MOVE_ABSOLUTE);
//    joint->SetMoveJoint(LWY, 0, 1000,MOVE_ABSOLUTE);

    printf("HowMotion done .. \n");


    HandPos_rx = 0.1; HandPos_lx= 0.3;
    HandPos_ry = -0.3; HandPos_ly= 0.15;
    HandPos_rz = 0.9; HandPos_lz= 1.1;

    usleep(100);

    WBmotion->addPELPosInfo(COM_z, MOTION_TIME);
    WBmotion->addLHPosInfo(HandPos_lx,HandPos_ly,HandPos_lz,MOTION_TIME);
    WBmotion->addRHPosInfo(HandPos_rx,HandPos_ry,HandPos_rz,MOTION_TIME);
    WBmotion->addRElbPosInfo(-20,MOTION_TIME);
    WBmotion->addLElbPosInfo(30,MOTION_TIME);

    qt_q_pelv=quat(vec3(1,0,0),-5*D2R);
    for(int k=0;k<4;k++)
        ds_q_pelv[k]=qt_q_pelv[k];

    WBmotion->addPELOriInfo(ds_q_pelv, MOTION_TIME);

    Command_LIFTBOX = MODE_SITDOWN_DONE;
}

void WalkReady()
{

}

