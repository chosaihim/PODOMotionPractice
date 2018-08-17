#include "liftbox.h"
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

    sprintf(__AL_NAME, "LiftBox");
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
         case INVERSECHECK_GO_AND_IK:
        {
            FILE_LOG(logSUCCESS) << "INVERSECHECK_GO_AND_IK received..\n";

            joint->RefreshToCurrentReference();
            StartWBIKmotion(-1);
            joint->SetAllMotionOwner();

            GoandIK();
            sharedData->COMMAND[PODO_NO].USER_COMMAND = INVERSECHECK_NO_ACT;
            break;
        }
        case INVERSECHECK_FK:
        {
            FILE_LOG(logSUCCESS) << "INVERSECHECK_FK received..\n";
            FK();
            sharedData->COMMAND[PODO_NO].USER_COMMAND = INVERSECHECK_NO_ACT;
            break;
        }
        default:
            sharedData->COMMAND[PODO_NO].USER_COMMAND = INVERSECHECK_NO_ACT;
            break;
        }
    }
    cout << ">>> Process InverseCheck is terminated..!!" << endl;
    return 0;
}

void RBTaskThread(void *)
{
     while(isTerminated == 0)
    {
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
            joint->SetJointRefAngle(RWY, WBmotion->Q_filt_34x1[idRWY]*R2D);
            joint->SetJointRefAngle(RWP, WBmotion->Q_filt_34x1[idRWP]*R2D);
            joint->SetJointRefAngle(RWY2, WBmotion->Q_filt_34x1[idRWY2]*R2D);

            joint->SetJointRefAngle(LSP, WBmotion->Q_filt_34x1[idLSP]*R2D);
            joint->SetJointRefAngle(LSR, WBmotion->Q_filt_34x1[idLSR]*R2D - OFFSET_LSR);
            joint->SetJointRefAngle(LSY, WBmotion->Q_filt_34x1[idLSY]*R2D);
            joint->SetJointRefAngle(LEB, WBmotion->Q_filt_34x1[idLEB]*R2D - OFFSET_ELB);
            joint->SetJointRefAngle(LWY, WBmotion->Q_filt_34x1[idLWY]*R2D);
            joint->SetJointRefAngle(LWP, WBmotion->Q_filt_34x1[idLWP]*R2D);
            joint->SetJointRefAngle(LWY2, WBmotion->Q_filt_34x1[idLWY2]*R2D);

            if(!CheckMotionOwned())
                WB_FLAG = false;
            for(int i=0;i<34;i++)
            {
                sharedData->WBIK_Q[i] = WBmotion->Q_filt_34x1[i]*R2D;
            }
            sharedData->WBIK_Q[idRSR] -= OFFSET_RSR;
            sharedData->WBIK_Q[idLSR] -= OFFSET_LSR;
            sharedData->WBIK_Q[idREB] -= OFFSET_ELB;
            sharedData->WBIK_Q[idLEB] -= OFFSET_ELB;

            userData->M2G.OutputX = WBmotion->pRH_3x1[0];
            userData->M2G.OutputY = WBmotion->pRH_3x1[1];
            userData->M2G.OutputZ = WBmotion->pRH_3x1[2];

            rpy outputrpy = rpy(quat(WBmotion->qRH_4x1[0],WBmotion->qRH_4x1[1],WBmotion->qRH_4x1[2],WBmotion->qRH_4x1[3]));
            userData->M2G.OutputRoll = outputrpy.r*R2D;
            userData->M2G.OutputPitch = outputrpy.p*R2D;
            userData->M2G.OutputYaw = outputrpy.y*R2D;
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


void GoandIK()
{
    double inputX = userData->G2M.InputX;
    double inputY = userData->G2M.InputY;
    double inputZ = userData->G2M.InputZ;
    double inputRoll = userData->G2M.InputRoll*D2R;
    double inputPitch = userData->G2M.InputPitch*D2R;
    double inputYaw = userData->G2M.InputYaw*D2R;
    double _sTime = 3.0;
    rpy outputrpy = rpy(quat(WBmotion->qRH_4x1[0],WBmotion->qRH_4x1[1],WBmotion->qRH_4x1[2],WBmotion->qRH_4x1[3]));
    double Roll=outputrpy.r,Pitch=outputrpy.p,Yaw=outputrpy.y;
    TRInfo tempInfo;

    printf("OnOff = %d %d %d %d %d %d\n",userData->G2M.OnOff[0],userData->G2M.OnOff[1],userData->G2M.OnOff[2],userData->G2M.OnOff[3],userData->G2M.OnOff[4],userData->G2M.OnOff[5]);
    if(userData->G2M.OnOff[0] == true)
    {
        printf("Move X\n");
        tempInfo = new TrajectoryCosine(_sTime, inputX);
        WBmotion->wbPosRH[0]->AddTrajInfo(tempInfo);
    }
    if(userData->G2M.OnOff[1] == true)
    {
        printf("Move Y\n");
        tempInfo = new TrajectoryCosine(_sTime, inputY);
        WBmotion->wbPosRH[1]->AddTrajInfo(tempInfo);
    }
    if(userData->G2M.OnOff[2] == true)
    {
        printf("Move Z\n");
        tempInfo = new TrajectoryCosine(_sTime, inputZ);
        WBmotion->wbPosRH[2]->AddTrajInfo(tempInfo);
    }

    if(userData->G2M.OnOff[3] == true)//roll
    {
        printf("Move roll\n");
        Roll = inputRoll;
    }
    if(userData->G2M.OnOff[4] == true)//pitch
    {
        printf("Move pitch\n");
        Pitch = inputPitch;
    }
    if(userData->G2M.OnOff[5] == true)//yaw
    {
        printf("Move yaw\n");
        Yaw = inputYaw;
    }


    quat RHori = quat(rpy(Roll,Pitch,Yaw));
    doubles rhori(4);
    for(int k=0;k<4;k++)
        rhori[k] = RHori[k];

    WBmotion->addRHOriInfo(rhori,_sTime);

}

void FK()
{
        printf("C_Pelvis = %f, %f, %f\n",WBmotion->kine_drc.C_Pelvis[0],WBmotion->kine_drc.C_Pelvis[1],WBmotion->kine_drc.C_Pelvis[2]);
        printf("C_Pelvis = %f, %f, %f\n",WBmotion->kine_drc.C_RightUpperLeg[0],WBmotion->kine_drc.C_RightUpperLeg[1],WBmotion->kine_drc.C_RightUpperLeg[2]);
        printf("C_Pelvis = %f, %f, %f\n",WBmotion->kine_drc.C_RightLowerLeg[0],WBmotion->kine_drc.C_RightLowerLeg[1],WBmotion->kine_drc.C_RightLowerLeg[2]);
        printf("C_Pelvis = %f, %f, %f\n",WBmotion->kine_drc.C_RightFoot[0],WBmotion->kine_drc.C_RightFoot[1],WBmotion->kine_drc.C_RightFoot[2]);
        printf("C_Pelvis = %f, %f, %f\n",WBmotion->kine_drc.C_Pelvis[0],WBmotion->kine_drc.C_Pelvis[1],WBmotion->kine_drc.C_Pelvis[2]);
        printf("C_Pelvis = %f, %f, %f\n",WBmotion->kine_drc.C_Pelvis[0],WBmotion->kine_drc.C_Pelvis[1],WBmotion->kine_drc.C_Pelvis[2]);
        printf("C_Pelvis = %f, %f, %f\n",WBmotion->kine_drc.C_Pelvis[0],WBmotion->kine_drc.C_Pelvis[1],WBmotion->kine_drc.C_Pelvis[2]);
        printf("C_Pelvis = %f, %f, %f\n",WBmotion->kine_drc.C_Pelvis[0],WBmotion->kine_drc.C_Pelvis[1],WBmotion->kine_drc.C_Pelvis[2]);
        printf("C_Pelvis = %f, %f, %f\n",WBmotion->kine_drc.C_Pelvis[0],WBmotion->kine_drc.C_Pelvis[1],WBmotion->kine_drc.C_Pelvis[2]);
        printf("C_Pelvis = %f, %f, %f\n",WBmotion->kine_drc.C_Pelvis[0],WBmotion->kine_drc.C_Pelvis[1],WBmotion->kine_drc.C_Pelvis[2]);
        printf("C_Pelvis = %f, %f, %f\n",WBmotion->kine_drc.C_Pelvis[0],WBmotion->kine_drc.C_Pelvis[1],WBmotion->kine_drc.C_Pelvis[2]);
        printf("C_Pelvis = %f, %f, %f\n",WBmotion->kine_drc.C_Pelvis[0],WBmotion->kine_drc.C_Pelvis[1],WBmotion->kine_drc.C_Pelvis[2]);
        printf("C_Pelvis = %f, %f, %f\n",WBmotion->kine_drc.C_Pelvis[0],WBmotion->kine_drc.C_Pelvis[1],WBmotion->kine_drc.C_Pelvis[2]);
        printf("C_Pelvis = %f, %f, %f\n",WBmotion->kine_drc.C_Pelvis[0],WBmotion->kine_drc.C_Pelvis[1],WBmotion->kine_drc.C_Pelvis[2]);
        printf("C_Pelvis = %f, %f, %f\n",WBmotion->kine_drc.C_Pelvis[0],WBmotion->kine_drc.C_Pelvis[1],WBmotion->kine_drc.C_Pelvis[2]);
        printf("C_Pelvis = %f, %f, %f\n",WBmotion->kine_drc.C_Pelvis[0],WBmotion->kine_drc.C_Pelvis[1],WBmotion->kine_drc.C_Pelvis[2]);
        printf("C_Pelvis = %f, %f, %f\n",WBmotion->kine_drc.C_Pelvis[0],WBmotion->kine_drc.C_Pelvis[1],WBmotion->kine_drc.C_Pelvis[2]);


//    double jointvalue[34];
//    double outputpos[3];
//    double outputquat[4];

//    for(int i=0;i<34;i++)
//    {
//        jointvalue[i] = sharedData->JointRef[i]*D2R;
//    }
//    for(int i=0;i<3;i++)
//        outputpos[i] = WBmotion->pRH_3x1[i];

//    for(int i=0;i<4;i++)
//        outputquat[i] = WBmotion->qRH_4x1[i];

//    double elb_ang = WBmotion->RElb_ang;
//    //printf("pRH = %f %f %f %f\n",WBmotion->qRH_4x1[0],WBmotion->qRH_4x1[1],WBmotion->qRH_4x1[2],WBmotion->qRH_4x1[3]);
//    WBmotion->kine_drc.FK_RightHand_Global(jointvalue,outputpos,outputquat,elb_ang);

//    /* rpy to quat */
//    double roll = 0.;
//    double pitch = -90.;
//    double yaw = 0.;

//    double cy = cos(yaw*0.5);
//    double sy = sin(yaw*0.5);
//    double cr = cos(roll*0.5);
//    double sr = sin(roll*0.5);
//    double cp = cos(pitch*0.5);
//    double sp = sin(pitch*0.5);

//    quat temp;
//    temp.w = cy*cr*cp + sy*sr*sp;
//    temp.x = cy*sr*cp - sy*cr*sp;
//    temp.y = cy*cr*sp + sy*sr*cp;
//    temp.z = sy*cr*cp - cy*sr*sp;

//    quat maybe = quat(outputquat[0],outputquat[1],outputquat[2],outputquat[2]);

//    double rroll,rpitch,ryaw;
//    double mroll,mpitch,myaw;

//    double sinr = +2.0 * (temp.w * temp.x + temp.y * temp.z);
//    double cosr = +1.0 - 2.0 * (temp.x * temp.x + temp.y * temp.y);
//    rroll = atan2(sinr, cosr);

//    // pitch (y-axis rotation)
//    double sinp = +2.0 * (temp.w * temp.y - temp.z * temp.x);
//    if (fabs(sinp) >= 1)
//        rpitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
//    else
//        rpitch = asin(sinp);

//    // yaw (z-axis rotation)
//    double siny = +2.0 * (temp.w * temp.z + temp.x * temp.y);
//    double cosy = +1.0 - 2.0 * (temp.y * temp.y + temp.z * temp.z);
//    ryaw = atan2(siny, cosy);


//    double msinr = +2.0 * (maybe.w * maybe.x + maybe.y * maybe.z);
//    double mcosr = +1.0 - 2.0 * (maybe.x * maybe.x + maybe.y * maybe.y);
//    mroll = atan2(msinr, mcosr);

//    // pitch (y-axis rotation)
//    double msinp = +2.0 * (maybe.w * maybe.y - maybe.z * maybe.x);
//    if (fabs(msinp) >= 1)
//        mpitch = copysign(M_PI / 2, msinp); // use 90 degrees if out of range
//    else
//        mpitch = asin(msinp);

//    // yaw (z-axis rotation)
//    double msiny = +2.0 * (maybe.w * maybe.z + maybe.x * maybe.y);
//    double mcosy = +1.0 - 2.0 * (maybe.y * maybe.y + maybe.z * maybe.z);
//    myaw = atan2(msiny, mcosy);

//    rpy kkk = rpy(temp);
//    rpy outputrpy = rpy(maybe);

//    /**************/
//    printf("WBmotion = %f %f %f %f\n\n",WBmotion->qRH_4x1[0],WBmotion->qRH_4x1[1],WBmotion->qRH_4x1[2],WBmotion->qRH_4x1[3]);

//    printf("mquat = %f %f %f %f\n",outputquat[0],outputquat[1],outputquat[2],outputquat[3]);
//    printf("mmmmm = %f %f %f %f\n",maybe[0],maybe[1],maybe[2],maybe[3]);
//    printf("rquat = %f %f %f %f\n\n",temp[0],temp[1],temp[2],temp[3]);
//    printf("rquat = %f %f %f %f\n\n",temp.w,temp.x,temp.y,temp.z);


//    printf("MY CODE=====================\n");
//    printf("mrpy = %f %f %f\n",mroll*R2D, mpitch*R2D, myaw*R2D);
//    printf("rrpy = %f %f %f\n\n",rroll*R2D, rpitch*R2D, ryaw*R2D);


//    printf("IN CODE=====================\n");
//    printf("printf = %f %f %f\n",outputrpy[0]*R2D,outputrpy[1]*R2D,outputrpy[2]*R2D);
//    printf("kkk = %f %f %f\n",kkk[0]*R2D,kkk[1]*R2D,kkk[2]*R2D);


//    //printf("OutputRoll = %f , SY = %f\n", outputrpy[0], jointvalue[RSY]);

//    userData->M2G.OutputX = outputpos[0];
//    userData->M2G.OutputY = outputpos[1];
//    userData->M2G.OutputZ = outputpos[2];
//    userData->M2G.OutputRoll = outputrpy[0]*R2D;
//    userData->M2G.OutputPitch = outputrpy[1]*R2D;
//    userData->M2G.OutputYaw = outputrpy[2]*R2D;
}
