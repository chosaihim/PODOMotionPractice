#include "MissionDoor.h"
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

    sprintf(__AL_NAME, "MissionDoor");
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
        case MISSIONDOOR_COMPLIANCE_START:
        {
            FILE_LOG(logSUCCESS) << "Arm Compliance control start..\n";
            ShutDownAllFlag();

            StartWBIKmotion(0);

            OnOff_compliance = true;
            //OnOff_zmpcontrol = true;
            sharedData->COMMAND[PODO_NO].USER_COMMAND = MISSIONDOOR_NO_ACT;
            break;
        }
        case MISSIONDOOR_COMPLIANCE_STOP:
        {
            FILE_LOG(logSUCCESS) << "Arm Compliance control stop..\n";
            OnOff_compliance = false;
            OnOff_zmpcontrol = false;
            ShutDownAllFlag();
            sharedData->COMMAND[PODO_NO].USER_COMMAND = MISSIONDOOR_NO_ACT;
            break;
        }
        case MISSIONDOOR_PUSH_DOOR:
        {
            FILE_LOG(logSUCCESS) << "Push Door start..\n";
            FLAG_pushdoor = true;
            sharedData->COMMAND[PODO_NO].USER_COMMAND = MISSIONDOOR_NO_ACT;
            break;
        }
        case MISSIONDOOR_LEANED_FORWARD:
        {
            if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[1] == 1)
            {
                FILE_LOG(logSUCCESS) << "Hubo 1111leaned forward!!!!!!!\n";
                ShutDownAllFlag();
                StartWBIKmotion(0);

                FLAG_leaned = true;
                LeanedFORWARD();
                sharedData->COMMAND[PODO_NO].USER_COMMAND = MISSIONDOOR_NO_ACT;
                break;
            }
            else
            {
                if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 1 || sharedData->COMMAND[PODO_NO].USER_PARA_INT[2] == 1)
                {
                    if(WB_FLAG == false) StartWBIKmotion(0);

                    FLAG_leaned = true;
                }
                sharedData->COMMAND[PODO_NO].USER_COMMAND = MISSIONDOOR_NO_ACT;
                break;
            }

        }
        case MISSIONDOOR_SAVE:
        {
            if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 0)
            {
                FILE_LOG(logSUCCESS) << "Command Data Save received..";
                fp = fopen("data.txt","w");
                for(int i=0;i<ROW;i++)
                {
                    for(int j=0;j<COL;j++)fprintf(fp,"%g\t", Save_Data[j][i]);
                    fprintf(fp,"\n");
                }
                fclose(fp);
                FILE_LOG(logSUCCESS) << "Data Save Complete ~!..";
            }else
            {
                FILE_LOG(logSUCCESS) << "Command Reset Data received..";
                Save_Index = 0;
            }
            sharedData->COMMAND[PODO_NO].USER_COMMAND = MISSIONDOOR_NO_ACT;
            break;
        }
        default:
            sharedData->COMMAND[PODO_NO].USER_COMMAND = MISSIONDOOR_NO_ACT;
            break;
        }
    }
    cout << ">>> Process MissionDoor is terminated..!!" << endl;
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

            if(OnOff_compliance == true) StartComplianceControl();

            if(OnOff_zmpcontrol == true) ZMPControl();

            if(FLAG_pushdoor == true) StartPushDoor();

            if(FLAG_leaned == true)
            {
                if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 1)
                {
                    LeanedFrontCOUNT++;
                    //printf("Hubo leaned forward!!!!!!! cnt = %d\n",LeanedFrontCOUNT);
                    LeanedFORWARD(LeanedFrontCOUNT);
                    printf("cnt = %d\n",LeanedFrontCOUNT);
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[2] == 1)
                {
                    LeanedBackCOUNT++;
                    //printf("Hubo leaned backward!!!!!!! cnt = %d\n",LeanedBackCOUNT);
                    LeanedBACKWARD(LeanedBackCOUNT);
                } else
                {
                    FLAG_leaned = false;
                    LeanedFrontCOUNT = LeanedBackCOUNT = 0;
                }

                userData->M2G.pCOM[0] = WBmotion->pCOM_2x1[0];
            }


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
        }
        joint->MoveAllJoint();
        rt_task_suspend(&rtTaskCon);
        save();
        SendDatatoGUI();
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

void SendDatatoGUI()
{
    userData->M2G.Des_posX = PosX[0];
    userData->M2G.Des_velX = VelX[0];
    userData->M2G.LPF_Fz = LPF_RH_Fz;
    userData->M2G.LPF_Fx = LPF_RH_Fx;
    userData->M2G.LPF_Fy = LPF_RH_Fy;
    userData->M2G.curZMP[0] = Y_ZMP_Local;
    userData->M2G.curZMP[1] = Local[1]*1000.;
    //userData->M2G.curZMP[2] = Local_bar[1]*1000.;
}

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

    WBmotion->WBIK_PARA_CHANGE();
    // RF_or_LF: 1=LF, -1=RF, 0=PC
    WBmotion->ResetGlobalCoord(_mode);

    WBmotion->StopAll();

    WBmotion->RefreshToCurrentReference();

    joint->SetAllMotionOwner();

    WB_FLAG = true;
}

void ShutDownAllFlag()
{
    //WB_FLAG = false;
    OnOff_compliance = false;
    FLAG_leaned = false;
    FLAG_pushdoor = false;

    /*Init Variables*/
    for(int i=0;i<2;i++)
    {
        PosX[i] = PosY[i] = PosZ[i] = VelX[i] = VelY[i] = VelZ[i] = 0.;
    }
    //ZMPControlX = ZMPControlY = Del_PC_X_DSP_XZMP_CON = Del_PC_Y_DSP_YZMP_CON = I_ZMP_CON_X = 0.;

}

/***************************************************/
void StartComplianceControl()
{
    /* Fz LowPassFilter */
    float LPF_Gain_X;
    float LPF_Gain_Y;
    float LPF_Gain_Z;
    if(FLAG_leaned == true)
    {
        LPF_Gain_X = 0.05;
        LPF_Gain_Y = 0.05;
        LPF_Gain_Z = 0.01;
    }
    else
    {
        LPF_Gain_X = 0.05;
        LPF_Gain_Y = 0.05;
        LPF_Gain_Z = 0.1;
    }

    if(sharedData->FT[3].Fz > 5. || sharedData->FT[3].Fz < -5.)
    {
        LPF_LH_Fz = sharedData->FT[3].Fz;
    } else
    {
        LPF_LH_Fz = 0.;
    }
    if(sharedData->FT[3].Fx > 3. || sharedData->FT[3].Fx < -3.)
    {
        LPF_LH_Fx = sharedData->FT[3].Fx;
    } else
    {
        LPF_LH_Fx = 0.;
    }
    if(sharedData->FT[3].Fy > 0.5 || sharedData->FT[3].Fy < -0.5)
    {
        LPF_LH_Fy = sharedData->FT[3].Fy;
    } else
    {
        LPF_LH_Fy = 0.;
    }

    if(sharedData->FT[2].Fz > 5. || sharedData->FT[2].Fz < -5.)
    {
        LPF_RH_Fz = sharedData->FT[2].Fz;
    } else
    {
        LPF_RH_Fz = 0.;
    }
    if(sharedData->FT[2].Fx > 3. || sharedData->FT[2].Fx < -3.)
    {
        LPF_RH_Fx = sharedData->FT[2].Fx;
    } else
    {
        LPF_RH_Fx = 0.;
    }
    if(sharedData->FT[2].Fy > 0.5 || sharedData->FT[2].Fy < -0.5)
    {
        LPF_RH_Fy = sharedData->FT[2].Fy;

    } else
    {
        LPF_RH_Fy = 0.;
    }

    LPF_RH_Fz = sharedData->FT[2].Fz;
    LPF_RH_Fx = sharedData->FT[2].Fx;
    LPF_RH_Fy = sharedData->FT[2].Fy;

    LPF_LH_Fz = sharedData->FT[3].Fz;
    LPF_LH_Fx = sharedData->FT[3].Fx;
    LPF_LH_Fy = sharedData->FT[3].Fy;

    LPF_RH_Fz = LPF_Gain_Z*LPF_RH_Fz + (1-LPF_Gain_Z)*Before_RH_Fz;
    LPF_RH_Fx = LPF_Gain_X*LPF_RH_Fx + (1-LPF_Gain_X)*Before_RH_Fx;
    LPF_RH_Fy = LPF_Gain_Y*LPF_RH_Fy + (1-LPF_Gain_Y)*Before_RH_Fy;

    LPF_LH_Fz = LPF_Gain_Z*LPF_LH_Fz + (1-LPF_Gain_Z)*Before_LH_Fz;
    LPF_LH_Fx = LPF_Gain_X*LPF_LH_Fx + (1-LPF_Gain_X)*Before_LH_Fx;
    LPF_LH_Fy = LPF_Gain_Y*LPF_LH_Fy + (1-LPF_Gain_Y)*Before_LH_Fy;

    Before_RH_Fz = LPF_RH_Fz;
    Before_RH_Fx = LPF_RH_Fx;
    Before_RH_Fy = LPF_RH_Fy;

    Before_LH_Fz = LPF_LH_Fz;
    Before_LH_Fx = LPF_LH_Fx;
    Before_LH_Fy = LPF_LH_Fy;

    LPFLPF_RH_Fz = LPF_RH_Fz;
    LPFLPF_LH_Fz = LPF_LH_Fz;

    LPFLPF_RH_Fz = LPF_Gain_Z*LPFLPF_RH_Fz + (1-LPF_Gain_Z)*Before_RH_Fz2;
    LPFLPF_LH_Fz = LPF_Gain_Z*LPFLPF_LH_Fz + (1-LPF_Gain_Z)*Before_LH_Fz2;

    Before_RH_Fz2 = LPFLPF_RH_Fz;
    Before_LH_Fz2 = LPFLPF_LH_Fz;

    /* RHand Compliance control */
    float Compliance_Gain_X = 0.01;
    float Resilence_Gain_X = 5.5;

    if(FLAG_leaned == true)
    {
        Compliance_Gain_X = 0.005;
        Resilence_Gain_X = 2.5;
    }

    float Compliance_Gain_Y = 0.01;
    float Resilence_Gain_Y = 5.5;

    float Compliance_Gain_Z = 0.005;
    float Resilence_Gain_Z = 2.5;

    Measured_Force_Z[0] = LPF_RH_Fz;
    Measured_Force_Y[0] = LPF_RH_Fy;
    Measured_Force_X[0] = LPF_RH_Fx;

    Measured_Force_Z[1] = LPF_LH_Fz;
    Measured_Force_Y[1] = LPF_LH_Fy;
    Measured_Force_X[1] = LPF_LH_Fx;

    if(FLAG_leaned == true)
    {
        Measured_Force_Z[0] = LPFLPF_RH_Fz;
        Measured_Force_Z[1] = LPFLPF_LH_Fz;
    }

    for(int i=0;i<2;i++)
    {
        Desired_Force_X[i] = 10;
        Desired_Force_Y[i] = 0;
        Desired_Force_Z[i] = 0;

        if(FLAG_leaned == true)
            Desired_Force_X[0] = 10;

        VelX[i] = Compliance_Gain_X*(Desired_Force_X[i] - Measured_Force_Z[i]) - Resilence_Gain_X*PosX[i];
        PosX[i] += 0.005*VelX[i];

        VelY[i] = Compliance_Gain_Y*(Desired_Force_Y[i] + Measured_Force_Y[i]) - Resilence_Gain_Y*PosY[i];
        PosY[i] += 0.005*VelY[i];

        VelZ[i] = Compliance_Gain_Z*(Desired_Force_Z[i] + Measured_Force_X[i]) - Resilence_Gain_Z*PosZ[i];
        PosZ[i] += 0.005*VelZ[i];

        if(PosX[i] > 0.1)
        {
            PosX[i] = 0.1;
        }else if(PosX[i] <-0.1)
        {
            PosX[i] = -0.1;
        }

        if(PosY[i] > 0.07)
        {
            PosY[i] = 0.07;
        }else if(PosY[i] <-0.07)
        {
            PosY[i] = -0.07;
        }

        if(PosZ[i] > 0.07)
        {
            PosZ[i] = 0.07;
        }else if(PosZ[i] <-0.07)
        {
            PosZ[i] = -0.07;
        }
    }

}

void StartPushDoor()
{
    double _sTime = 3.0;
    double _xArm = 0.4;

    TRInfo tempInfo;
    tempInfo = new TrajectoryCosine(_sTime, _xArm);
    WBmotion->wbPosRH[0]->AddTrajInfo(tempInfo);
}

void LeanedFORWARD()
{
    _FOOT_CENT_X = (WBmotion->pRF_3x1[0] + WBmotion->pLF_3x1[0]/2.);
    WBmotion->addCOMInfo(_FOOT_CENT_X+0.05, 0., 3.);
    printf("Footcent = %f\n",_FOOT_CENT_X);
}

void LeanedFORWARD(int cnt)
{
    COMpos = WBmotion->pCOM_2x1[0];


    if(cnt == 1)
    {
        InitCOMpos = WBmotion->pCOM_2x1[0];
    }
    if(cnt == 1)
    {
        for(int i=0;i<3;i++)
        {
            RHpos[i] = WBmotion->pRH_3x1[i];
            LHpos[i] = WBmotion->pLH_3x1[i];
        }
    }


    WBmotion->addCOMInfo(COMpos+0.0001, 0., 0.005);

    TRInfo tempInfoR;
    tempInfoR = new TrajectoryCosine(0.005, RHpos[0]+COMpos-InitCOMpos);
    WBmotion->wbPosRH[0]->AddTrajInfo(tempInfoR);

    TRInfo tempInfoL;
    tempInfoL = new TrajectoryCosine(0.005, LHpos[0]+COMpos-InitCOMpos);
    WBmotion->wbPosLH[0]->AddTrajInfo(tempInfoL);
}

void LeanedBACKWARD(int cnt)
{
    COMpos = WBmotion->pCOM_2x1[0];

    if(cnt == 1)
    {
        InitCOMpos = WBmotion->pCOM_2x1[0];
    }

    if(cnt == 1)
    {
        for(int i=0;i<3;i++)
        {
            RHpos[i] = WBmotion->pRH_3x1[i];
            LHpos[i] = WBmotion->pLH_3x1[i];
        }
    }

    WBmotion->addCOMInfo(COMpos-0.0001, 0., 0.005);

    TRInfo tempInfoR;
    tempInfoR = new TrajectoryCosine(0.005, RHpos[0]+COMpos-InitCOMpos);
    WBmotion->wbPosRH[0]->AddTrajInfo(tempInfoR);

    TRInfo tempInfoL;
    tempInfoL = new TrajectoryCosine(0.005, LHpos[0]+COMpos-InitCOMpos);
    WBmotion->wbPosLH[0]->AddTrajInfo(tempInfoL);

}


void ZMPControl()
{
    get_zmp2();
    Kirk_Control();
    Local[0] = -0.001*Del_PC_X_DSP_XZMP_CON;
    Local[1] = -0.001*Del_PC_Y_DSP_YZMP_CON;
    Local[2] = 0;

    Local2Global(Local,Global);
    ZMPControlX = Local[0];
    ZMPControlY = Local[1];
}

void Kirk_Control()
{
    ZMP_intergral_control();

    Del_PC_X_DSP_XZMP_CON = kirkZMPCon_XP2(-Del_PC_X_DSP_XZMP_CON, X_ZMP_Local, 1);
    Del_PC_Y_DSP_YZMP_CON = kirkZMPCon_YP2(-Del_PC_Y_DSP_YZMP_CON, Y_ZMP_Local, 1);

    //printf("ZMP LOCAL %f    Del_PC_X_DSP_XZMP_CON: %f \n",X_ZMP_Local,Del_PC_X_DSP_XZMP_CON);
}

double kirkZMPCon_XP2(double u, double ZMP, int zero)
{
    int i;
    const double A[2][2] = {{0.000000000000, 1.000000000000}, {-93.44,-1.0204}};
    const double B[2] = {0,153.5062};
    const double C[2] = {5.3672,0.0510};
    const double D = -7.6675;
    const double Kg[2] = {-0.1917,0.0976};//
    const double Og[2] = {3.5220,1.4988};

    static double x_old[2], x_new[2], Temp_1[2], Temp_2[2], Temp_3[2];
    double y;

    Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
    Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

    Temp_2[0] = B[0]*u;
    Temp_2[1] = B[1]*u;

    Temp_3[0] = Og[0]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);
    Temp_3[1] = Og[1]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);

    if(zero == 0)
    {
        for(i=0; i<2; i++)
        {
            x_old[i] = 0.;
            x_new[i] = 0.;
            Temp_1[i] = 0.;
            Temp_2[i] = 0.;
            Temp_3[i] = 0.;
        }
    }

    x_new[0] = x_old[0] + DEL_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
    x_new[1] = x_old[1] + DEL_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);

    y = Kg[0]*x_new[0] + Kg[1]*x_new[1];

    x_old[0] = x_new[0];
    x_old[1] = x_new[1];

    if(y > 40.0) y = 40.0;
    else if(y < -40.0) y = -40.0;

    return y ;
}
double kirkZMPCon_YP2(double u, double ZMP, int zero)
{
    int i;


    const double A[2][2] = {{0.000000000000, 1.000000000000}, {-93.44,-1.5306}};
    const double B[2] = {0,153.5062};
    const double C[2] = {5.3672,0.0765};
    const double D = -7.6675;
    const double Kg[2] = {-0.0809742090339354,	0.107288107510564};
    const double Og[2] = {3.43082537995055,0.723663240519607};


    static double x_old[2], x_new[2], Temp_1[2], Temp_2[2], Temp_3[2];
    double y;

    Temp_1[0] = A[0][0]*x_old[0] + A[0][1]*x_old[1];
    Temp_1[1] = A[1][0]*x_old[0] + A[1][1]*x_old[1];

    Temp_2[0] = B[0]*u;
    Temp_2[1] = B[1]*u;

    Temp_3[0] = Og[0]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);
    Temp_3[1] = Og[1]*(ZMP - C[0]*x_old[0] - C[1]*x_old[1] -D*u);

    if(zero == 0)
    {
        for(i=0; i<2; i++)
        {
            x_old[i] = 0.;
            x_new[i] = 0.;
            Temp_1[i] = 0.;
            Temp_2[i] = 0.;
            Temp_3[i] = 0.;
        }
    }

    x_new[0] = x_old[0] + DEL_T*(Temp_1[0] + Temp_2[0] + Temp_3[0]);
    x_new[1] = x_old[1] + DEL_T*(Temp_1[1] + Temp_2[1] + Temp_3[1]);

    y = Kg[0]*(x_new[0]) + Kg[1]*(x_new[1]);

    x_old[0] = x_new[0];
    x_old[1] = x_new[1];

    if(y > 60.0) y = 60.0;
    else if(y < -60.0) y = -60.0;

    return y ;
}

void ZMP_intergral_control()
{

    I_ZMP_CON_X += -0.001*(0.001*X_ZMP_Local - (X_ZMP_REF_Local));//-X_ZMP_n_OFFSET_BP
    I_ZMP_CON_Y += -0.001*(0.001*Y_ZMP_Local - (Y_ZMP_REF_Local));

    if(I_ZMP_CON_X > 0.04)I_ZMP_CON_X=0.04;
    else if(I_ZMP_CON_X < -0.04)I_ZMP_CON_X=-0.04;
    if(I_ZMP_CON_Y > 0.04)I_ZMP_CON_Y=0.04;
    else if(I_ZMP_CON_Y < -0.04)I_ZMP_CON_Y=-0.04;

}

void get_zmp2()
{
    // Foot Center in Global Coord.
        pCenter[0] = (WBmotion->des_pRF_3x1[0] + WBmotion->des_pLF_3x1[0])/2.;
        pCenter[1] = (WBmotion->des_pRF_3x1[1] + WBmotion->des_pLF_3x1[1])/2.;
        pCenter[2] = (WBmotion->des_pRF_3x1[2] + WBmotion->des_pLF_3x1[2])/2.;

//        printf("********** %f  %f   %f \n",WBmotion->des_pRF_3x1[0],WBmotion->des_pRF_3x1[1],WBmotion->des_pRF_3x1[2]);

        qtRZ((0. + 0.)/2.,qCenter);
        if(sharedData->FT[RAFT].Fz + sharedData->FT[LAFT].Fz > 50.)
        {
            M_LF[0] =  sharedData->FT[LAFT].Mx;
            M_LF[1] =  sharedData->FT[LAFT].My;
            M_LF[2] =  sharedData->FT[LAFT].Mz;

            QTtransform(WBmotion->des_qLF_4x1,M_LF,M_LF_Global);

            M_RF[0] =  sharedData->FT[RAFT].Mx;
            M_RF[1] =  sharedData->FT[RAFT].My;
            M_RF[2] =  sharedData->FT[RAFT].Mz;

            QTtransform(WBmotion->des_qRF_4x1,M_RF,M_RF_Global);

            F_LF[0] = sharedData->FT[LAFT].Fx;
            F_LF[1] = sharedData->FT[LAFT].Fy;
            F_LF[2] = sharedData->FT[LAFT].Fz;

            QTtransform(WBmotion->des_qLF_4x1,F_LF,F_LF_Global);

            F_RF[0] = sharedData->FT[RAFT].Fx;
            F_RF[1] = sharedData->FT[RAFT].Fy;
            F_RF[2] = sharedData->FT[RAFT].Fz;

            QTtransform(WBmotion->des_qRF_4x1,F_RF,F_RF_Global);



            double temp1[3],temp2[3],temp3[3],temp4[3];

            diff_vv(WBmotion->des_pRF_3x1,3,pCenter,temp1);// (despRF - pCenter)
            diff_vv(WBmotion->des_pLF_3x1,3,pCenter,temp2);// (despLF - pCenter)

            cross(1,temp1,F_RF_Global,temp3);// (despRF - pCenter)x(F_RF_Global)
            cross(1,temp2,F_LF_Global,temp4);// (despLF - pCenter)x(F_LF_Global)


            sum_vv(temp3,3,temp4,temp3); // (despRF - pCenter)x(F_RF_Global) + (despLF - pCenter)x(F_LF_Global)
            sum_vv(temp3,3,M_RF_Global,temp3); // (despRF - pCenter)x(F_RF_Global) + (despLF - pCenter)x(F_LF_Global) + M_RF_Global
            sum_vv(temp3,3,M_LF_Global,temp3); // (despRF - pCenter)x(F_RF_Global) + (despLF - pCenter)x(F_LF_Global) + M_RF_Global + M_LF_Global

            zmp[0] = (-temp3[1])/(F_RF_Global[2] + F_LF_Global[2]) + pCenter[0];
            zmp[1] = (temp3[0])/(F_RF_Global[2] + F_LF_Global[2]) + pCenter[1];
            zmp[2] = 0.;


            diff_vv(zmp,3,pCenter,temp1); // zmp - pCenter
            qCenter_bar[0] = qCenter[0];
            qCenter_bar[1] = -qCenter[1];
            qCenter_bar[2] = -qCenter[2];
            qCenter_bar[3] = -qCenter[3];

            QTtransform(qCenter_bar, temp1, zmp_local); // qCenter_bar*(zmp-pCenter)
            temp2[0] = 0;
            temp2[1] = 0;
            temp2[2] = 0;
            diff_vv(temp2,3,pCenter,temp1);
            QTtransform(qCenter_bar, temp1, zmp_ref_local);

            X_ZMP_Local = 1000.*zmp_local[0];
            Y_ZMP_Local = 1000.*zmp_local[1];

            X_ZMP_Global = 1000.*zmp[0];
            Y_ZMP_Global = 1000.*zmp[1];

            X_ZMP_REF_Local = zmp_ref_local[0];
            Y_ZMP_REF_Local = zmp_ref_local[1];
        }
}

void Local2Global(double _Local[],double _Global[])
{
    double pCenter[3],qCenter[4];
    double temp1[3];

    // Foot Center in Global Coord.
    pCenter[0] = (WBmotion->des_pRF_3x1[0] + WBmotion->des_pLF_3x1[0])/2.;
    pCenter[1] = (WBmotion->des_pRF_3x1[1] + WBmotion->des_pLF_3x1[1])/2.;
    pCenter[2] = (WBmotion->des_pRF_3x1[2] + WBmotion->des_pLF_3x1[2])/2.;

    qtRZ((0. + 0.)/2.,qCenter);

    QTtransform(qCenter, _Local, temp1);
    sum_vv(temp1,3,pCenter,_Global); // zmp - pCenter
}

void save()
{
    if(Save_Index < ROW)
    {

            Save_Data[0][Save_Index] = X_ZMP_Local;
            Save_Data[1][Save_Index] = Y_ZMP_Local;
            Save_Data[2][Save_Index] = Del_PC_X_DSP_XZMP_CON;
            Save_Data[3][Save_Index] = Del_PC_Y_DSP_YZMP_CON;
            Save_Data[4][Save_Index] = Global[0];
            Save_Data[5][Save_Index] = Global[1];
            Save_Data[6][Save_Index] = Save_Index;

            Save_Data[7][Save_Index] = F_RF_Global[1];
            Save_Data[8][Save_Index] = F_LF_Global[1];
            Save_Data[9][Save_Index] = M_RF_Global[1];
            Save_Data[10][Save_Index] = M_LF_Global[1];
            Save_Data[11][Save_Index] = pCenter[0];
            Save_Data[12][Save_Index] = pCenter[1];


            Save_Data[13][Save_Index] = WBmotion->des_pRH_3x1[0];
            Save_Data[14][Save_Index] = WBmotion->des_pCOM_2x1[0];

            Save_Data[15][Save_Index] = Local[0];
            Save_Data[16][Save_Index] = Local[1];
            Save_Data[17][Save_Index] = I_ZMP_CON_X;
            Save_Data[18][Save_Index] = I_ZMP_CON_Y;
            Save_Data[19][Save_Index] = _FOOT_CENT_X;

            Save_Data[20][Save_Index] = COMpos;

            Save_Data[21][Save_Index] = InitCOMpos;
            Save_Data[22][Save_Index] = RHpos[0];

            Save_Data[23][Save_Index] = sharedData->FT[2].Fz;

            Save_Data[24][Save_Index] = PosX[0];
            Save_Data[25][Save_Index] = LPF_RH_Fz;
            Save_Data[26][Save_Index] = LPFLPF_RH_Fz;

            Save_Index++;

            if(Save_Index >= ROW) Save_Index = 0;
    }
}

