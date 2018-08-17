#include "approachbox.h"
#include "manualwalking.cpp"
#include "controller.cpp"


/****************************** 1. main *************************************/
int main(int argc, char *argv[])
{
    // Termination signal ---------------------------------
    signal(SIGTERM, CatchSignals);   // "kill" from shell
    signal(SIGINT,  CatchSignals);    // Ctrl-c
    signal(SIGHUP,  CatchSignals);    // shell termination
    signal(SIGKILL, CatchSignals);
    signal(SIGSEGV, CatchSignals);

    // Block memory swapping ------------------------------
    mlockall(MCL_CURRENT|MCL_FUTURE);

    // Setting the AL Name <-- Must be a unique name!!
    sprintf(__AL_NAME, "WBWalk");
    CheckArguments(argc, argv);

    if(PODO_NO == -1){
        FILE_LOG(logERROR) << "Please check the AL Number";
        FILE_LOG(logERROR) << "Terminate this AL..";
        return 0;
    }

    // Initialize RBCore -----------------------------------
    if(RBInitialize() == false)
        __IS_WORKING = false;

    // Initialize MPCWalking -------------------------------
    //************* Get F.K
    First_Initialize();

    // QP solver setting
    #if (NUMTESTS > 0)
      int i;
      double time;
      double time_per;
    #endif
      set_defaults();
      setup_indexing();
      /* Solve problem instance for the record. */
      settings.verbose = 1;

    // MPC Precomputing for Fast calculation
    PreComputeQP();

    while(__IS_WORKING){
        usleep(100*1000);

        switch(sharedData->COMMAND[PODO_NO].USER_COMMAND)
        {
        case WBWALK_GO_CART:
        {
            initialize();
            FILE_LOG(logSUCCESS) << "Start Action : Approach Handle";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();
            FLAG_UBIK = true;
            Command_CART = APPROACH_HANDLE;
            //Command_GRIPPER = GRIPPER_OPEN;
            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case WBWALK_GRASP_CART:
        {
            FILE_LOG(logSUCCESS) << "Start Action : Grasp Handle";
            Command_CART = GRASP_HANDLE;
            //Command_GRIPPER = GRIPPER_CLOSE;
            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case WBWALK_WALKING:
        {
            if(walk_flag == 0)
            {
                FILE_LOG(logSUCCESS) << "Start Action : Walking Forward with cart";

                sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0]=1;//fog zero
                sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_SENSOR_FOG_NULL;
                pv_Index = 0;
                /* walking initialize */
                Walking_initialize();

                FLAG_UBIK = false;
                FPG_TEST(WALK_FORWARD, userData->G2M.StepNum,userData->G2M.StepLength,userData->G2M.StepAngle,userData->G2M.StepOffset, kine_drc_hubo4.L_PEL2PEL);
                usleep(200*1000);
                walk_flag = 1;

                Command_CART = WALKING;
                Command_GRIPPER = GRIPPER_STOP;
                jCon->RefreshToCurrentReference();
                jCon->SetAllMotionOwner();
            }
            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case WBWALK_RELEASE_CART:
        {
            FILE_LOG(logSUCCESS) << "Start Action : Quit Handle";
            Command_CART = QUIT_HANDLE;
            //Command_GRIPPER = GRIPPER_OPEN;
            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case WBWALK_SAVE:
        {
            if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 0)
            {
                FILE_LOG(logSUCCESS) << "Command Data Save received..";
                fp = fopen("WBWALK.txt","w");
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
            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        default:
            break;
        }//end switch
    }//end while
    FILE_LOG(logERROR) << "Process \"" << __AL_NAME << "\" is terminated" << endl;
    return 0;
}//end main

/****************************** 2. read *******************************/
void RBTaskThread(void *)
{
    while(__IS_WORKING)
    {
        /* Check [Command_GRIPPER] and Action */
        GripperTH();

        /* Check [Command_CART] and Action */
        CartTH();

        /* save data */
        save();

        if(MODE_compliance == ON)
        {
            StartComplianceControl();
        }


        /* Kalman, High Pass Filter */
        State_Estimator(sharedData->FOG.RollVel,sharedData->FOG.PitchVel, sharedData->FOG.YawVel, sharedData->IMU[0].AccX, sharedData->IMU[0].AccY, Estimated_Orientation);

        /* Update target_foot & make Trajectory ZMP,Foot reference */
        update_window();
        /* Calculate the ZMP using value of FT sensor */
        get_zmp2();

        /* Calculate the COM x,y(GLOBAL_X,Y) */
        WMG();

        /* Control on/off */
        Controller();


        /* Calculate desired value and IK */
        if(FLAG_UBIK == true)
            UBIK();
        else
            WBIK();

        SendDatatoGUI();

        if(walk_flag == false)
        {
            if(LandingState == FINAL)
            {
                printf("******************* Walking Is Finishied!!!");
                Walking_initialize();
            }
            LandingState = END;
        }

        for(int i=0;i<34;i++)
        {
            sharedData->WBIK_Q[i] = wbwalk.fkik.WBIK_Q[i];
        }

        wbwalk.RefreshCurrent();
        jCon->MoveAllJoint();
        rt_task_suspend(&rtTaskCon);
    }
}


void initialize()
{

    WBIK_PARA_CHANGE();
    printf("WBIK_Q0[idRSP] = %f\n",wbwalk.fkik.WBIK_Q[idRSP]*R2D);

    wbwalk.fkik.desired.pCOM_3x1[0] = userData->WalkReadyCOM[0] = 0.0;//0.0237f;
    wbwalk.fkik.desired.pCOM_3x1[1] = userData->WalkReadyCOM[1] = 0.0;
    wbwalk.fkik.desired.pCOM_3x1[2] = userData->WalkReadyCOM[2] = 0.77;// + 0.11;//(61.8kg:0.74)//59//0.8;//71kg

    wbwalk.fkik.desired.qPEL_4x1[0] = 1.;
    wbwalk.fkik.desired.qPEL_4x1[1] = 0.;
    wbwalk.fkik.desired.qPEL_4x1[2] = 0.;
    wbwalk.fkik.desired.qPEL_4x1[3] = 0.;

    wbwalk.fkik.desired.pRF_3x1[0] = 0.;
    wbwalk.fkik.desired.pRF_3x1[1] = -kine_drc_hubo4.L_PEL2PEL/2.;//-0.13;//-kine_drc_hubo4.L_PEL2PEL/2;//-0.135;//
    wbwalk.fkik.desired.pRF_3x1[2] = 0.;

    wbwalk.fkik.desired.qRF_4x1[0] = 1.;
    wbwalk.fkik.desired.qRF_4x1[1] = 0.;
    wbwalk.fkik.desired.qRF_4x1[2] = 0.;
    wbwalk.fkik.desired.qRF_4x1[3] = 0.;

    wbwalk.fkik.desired.pLF_3x1[0] = 0.;
    wbwalk.fkik.desired.pLF_3x1[1] = kine_drc_hubo4.L_PEL2PEL/2.;//0.13;//kine_drc_hubo4.L_PEL2PEL/2;//0.135;//
    wbwalk.fkik.desired.pLF_3x1[2] = 0.;

    wbwalk.fkik.desired.qLF_4x1[0] = 1.;
    wbwalk.fkik.desired.qLF_4x1[1] = 0.;
    wbwalk.fkik.desired.qLF_4x1[2] = 0.;
    wbwalk.fkik.desired.qLF_4x1[3] = 0.;

    get_WBIK_Q_from_RefAngleCurrent();

   //memcpy(wbwalk.fkik.WBIK_Q,wbwalk.fkik.WBIK_Q0,34*sizeof(double));

    usleep(100*1000);
    printf("COM : %f  %f  %f \n",wbwalk.fkik.desired.pCOM_3x1[0],wbwalk.fkik.desired.pCOM_3x1[1],wbwalk.fkik.desired.pCOM_3x1[2]);
    printf("qPel : %f  %f  %f  %f \n",wbwalk.fkik.desired.qPEL_4x1[0],wbwalk.fkik.desired.qPEL_4x1[1],wbwalk.fkik.desired.qPEL_4x1[2],wbwalk.fkik.desired.qPEL_4x1[3]);
    printf("pRF : %f  %f  %f \n",wbwalk.fkik.desired.pRF_3x1[0],wbwalk.fkik.desired.pRF_3x1[1],wbwalk.fkik.desired.pRF_3x1[2]);
    printf("qRF : %f  %f  %f  %f \n",wbwalk.fkik.desired.qRF_4x1[0],wbwalk.fkik.desired.qRF_4x1[1],wbwalk.fkik.desired.qRF_4x1[2],wbwalk.fkik.desired.qRF_4x1[3]);
    printf("pLF : %f  %f  %f \n",wbwalk.fkik.desired.pLF_3x1[0],wbwalk.fkik.desired.pLF_3x1[1],wbwalk.fkik.desired.pLF_3x1[2]);
    printf("qLF : %f  %f  %f  %f \n",wbwalk.fkik.desired.qLF_4x1[0],wbwalk.fkik.desired.qLF_4x1[1],wbwalk.fkik.desired.qLF_4x1[2],wbwalk.fkik.desired.qLF_4x1[3]);
    printf("pRH : %f  %f  %f \n",wbwalk.fkik.desired.pRH_3x1[0],wbwalk.fkik.desired.pRH_3x1[1],wbwalk.fkik.desired.pRH_3x1[2]);
    printf("qRH : %f  %f  %f  %f \n",wbwalk.fkik.desired.qRH_4x1[0],wbwalk.fkik.desired.qRH_4x1[1],wbwalk.fkik.desired.qRH_4x1[2],wbwalk.fkik.desired.qRH_4x1[3]);
    printf("pLH : %f  %f  %f \n",wbwalk.fkik.desired.pLH_3x1[0],wbwalk.fkik.desired.pLH_3x1[1],wbwalk.fkik.desired.pLH_3x1[2]);
    printf("qLH : %f  %f  %f  %f \n",wbwalk.fkik.desired.qLH_4x1[0],wbwalk.fkik.desired.qLH_4x1[1],wbwalk.fkik.desired.qLH_4x1[2],wbwalk.fkik.desired.qLH_4x1[3]);

    get_WBIK_Q_from_RefAngleCurrent();


    wbwalk.fkik.WBIK_Q[idQ0] = 1;
    wbwalk.fkik.WBIK_Q[idQ1] = 0;
    wbwalk.fkik.WBIK_Q[idQ2] = 0;
    wbwalk.fkik.WBIK_Q[idQ3] = 0;

    wbwalk.fkik.WBIK_Q[idX] = wbwalk.fkik.WBIK_Q[idX] - wbwalk.fkik.FK.pCOM_3x1[0];//reset to 0;
    wbwalk.fkik.WBIK_Q[idY] = wbwalk.fkik.WBIK_Q[idY] - wbwalk.fkik.FK.pCOM_3x1[1];//reset to 0;
    wbwalk.fkik.WBIK_Q[idZ] = wbwalk.fkik.WBIK_Q[idZ] - wbwalk.fkik.FK.pCOM_3x1[2] + userData->WalkReadyCOM[2];// + fsm->AddComInfos[0][2];//0;

    kine_drc_hubo4.FK_RightFoot_Global(wbwalk.fkik.WBIK_Q,wbwalk.fkik.FK.pRF_3x1,  wbwalk.fkik.FK.qRF_4x1);
    kine_drc_hubo4.FK_LeftFoot_Global(wbwalk.fkik.WBIK_Q,wbwalk.fkik.FK.pLF_3x1,  wbwalk.fkik.FK.qLF_4x1);
    kine_drc_hubo4.FK_RightHand_Local(wbwalk.fkik.WBIK_Q,2,wbwalk.fkik.FK.pRH_3x1,wbwalk.fkik.FK.qRH_4x1,wbwalk.fkik.FK.REB_ang);
    printf("wbwalk.fkik.FK.pRH_3x1 : %f, %f, %f\n",wbwalk.fkik.FK.pRH_3x1[0],wbwalk.fkik.FK.pRH_3x1[1],wbwalk.fkik.FK.pRH_3x1[2]);
    kine_drc_hubo4.FK_LeftHand_Local(wbwalk.fkik.WBIK_Q,2,wbwalk.fkik.FK.pLH_3x1,wbwalk.fkik.FK.qLH_4x1,wbwalk.fkik.FK.LEB_ang);
    kine_drc_hubo4.FK_COM_Global(wbwalk.fkik.WBIK_Q,wbwalk.fkik.FK.pCOM_3x1);

    usleep(100*1000);
    printf("COM : %f  %f  %f \n",wbwalk.fkik.desired.pCOM_3x1[0],wbwalk.fkik.desired.pCOM_3x1[1],wbwalk.fkik.desired.pCOM_3x1[2]);
    printf("qPel : %f  %f  %f  %f \n",wbwalk.fkik.desired.qPEL_4x1[0],wbwalk.fkik.desired.qPEL_4x1[1],wbwalk.fkik.desired.qPEL_4x1[2],wbwalk.fkik.desired.qPEL_4x1[3]);
    printf("pRF : %f  %f  %f \n",wbwalk.fkik.desired.pRF_3x1[0],wbwalk.fkik.desired.pRF_3x1[1],wbwalk.fkik.desired.pRF_3x1[2]);
    printf("qRF : %f  %f  %f  %f \n",wbwalk.fkik.desired.qRF_4x1[0],wbwalk.fkik.desired.qRF_4x1[1],wbwalk.fkik.desired.qRF_4x1[2],wbwalk.fkik.desired.qRF_4x1[3]);
    printf("pLF : %f  %f  %f \n",wbwalk.fkik.desired.pLF_3x1[0],wbwalk.fkik.desired.pLF_3x1[1],wbwalk.fkik.desired.pLF_3x1[2]);
    printf("qLF : %f  %f  %f  %f \n",wbwalk.fkik.desired.qLF_4x1[0],wbwalk.fkik.desired.qLF_4x1[1],wbwalk.fkik.desired.qLF_4x1[2],wbwalk.fkik.desired.qLF_4x1[3]);
    printf("pRH : %f  %f  %f \n",wbwalk.fkik.desired.pRH_3x1[0],wbwalk.fkik.desired.pRH_3x1[1],wbwalk.fkik.desired.pRH_3x1[2]);
    printf("qRH : %f  %f  %f  %f \n",wbwalk.fkik.desired.qRH_4x1[0],wbwalk.fkik.desired.qRH_4x1[1],wbwalk.fkik.desired.qRH_4x1[2],wbwalk.fkik.desired.qRH_4x1[3]);
    printf("pLH : %f  %f  %f \n",wbwalk.fkik.desired.pLH_3x1[0],wbwalk.fkik.desired.pLH_3x1[1],wbwalk.fkik.desired.pLH_3x1[2]);
    printf("qLH : %f  %f  %f  %f \n",wbwalk.fkik.desired.qLH_4x1[0],wbwalk.fkik.desired.qLH_4x1[1],wbwalk.fkik.desired.qLH_4x1[2],wbwalk.fkik.desired.qLH_4x1[3]);


}

/****************************** 3. FlagThread *******************************/
void RBFlagThread(void *)
{
    rt_task_set_periodic(NULL, TM_NOW, 300*1000);        // 300 usec

    while(__IS_WORKING)
    {
        rt_task_wait_period(NULL);

        if(HasAnyOwnership()){
            if(sharedData->SYNC_SIGNAL[PODO_NO] == true){
                jCon->JointUpdate();
                rt_task_resume(&rtTaskCon);
            }
        }
    }
}

/****************************** 4. MissionDoor ******************************/
void SendDatatoGUI()
{
    userData->M2G.Des_posX = PosX[0];
    userData->M2G.Des_velX = VelX[0];
    userData->M2G.LPF_Fz = LPF_RH_Fz;
    userData->M2G.LPF_Fx = LPF_RH_Fx;
    userData->M2G.LPF_Fy = LPF_RH_Fy;
    userData->M2G.curZMP[0] = Y_ZMP_Local;
    userData->M2G.curZMP[1] = Local[1]*1000.;
}
void ShutDownAllFlag()
{
    //WB_FLAG = false;
    MODE_compliance = OFF;

    /*Init Variables*/
    for(int i=0;i<2;i++)
    {
        PosX[i] = PosY[i] = PosZ[i] = VelX[i] = VelY[i] = VelZ[i] = 0.;
    }
    //ZMPControlX = ZMPControlY = Del_PC_X_DSP_XZMP_CON = Del_PC_Y_DSP_YZMP_CON = I_ZMP_CON_X = 0.;
}

void StartComplianceControl()
{
    /* Fz LowPassFilter */
    float LPF_Gain_X;
    float LPF_Gain_Y;
    float LPF_Gain_Z;

    LPF_Gain_X = 0.05;
    LPF_Gain_Y = 0.05;
    LPF_Gain_Z = 0.1;


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



    for(int i=0;i<2;i++)
    {
        Desired_Force_X[i] = 10;
        Desired_Force_Y[i] = 0;
        Desired_Force_Z[i] = 0;



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

        ComplianceX[i] = PosX[i] - PosXbef[i];
        ComplianceY[i] = PosY[i] - PosYbef[i];
        ComplianceZ[i] = PosZ[i] - PosZbef[i];

        PosXbef[i] = PosX[i];
        PosYbef[i] = PosY[i];
        PosZbef[i] = PosZ[i];
    }

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

/****************************** 5. Motion Generator *************************/
void WMG()
{
    int Preview_time = 15,cn = 20;
    double disturbance[2][3] = {{0,},},FOOT_REF[4] = {0.0,},temp_zmp[15]={0.0,},temp_state[3] = {0.0,},temp_vec[15] = {0.0,},FOOT_IND[2] = {1.,1.},optimal_U[15] = {0.,};
    static double pv_state[2][3] = {{0.0}}, pv_state_old[2][3] = {{0.0}};
    double UK[15][3]={{0.0,},},U0[15]={0.0,},MPC_pk[18] = {0.,},MPC_zmp[2]={0.0,};
    static double refx[3] = {0.,},refy[3] = {0.,},px0[3] = {0.,},py0[3] = {0.,},pxf[3] = {0.,},pyf[3] = {0.,},t1=0,t2=0,tf=0.1;

    if(pv_Index == 0)
    {// initialize the parameter (pv_Index : The num of times WMG() was executed.)

        pv_state_old[0][0] = 0.;
        pv_state_old[0][1] = 0.;
        pv_state_old[0][2] = 0.;
        pv_state_old[1][0] = 0.;
        pv_state_old[1][1] = 0.;
        pv_state_old[1][2] = 0.;

        printf(">>>>>>>> WMG  Initial CoM  <<<<<<<<< \n");
        printf("%f  %f  %f \n",pv_state_old[0][0],pv_state_old[0][1],pv_state_old[0][2]);
        printf("%f  %f  %f \n",pv_state_old[1][0],pv_state_old[1][1],pv_state_old[1][2]);

        for(int i = 0;i<Preview_time;i++)
        {// 15 times.
            temp_vec[i] = 0.;
            t1 = 0;
            t2 = 0;
        }
    }

    if(pv_Index == 700)
    {//3.5sec?
            disturbance[1][0] = 0.;
            disturbance[1][1] = 0.;
            disturbance[1][2] = 0.;

            disturbance[0][0] = 0.;
            disturbance[0][1] = 0.;
            disturbance[0][2] = 0.;
    }


    if(pv_Index%cn == 0)
    {//0.1sec

        /* Set U0, UK, FOOT_IND (But U0, UK not used)*/
        UKSEL(20,U0,UK,FOOT_IND);

        t1 = 0;
        t2 = 0;
        for(int i=0;i<2;i++)
        {
            if(i==0)
            {
                px0[0] = pv_state_old[i][0] + disturbance[i][0];
                px0[1] = pv_state_old[i][1] + disturbance[i][1];
                px0[2] = pv_state_old[i][2] + disturbance[i][2];
            }else
            {
                py0[0] = pv_state_old[i][0] + disturbance[i][0];
                py0[1] = pv_state_old[i][1] + disturbance[i][1];
                py0[2] = pv_state_old[i][2] + disturbance[i][2];
            }

            temp_state[0] = pv_state_old[i][0] + disturbance[i][0];
            temp_state[1] = pv_state_old[i][1] + disturbance[i][1];
            temp_state[2] = pv_state_old[i][2] + disturbance[i][2];

            /* MPC_pk */
            mat15by3x3by1(MPC_pc,temp_state,temp_vec);
            for(int asdf = 0;asdf<15;asdf++)
            {
                    if(i >0)
                    {
                        temp_zmp[asdf] = window[(asdf+1)*cn].zmp.y;
                    }else
                    {
                        temp_zmp[asdf] = window[(asdf+1)*cn].zmp.x;
                    }
            }
            temp_debug[7] = temp_zmp[0];
            for(int asdf = 0;asdf<15;asdf++)
            {
                    MPC_pk[asdf] = temp_vec[asdf] - MPC_beta*temp_zmp[asdf];
            }

            /* ------Cix>=ci */
            for(int sk = 0;sk<Preview_time*2;sk++)
            {
                if(sk<15)
                {
                    MPC_ci[sk] = MPC_sp[i] + temp_zmp[sk];
                }else
                {
                    MPC_ci[sk] = MPC_sp[i] - temp_zmp[sk - Preview_time];
                }
            }

            for(int s=0;s<15;s++)
            {
                for(int t=0;t<15;t++)
                {
                    params.Q[s + t*15 ] = MPC_Q[s][t];
                }
            }

            for(int s=0;s<15;s++)
            {
                params.c[s] = MPC_pk[s];
            }

            for(int s=0;s<30;s++)
            {
                for(int t=0;t<15;t++)
                {
                    params.A[s + t*30] = MPC_Ci[s][t];
                }
            }

            for(int s=0;s<30;s++)
            {
                params.b[s] = MPC_ci[s];
            }

            /* solving optimization problem */
            solve();

            mat15by3x3by1(Pzs,temp_state,temp_vec);

           for(int si=0;si<15;si++)
           {
               temp_vec[si] = vars.x[si] - temp_vec[si];
           }

           mat15by15x15by1(Pzu_inv,temp_vec,optimal_U);

            pv_state[i][0] = (MPC_A[0][0]*pv_state_old[i][0] + MPC_A[0][1]*pv_state_old[i][1] + MPC_A[0][2]*pv_state_old[i][2]) + (MPC_B[0])*optimal_U[0];
            pv_state[i][1] = (MPC_A[1][0]*pv_state_old[i][0] + MPC_A[1][1]*pv_state_old[i][1] + MPC_A[1][2]*pv_state_old[i][2]) + (MPC_B[1])*optimal_U[0];
            pv_state[i][2] = (MPC_A[2][0]*pv_state_old[i][0] + MPC_A[2][1]*pv_state_old[i][1] + MPC_A[2][2]*pv_state_old[i][2]) + (MPC_B[2])*optimal_U[0];

            /* ZMP Output from LIPM */
            MPC_zmp[i] = MPC_C[0]*pv_state[i][0] + MPC_C[1]*pv_state[i][1]+ MPC_C[2]*pv_state[i][2];

            pv_state_old[i][0] = pv_state[i][0];
            pv_state_old[i][1] = pv_state[i][1];
            pv_state_old[i][2] = pv_state[i][2];

            if(i == 0)
            {
                temp_debug[0] = temp_zmp[0];
                temp_debug[1] = MPC_zmp[i];
                temp_debug[2] = pv_state[i][0];
                temp_debug[3] = pv_state[i][1];
            }else
            {
                temp_debug[5] = temp_zmp[0];
                temp_debug[6] = MPC_zmp[i];
                temp_debug[7] = pv_state[i][0];
                temp_debug[8] = pv_state[i][1];
            }
        }

        temp_debug[10] = FOOT_REF[0];
        temp_debug[11] = FOOT_IND[0];

        pxf[0] = pv_state_old[0][0];
        pxf[1] = pv_state_old[0][1];
        pxf[2] = pv_state_old[0][2];
        pyf[0] = pv_state_old[1][0];
        pyf[1] = pv_state_old[1][1];
        pyf[2] = pv_state_old[1][2];

    }

    t1 = t1 + 0.005;
    t2 = t2 + 0.005;

    Fifth(t1,tf,px0,pxf,refx);
    Fifth(t2,tf,py0,pyf,refy);

    _temp_debug_data[0] = py0[0];
    _temp_debug_data[1] = py0[1];
    _temp_debug_data[2] = py0[2];

    _temp_debug_data[3] = refx[0];
    _temp_debug_data[4] = refx[1];
    _temp_debug_data[5] = refx[2];

    _temp_debug_data[6] = refy[0];
    _temp_debug_data[7] = refy[1];
    _temp_debug_data[8] = refy[2];

    GLOBAL_Y_LIPM = refy[0];
    GLOBAL_X_LIPM = refx[0];
    GLOBAL_Z_LIPM = userData->WalkReadyCOM[Zdir];

    GLOBAL_Y_LIPM_d = refy[1];
    GLOBAL_X_LIPM_d = refx[1];

    GLOBAL_X_RF = window[0].right_foot_ref.x;
    GLOBAL_Y_RF = window[0].right_foot_ref.y;

    GLOBAL_X_LF = window[0].left_foot_ref.x;
    GLOBAL_Y_LF = window[0].left_foot_ref.y;

    GLOBAL_ZMP_REF_X = window[0].zmp.x;
    GLOBAL_ZMP_REF_Y = window[0].zmp.y;

    double Global[3],Local[3];
    Global[0] = GLOBAL_X_LIPM;
    Global[1] = GLOBAL_Y_LIPM;
    Global[2] = 0;
    Global2Local(Global,Local);
    GLOBAL_X_LIPM_n =  Local[0];
    GLOBAL_Y_LIPM_n =  Local[1];

    Global[0] = GLOBAL_X_LIPM_d;
    Global[1] = GLOBAL_Y_LIPM_d;
    Global[2] = 0;
    Global2Local(Global,Local);
    GLOBAL_X_LIPM_d_n =  Local[0];
    GLOBAL_Y_LIPM_d_n =  Local[1];

    Global[0] = GLOBAL_X_RF;
    Global[1] = GLOBAL_Y_RF;
    Global[2] = 0;
    Global2Local(Global,Local);
    GLOBAL_X_RF_n = Local[0];
    GLOBAL_Y_RF_n = Local[1];

    Global[0] = GLOBAL_X_LF;
    Global[1] = GLOBAL_Y_LF;
    Global[2] = 0;
    Global2Local(Global,Local);
    GLOBAL_X_LF_n = Local[0];
    GLOBAL_Y_LF_n = Local[1];

    if(pv_Index < 1)
    {
        printf(">>>>>>>>>>>>>>>>>>>  WMG \n");
        printf("zmp[0].x: %f   zmp[0].y: %f  \n",window[0].zmp.x,window[0].zmp.y);
        printf("com x: %f com y: %f  \n",GLOBAL_X_LIPM,GLOBAL_Y_LIPM);
        printf("GLOBAL_X_RF : %f   GLOBAL_Y_RF y: %f  \n",GLOBAL_X_RF,GLOBAL_Y_RF);
        printf("GLOBAL_X_LF : %f   GLOBAL_Y_LF y: %f  \n",GLOBAL_X_LF,GLOBAL_Y_LF);
    }
    pv_Index++ ;
}


void UBIK()
{
    memcpy(wbwalk.fkik.WBIK_Q0,wbwalk.fkik.WBIK_Q,34*sizeof(double));

    kine_drc_hubo4.FK_COM_Global(wbwalk.fkik.WBIK_Q, wbwalk.fkik.FK.pCOM_3x1);
    kine_drc_hubo4.FK_RightFoot_Global(wbwalk.fkik.WBIK_Q, wbwalk.fkik.FK.pRF_3x1, wbwalk.fkik.FK.qRF_4x1);
    kine_drc_hubo4.FK_LeftFoot_Global(wbwalk.fkik.WBIK_Q, wbwalk.fkik.FK.pLF_3x1, wbwalk.fkik.FK.qLF_4x1);
    kine_drc_hubo4.FK_RightHand_Local(wbwalk.fkik.WBIK_Q, wbwalk.fkik.RH_ref_frame, wbwalk.fkik.FK.pRH_3x1, wbwalk.fkik.FK.qRH_4x1, wbwalk.fkik.FK.REB_ang);
    kine_drc_hubo4.FK_LeftHand_Local(wbwalk.fkik.WBIK_Q, wbwalk.fkik.LH_ref_frame, wbwalk.fkik.FK.pLH_3x1, wbwalk.fkik.FK.qLH_4x1, wbwalk.fkik.FK.LEB_ang);

    for(int i=0;i<3;i++)
    {
        wbwalk.fkik.desired.pRH_3x1[i] = wbwalk.fkik.FK.pRH_3x1[i];
        wbwalk.fkik.desired.pLH_3x1[i] = wbwalk.fkik.FK.pLH_3x1[i];
    }
    for(int i=0;i<4;i++)
    {
        wbwalk.fkik.desired.qRH_4x1[i] = wbwalk.fkik.FK.qRH_4x1[i];
        wbwalk.fkik.desired.qLH_4x1[i] = wbwalk.fkik.FK.qLH_4x1[i];
    }
    wbwalk.fkik.desired.REB_ang = wbwalk.fkik.FK.REB_ang;
    wbwalk.fkik.desired.LEB_ang = wbwalk.fkik.FK.LEB_ang;


    /********** YUJIN add RH pos ***************/
//    wbwalk.fkik.desired.pRH_3x1[1] += 0.00005;
//    wbwalk.fkik.desired.pLH_3x1[1] += 0.00005;


    for(int i=0;i<3;i++)
    {
        wbwalk.fkik.desired.pLH_3x1[i] += wbwalk.pLH[i].reference;
        wbwalk.fkik.desired.pRH_3x1[i] += wbwalk.pRH[i].reference;
    }
    for(int i=0;i<4;i++)
    {
        wbwalk.fkik.desired.qRH_4x1[i] = wbwalk.qRH.reference[i];
        wbwalk.fkik.desired.qLH_4x1[i] = wbwalk.qLH.reference[i];
    }

    if(MODE_compliance == ON)
    {
        wbwalk.fkik.desired.pRH_3x1[0] += ComplianceX[0];
        wbwalk.fkik.desired.pRH_3x1[1] += ComplianceY[0];
        wbwalk.fkik.desired.pRH_3x1[2] += ComplianceZ[0];

        wbwalk.fkik.desired.pLH_3x1[0] += ComplianceX[1];
        wbwalk.fkik.desired.pLH_3x1[1] += ComplianceY[1];
        wbwalk.fkik.desired.pLH_3x1[2] += ComplianceZ[1];
    }

    wbwalk.fkik.desired.WST_ang += wbwalk.thWST.reference;
    wbwalk.fkik.desired.REB_ang += wbwalk.thREB.reference;
    wbwalk.fkik.desired.LEB_ang += wbwalk.thLEB.reference;

    kine_drc_hubo4.IK_UpperBody_Local(wbwalk.fkik.WBIK_Q0, wbwalk.fkik.desired.pRH_3x1, wbwalk.fkik.desired.qRH_4x1, wbwalk.fkik.desired.REB_ang, wbwalk.fkik.RH_ref_frame, wbwalk.fkik.desired.pLH_3x1, wbwalk.fkik.desired.qLH_4x1, wbwalk.fkik.desired.LEB_ang, wbwalk.fkik.LH_ref_frame, wbwalk.fkik.desired.WST_ang, wbwalk.fkik.WBIK_Q);

    wbwalk.fkik.Qub[idRSR] = wbwalk.fkik.WBIK_Q[idRSR];
    wbwalk.fkik.Qub[idRSP] = wbwalk.fkik.WBIK_Q[idRSP];
    wbwalk.fkik.Qub[idRSY] = wbwalk.fkik.WBIK_Q[idRSY];
    wbwalk.fkik.Qub[idREB] = wbwalk.fkik.WBIK_Q[idREB];
    wbwalk.fkik.Qub[idRWY] = wbwalk.fkik.WBIK_Q[idRWY];
    wbwalk.fkik.Qub[idRWP] = wbwalk.fkik.WBIK_Q[idRWP];
    wbwalk.fkik.Qub[idRWY] = wbwalk.fkik.WBIK_Q[idRWY];
    wbwalk.fkik.Qub[idRWY2] =wbwalk.fkik.WBIK_Q[idRWY2];

    wbwalk.fkik.Qub[idLSR] = wbwalk.fkik.WBIK_Q[idLSR];
    wbwalk.fkik.Qub[idLSP] = wbwalk.fkik.WBIK_Q[idLSP];
    wbwalk.fkik.Qub[idLSY] = wbwalk.fkik.WBIK_Q[idLSY];
    wbwalk.fkik.Qub[idLEB] = wbwalk.fkik.WBIK_Q[idLEB];
    wbwalk.fkik.Qub[idLWY] = wbwalk.fkik.WBIK_Q[idLWY];
    wbwalk.fkik.Qub[idLWP] = wbwalk.fkik.WBIK_Q[idLWP];
    wbwalk.fkik.Qub[idLWY] = wbwalk.fkik.WBIK_Q[idLWY];
    wbwalk.fkik.Qub[idLWY2] =wbwalk.fkik.WBIK_Q[idLWY2];

    wbwalk.fkik.Qub[idWST] = wbwalk.fkik.WBIK_Q[idWST];

    kine_drc_hubo4.IK_LowerBody_Global(wbwalk.fkik.WBIK_Q0,wbwalk.fkik.Qub,wbwalk.fkik.FK.pCOM_3x1, wbwalk.fkik.FK.qPEL_4x1, wbwalk.fkik.FK.pRF_3x1, wbwalk.fkik.FK.qRF_4x1, wbwalk.fkik.FK.pLF_3x1, wbwalk.fkik.FK.qLF_4x1,wbwalk.fkik.WBIK_Q);

    for(int i=0; i<=LAR; i++)
    {
        wbwalk.fkik.FWRefAngleCurrent[i] = wbwalk.fkik.WBIK_Q[i+7]*R2D;
        jCon->Joints[i]->RefAngleCurrent = wbwalk.fkik.FWRefAngleCurrent[i];
    }

    jCon->Joints[RSP]->RefAngleCurrent = wbwalk.fkik.WBIK_Q[idRSP]*R2D;
    jCon->Joints[RSR]->RefAngleCurrent = (wbwalk.fkik.WBIK_Q[idRSR])*R2D-OFFSET_RSR;
    jCon->Joints[RSY]->RefAngleCurrent = wbwalk.fkik.WBIK_Q[idRSY]*R2D;
    jCon->Joints[REB]->RefAngleCurrent = (wbwalk.fkik.WBIK_Q[idREB])*R2D-OFFSET_ELB;
    jCon->Joints[RWY]->RefAngleCurrent = wbwalk.fkik.WBIK_Q[idRWY]*R2D;
    jCon->Joints[RWP]->RefAngleCurrent = wbwalk.fkik.WBIK_Q[idRWP]*R2D;
    jCon->Joints[RWY2]->RefAngleCurrent= wbwalk.fkik.WBIK_Q[idRWY2]*R2D;

    jCon->Joints[LSP]->RefAngleCurrent = wbwalk.fkik.WBIK_Q[idLSP]*R2D;
    jCon->Joints[LSR]->RefAngleCurrent = (wbwalk.fkik.WBIK_Q[idLSR])*R2D-OFFSET_LSR;
    jCon->Joints[LSY]->RefAngleCurrent = wbwalk.fkik.WBIK_Q[idLSY]*R2D;
    jCon->Joints[LEB]->RefAngleCurrent = (wbwalk.fkik.WBIK_Q[idLEB])*R2D-OFFSET_ELB;
    jCon->Joints[LWY]->RefAngleCurrent = wbwalk.fkik.WBIK_Q[idLWY]*R2D;
    jCon->Joints[LWP]->RefAngleCurrent = wbwalk.fkik.WBIK_Q[idLWP]*R2D;
    jCon->Joints[LWY2]->RefAngleCurrent= wbwalk.fkik.WBIK_Q[idLWY2]*R2D;
    jCon->Joints[WST]->RefAngleCurrent= wbwalk.fkik.WBIK_Q[idWST]*R2D;

}

void WBIK()
{
    double temp1des_qPEL_4x1[4],temp2des_qPEL_4x1[4],temp3des_qPEL_4x1[4],temp4des_qPEL_4x1[4];
    double temp1des_qRF_4x1[4],temp2des_qRF_4x1[4],temp3des_qRF_4x1[4],temp4des_qRF_4x1[4],temp5des_qRF_4x1[4];
    double temp1des_qLF_4x1[4],temp2des_qLF_4x1[4],temp3des_qLF_4x1[4],temp4des_qLF_4x1[4],temp5des_qLF_4x1[4];
    double RightYaw,RightRoll,RightPitch,LeftYaw,LeftRoll,LeftPitch;

    // Task Space Command

    if(pv_Index ==1)
    {
        // ankle torque control;

        RDPitch = RMYC(2,2,0,0.,0.,0./1000.0,0./1000.0,wbwalk.fkik.desired.pLF_3x1[0],wbwalk.fkik.desired.pRF_3x1[0],wbwalk.fkik.desired.pLF_3x1[1],wbwalk.fkik.desired.pRF_3x1[1],0.,0.);
        LDPitch = LMYC(2,2,0,0.,0.,0./1000.0,0./1000.0,wbwalk.fkik.desired.pLF_3x1[0],wbwalk.fkik.desired.pRF_3x1[0],wbwalk.fkik.desired.pLF_3x1[1],wbwalk.fkik.desired.pRF_3x1[1],0.,0.);
        RDPitch =0.;
        LDPitch =0.;
        RDRoll = RMXC(2,2,0,0.,0.,0./1000.0,0./1000.0,wbwalk.fkik.desired.pLF_3x1[0],wbwalk.fkik.desired.pRF_3x1[0],wbwalk.fkik.desired.pLF_3x1[1],wbwalk.fkik.desired.pRF_3x1[1],0.,0.);
        LDRoll = LMXC(2,2,0,0.,0.,0./1000.0,0./1000.0,wbwalk.fkik.desired.pLF_3x1[0],wbwalk.fkik.desired.pRF_3x1[0],wbwalk.fkik.desired.pLF_3x1[1],wbwalk.fkik.desired.pRF_3x1[1],0.,0.);
        RDRoll =0.;
        LDRoll =0.;

        RDPitch2 = RMYC2(2,2,0,0.,0.,0./1000.0,0./1000.0,wbwalk.fkik.desired.pLF_3x1[0],wbwalk.fkik.desired.pRF_3x1[0],wbwalk.fkik.desired.pLF_3x1[1],wbwalk.fkik.desired.pRF_3x1[1],0.,0.);
        LDPitch2= LMYC2(2,2,0,0.,0.,0./1000.0,0./1000.0,wbwalk.fkik.desired.pLF_3x1[0],wbwalk.fkik.desired.pRF_3x1[0],wbwalk.fkik.desired.pLF_3x1[1],wbwalk.fkik.desired.pRF_3x1[1],0.,0.);
        RDPitch2 =0.;
        LDPitch2 =0.;

        RDRoll2 = RMXC2(2,2,0,0.,0.,0./1000.0,0./1000.0,wbwalk.fkik.desired.pLF_3x1[0],wbwalk.fkik.desired.pRF_3x1[0],wbwalk.fkik.desired.pLF_3x1[1],wbwalk.fkik.desired.pRF_3x1[1],0.,0.);
        LDRoll2= LMXC2(2,2,0,0.,0.,0./1000.0,0./1000.0,wbwalk.fkik.desired.pLF_3x1[0],wbwalk.fkik.desired.pRF_3x1[0],wbwalk.fkik.desired.pLF_3x1[1],wbwalk.fkik.desired.pRF_3x1[1],0.,0.);
        RDRoll2 =0.;
        LDRoll2 =0.;

        Zctrl = FootForceControl(0,RDF,LDF,sharedData->FT[RAFT].Fz,sharedData->FT[LAFT].Fz,1,0.0);
        Zctrl = 0.;
        Zctrl2 = FootForceControl2(0,RDF,LDF,sharedData->FT[RAFT].Fz,sharedData->FT[LAFT].Fz,1,0.0);
        Zctrl2 = 0.;

        HUBO2ZMPInitLegLength(0.,0.,0);
        Add_FootTask[RIGHT][Zdir] = 0.;
        Add_FootTask[LEFT][Zdir] = 0.;

        RecoverRightLegLength(0.,0.,0);
        RecoverLeftLegLength(0.,0.,0);
        Add_Leg_Recovery[RIGHT][Zdir] = 0.;
        Add_Leg_Recovery[LEFT][Zdir] = 0.;

        // zmp control
        kirkZMPCon_XP2(0,0,0);
        kirkZMPCon_YP2(0,0,0);
        Del_PC_X_DSP_XZMP_CON = 0;
        Del_PC_Y_DSP_YZMP_CON = 0;

        I_ZMP_CON_X = 0.0f;
        I_ZMP_CON_Y = 0.0f;

        y_i_1 = 0;
        y_i_11= 0;
        u_i_1 = 0;
        u_i_11 = 0;
        NotchFilter_GyroRollControlInput(0,0);
        NotchFilter_GyroPitchControlInput(0,0);
        NotchFilter_GyroRollVel(0,0);
        NotchFilter_GyroPitchVel(0,0);
        GLOBAL_Xori_RF_last = 0;
        GLOBAL_Xori_LF_last = 0;
        GLOBAL_Yori_RF_last = 0;
        GLOBAL_Yori_LF_last = 0;

        GLOBAL_Xori_RF2_last = 0;
        GLOBAL_Xori_LF2_last = 0;
        GLOBAL_Yori_RF2_last = 0;

        GLOBAL_Yori_LF2_last = 0;

        U_Gain = 0.;
        GLOBAL_Xori_RF = 0.;
        GLOBAL_Xori_LF = 0.;
        GLOBAL_Yori_RF = 0.;
        GLOBAL_Yori_LF = 0.;

        printf("WBIK >>>>>>>>>>>>>>>>> \n");
        printf("GLOBAL_X_LIPM_n : %f   GLOBAL_Y_LIPM_n:%f \n",GLOBAL_X_LIPM_n,GLOBAL_Y_LIPM_n);
        printf("GLOBAL_Y_LF_n : %f   GLOBAL_Y_RF_n:%f \n",GLOBAL_Y_LF_n,GLOBAL_Y_RF_n);
        printf("GLOBAL_X_RF : %f   GLOBAL_Y_RF:%f \n",GLOBAL_X_RF,GLOBAL_Y_RF);
        printf("GLOBAL_X_LF : %f   GLOBAL_Y_LF:%f \n",GLOBAL_X_LF,GLOBAL_Y_LF);
    }

    // pelvis
    qtRX(1.0*0*D2R, temp2des_qPEL_4x1);
    qtRY(1.0*0*D2R, temp4des_qPEL_4x1);

    QTcross(temp1des_qPEL_4x1,temp2des_qPEL_4x1,temp3des_qPEL_4x1);
    QTcross(temp3des_qPEL_4x1,temp4des_qPEL_4x1,wbwalk.fkik.desired.qPEL_4x1);

    //------------- CoM
    Local[0] = GLOBAL_X_LIPM_n + G_DSP_X*(- 0.001*Del_PC_X_DSP_XZMP_CON*G_DSP_X + I_ZMP_CON_X*1.)*ZMP_FeedBack_ONOFF;// + ZMPControlX*OnOff_compliance;
    if(Inv_ONOFF == 0){
        U_Gain = 0;
    }
    CONT_Y = Local[1] =  (GLOBAL_Y_LIPM_n)*(U0_Gain)+ (-0.001*Del_PC_Y_DSP_YZMP_CON*G_DSP_Y - 0.001*Del_PC_Y_SSP_YZMP_CON*0+ I_ZMP_CON_Y*0.)*ZMP_FeedBack_ONOFF;// + ZMPControlY*OnOff_compliance;

    Local[2]=0;
    Local2Global(Local,Global);

    wbwalk.fkik.desired_hat.pCOM_3x1[Xdir] = Global[0];
    wbwalk.fkik.desired_hat.pCOM_3x1[Ydir] = Global[1];
    wbwalk.fkik.desired_hat.pCOM_3x1[Zdir] = userData->WalkReadyCOM[Zdir];// + GLOBAL_Z_LIPM;// - (fsm->AddRightFootInfos[0][2] + fsm->AddLeftFootInfos[0][2])*0.7;

    double RotX[9],RotY[9],RotZ[9],RotYX[9],TorsoOri[3],TorsoOri_n[3];
    TorsoOri[0] = 0.*D2R;
    TorsoOri[1] = 0.*D2R;
    TorsoOri[2] = 0.;

    RZ((window[0].right_foot_ref.yaw*D2R + window[0].left_foot_ref.yaw*D2R)/2.,RotZ);
    mult_mv(RotZ,3,3,TorsoOri,TorsoOri_n);

    RX(TorsoOri_n[0],RotX);
    RY(TorsoOri_n[1],RotY);

    mult_mm(RotY,3,3,RotX,3,RotYX);

    mult_mv(RotYX,3,3,wbwalk.fkik.desired_hat.pCOM_3x1,wbwalk.fkik.desired.pCOM_3x1);
    //---------------

    // ------------ RF
    wbwalk.fkik.desired_hat.pRF_3x1[Xdir] = GLOBAL_X_RF;
    wbwalk.fkik.desired_hat.pRF_3x1[Ydir] = GLOBAL_Y_RF;
    wbwalk.fkik.desired_hat.pRF_3x1[Zdir] = GLOBAL_Z_RF;// + Zctrl*0.5*impONOFF+ Zctrl2*0.5*impONOFF2+ Add_FootTask[RIGHT][Zdir]*Leg_Length_FeedBack_ONOFF + Add_Leg_Recovery[RIGHT][Zdir]*Leg_Length_Recover_ONOFF;

    mult_mv(RotYX,3,3,wbwalk.fkik.desired_hat.pRF_3x1,wbwalk.fkik.desired.pRF_3x1);

    // ------------ RF Orietation
    double RY,RP,RR;
    double temp_QT[4]={1,0,0,0};
    QTcross(temp_QT,temp2des_qPEL_4x1,temp1des_qPEL_4x1);
    QTcross(temp1des_qPEL_4x1,temp4des_qPEL_4x1,temp3des_qPEL_4x1);
    QT2YPR(temp3des_qPEL_4x1,RY,RP,RR);

    RightYaw = window[0].right_foot_ref.yaw*D2R;
    RightPitch = window[0].right_foot_ref.pitch*D2R;
    RightRoll  = window[0].right_foot_ref.roll*D2R;

    qtRZ(RightYaw, temp1des_qRF_4x1);
    qtRY(RightPitch, temp4des_qRF_4x1);
    qtRX(RightRoll, temp2des_qRF_4x1);

    QTcross(temp1des_qRF_4x1,temp4des_qRF_4x1,temp3des_qRF_4x1);
    QTcross(temp3des_qRF_4x1,temp2des_qRF_4x1,temp5des_qRF_4x1);

    wbwalk.fkik.desired.qRF_4x1[0] = temp5des_qRF_4x1[0];
    wbwalk.fkik.desired.qRF_4x1[1] = temp5des_qRF_4x1[1];
    wbwalk.fkik.desired.qRF_4x1[2] = temp5des_qRF_4x1[2];
    wbwalk.fkik.desired.qRF_4x1[3] = temp5des_qRF_4x1[3];

    // ------------ LF
    wbwalk.fkik.desired_hat.pLF_3x1[Xdir] = GLOBAL_X_LF;
    wbwalk.fkik.desired_hat.pLF_3x1[Ydir] = GLOBAL_Y_LF;
    wbwalk.fkik.desired_hat.pLF_3x1[Zdir] = GLOBAL_Z_LF;// - Zctrl*0.5*impONOFF- Zctrl2*0.5*impONOFF2+ Add_FootTask[LEFT][Zdir]*Leg_Length_FeedBack_ONOFF + Add_Leg_Recovery[LEFT][Zdir]*Leg_Length_Recover_ONOFF;

    mult_mv(RotYX,3,3,wbwalk.fkik.desired_hat.pLF_3x1,wbwalk.fkik.desired.pLF_3x1);

    // ------------ LF Orietation
    LeftYaw   = window[0].left_foot_ref.yaw*D2R;
    LeftPitch = window[0].left_foot_ref.pitch*D2R;
    LeftRoll  = window[0].left_foot_ref.roll*D2R;

    qtRZ(LeftYaw, temp1des_qLF_4x1);
    qtRY(LeftPitch, temp4des_qLF_4x1);
    qtRX(LeftRoll, temp2des_qLF_4x1);

    QTcross(temp1des_qLF_4x1,temp4des_qLF_4x1,temp3des_qLF_4x1);
    QTcross(temp3des_qLF_4x1,temp2des_qLF_4x1,temp5des_qLF_4x1);

    wbwalk.fkik.desired.qLF_4x1[0] = temp5des_qLF_4x1[0];
    wbwalk.fkik.desired.qLF_4x1[1] = temp5des_qLF_4x1[1];
    wbwalk.fkik.desired.qLF_4x1[2] = temp5des_qLF_4x1[2];
    wbwalk.fkik.desired.qLF_4x1[3] = temp5des_qLF_4x1[3];


    if(pv_Index <=1)
    {
        printf("%d WBIK*********** right x: %f  y:%f   z:%f  \n",pv_Index, wbwalk.fkik.desired.pRF_3x1[0],wbwalk.fkik.desired.pRF_3x1[1],wbwalk.fkik.desired.pRF_3x1[2]);
        printf("%d WBIK*********** left  x: %f  y:%f   z:%f  \n",pv_Index,wbwalk.fkik.desired.pLF_3x1[0],wbwalk.fkik.desired.pLF_3x1[1],wbwalk.fkik.desired.pLF_3x1[2]);
        printf("%d WBIK COM x: %f  y: %f   z:%f \n",pv_Index,wbwalk.fkik.desired.pCOM_3x1[0],wbwalk.fkik.desired.pCOM_3x1[1],wbwalk.fkik.desired.pCOM_3x1[2]);
        printf("%d  Right WBIK_Q : %f  %f  %f  %f  %f  %f \n",pv_Index,wbwalk.fkik.WBIK_Q[7],wbwalk.fkik.WBIK_Q[8],wbwalk.fkik.WBIK_Q[9],wbwalk.fkik.WBIK_Q[10],wbwalk.fkik.WBIK_Q[11],wbwalk.fkik.WBIK_Q[12]);
        printf("%d  Left  WBIK_Q : %f  %f  %f  %f  %f  %f \n",pv_Index,wbwalk.fkik.WBIK_Q[13],wbwalk.fkik.WBIK_Q[14],wbwalk.fkik.WBIK_Q[15],wbwalk.fkik.WBIK_Q[16],wbwalk.fkik.WBIK_Q[17],wbwalk.fkik.WBIK_Q[18]);
        printf("%d  WBIK Pelvis :  %f   %f   %f \n",pv_Index,wbwalk.fkik.WBIK_Q[0],wbwalk.fkik.WBIK_Q[1],wbwalk.fkik.WBIK_Q[2]);
    }

    // 1 . PELVIS Orientation
    Pel_Yaw = (window[0].right_foot_ref.yaw + window[0].left_foot_ref.yaw)*D2R/2.;
    qtRZ(Pel_Yaw, temp1des_qPEL_4x1);
    qtRX(1.0*0*D2R, temp2des_qPEL_4x1);
    qtRY(1.0*0*D2R, temp4des_qPEL_4x1);

    QTcross(temp1des_qPEL_4x1,temp2des_qPEL_4x1,temp3des_qPEL_4x1);
    QTcross(temp3des_qPEL_4x1,temp4des_qPEL_4x1,wbwalk.fkik.desired.qPEL_4x1);

    // 2. Right Foot Orientation
    RightYaw = window[0].right_foot_ref.yaw*D2R;
    RightPitch = window[0].right_foot_ref.pitch*D2R;
    RightRoll  = window[0].right_foot_ref.roll*D2R;

    qtRZ(RightYaw, temp1des_qRF_4x1);
    qtRY(RightPitch, temp4des_qRF_4x1);
    qtRX(RightRoll, temp2des_qRF_4x1);

    QTcross(temp1des_qRF_4x1,temp4des_qRF_4x1,temp3des_qRF_4x1);
    QTcross(temp3des_qRF_4x1,temp2des_qRF_4x1,temp5des_qRF_4x1);

    wbwalk.fkik.desired.qRF_4x1[0] = temp5des_qRF_4x1[0];
    wbwalk.fkik.desired.qRF_4x1[1] = temp5des_qRF_4x1[1];
    wbwalk.fkik.desired.qRF_4x1[2] = temp5des_qRF_4x1[2];
    wbwalk.fkik.desired.qRF_4x1[3] = temp5des_qRF_4x1[3];

    // 6 . LF Orietation
    LeftYaw = window[0].left_foot_ref.yaw*D2R ;
    LeftPitch = window[0].left_foot_ref.pitch*D2R ;
    LeftRoll = window[0].left_foot_ref.roll*D2R ;

    qtRZ(LeftYaw, temp1des_qLF_4x1);
    qtRY(LeftPitch, temp4des_qLF_4x1);
    qtRX(LeftRoll, temp2des_qLF_4x1);

    QTcross(temp1des_qLF_4x1,temp4des_qLF_4x1,temp3des_qLF_4x1);
    QTcross(temp3des_qLF_4x1,temp2des_qLF_4x1,temp5des_qLF_4x1);

    wbwalk.fkik.desired.qLF_4x1[0] = temp5des_qLF_4x1[0];
    wbwalk.fkik.desired.qLF_4x1[1] = temp5des_qLF_4x1[1];
    wbwalk.fkik.desired.qLF_4x1[2] = temp5des_qLF_4x1[2];
    wbwalk.fkik.desired.qLF_4x1[3] = temp5des_qLF_4x1[3];

    memcpy(wbwalk.fkik.WBIK_Q0,wbwalk.fkik.WBIK_Q,34*sizeof(double));

    kine_drc_hubo4.FK_COM_Global(wbwalk.fkik.WBIK_Q0,FK0com);
    kine_drc_hubo4.FK_RightFoot_Global(wbwalk.fkik.WBIK_Q, wbwalk.fkik.FK.pRF_3x1, wbwalk.fkik.FK.qRF_4x1);
    kine_drc_hubo4.FK_LeftFoot_Global(wbwalk.fkik.WBIK_Q, wbwalk.fkik.FK.pLF_3x1, wbwalk.fkik.FK.qLF_4x1);
    kine_drc_hubo4.FK_RightHand_Local(wbwalk.fkik.WBIK_Q, wbwalk.fkik.RH_ref_frame, wbwalk.fkik.FK.pRH_3x1, wbwalk.fkik.FK.qRH_4x1, wbwalk.fkik.FK.REB_ang);
    kine_drc_hubo4.FK_LeftHand_Local(wbwalk.fkik.WBIK_Q, wbwalk.fkik.LH_ref_frame, wbwalk.fkik.FK.pLH_3x1, wbwalk.fkik.FK.qLH_4x1, wbwalk.fkik.FK.LEB_ang);

    kine_drc_hubo4.FK_RightHand_Local(wbwalk.fkik.WBIK_Q, 2, wbwalk.fkik.desired.pRH_3x1, wbwalk.fkik.desired.qRH_4x1, wbwalk.fkik.desired.REB_ang);
    kine_drc_hubo4.FK_LeftHand_Local(wbwalk.fkik.WBIK_Q, 2, wbwalk.fkik.desired.pLH_3x1, wbwalk.fkik.desired.qLH_4x1, wbwalk.fkik.desired.LEB_ang);

    /********** YUJIN add RH pos ***************/
    for(int i=0;i<3;i++)
    {
        wbwalk.fkik.desired.pRH_3x1[i] += wbwalk.pRH[i].reference;
        wbwalk.fkik.desired.pLH_3x1[i] += wbwalk.pLH[i].reference;
    }
    for(int i=0;i<4;i++)
    {
        wbwalk.fkik.desired.qRH_4x1[i] = wbwalk.qRH.reference[i];
        wbwalk.fkik.desired.qLH_4x1[i] = wbwalk.qLH.reference[i];
    }
    wbwalk.fkik.desired.REB_ang = wbwalk.fkik.FK.REB_ang;
    wbwalk.fkik.desired.LEB_ang = wbwalk.fkik.FK.LEB_ang;


    wbwalk.fkik.desired.WST_ang += wbwalk.thWST.reference;
    wbwalk.fkik.desired.REB_ang += wbwalk.thREB.reference;
    wbwalk.fkik.desired.LEB_ang += wbwalk.thLEB.reference;

    if(MODE_compliance == ON)
    {
        wbwalk.fkik.desired.pRH_3x1[0] += ComplianceX[0];
        wbwalk.fkik.desired.pRH_3x1[1] += ComplianceY[0];
        wbwalk.fkik.desired.pRH_3x1[2] += ComplianceZ[0];

        wbwalk.fkik.desired.pLH_3x1[0] += ComplianceX[1];
        wbwalk.fkik.desired.pLH_3x1[1] += ComplianceY[1];
        wbwalk.fkik.desired.pLH_3x1[2] += ComplianceZ[1];
    }

    kine_drc_hubo4.IK_UpperBody_Local(wbwalk.fkik.WBIK_Q0, wbwalk.fkik.desired.pRH_3x1, wbwalk.fkik.desired.qRH_4x1, wbwalk.fkik.desired.REB_ang, wbwalk.fkik.RH_ref_frame, wbwalk.fkik.desired.pLH_3x1, wbwalk.fkik.desired.qLH_4x1, wbwalk.fkik.desired.LEB_ang, wbwalk.fkik.LH_ref_frame, wbwalk.fkik.desired.WST_ang, wbwalk.fkik.WBIK_Q);

    wbwalk.fkik.Qub[idRSR] = wbwalk.fkik.WBIK_Q[idRSR];
    wbwalk.fkik.Qub[idRSP] = wbwalk.fkik.WBIK_Q[idRSP];
    wbwalk.fkik.Qub[idRSY] = wbwalk.fkik.WBIK_Q[idRSY];
    wbwalk.fkik.Qub[idREB] = wbwalk.fkik.WBIK_Q[idREB];
    wbwalk.fkik.Qub[idRWY] = wbwalk.fkik.WBIK_Q[idRWY];
    wbwalk.fkik.Qub[idRWP] = wbwalk.fkik.WBIK_Q[idRWP];
    wbwalk.fkik.Qub[idRWY] = wbwalk.fkik.WBIK_Q[idRWY];
    wbwalk.fkik.Qub[idRWY2] = wbwalk.fkik.WBIK_Q[idRWY2];

    wbwalk.fkik.Qub[idLSR] = wbwalk.fkik.WBIK_Q[idLSR];
    wbwalk.fkik.Qub[idLSP] = wbwalk.fkik.WBIK_Q[idLSP];
    wbwalk.fkik.Qub[idLSY] = wbwalk.fkik.WBIK_Q[idLSY];
    wbwalk.fkik.Qub[idLEB] = wbwalk.fkik.WBIK_Q[idLEB];
    wbwalk.fkik.Qub[idLWY] = wbwalk.fkik.WBIK_Q[idLWY];
    wbwalk.fkik.Qub[idLWP] = wbwalk.fkik.WBIK_Q[idLWP];
    wbwalk.fkik.Qub[idLWY] = wbwalk.fkik.WBIK_Q[idLWY];
    wbwalk.fkik.Qub[idLWY2] = wbwalk.fkik.WBIK_Q[idLWY2];

    wbwalk.fkik.Qub[idWST] = wbwalk.fkik.WBIK_Q[idWST];

    kine_drc_hubo4.IK_LowerBody_Global(wbwalk.fkik.WBIK_Q0,wbwalk.fkik.Qub,wbwalk.fkik.desired.pCOM_3x1, wbwalk.fkik.desired.qPEL_4x1, wbwalk.fkik.desired.pRF_3x1, wbwalk.fkik.desired.qRF_4x1, wbwalk.fkik.desired.pLF_3x1, wbwalk.fkik.desired.qLF_4x1,wbwalk.fkik.WBIK_Q);


    // Ankle Joint control using gyro

    double temprhh[4], temprhhh;
    kine_drc_hubo4.FK_COM_Global(wbwalk.fkik.WBIK_Q,FKcom);
   // kine_drc_hubo4.FK_RightHand_Global(wbwalk.fkik.WBIK_Q, FKrh, temprhh, temprhhh);

    GLOBAL_Xori_RF_n = GLOBAL_Xori_RF*cos(wbwalk.fkik.WBIK_Q[idRHY]) + GLOBAL_Yori_RF*sin(wbwalk.fkik.WBIK_Q[idRHY]);
    GLOBAL_Yori_RF_n =-GLOBAL_Xori_RF*sin(wbwalk.fkik.WBIK_Q[idRHY]) + GLOBAL_Yori_RF*cos(wbwalk.fkik.WBIK_Q[idRHY]);

    GLOBAL_Xori_LF_n = GLOBAL_Xori_LF*cos(wbwalk.fkik.WBIK_Q[idLHY]) + GLOBAL_Yori_LF*sin(wbwalk.fkik.WBIK_Q[idLHY]);
    GLOBAL_Yori_LF_n =-GLOBAL_Xori_LF*sin(wbwalk.fkik.WBIK_Q[idLHY]) + GLOBAL_Yori_LF*cos(wbwalk.fkik.WBIK_Q[idLHY]);


    for(int i=0; i<=LAR; i++)
    {
        wbwalk.fkik.FWRefAngleCurrent[i] = wbwalk.fkik.WBIK_Q[i+7]*R2D;

        wbwalk.fkik.FWRefAngleCurrent[RAR] = wbwalk.fkik.WBIK_Q[RAR+7]*R2D + GLOBAL_Xori_RF*Gyro_Ankle_FeedBack_ONOFF + deflection_comp_RAR*Sagging_Comp_ONOFF - RDRoll*ssp_torque_ONOFF;
        wbwalk.fkik.FWRefAngleCurrent[RAP] = wbwalk.fkik.WBIK_Q[RAP+7]*R2D + GLOBAL_Yori_RF*Gyro_Ankle_FeedBack_ONOFF - RDPitch*ssp_torque_ONOFF;

        wbwalk.fkik.FWRefAngleCurrent[LAR] = wbwalk.fkik.WBIK_Q[LAR+7]*R2D + GLOBAL_Xori_LF*Gyro_Ankle_FeedBack_ONOFF + deflection_comp_LAR*Sagging_Comp_ONOFF - LDRoll*ssp_torque_ONOFF;
        wbwalk.fkik.FWRefAngleCurrent[LAP] = wbwalk.fkik.WBIK_Q[LAP+7]*R2D + GLOBAL_Yori_LF*Gyro_Ankle_FeedBack_ONOFF - LDPitch*ssp_torque_ONOFF;

        jCon->Joints[i]->RefAngleCurrent = wbwalk.fkik.FWRefAngleCurrent[i];
    }

    jCon->Joints[RSP]->RefAngleCurrent = wbwalk.fkik.WBIK_Q[idRSP]*R2D;
    jCon->Joints[RSR]->RefAngleCurrent = (wbwalk.fkik.WBIK_Q[idRSR])*R2D-OFFSET_RSR;
    jCon->Joints[RSY]->RefAngleCurrent = wbwalk.fkik.WBIK_Q[idRSY]*R2D;
    jCon->Joints[REB]->RefAngleCurrent = (wbwalk.fkik.WBIK_Q[idREB])*R2D-OFFSET_ELB;
    jCon->Joints[RWY]->RefAngleCurrent = wbwalk.fkik.WBIK_Q[idRWY]*R2D;
    jCon->Joints[RWP]->RefAngleCurrent = wbwalk.fkik.WBIK_Q[idRWP]*R2D;
    jCon->Joints[RWY2]->RefAngleCurrent= wbwalk.fkik.WBIK_Q[idRWY2]*R2D;
    jCon->Joints[LSP]->RefAngleCurrent = wbwalk.fkik.WBIK_Q[idLSP]*R2D;
    jCon->Joints[LSR]->RefAngleCurrent = (wbwalk.fkik.WBIK_Q[idLSR])*R2D-OFFSET_LSR;
    jCon->Joints[LSY]->RefAngleCurrent = wbwalk.fkik.WBIK_Q[idLSY]*R2D;
    jCon->Joints[LEB]->RefAngleCurrent = (wbwalk.fkik.WBIK_Q[idLEB])*R2D-OFFSET_ELB;
    jCon->Joints[LWY]->RefAngleCurrent = wbwalk.fkik.WBIK_Q[idLWY]*R2D;
    jCon->Joints[LWP]->RefAngleCurrent = wbwalk.fkik.WBIK_Q[idLWP]*R2D;
    jCon->Joints[LWY2]->RefAngleCurrent= wbwalk.fkik.WBIK_Q[idLWY2]*R2D;
    jCon->Joints[WST]->RefAngleCurrent= wbwalk.fkik.WBIK_Q[idWST]*R2D;


    if(MODE_compliance == ON && Command_CART != WALKING)
    {

    } else if(window[0].state == STATE_EMPTY)
    {
        FINAL_TIMER  = FINAL_TIMER + 0.005;

        if(FINAL_TIMER > 3.0)
        {
            printf("Walking is finished and Walkflag is set to 0 \n");
            walk_flag = false;
            FLAG_UBIK = true;

            LandingState = FINAL;
        }
    }
}
void UKSEL(int sampling_tic,double V[15],double U[15][3],double IND[2])
{
    /*
     * Initialize every control period(0.1sec)
     */
    int flag1 = 0,flag2 = 0 ,Rflag1 = 0,Rflag2 = 0,Lflag1 = 0,Lflag2 = 0;
    static int before_ind = 1;

    for(int i = 0;i<3;i++)
    {
        for(int j = 0;j<15;j++)
        {
            U[j][i] = 0.;
            V[j] = 0;
        }
    }

    if(window[0].state == DSP_INIT_RF)
    {
        // Left foot = 1, right foot  = -1
        IND[0] = 1.;
        for(int i=0;i<=14;i++)
        {
                if((window[i*sampling_tic].state == DSP_INIT_RF) && flag1 == 0)
                {
                    U[i][0] = 0;
                    V[i] = 1;
                } else if((window[i*sampling_tic].state == SSP_RF) && flag2 == 0)
                {
                    U[i][0] = 1;
                    V[i] = 0;
                    flag1 = 1;
                } else if((window[i*sampling_tic].state == DSP_LF || window[i*sampling_tic].state == SSP_LF) && flag1 == 1 )
                {
                    U[i][1] = 1;
                    V[i] = 0;
                    flag2 = 1;
                } else if((window[i*sampling_tic].state == DSP_RF || window[i*sampling_tic].state == SSP_RF) && flag2 == 1 )
                {
                    U[i][2] = 1;
                    V[i] = 0;
                }
        }
    } else if(window[0].state == DSP_INIT_LF)
    {
        // Left foot = 1, right foot  = -1
        IND[0] = -1.;
        for(int i=0;i<=14;i++)
        {
                if((window[i*sampling_tic].state == DSP_INIT_LF) && flag1 == 0)
                {
                    U[i][0] = 0;
                    V[i] = 1;
                } else if((window[i*sampling_tic].state == SSP_LF ) && flag2 == 0)
                {
                    U[i][0] = 1;
                    V[i] = 0;
                    flag1 = 1;
                } else if((window[i*sampling_tic].state == DSP_RF || window[i*sampling_tic].state == SSP_RF) && flag1 == 1 )
                {
                    U[i][1] = 1;
                    V[i] = 0;
                } else if((window[i*sampling_tic].state == DSP_LF || window[i*sampling_tic].state == SSP_LF) && flag1 == 1 )
                {
                    U[i][2] = 1;
                    V[i] = 0;
                }
        }

    } else if(window[0].state == SSP_RF || window[0].state == DSP_RF)
    {
        IND[0] = 1;
        before_ind = 1;

        IND[0] = IND[0] + 1;

        if(window[0 + sampling_tic].state == DSP_LF)
        {
            IND[1] = 0;
        }

        Rflag1 = 0; Rflag2=0;
        for(int i=0;i<=14;i++)
        {
                //L SSP 0
                if((window[i*sampling_tic].state == SSP_RF || window[i*sampling_tic].state == DSP_RF) && Lflag1 == 0)
                {
                    U[i][0] = 0;
                    V[i] = 1;
                    Lflag1 = 0;
                    Lflag2 = 0;
                // R SSP 1
                } else if((window[i*sampling_tic].state == SSP_LF ||window[i*sampling_tic].state == DSP_LF) && Lflag2 == 0)
                {
                    U[i][0] = 1;
                    V[i] = 0;
                    Lflag1 = 1;
                // L SSP 2
                } else if((window[i*sampling_tic].state == SSP_RF || window[i*sampling_tic].state == DSP_RF) && Lflag1 == 1)
                {
                    U[i][1] = 1;
                    V[i] = 0;
                    Lflag2 = 1;
                // R SSP 3
                } else if((window[i*sampling_tic].state == SSP_LF ||window[i*sampling_tic].state == DSP_LF) && Lflag2 == 1)
                {
                    U[i][2] = 1;
                    V[i] = 0;
                    Lflag1 = 0;
                } else if((window[i*sampling_tic].state == DSP_FINAL ||window[i*sampling_tic].state == STATE_EMPTY))
                {
                    U[i][0] = 0;
                    U[i][1] = 0;
                    U[i][2] = 0;
                    V[i] = 1;
                }
        }
    // R SSP
    } else if(window[0].state == SSP_LF || window[0].state == DSP_LF)
    {
        // left swing phase
        IND[0] = -1.;
        before_ind = -1;
        IND[1] = IND[1] + 1;

        if(window[0 + sampling_tic].state == DSP_RF)
        {
                IND[1] = 0;
        }

        for(int i=0;i<=14;i++)
        {
            Lflag1 = 0; Lflag2=0;
            //R SSP 0
            if((window[i*sampling_tic].state == SSP_LF || window[i*sampling_tic].state == DSP_LF) && Rflag1 == 0)
            {
                U[i][0] = 0;
                V[i] = 1;
                Rflag1 = 0;
                Rflag2 = 0;
            // R SSP 1
            } else if((window[i*sampling_tic].state == SSP_RF || window[i*sampling_tic].state == DSP_RF) && Rflag2 == 0)
            {
                U[i][0] = 1;
                V[i] = 0;
                Rflag1 = 1;
            // L SSP 2
            } else if((window[i*sampling_tic].state == SSP_LF || window[i*sampling_tic].state == DSP_LF) && Rflag1 == 1)
            {
                U[i][1] = 1;
                V[i] = 0;
                Rflag2 = 1;
            // R SSP 3
            } else if((window[i*sampling_tic].state == SSP_RF || window[i*sampling_tic].state == DSP_RF ) && Rflag2 == 1)
            {
                U[i][2] = 1;
                V[i] = 0;
                Rflag1 = 0;
            } else if((window[i*sampling_tic].state == DSP_FINAL || window[i*sampling_tic].state == STATE_EMPTY ))
            {
                U[i][0] = 0;
                U[i][1] = 0;
                U[i][2] = 0;
                V[i] = 1;
            }
        }
    }
}

void GripperTH()
{
    int Grasping_value = 0;
    int limit_cnt = 200; // 200*5 = 1000ms = 1sec

    switch(Command_GRIPPER)
    {
    case GRIPPER_STOP:
    {
        Grasping_value = 0;
        jCon->SetJointRefAngle(RHAND, Grasping_value);
        jCon->SetJointRefAngle(LHAND, Grasping_value);
        break;
    }
    case GRIPPER_OPEN:
    {
        static int cnt1 = 0;
        Grasping_value = -125;

        if(cnt1 < limit_cnt)
        {
            jCon->SetJointRefAngle(RHAND, Grasping_value);
            jCon->SetJointRefAngle(LHAND, Grasping_value);
        }else
        {
            Command_GRIPPER = GRIPPER_STOP;
            cnt1 = 0;
        }
        break;
    }
    case GRIPPER_CLOSE:
    {
        static int cnt2 = 0;
        Grasping_value = 125;

        if(cnt2 < limit_cnt)
        {
            jCon->SetJointRefAngle(RHAND, Grasping_value);
            jCon->SetJointRefAngle(LHAND, Grasping_value);
        }else
        {
            Command_GRIPPER = GRIPPER_STOP;
            cnt2 = 0;
        }
        break;
    }
    default:
    {
        break;
    }
    }


}

void CartTH()
{
    switch(Command_CART)
    {
    case APPROACH_HANDLE:
    {
        wbwalk.AddRHpos(0.5, -0.25, 0.25, 1.);
        wbwalk.AddLHpos(0.5, 0.25, 0.25, 1.);
        Command_CART = CART_NO_ACT;
        break;
    }
    case GRASP_HANDLE:
    {
        usleep(1500*1000);
        FILE_LOG(logSUCCESS) << "MODE CHANGED : Compliance Mode OFF -> ON";
        Command_CART = CART_NO_ACT;
        MODE_compliance = ON;
        break;
    }
    case WALKING:
    {

        break;
    }
    case QUIT_HANDLE:
    {
        FILE_LOG(logSUCCESS) << "MODE CHANGED : Compliance Mode ON -> OFF";
        MODE_compliance = OFF;
        break;
    }
    default:
    {
        break;
    }
    }
}

/****************************** 6. Controller *******************************/
void Controller(){

    double init_start_time = 0.6,init_final_time = 1.5;

    switch(window[0].state)
    {
    case DSP_INIT_RF:
    {// For continous walking
        LandingState = DSP;

        if((window[0].timer.current>=init_start_time)&&(window[0].timer.current<=init_final_time))
        {
            U_Gain = 0.5*(1-cos(PI*(window[0].timer.current - init_start_time)/(init_final_time - init_start_time)));
        }
        if(window[0].timer.current<=0.7)
        {
            G_DSP_X = G_DSP_Y = U_Gain_DSP = 0.5*(1-cos(PI*(window[0].timer.current)/(0.7)));
        }
        Leg_Length_Control();
        break;
    }
    case DSP_INIT_LF:
    {// For continous walking
        LandingState = DSP;

        if((window[0].timer.current>=init_start_time)&&(window[0].timer.current<=init_final_time))
        {
            U_Gain = 0.5*(1-cos(PI*(window[0].timer.current - init_start_time)/(init_final_time - init_start_time)));
        }
        if(window[0].timer.current<=0.7)
        {
            G_DSP_X = G_DSP_Y = U_Gain_DSP = 0.5*(1-cos(PI*(window[0].timer.current)/(0.7)));
        }
        Leg_Length_Control();
        break;
    }
    case DSP_FINAL:
    {
        if(LandingState == RSSP)
        {
            Pre_LandingState = RSSP;
        } else if(LandingState == LSSP)
        {
            Pre_LandingState = LSSP;
        }
        LandingState = FINAL;

        Upperbody_Gain_Lock();
        Leg_Length_Control();
        RecoverLegLength();
        break;
    }
    case DSP_RF:
    {
        if(LandingState == RSSP)
        {
            Pre_LandingState = RSSP;
        } else if(LandingState == LSSP)
        {
            Pre_LandingState = LSSP;
        }
        LandingState = DSP;
        ReactiveControl(2,2,1);
        break;
    }
    case DSP_LF:
    {
        if(LandingState == RSSP)
        {
            Pre_LandingState = RSSP;
        } else if(LandingState == LSSP)
        {
            Pre_LandingState = LSSP;
        }
        LandingState = DSP;
        ReactiveControl(2,2,1);
        break;
    }
    case SSP_RF:
    {
        //reset control RX_TC while RF swing
        if((window[0].timer.current >= 0.1) && (window[0].timer.current <= 0.5))
        {// Recovery
            ReactiveControl(0,2,1);
        }
        else if((window[0].timer.current > 0.6))
        {// Control
            ReactiveControl(1,2,1);
        }else
        {// no control
            ReactiveControl(2,2,1);
        }
        LandingState = RSSP;
        break;
    }
    case SSP_LF:
    {
        LandingState = LSSP;
        if((window[0].timer.current >= 0.1) && (window[0].timer.current <= 0.5))
        {// Recovery
            ReactiveControl(2,0,1);

        }
        else if((window[0].timer.current > 0.6))
        {// Control
            ReactiveControl(2,1,1);
        }else
        {// no control
            ReactiveControl(2,2,1);
        }
    }
    case STATE_EMPTY:
    {
        Leg_Length_Control();
        RecoverLegLength();
        break;
    }
    default:
        break;

    }


    Compensator_deflection(window[0].state);
    Kirk_Control();
    LandingControl(window[0].timer.current,window[0].state,sharedData->FT[RAFT].Fz,sharedData->FT[LAFT].Fz);
    Gyro_Feedback();



}

void State_Estimator(double p,double q, double r, double ax, double ay, double orientation[3])
{
    double eye[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
    double dA[4][4] = {{0.,-p*0.5*0.005,-q*0.5*0.005,-r*0.5*0.005},{p*0.5*0.005,0.,r*0.5*0.005,-q*0.5*0.005},{q*0.5*0.005,-r*0.5*0.005,0.,p*0.5*0.005},{r*0.5*0.005,q*0.5*0.005,-p*0.5*0.005,0.}};
    double A[4][4] = {{0.,},},z[4] = {0.,};
    double ang[3] = {0.,},state_global_pelvis[6] = {0.,};

    /* Kalman filter for orientation estimation */
    mat4by4plus4by4(eye,dA,A);

    ang[0] = ax*D2R;
    ang[1] = ay*D2R;
    ang[2] = 0.;

    EulerToQt(ang,z);

    Kalman(A,z,orientation);

    /* High pass filter */
    double a = 0.95;
    HPF_Estimated_Orientation[0] = a*(Old_HPF_Estimated_Orientation[0] + orientation[0] - Old_Estimated_Orientation[0]);
    HPF_Estimated_Orientation[1] = a*(Old_HPF_Estimated_Orientation[1] + orientation[1] - Old_Estimated_Orientation[1]);
    HPF_Estimated_Orientation[2] = a*(Old_HPF_Estimated_Orientation[2] + orientation[2] - Old_Estimated_Orientation[2]);

    Old_Estimated_Orientation[0] = orientation[0];
    Old_Estimated_Orientation[1] = orientation[1];
    Old_Estimated_Orientation[2] = orientation[2];

    Old_HPF_Estimated_Orientation[0] = HPF_Estimated_Orientation[0];
    Old_HPF_Estimated_Orientation[1] = HPF_Estimated_Orientation[1];
    Old_HPF_Estimated_Orientation[2] = HPF_Estimated_Orientation[2];

    /* Set state global pelvis, comp orientation */
    double comp_a = 0.6;
    Comp_Orientation[0] = ang[0]*(1. - comp_a) + comp_a*p*R2D;
    Comp_Orientation[1] = ang[1]*(1. - comp_a) + comp_a*q*R2D;
    Comp_Orientation[2] = ang[2]*(1. - comp_a) + comp_a*r*R2D;

    state_global_pelvis[0] = wbwalk.fkik.WBIK_Q[0];
    state_global_pelvis[1] = wbwalk.fkik.WBIK_Q[1];
    state_global_pelvis[2] = wbwalk.fkik.WBIK_Q[2];

    state_global_pelvis[3] = 0.;
    state_global_pelvis[4] = 0.;
    state_global_pelvis[5] = 0.;
}

void ZMP_intergral_control()
{

    I_ZMP_CON_X += -0.001*(0.001*X_ZMP_Local - (X_ZMP_REF_Local ));//-X_ZMP_n_OFFSET_BP
    I_ZMP_CON_Y += -0.001*(0.001*Y_ZMP_Local - (Y_ZMP_REF_Local ));

    if(I_ZMP_CON_X > 0.04)I_ZMP_CON_X=0.04;
    else if(I_ZMP_CON_X < -0.04)I_ZMP_CON_X=-0.04;
    if(I_ZMP_CON_Y > 0.04)I_ZMP_CON_Y=0.04;
    else if(I_ZMP_CON_Y < -0.04)I_ZMP_CON_Y=-0.04;

}

double kirkZMPCon_XP2(double u, double ZMP, int zero)
{
    int i;
    const double A[2][2] = {{0.000000000000, 1.000000000000}, {-93.44,-1.0204}};
    const double B[2] = {0,153.5062};
    const double C[2] = {5.3672,0.0510};
    const double D = -7.6675;
    const double Kg[2] = {-0.1917,0.0976};
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

void Gyro_Feedback()
{
    if(pv_Index == 1)
    {
        y_i_1 = 0;
        y_i_11= 0;
        u_i_1 = 0;
        u_i_11 = 0;
        NotchFilter_GyroRollControlInput(0,0);
        NotchFilter_GyroPitchControlInput(0,0);
        NotchFilter_GyroRollVel(0,0);
        NotchFilter_GyroPitchVel(0,0);
        GLOBAL_Xori_RF_last = 0;
        GLOBAL_Xori_LF_last = 0;
        GLOBAL_Yori_RF_last = 0;
        GLOBAL_Yori_LF_last = 0;

        GLOBAL_Xori_RF2_last = 0;
        GLOBAL_Xori_LF2_last = 0;
        GLOBAL_Yori_RF2_last = 0;

        GLOBAL_Yori_LF2_last = 0;

//        U_Gain = 0.;
        GLOBAL_Xori_RF = 0.;
        GLOBAL_Xori_LF = 0.;
        GLOBAL_Yori_RF = 0.;
        GLOBAL_Yori_LF = 0.;

        GLOBAL_Xori_RF_last2=0;
        GLOBAL_Yori_RF_last2=0;
        GLOBAL_Xori_LF_last2=0;
        GLOBAL_Yori_LF_last2=0;
    }

    static int CNT_AnkleControl1=0,CNT_AnkleControl2=0,CNT_AnkleControl3=0,CNT_AnkleControl4=0;

    FOGRollVel_NF2 = NotchFilter_GyroRollVel(sharedData->FOG.RollVel,1);
    FOGPitchVel_NF2 = NotchFilter_GyroPitchVel(sharedData->FOG.PitchVel,1);

    FOGRollVel_LPF  = (1.f-2.f*PI*8.0*(double)RT_TIMER_PERIOD_MS*0.001f)*FOGRollVel_LPF  + 2.f*PI*8.0*(double)RT_TIMER_PERIOD_MS*0.001f*FOGRollVel_NF2;
    FOGPitchVel_LPF = (1.f-2.f*PI*8.0*(double)RT_TIMER_PERIOD_MS*0.001f)*FOGPitchVel_LPF + 2.f*PI*8.0*(double)RT_TIMER_PERIOD_MS*0.001f*FOGPitchVel_NF2;


    AnkleControl1 = (sharedData->FOG.Roll*1.0f + sharedData->FOG.RollVel*D2R*2.);
    AnkleControl2 = (sharedData->FOG.Pitch*1.0f + sharedData->FOG.PitchVel*D2R*2.0);


    den_a1 = 1;
    den_a2 = -0.801151070558751;//-0.854080685463467;//-0.909929988177738;
    num_b1 = 0.099424464720624;//0.072959657268267;//0.045035005911131;
    num_b2 = 0.099424464720624;//0.072959657268267;//0.045035005911131;

    u_i = AnkleControl1;
    y_i = -den_a2*y_i_1 + num_b1*u_i + num_b2*u_i_1; //1st order
    y_i_1 = y_i;
    u_i_1 = u_i;

    u_i1 = AnkleControl2;
    y_i1 = -den_a2*y_i_11 + num_b1*u_i1 + num_b2*u_i_11; //1st order
    y_i_11 = y_i1;
    u_i_11 = u_i1;

    if(y_i>20) y_i = 20;
    if(y_i<-20) y_i = -20;
    if(y_i1>20) y_i1 = 20;
    if(y_i1<-20) y_i1 = -20;

    FOGRollVel_NF = NotchFilter_GyroRollControlInput(y_i,1);
    FOGPitchVel_NF = NotchFilter_GyroPitchControlInput(y_i1,1);





    if(window[0].state == SSP_RF)
    {
        Foot_gainLF = 0.5*(1 - cos(PI*CNT_AnkleControl2/20));
        if(CNT_AnkleControl2<20)CNT_AnkleControl2++;

        if(EarlyLandingFlag[RIGHT] == 0)// If the right foot is not landed earlyer,
        {
            //supporting foot
            GLOBAL_Xori_LF = Foot_gainLF*(FOGRollVel_NF) + GLOBAL_Xori_LF_last*(1-Foot_gainLF);
            GLOBAL_Yori_LF = Foot_gainLF*(FOGPitchVel_NF) + GLOBAL_Yori_LF_last*(1-Foot_gainLF);
            GLOBAL_Xori_LF_last2 = GLOBAL_Xori_LF;
            GLOBAL_Yori_LF_last2 = GLOBAL_Yori_LF;
        }

        //swing foot
        GLOBAL_Xori_RF = GLOBAL_Xori_RF_last*(1-Foot_gainLF);
        GLOBAL_Yori_RF = GLOBAL_Yori_RF_last*(1-Foot_gainLF);

        GLOBAL_Xori_RF2 = Foot_gainLF*(0);
        GLOBAL_Yori_RF2 = Foot_gainLF*(0);

        GLOBAL_Xori_RF2_last = GLOBAL_Xori_RF2;
        GLOBAL_Yori_RF2_last = GLOBAL_Yori_RF2;

        CNT_AnkleControl4 = 0;

    }else if(window[0].state == SSP_LF)
    {
        Foot_gainRF = 0.5*(1 - cos(PI*CNT_AnkleControl1/20));
        if(CNT_AnkleControl1<20)CNT_AnkleControl1++;

        if(EarlyLandingFlag[LEFT] == 0)// If the left foot is not landed earlyer,
        {
            //supporting foot
            GLOBAL_Xori_RF = Foot_gainRF*(FOGRollVel_NF) + GLOBAL_Xori_RF_last*(1-Foot_gainRF);
            GLOBAL_Yori_RF = Foot_gainRF*(FOGPitchVel_NF) + GLOBAL_Yori_RF_last*(1-Foot_gainRF);
            GLOBAL_Xori_RF_last2 = GLOBAL_Xori_RF;
            GLOBAL_Yori_RF_last2 = GLOBAL_Yori_RF;
        }

        //swing foot
        GLOBAL_Xori_LF = GLOBAL_Xori_LF_last*(1-Foot_gainRF);
        GLOBAL_Yori_LF = GLOBAL_Yori_LF_last*(1-Foot_gainRF);

        GLOBAL_Xori_LF2 = Foot_gainRF*(0);
        GLOBAL_Yori_LF2 = Foot_gainRF*(0);

        GLOBAL_Xori_LF2_last = GLOBAL_Xori_LF2;
        GLOBAL_Yori_LF2_last = GLOBAL_Yori_LF2;
        CNT_AnkleControl4 = 0;

    }else if(window[0].state == DSP_LF || window[0].state == DSP_RF)
    {
        Foot_gainLF = 1 - 0.5*(1 - cos(PI*CNT_AnkleControl4/50));
        if(CNT_AnkleControl4<50)CNT_AnkleControl4++;

        if(Pre_LandingState == RSSP)
        {
            GLOBAL_Xori_LF = Foot_gainLF*(GLOBAL_Xori_LF_last2);// when dsp, set Xori to zero
            GLOBAL_Yori_LF = Foot_gainLF*(GLOBAL_Yori_LF_last2);

            GLOBAL_Xori_RF2 = Foot_gainLF*(GLOBAL_Xori_RF2_last);
            GLOBAL_Yori_RF2 = Foot_gainLF*(GLOBAL_Yori_RF2_last);
        }
        if(Pre_LandingState == LSSP)
        {
            GLOBAL_Xori_RF = Foot_gainLF*(GLOBAL_Xori_RF_last2);// when dsp, set Xori to zero
            GLOBAL_Yori_RF = Foot_gainLF*(GLOBAL_Yori_RF_last2);

            GLOBAL_Xori_LF2 = Foot_gainLF*(GLOBAL_Xori_LF2_last);
            GLOBAL_Yori_LF2 = Foot_gainLF*(GLOBAL_Yori_LF2_last);
        }

        CNT_AnkleControl1 = CNT_AnkleControl2 = CNT_AnkleControl3 = 0;

        GLOBAL_Xori_LF_last = GLOBAL_Xori_LF;
        GLOBAL_Yori_LF_last = GLOBAL_Yori_LF;

        GLOBAL_Xori_RF_last = GLOBAL_Xori_RF;
        GLOBAL_Yori_RF_last = GLOBAL_Yori_RF;

    }else if(window[0].state == DSP_FINAL)
    {
        Foot_gainLF = 1 - 0.5*(1 - cos(PI*CNT_AnkleControl3/50));
        if(CNT_AnkleControl3<50)CNT_AnkleControl3++;

        if(Pre_LandingState == RSSP)
        {
            GLOBAL_Xori_LF = Foot_gainLF*(GLOBAL_Xori_LF_last2);
            GLOBAL_Yori_LF = Foot_gainLF*(GLOBAL_Yori_LF_last2);
            GLOBAL_Xori_RF2 = Foot_gainLF*(GLOBAL_Xori_RF2_last);
            GLOBAL_Yori_RF2 = Foot_gainLF*(GLOBAL_Yori_RF2_last);
        }

        if(Pre_LandingState == LSSP)
        {
            GLOBAL_Xori_RF = Foot_gainLF*(GLOBAL_Xori_RF_last2);
            GLOBAL_Yori_RF = Foot_gainLF*(GLOBAL_Yori_RF_last2);
            GLOBAL_Xori_LF2 = Foot_gainLF*(GLOBAL_Xori_LF2_last);
            GLOBAL_Yori_LF2 = Foot_gainLF*(GLOBAL_Yori_LF2_last);
        }


        CNT_AnkleControl1 = CNT_AnkleControl2 = 0;

        GLOBAL_Xori_LF_last = GLOBAL_Xori_LF;
        GLOBAL_Yori_LF_last = GLOBAL_Yori_LF;

        GLOBAL_Xori_RF_last = GLOBAL_Xori_RF;
        GLOBAL_Yori_RF_last = GLOBAL_Yori_RF;

        GLOBAL_Xori_LF2_last = GLOBAL_Xori_LF2;
        GLOBAL_Yori_LF2_last = GLOBAL_Yori_LF2;

        GLOBAL_Xori_RF2_last = GLOBAL_Xori_RF2;
        GLOBAL_Yori_RF2_last = GLOBAL_Yori_RF2;
    }


}

double HUBO2ZMPInitLegLength(double _ref, double _force, int _zero)
{
    static double y;
    static double sume = 0.;
    const double KI = 0.000011;

    y = 0.0*(_ref - _force) + KI*sume;

    sume += _ref - _force;

    if(sume > 0.08/KI) sume = .08/KI;
    else if(sume < -0.08/KI) sume = -.08/KI;

    if(y > 0.08) y = 0.08;
    else if(y < -0.08) y = -0.08;

    if(_zero == 0) sume = 0.;

    return y;
}

double RecoverRightLegLength(double _ref, double _force, int _zero)
{
    static double y;
    static double sume = 0.;
    const double KI = 0.051;

    y = 0.0*(_ref - _force) + KI*sume;

    sume += _ref - _force;


    if(y > 0.08) y = 0.08;
    else if(y < -0.08) y = -0.08;

    if(_zero == 0) sume = 0.;

    return y;
}

double RecoverLeftLegLength(double _ref, double _force, int _zero)
{
    static double y;
    static double sume = 0.;
    const double KI = 0.051;

    y = 0.0*(_ref - _force) + KI*sume;

    sume += _ref - _force;

    if(y > 0.08) y = 0.08;
    else if(y < -0.08) y = -0.08;

    if(_zero == 0) sume = 0.;

    return y;
}

double NotchFilter_GyroRollControlInput(double _input, int _reset)
{

    double a[3] = {1,-0.974482283357440,0.948964566714880};
    double b[3] = {0.974482283357440,-0.974482283357440,0.974482283357440};
    static double Yi =0,Yi_1=0,Yi_2=0,Ui=0,Ui_1=0,Ui_2=0;

    Ui = _input;

    if(_reset == 0) Yi_1 = Yi_2 = Ui_2 = Ui_1 = 0;

    Yi = -a[1]*Yi_1 - a[2]*Yi_2 + b[0]*Ui + b[1]*Ui_1 + b[2]*Ui_2;
    Yi_2 = Yi_1;
    Yi_1 = Yi;
    Ui_2 = Ui_1;
    Ui_1 = Ui;

    return Yi;
}
double NotchFilter_GyroPitchControlInput(double _input, int _reset)
{

    double a[3] = {1,-0.974482283357440,0.948964566714880};
    double b[3] = {0.974482283357440,-0.974482283357440,0.974482283357440};
    static double Yi =0,Yi_1=0,Yi_2=0,Ui=0,Ui_1=0,Ui_2=0;

    Ui = _input;

    if(_reset == 0) Yi_1 = Yi_2 = Ui_2 = Ui_1 = 0;

    Yi = -a[1]*Yi_1 - a[2]*Yi_2 + b[0]*Ui + b[1]*Ui_1 + b[2]*Ui_2;
    Yi_2 = Yi_1;
    Yi_1 = Yi;
    Ui_2 = Ui_1;
    Ui_1 = Ui;

    return Yi;
}
double NotchFilter_GyroRollVel(double _input, int _reset)
{
    double a[3] = {1,-0.974482283357440,0.948964566714880};
    double b[3] = {0.974482283357440,-0.974482283357440,0.974482283357440};
    static double Yi =0,Yi_1=0,Yi_2=0,Ui=0,Ui_1=0,Ui_2=0;

    Ui = _input;

    if(_reset == 0) Yi_1 = Yi_2 = Ui_2 = Ui_1 = 0;

    Yi = -a[1]*Yi_1 - a[2]*Yi_2 + b[0]*Ui + b[1]*Ui_1 + b[2]*Ui_2;
    Yi_2 = Yi_1;
    Yi_1 = Yi;
    Ui_2 = Ui_1;
    Ui_1 = Ui;

    return Yi;
}
double NotchFilter_GyroPitchVel(double _input, int _reset)
{

    double a[3] = {1,-0.974482283357440,0.948964566714880};
    double b[3] = {0.974482283357440,-0.974482283357440,0.974482283357440};
    static double Yi =0,Yi_1=0,Yi_2=0,Ui=0,Ui_1=0,Ui_2=0;

    Ui = _input;

    if(_reset == 0) Yi_1 = Yi_2 = Ui_2 = Ui_1 = 0;

    Yi = -a[1]*Yi_1 - a[2]*Yi_2 + b[0]*Ui + b[1]*Ui_1 + b[2]*Ui_2;
    Yi_2 = Yi_1;
    Yi_1 = Yi;
    Ui_2 = Ui_1;
    Ui_1 = Ui;

    return Yi;
}

void Kirk_Control()
{
    //printf("ZMP LOCAL %f \   Del_PC_X_DSP_XZMP_CON: %f \n",X_ZMP_Local,Del_PC_X_DSP_XZMP_CON);

    ZMP_intergral_control();
    final_gain_DSP_ZMP_CON = 0.5*(1-cos(PI*CNT_final_gain_DSP_ZMP_CON/40));

    Del_PC_X_DSP_XZMP_CON = kirkZMPCon_XP2(-Del_PC_X_DSP_XZMP_CON, X_ZMP_Local - (X_ZMP_REF_Local)*1000 , 1);
    Del_PC_Y_DSP_YZMP_CON = kirkZMPCon_YP2(-Del_PC_Y_DSP_YZMP_CON, Y_ZMP_Local - (Y_ZMP_REF_Local)*1000 , 1);

    LPF_Del_PC_X_DSP_XZMP_CON = (1.f-2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f)*LPF_Del_PC_X_DSP_XZMP_CON + 2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f*Del_PC_X_DSP_XZMP_CON;
    LPF_Del_PC_Y_DSP_YZMP_CON = (1.f-2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f)*LPF_Del_PC_Y_DSP_YZMP_CON + 2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f*Del_PC_Y_DSP_YZMP_CON;

    Old_Del_PC_X_DSP_XZMP_CON = Del_PC_X_DSP_XZMP_CON;
    Old_Del_PC_Y_DSP_YZMP_CON = Del_PC_Y_DSP_YZMP_CON;
    Old_Del_PC_X_DSP_XZMP_CON2 = Del_PC_X_DSP_XZMP_CON;
    Old_Del_PC_Y_DSP_YZMP_CON2 = Del_PC_Y_DSP_YZMP_CON;
    Old_I_ZMP_CON_X = I_ZMP_CON_X;
    Old_I_ZMP_CON_Y = I_ZMP_CON_Y;
}

double RMYC2(int state,int state2,int state3,double ref_zmp_x,double ref_zmp_y,double zmp_x,double zmp_y,double LFoot_x,double RFoot_x,double LFoot_y,double RFoot_y,double FTmx,double Qd0)
{
    static double alpha,RDF,LDF,torqueX=0,dTorque=0,mTorque=0;
    static double M = 80.36,g=9.81;

    alpha = fabs(ref_zmp_y - LFoot_y)/fabs(LFoot_y - RFoot_y);
    RDF = (alpha*M*g);
    LDF = ((1.0 - alpha)*M*g);

    //desired
    dTorque = 0.;

    //measured
    torqueX = (-(RFoot_x - zmp_x)*RDF - (LFoot_x-zmp_x)*LDF);


//    RmTorqueX = mTorque = alpha*torqueX;
    if(state ==1)
    {
        mTorque = RDF*(HPF_Estimated_Orientation[1]*R2D*0.3 + sharedData->FOG.PitchVel*.7);//(RFoot_x - ref_zmp_x)*RDF + (LFoot_x - ref_zmp_x)*LDF;
    }else if(state == 0)
    {
        mTorque = 0;
    }

    static double CL=0.0;
    static double d = 3000, m = 1.5,dt = 0.005;
    static double Qd1=0.0,Qd2=0.0,Q[3]={0.0,},T = 0.4;

    if(state == 1)
    {
        d = 2500;//1125;
        T = 1.0;
    }else if(state == 0)
    {
        d = 2500;
        T = 0.05;
    }

    if(state3 == 0)
    {
        Q[0] = 0;
        Q[1] = 0;
        Q[2] = 0;
        CL = 0;
    }


    if(state != 2){


        if(pv_Index >=2)
        {

            Q[0] = ( dTorque - mTorque )/d - (1.0/T)*Q[1];

            CL = Q[2] = Q[1] + Q[0]*dt;

            Q[1] = Q[2];

            if(CL > 0.2)
            {
                CL = 0.2;
            }else if( CL < -0.2)
            {
                CL = -0.2;
            }

        }
    }


        return CL*R2D;



}
double LMYC2(int state,int state2,int state3,double ref_zmp_x,double ref_zmp_y,double zmp_x,double zmp_y,double LFoot_x,double RFoot_x,double LFoot_y,double RFoot_y,double FTmx,double Qd0)
{
    static double alpha,RDF,LDF,torqueX=0,dTorque=0,mTorque=0;
    static double M = 80.36,g=9.81;

    alpha = fabs(ref_zmp_y - LFoot_y)/fabs(LFoot_y - RFoot_y);
    RDF = (alpha*M*g);
    LDF = ((1.0 - alpha)*M*g);

    //desired

        dTorque = 0.;

    //measured
    torqueX = (-(RFoot_x - zmp_x)*RDF - (LFoot_x-zmp_x)*LDF);


    if(state2 ==1)
    {
        mTorque = LDF*(HPF_Estimated_Orientation[1]*R2D*0.3 + sharedData->FOG.PitchVel*.7);//(RFoot_x - ref_zmp_x)*RDF + (LFoot_x - ref_zmp_x)*LDF;
    }else if(state2 == 0)
    {
        mTorque = 0;
    }


    static double CL=0.0;
    static double d = 3000, m = 1.5,dt = 0.005;
    static double Qd1=0.0,Qd2=0.0,Q[3]={0.0,},T = .4;

    if(state2 == 1)
    {
        d = 3000;//1125;
        T = 1.0;
    }else if(state2 == 0)
    {
        d = 3000;
        T = 0.05;
    }

    if(state3 == 0)
    {
        Q[0] = 0;
        Q[1] = 0;
        Q[2] = 0;
        CL = 0;
    }

    if(state2 != 2){


        if(pv_Index >=2)
        {

            Q[0] = ( dTorque - mTorque )/d - (1.0/T)*Q[1];

            CL = Q[2] = Q[1] + Q[0]*dt;

            Q[1] = Q[2];

            if(CL > 0.2)
            {
                CL = 0.2;
            }else if( CL < -0.2)
            {
                CL = -0.2;
            }

        }
    }




        return CL*R2D;



}
double RMXC2(int state,int state2,int state3,double ref_zmp_x,double ref_zmp_y,double zmp_x,double zmp_y,double LFoot_x,double RFoot_x,double LFoot_y,double RFoot_y,double FTmx,double Qd0)
{
    static double alpha,RDF,LDF,torqueX=0,dTorque=0,mTorque=0;
    static double M = 80.36,g=9.81;

    alpha = fabs(ref_zmp_y - LFoot_y)/fabs(LFoot_y - RFoot_y);
    RDF = (alpha*M*g);
    LDF = ((1.0 - alpha)*M*g);

    //desired
    dTorque = 0.;

    //measured
    torqueX = (-(RFoot_x - zmp_x)*RDF - (LFoot_x-zmp_x)*LDF);


    if(state ==1)
    {
        mTorque = RDF*(HPF_Estimated_Orientation[0]*R2D*0.3 + sharedData->FOG.RollVel*.7);//(RFoot_x - ref_zmp_x)*RDF + (LFoot_x - ref_zmp_x)*LDF;
    }else if(state == 0)
    {
        mTorque = 0;
    }

    static double CL=0.0;
    static double d = 3000, m = 1.5,dt = 0.005;
    static double Qd1=0.0,Qd2=0.0,Q[3]={0.0,},T = 0.4;

    if(state == 1)
    {
        d = 2500;//1125;
        T = 1.0;
    }else if(state == 0)
    {
        d = 2500;
        T = 0.05;
    }

    if(state3 == 0)
    {
        Q[0] = 0;
        Q[1] = 0;
        Q[2] = 0;
        CL = 0;
    }


    if(state != 2){


        if(pv_Index >=2)
        {

            Q[0] = ( dTorque - mTorque )/d - (1.0/T)*Q[1];

            CL = Q[2] = Q[1] + Q[0]*dt;

            Q[1] = Q[2];

            if(CL > 0.2)
            {
                CL = 0.2;
            }else if( CL < -0.2)
            {
                CL = -0.2;
            }

        }
    }


        return CL*R2D;



}
double LMXC2(int state,int state2,int state3,double ref_zmp_x,double ref_zmp_y,double zmp_x,double zmp_y,double LFoot_x,double RFoot_x,double LFoot_y,double RFoot_y,double FTmx,double Qd0)
{
    static double alpha,RDF,LDF,torqueX=0,dTorque=0,mTorque=0;
    static double M = 80.36,g=9.81;

    alpha = fabs(ref_zmp_y - LFoot_y)/fabs(LFoot_y - RFoot_y);
    RDF = (alpha*M*g);
    LDF = ((1.0 - alpha)*M*g);

    //desired
        dTorque = 0.;

    //measured
    torqueX = (-(RFoot_x - zmp_x)*RDF - (LFoot_x-zmp_x)*LDF);


    if(state2 ==1)
    {
        mTorque = LDF*(HPF_Estimated_Orientation[0]*R2D + sharedData->FOG.RollVel*.7);//(RFoot_x - ref_zmp_x)*RDF + (LFoot_x - ref_zmp_x)*LDF;
    }else if(state2 == 0)
    {
        mTorque = 0;
    }


    static double CL=0.0;
    static double d = 3000, m = 1.5,dt = 0.005;
    static double Qd1=0.0,Qd2=0.0,Q[3]={0.0,},T = .4;

    if(state2 == 1)
    {
        d = 2500;//3000;
        T = 1.0;
    }else if(state2 == 0)
    {
        d = 2500;
        T = 0.05;
    }

    if(state3 == 0)
    {
        Q[0] = 0;
        Q[1] = 0;
        Q[2] = 0;
        CL = 0;
    }

    if(state2 != 2){


        if(pv_Index >=2)
        {

            Q[0] = ( dTorque - mTorque )/d - (1.0/T)*Q[1];

            CL = Q[2] = Q[1] + Q[0]*dt;

            Q[1] = Q[2];

            if(CL > 0.2)
            {
                CL = 0.2;
            }else if( CL < -0.2)
            {
                CL = -0.2;
            }

        }
    }




        return CL*R2D;



}

void get_zmp2()
{
        // Foot Center in Global Coord.
        pCenter[0] = (wbwalk.fkik.desired.pRF_3x1[0] + wbwalk.fkik.desired.pLF_3x1[0])/2.;
        pCenter[1] = (wbwalk.fkik.desired.pRF_3x1[1] + wbwalk.fkik.desired.pLF_3x1[1])/2.;
        pCenter[2] = (wbwalk.fkik.desired.pRF_3x1[2] + wbwalk.fkik.desired.pLF_3x1[2])/2.;

        qtRZ((window[0].right_foot_ref.yaw*D2R + window[0].left_foot_ref.yaw*D2R)/2.,qCenter);


        if(sharedData->FT[RAFT].Fz + sharedData->FT[LAFT].Fz > 50.)
        {
            M_LF[0] =  sharedData->FT[LAFT].Mx;
            M_LF[1] =  sharedData->FT[LAFT].My;
            M_LF[2] =  sharedData->FT[LAFT].Mz;

            QTtransform(wbwalk.fkik.desired.qLF_4x1,M_LF,M_LF_Global);

            M_RF[0] =  sharedData->FT[RAFT].Mx;
            M_RF[1] =  sharedData->FT[RAFT].My;
            M_RF[2] =  sharedData->FT[RAFT].Mz;

            QTtransform(wbwalk.fkik.desired.qRF_4x1,M_RF,M_RF_Global);

            F_LF[0] = sharedData->FT[LAFT].Fx;
            F_LF[1] = sharedData->FT[LAFT].Fy;
            F_LF[2] = sharedData->FT[LAFT].Fz;

            QTtransform(wbwalk.fkik.desired.qLF_4x1,F_LF,F_LF_Global);

            F_RF[0] = sharedData->FT[RAFT].Fx;
            F_RF[1] = sharedData->FT[RAFT].Fy;
            F_RF[2] = sharedData->FT[RAFT].Fz;

            QTtransform(wbwalk.fkik.desired.qRF_4x1,F_RF,F_RF_Global);

            double temp1[3],temp2[3],temp3[3],temp4[3];

            diff_vv(wbwalk.fkik.desired.pRF_3x1,3,pCenter,temp1);// (despRF - pCenter)
            diff_vv(wbwalk.fkik.desired.pLF_3x1,3,pCenter,temp2);// (despLF - pCenter)

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
            temp2[0] = GLOBAL_ZMP_REF_X;
            temp2[1] = GLOBAL_ZMP_REF_Y;
            temp2[2] = 0;
            diff_vv(temp2,3,pCenter,temp1);
            QTtransform(qCenter_bar, temp1, zmp_ref_local);

            X_ZMP_Local = 1000.*zmp_local[0];
            Y_ZMP_Local = 1000.*zmp_local[1];

            X_ZMP_Global = 1000.*zmp[0];
            Y_ZMP_Global = 1000.*zmp[1];

            X_ZMP_REF_Local = zmp_ref_local[0];
            Y_ZMP_REF_Local = zmp_ref_local[1];

            X_ZMP_REF_Global = GLOBAL_ZMP_REF_X;
            Y_ZMP_REF_Global = GLOBAL_ZMP_REF_Y;


        }
}

void LandingControl(int cur_time,int state,double rForce,double lForce)
{
    static double temp_Z_RF[3]={0.,},temp_Z_LF[3]={0.};
    double Force_Threshold = 150.,target_time = 0.4;

    if(state == SSP_RF)
    {
        // Recovery left supporting leg
        if(window[0].timer.current < 0.4){
            GLOBAL_Z_LF_goal[0] = window[0].left_foot_ref.z;    GLOBAL_Z_LF_goal[1] = 0.;   GLOBAL_Z_LF_goal[2] = 0.;
            Fifth( window[0].timer.current , 0.4, GLOBAL_Z_LF_last ,GLOBAL_Z_LF_goal  , temp_Z_LF);
        }else{
            GLOBAL_Z_LF_last[0] = GLOBAL_Z_LF;
            temp_Z_LF[0]= window[0].left_foot_ref.z;
        }

        GLOBAL_Z_LF = temp_Z_LF[0];
        // ----------------------------

        // Detect early landing of Right foot
        if(window[0].timer.current <= 0.4)
        {
            EarlyLandingFlag[RIGHT] = 0;
            GLOBAL_Z_RF = window[0].right_foot_ref.z;
        }else
        {
            if(rForce > Force_Threshold && window[0].timer.current>= 0.65){
                if(EarlyLandingFlag[RIGHT] == 0)
                {
                    GLOBAL_Z_RF_last_earlylanding = GLOBAL_Z_RF;
                }

                EarlyLandingFlag[RIGHT] = 1;

            }

            if(EarlyLandingFlag[RIGHT] == 0){ // normal walking
                GLOBAL_Z_RF = window[0].right_foot_ref.z;
            }else{
                GLOBAL_Z_RF = GLOBAL_Z_RF_last_earlylanding;

            }
            GLOBAL_Z_RF_last[0] = GLOBAL_Z_RF;

        }

    }else if(state == SSP_LF){

        // Recovery right supporting leg up to left off phase
        if(window[0].timer.current < 0.4){

            GLOBAL_Z_RF_goal[0] = window[0].right_foot_ref.z;    GLOBAL_Z_RF_goal[1] = 0.;   GLOBAL_Z_RF_goal[2] = 0.;

            Fifth( window[0].timer.current , 0.4, GLOBAL_Z_RF_last , GLOBAL_Z_RF_goal , temp_Z_RF);

        }else{
            GLOBAL_Z_RF_last[0] = GLOBAL_Z_RF;
            temp_Z_RF[0]= window[0].right_foot_ref.z;
        }

        GLOBAL_Z_RF = temp_Z_RF[0];
        // ----------------------------

        // Detect early landing of Left foot
        if(window[0].timer.current <= 0.4)
        {
            EarlyLandingFlag[LEFT] = 0;
            GLOBAL_Z_LF = window[0].left_foot_ref.z;

        }else
        {
            if(lForce > Force_Threshold && window[0].timer.current>= 0.65){

                if(EarlyLandingFlag[LEFT] == 0){

                    GLOBAL_Z_LF_last_earlylanding = GLOBAL_Z_LF;
                }
                EarlyLandingFlag[LEFT] = 1;

            }

            if(EarlyLandingFlag[LEFT] == 0){ // normal walking
                GLOBAL_Z_LF = window[0].left_foot_ref.z;
            }else{
                GLOBAL_Z_LF = GLOBAL_Z_LF_last_earlylanding;
            }

            GLOBAL_Z_LF_last[0] = GLOBAL_Z_LF;
        }



    }else if(state == DSP_FINAL)
    {




    }


    if(EarlyLanding_ONOFF == 0)
    {
        GLOBAL_Z_LF = window[0].left_foot_ref.z;
        GLOBAL_Z_RF = window[0].right_foot_ref.z;
    }





}
void Leg_Length_Control()
{
    // Integral control for nutral state
    // Leg_length Control
    double des_pRF_3x1_n[3],des_pLF_3x1_n[3],qLF[4],qRF[4];

    if(window[0].state == DSP_INIT_RF || window[0].state == DSP_INIT_LF)
    {
        kine_drc_hubo4.FK_LeftFoot_Local(wbwalk.fkik.WBIK_Q,des_pLF_3x1_n,qLF);
        kine_drc_hubo4.FK_RightFoot_Local(wbwalk.fkik.WBIK_Q,des_pRF_3x1_n,qRF);

        des_pLF_3x1_n[2] =  wbwalk.fkik.desired.pLF_3x1[2];
        des_pRF_3x1_n[2] =  wbwalk.fkik.desired.pRF_3x1[2];

        convert_euler(des_pRF_3x1_n, des_pLF_3x1_n, sharedData->IMU[0].AccX*D2R*cos(-yaw_angle)- sharedData->IMU[0].AccY*D2R*sin(-yaw_angle), ( sharedData->IMU[0].AccX*D2R)*sin(-yaw_angle)+ sharedData->IMU[0].AccY*D2R*cos(-yaw_angle), 0,BTW_FOOT_Angle_roll, BTW_FOOT_Angle_pitch, BTW_FOOT_Angle_yaw,BTW_FOOT_qPEL_comp_4x1);

        Add_FootTask[RIGHT][Zdir] = HUBO2ZMPInitLegLength(0., BTW_FOOT_Angle_roll*R2D, 1);
        Add_FootTask[LEFT][Zdir] = -Add_FootTask[RIGHT][Zdir];

    }
    else if(window[0].state == DSP_FINAL)
    {

        kine_drc_hubo4.FK_LeftFoot_Local(wbwalk.fkik.WBIK_Q,des_pLF_3x1_n,qLF);
        kine_drc_hubo4.FK_RightFoot_Local(wbwalk.fkik.WBIK_Q,des_pRF_3x1_n,qRF);

        des_pLF_3x1_n[2] =  wbwalk.fkik.desired.pLF_3x1[2];
        des_pRF_3x1_n[2] =  wbwalk.fkik.desired.pRF_3x1[2];

        convert_euler(des_pRF_3x1_n, des_pLF_3x1_n, sharedData->IMU[0].AccX*D2R*cos(-yaw_angle)- sharedData->IMU[0].AccY*D2R*sin(-yaw_angle), ( sharedData->IMU[0].AccX*D2R)*sin(-yaw_angle)+ sharedData->IMU[0].AccY*D2R*cos(-yaw_angle), 0,BTW_FOOT_Angle_roll, BTW_FOOT_Angle_pitch, BTW_FOOT_Angle_yaw,BTW_FOOT_qPEL_comp_4x1);
        Add_FootTask[RIGHT][Zdir] = HUBO2ZMPInitLegLength(0., BTW_FOOT_Angle_roll*R2D, 1);
        Add_FootTask[LEFT][Zdir] = -Add_FootTask[RIGHT][Zdir];

    }else if(window[0].state == STATE_EMPTY || window[0].timer.current < 2.0)
    {

    }

    if(Add_FootTask[RIGHT][Zdir] > 0.05)Add_FootTask[RIGHT][Zdir] =  0.05;
    if(Add_FootTask[RIGHT][Zdir] <-0.05)Add_FootTask[RIGHT][Zdir] = -0.05;

    if(Add_FootTask[LEFT][Zdir] > 0.05)Add_FootTask[LEFT][Zdir] =  0.05;
    if(Add_FootTask[LEFT][Zdir] <-0.05)Add_FootTask[LEFT][Zdir] = -0.05;
}
void RecoverLegLength()
{
    // Integral control for nutral state
    // Leg_length Control
    double des_pRF_3x1_n[3],des_pLF_3x1_n[3],qLF[4],qRF[4];

    if(window[0].state == DSP_INIT_RF || window[0].state == DSP_INIT_LF)
    {


    }
    else if(window[0].state == DSP_FINAL)
    {

            Add_Leg_Recovery[RIGHT][Zdir] = RecoverRightLegLength(0.,(GLOBAL_Z_RF + Add_Leg_Recovery[RIGHT][Zdir])- Init_Right_Leg , 1);
            Add_Leg_Recovery[LEFT][Zdir]  = RecoverLeftLegLength(0.,(GLOBAL_Z_LF + Add_Leg_Recovery[LEFT][Zdir])- Init_Left_Leg , 1);

            // Recovery Leg initial position


    }else if(window[0].state == STATE_EMPTY && window[0].timer.current < 2.0)
    {
        Add_Leg_Recovery[RIGHT][Zdir] = RecoverRightLegLength(0.,(GLOBAL_Z_RF + Add_Leg_Recovery[RIGHT][Zdir])- Init_Right_Leg , 1);
        Add_Leg_Recovery[LEFT][Zdir]  = RecoverLeftLegLength(0.,(GLOBAL_Z_LF + Add_Leg_Recovery[LEFT][Zdir])- Init_Left_Leg , 1);
    }

    if(Add_Leg_Recovery[RIGHT][Zdir] > 0.05)Add_Leg_Recovery[RIGHT][Zdir] =  0.05;
    if(Add_Leg_Recovery[RIGHT][Zdir] <-0.05)Add_Leg_Recovery[RIGHT][Zdir] = -0.05;

    if(Add_Leg_Recovery[LEFT][Zdir] > 0.05)Add_Leg_Recovery[LEFT][Zdir] =  0.05;
    if(Add_Leg_Recovery[LEFT][Zdir] <-0.05)Add_Leg_Recovery[LEFT][Zdir] = -0.05;
}

double FootForceControl(int state,double dRforce,double dLforce,double mRforce,double mLforce,int reset,double Qd0)
{
    double gain = 10600;
    static double CL=0.0;
    static double d = 7600, m = 2.5,dt = 0.005,T = 0.15;
    static double Qd1=0.0,Qd2=0.0,Q[3]={0.0,};

    if(reset == 1)
    {
        Q[0] = 0;
        Q[1] = 0;
        Q[2] = 0;
        CL = 0;
    }

        if(pv_Index >=2){

            Q[0] = ( (dLforce - dRforce) - (mLforce - mRforce) )/d - (1.0/T)*Q[1];

            CL = Q[2] = Q[1] + Q[0]*dt;

            Q[1] = Q[2];

        }
    return CL;
}
double RFootForceControl(int state,double dRforce,double dLforce,double mRforce,double mLforce,int reset,double Qd0 )
{
    static double CL=0.0;
    static double d1 = 10000,d2 = 15000, m = 2.5,dt = 0.005,T = 0.2;
    static double Qd1=0.0,Qd2=0.0,Q[3]={0.0,};


    if(reset == 1)
    {
        Q[0] = 0;
        Q[1] = 0;
        Q[2] = 0;
        CL = 0;
    }

    if(pv_Index >=2)
    {
        Q[0] = (dRforce  - mRforce )/d1 - (1.0/T)*Q[1];

        CL = Q[2] = Q[1] + Q[0]*dt;

        Q[1] = Q[2];

        if(CL > 0.05)
        {
            CL = 0.05;
        }else if( CL < -0.05)
        {
            CL = -0.05;
        }
    }


    return CL;
}
double LFootForceControl(int state,double dRforce,double dLforce,double mRforce,double mLforce,int reset,double Qd0 )
{

    static double CL=0.0;
    static double d1 = 10000,d2 = 15000, m = 2.5,dt = 0.005,T = 0.2;
    static double Qd1=0.0,Qd2=0.0,Q[3]={0.0,};

    if(reset == 1)
    {
        Q[0] = 0;
        Q[1] = 0;
        Q[2] = 0;
        CL = 0;
    }


    if(pv_Index >=2)
    {
        Q[0] = (dLforce  - mLforce )/d1 - (1.0/T)*Q[1];

        CL = Q[2] = Q[1] + Q[0]*dt;

        Q[1] = Q[2];

        if(CL > 0.05)
        {
            CL = 0.05;
        }else if( CL < -0.05)
        {
            CL = -0.05;
        }
    }


    return CL;
}
double FootForceControl2(int state,double dRforce,double dLforce,double mRforce,double mLforce,int reset,double Qd0)
{
    static double CL=0.0;
    static double d1 = 150,d2 = 28000, m = 2.5,dt = 0.005,T = 0.1;
    static double Qd1=0.0,Qd2=0.0,Q[3]={0.0,};



    if(reset == 1)
    {
        Q[0] = 0;
        Q[1] = 0;
        Q[2] = 0;
        CL = 0;
    }

    {

        if(pv_Index >=2)
        {
            double gain = 1.;//+ sqrt(sharedData->FOG.Pitch*sharedData->FOG.Pitch)*0.01;

            Q[0] = ( -HPF_Estimated_Orientation[0]*D2R*1.5 - sharedData->FOG.RollVel*1.28 )/d1 - (1.0/T)*Q[1];

            CL = Q[2] = Q[1] + Q[0]*dt;

            Q[1] = Q[2];

            if(CL > 0.05)
            {
                CL = 0.05;
            }else if( CL < -0.05)
            {
                CL = -0.05;
            }

        }

    }

    return CL;
}

double RightDesForce(int state,double ref_zmp_x,double ref_zmp_y,double LFoot_x,double RFoot_x,double LFoot_y,double RFoot_y)
{
    double alpha,RDF,LDF;
    double M = 78.36,g=-9.81;

    ALPHA = alpha = fabs(ref_zmp_y - LFoot_y)/fabs(LFoot_y - RFoot_y);
    RDF = -alpha*(M*g);
    LDF = -(1.0 - alpha)*M*g;

    return RDF;
}
double LeftDesForce(int state,double ref_zmp_x,double ref_zmp_y,double LFoot_x,double RFoot_x,double LFoot_y,double RFoot_y)
{
    double alpha,RDF,LDF;
    double M = 78.36,g=-9.81;

    alpha = fabs(ref_zmp_y - LFoot_y)/fabs(LFoot_y - RFoot_y);
    RDF = -alpha*M*g;
    LDF = -(1.0 - alpha)*M*g;


    return LDF;
}

double RMXC(int state,int state2,int state3,double ref_zmp_x,double ref_zmp_y,double zmp_x,double zmp_y,double LFoot_x,double RFoot_x,double LFoot_y,double RFoot_y,double FTmx,double Qd0)
{
    static double alpha,RDF,LDF,torqueX=0,dTorque=0,mTorque=0;
    static double M = 78.36,g=-9.81;

    alpha = fabs(ref_zmp_y - LFoot_y)/fabs(LFoot_y - RFoot_y);
    RDF = (-alpha*M*g);
    LDF = (-(1.0 - alpha)*M*g);

    //desired
    dTorque = 0.;//(RFoot_x - ref_zmp_x)*RDF + (LFoot_x - ref_zmp_x)*LDF;


    //measured
    torqueX = (-(RFoot_x - zmp_x)*RDF - (LFoot_x-zmp_x)*LDF);



    mTorque = FTmx;//alpha*(sharedData->FOGPitch*4.0 + sharedData->FOGPitchVel*2.0);


    static double CL=0.0;
    static double d = 30, m = 1.5,dt = 0.005;
    static double Qd1=0.0,Qd2=0.0,Q[3]={0.0,},T = 0.4;


    if(state == 1)
    {
        d = 50.;//1125;
        T = 0.4;
    }else if(state == 0)
    {
        d = 50.;
        T = 0.05;
    }else if(state == 3)
    {
        d = 2500;
        T = 0.5;
    }


    if(state3 == 0)
    {
        Q[0] = 0;
        Q[1] = 0;
        Q[2] = 0;
        CL = 0;
    }

    if(state != 2){


        if(pv_Index >=2)
        {

            Q[0] = ( 0 - mTorque )/d - (1.0/T)*Q[1];

            CL = Q[2] = Q[1] + Q[0]*dt;

            Q[1] = Q[2];

            if(CL > 0.5)
            {
                CL = 0.5;
            }else if( CL < -0.5)
            {
                CL = -0.5;
            }

        }
    }


        return CL*R2D;
}
double LMXC(int state,int state2,int state3,double ref_zmp_x,double ref_zmp_y,double zmp_x,double zmp_y,double LFoot_x,double RFoot_x,double LFoot_y,double RFoot_y,double FTmx,double Qd0)
{
    static double alpha,RDF,LDF,torqueX=0,dTorque=0,mTorque=0;
    static double M = 78.36,g=-9.81;

    alpha = fabs(ref_zmp_y - LFoot_y)/fabs(LFoot_y - RFoot_y);
    RDF = (-alpha*M*g);
    LDF = (-(1.0 - alpha)*M*g);

    //desired
dTorque = 0.;//(RFoot_x - ref_zmp_x)*RDF + (LFoot_x - ref_zmp_x)*LDF;


    //measured
    torqueX = (-(RFoot_x - zmp_x)*RDF - (LFoot_x-zmp_x)*LDF);


    mTorque = FTmx;//alpha*(sharedData->FOGPitch*4.0 + sharedData->FOGPitchVel*2.0);


    static double CL=0.0;
    static double d = 30, m = 1.5,dt = 0.005;
    static double Qd1=0.0,Qd2=0.0,Q[3]={0.0,},T = 0.4;



    if(state2 == 1)
    {

        d = 50.0;
        T = 0.4;
    }else if(state2 == 0)
    {
        d = 50;
        T = 0.05;
    }else if(state2 == 3)
    {
        d = 2500;
        T = 0.5;
    }

    if(state3 == 0)
    {
        Q[0] = 0;
        Q[1] = 0;
        Q[2] = 0;
        CL = 0;
    }

    if(state2 != 2){


        if(pv_Index >=2)
        {

            Q[0] = ( 0 - mTorque )/d - (1.0/T)*Q[1];

            CL = Q[2] = Q[1] + Q[0]*dt;

            Q[1] = Q[2];

            if(CL > 0.5)
            {
                CL = 0.5;
            }else if( CL < -0.5)
            {
                CL = -0.5;
            }

        }
    }
        return CL*R2D;
}
double RMYC(int state,int state2,int state3,double ref_zmp_x,double ref_zmp_y,double zmp_x,double zmp_y,double LFoot_x,double RFoot_x,double LFoot_y,double RFoot_y,double FTmx,double Qd0)
{
    static double alpha,RDF,LDF,torqueX=0,dTorque=0,mTorque=0;
    static double M = 78.36,g=9.81;

    alpha = fabs(ref_zmp_y - LFoot_y)/fabs(LFoot_y - RFoot_y);
    RDF = (alpha*M*g);
    LDF = ((1.0 - alpha)*M*g);

    //desired
        dTorque = 0.;

    //measured
    torqueX = (-(RFoot_x - zmp_x)*RDF - (LFoot_x-zmp_x)*LDF);



    {
        mTorque = FTmx;
    }

    static double CL=0.0;
    static double d = 30, m = 1.5,dt = 0.005;
    static double Qd1=0.0,Qd2=0.0,Q[3]={0.0,},T = 0.4;

    if(state == 1)
    {
        d = 50.;
        T = 0.4;

    }else if(state == 0)
    {
        d = 50.;
        T = 0.05;
    }else if(state == 3)
    {
        d = 2500;
        T = 0.5;
    }

    if(state3 == 0)
    {
        Q[0] = 0;
        Q[1] = 0;
        Q[2] = 0;
        CL = 0;
    }


    if(state != 2){


        if(pv_Index >=2)
        {

            Q[0] = ( dTorque - mTorque )/d - (1.0/T)*Q[1];

            CL = Q[2] = Q[1] + Q[0]*dt;

            Q[1] = Q[2];

            if(CL > 0.5)
            {
                CL = 0.5;
            }else if( CL < -0.5)
            {
                CL = -0.5;
            }

        }
    }


        return CL*R2D;



}
double LMYC(int state,int state2,int state3,double ref_zmp_x,double ref_zmp_y,double zmp_x,double zmp_y,double LFoot_x,double RFoot_x,double LFoot_y,double RFoot_y,double FTmx,double Qd0)
{
    static double alpha,RDF,LDF,torqueX=0,dTorque=0,mTorque=0;
    static double M = 78.36,g=9.81;

    alpha = fabs(ref_zmp_y - LFoot_y)/fabs(LFoot_y - RFoot_y);
    RDF = (alpha*M*g);
    LDF = ((1.0 - alpha)*M*g);

    //desired

        dTorque = 0.;

    //measured
    torqueX = (-(RFoot_x - zmp_x)*RDF - (LFoot_x-zmp_x)*LDF);



    {
        mTorque = FTmx;//alpha*(sharedData->FOGPitch*4.0 + sharedData->FOGPitchVel*2.0);
    }

    static double CL=0.0;
    static double d = 50, m = 1.5,dt = 0.005;
    static double Qd1=0.0,Qd2=0.0,Q[3]={0.0,},T = .4;

    if(state2 == 1)
    {
//        d = 10;
//        T = 0.2;
        d = 50.0;
        T = 0.4;
    }else if(state2 == 0)
    {
        d = 50;
        T = 0.05;
    }else if(state2 == 3)
    {
        d = 2500;
        T = 0.5;
    }

    if(state3 == 0)
    {
        Q[0] = 0;
        Q[1] = 0;
        Q[2] = 0;
        CL = 0;
    }

    if(state2 != 2){


        if(pv_Index >=2)
        {

            Q[0] = ( dTorque - mTorque )/d - (1.0/T)*Q[1];

            CL = Q[2] = Q[1] + Q[0]*dt;

            Q[1] = Q[2];

            if(CL > 0.5)
            {
                CL = 0.5;
            }else if( CL < -0.5)
            {
                CL = -0.5;
            }

        }
    }

        return CL*R2D;



}

void ReactiveControl(int state,int state2,int state3)
{
    RDPitch = RMYC(state,state2,state3, window[0].zmp.x,window[0].zmp.y,0,0, window[0].left_foot_ref.x,window[0].right_foot_ref.x,window[0].left_foot_ref.y,window[0].right_foot_ref.y,sharedData->FT[RAFT].My,0.);
    LDPitch = LMYC(state,state2,state3, window[0].zmp.x,window[0].zmp.y,0,0, window[0].left_foot_ref.x,window[0].right_foot_ref.x,window[0].left_foot_ref.y,window[0].right_foot_ref.y,sharedData->FT[LAFT].My,0.);

    RDRoll = RMXC(state,state2,state3, window[0].zmp.x,window[0].zmp.y,0,0, window[0].left_foot_ref.x,window[0].right_foot_ref.x,window[0].left_foot_ref.y,window[0].right_foot_ref.y,sharedData->FT[RAFT].Mx,0.);
    LDRoll = LMXC(state,state2,state3, window[0].zmp.x,window[0].zmp.y,0,0, window[0].left_foot_ref.x,window[0].right_foot_ref.x,window[0].left_foot_ref.y,window[0].right_foot_ref.y,sharedData->FT[LAFT].Mx,0.);

    RDF = RightDesForce(state, window[0].zmp.x,window[0].zmp.y, window[0].left_foot_ref.x,window[0].right_foot_ref.x,window[0].left_foot_ref.y,window[0].right_foot_ref.y);

    LDF = LeftDesForce(state, window[0].zmp.x,window[0].zmp.y, window[0].left_foot_ref.x,window[0].right_foot_ref.x,window[0].left_foot_ref.y,window[0].right_foot_ref.y);

    if(sharedData->FT[RAFT].Fz > 20 || sharedData->FT[LAFT].Fz > 20)
    {
        Zctrl = FootForceControl(window[0].state,RDF,LDF,sharedData->FT[RAFT].Fz,sharedData->FT[LAFT].Fz,0.0,0.);

        Zctrl2 = FootForceControl2(window[0].state,RDF,LDF,sharedData->FT[RAFT].Fz,sharedData->FT[LAFT].Fz,0,0.);
    }
}

/****************************** 8. Initialize *******************************/
void First_Initialize()
{

    Controller_initialize();

    //180725 WalkReady
    wbwalk.fkik.WBIK_Q0[idRHY] = 0.;
    wbwalk.fkik.WBIK_Q0[idRHR] = 0.;//-2.78*D2R;
    wbwalk.fkik.WBIK_Q0[idRHP] = -37.084*D2R;//-43.9*D2R;
    wbwalk.fkik.WBIK_Q0[idRKN] = 68.566*D2R;//80.58*D2R; //77.4*D2R;
    wbwalk.fkik.WBIK_Q0[idRAP] = -31.483*D2R;//-36.68*D2R;
    wbwalk.fkik.WBIK_Q0[idRAR] = 0.;//2.78*D2R;

    wbwalk.fkik.WBIK_Q0[idLHY] = 0.;
    wbwalk.fkik.WBIK_Q0[idLHR] = 0.;//2.78*D2R;
    wbwalk.fkik.WBIK_Q0[idLHP] = -37.084*D2R;//-43.9*D2R;
    wbwalk.fkik.WBIK_Q0[idLKN] = 68.566*D2R;//80.58*D2R;//77.4*D2R;
    wbwalk.fkik.WBIK_Q0[idLAP] = -31.483*D2R;//-36.68*D2R;
    wbwalk.fkik.WBIK_Q0[idLAR] = 0.;//-2.78*D2R;

    wbwalk.fkik.WBIK_Q0[idRSP] = 40.*D2R;
    wbwalk.fkik.WBIK_Q0[idLSP] = 40.*D2R;

    wbwalk.fkik.WBIK_Q0[idRSR] = 10.*D2R;
    wbwalk.fkik.WBIK_Q0[idLSR] = -10.*D2R;

    wbwalk.fkik.WBIK_Q0[idRSY] = 0.*D2R;
    wbwalk.fkik.WBIK_Q0[idLSY] = 0.*D2R;

    wbwalk.fkik.WBIK_Q0[idREB] = -130.*D2R;
    wbwalk.fkik.WBIK_Q0[idLEB] = -130.*D2R;

    wbwalk.fkik.WBIK_Q0[idRWY] = 0.*D2R;
    wbwalk.fkik.WBIK_Q0[idLWY] = 0.*D2R;

    wbwalk.fkik.WBIK_Q0[idRWP] = 20.*D2R;
    wbwalk.fkik.WBIK_Q0[idLWP] = 20.*D2R;
    wbwalk.fkik.WBIK_Q0[idRWY2] = 0;//20.*D2R;
    wbwalk.fkik.WBIK_Q0[idLWY2] = 0;//20.*D2R;
    wbwalk.fkik.WBIK_Q0[idWST] = 0;//-180*D2R;//20.*D2R;

    WBIK_PARA_CHANGE();

    cout << "IK Version: " << kine_drc_hubo4.get_version() << endl;

    wbwalk.fkik.desired.pCOM_3x1[0] = userData->WalkReadyCOM[0] = 0.0;//0.0237f;
    wbwalk.fkik.desired.pCOM_3x1[1] = userData->WalkReadyCOM[1] = 0.0;
    wbwalk.fkik.desired.pCOM_3x1[2] = userData->WalkReadyCOM[2] = 0.77;// + 0.11;//(61.8kg:0.74)//59//0.8;//71kg

    wbwalk.fkik.desired.qPEL_4x1[0] = 1.;
    wbwalk.fkik.desired.qPEL_4x1[1] = 0.;
    wbwalk.fkik.desired.qPEL_4x1[2] = 0.;
    wbwalk.fkik.desired.qPEL_4x1[3] = 0.;

    wbwalk.fkik.desired.pRF_3x1[0] = 0.;
    wbwalk.fkik.desired.pRF_3x1[1] = -kine_drc_hubo4.L_PEL2PEL/2.;//-0.13;//-kine_drc_hubo4.L_PEL2PEL/2;//-0.135;//
    wbwalk.fkik.desired.pRF_3x1[2] = 0.;

    wbwalk.fkik.desired.qRF_4x1[0] = 1.;
    wbwalk.fkik.desired.qRF_4x1[1] = 0.;
    wbwalk.fkik.desired.qRF_4x1[2] = 0.;
    wbwalk.fkik.desired.qRF_4x1[3] = 0.;

    wbwalk.fkik.desired.pLF_3x1[0] = 0.;
    wbwalk.fkik.desired.pLF_3x1[1] = kine_drc_hubo4.L_PEL2PEL/2.;//0.13;//kine_drc_hubo4.L_PEL2PEL/2;//0.135;//
    wbwalk.fkik.desired.pLF_3x1[2] = 0.;

    wbwalk.fkik.desired.qLF_4x1[0] = 1.;
    wbwalk.fkik.desired.qLF_4x1[1] = 0.;
    wbwalk.fkik.desired.qLF_4x1[2] = 0.;
    wbwalk.fkik.desired.qLF_4x1[3] = 0.;

    printf("First Init Upper Right wbwalk.fkik.WBIK_Q0 : %f  %f  %f  %f  %f  %f %f \n",wbwalk.fkik.WBIK_Q0[idRSP]*R2D,wbwalk.fkik.WBIK_Q0[idRSR]*R2D,wbwalk.fkik.WBIK_Q0[idRSY]*R2D,wbwalk.fkik.WBIK_Q0[idREB]*R2D,wbwalk.fkik.WBIK_Q0[idRWY]*R2D,wbwalk.fkik.WBIK_Q0[idRWP]*R2D,wbwalk.fkik.WBIK_Q0[idRWY2]*R2D);

    get_WBIK_Q_from_RefAngleCurrent();
    printf("First Init Upper Right wbwalk.fkik.WBIK_Q : %f  %f  %f  %f  %f  %f %f \n",wbwalk.fkik.WBIK_Q[idRSP]*R2D,wbwalk.fkik.WBIK_Q[idRSR]*R2D,wbwalk.fkik.WBIK_Q[idRSY]*R2D,wbwalk.fkik.WBIK_Q[idREB]*R2D,wbwalk.fkik.WBIK_Q[idRWY]*R2D,wbwalk.fkik.WBIK_Q[idRWP]*R2D,wbwalk.fkik.WBIK_Q[idRWY2]*R2D);

    printf("First Init Lower Right wbwalk.fkik.WBIK_Q : %f  %f  %f  %f  %f  %f \n",wbwalk.fkik.WBIK_Q[7]*R2D,wbwalk.fkik.WBIK_Q[8]*R2D,wbwalk.fkik.WBIK_Q[9]*R2D,wbwalk.fkik.WBIK_Q[10]*R2D,wbwalk.fkik.WBIK_Q[11]*R2D,wbwalk.fkik.WBIK_Q[12]*R2D);
    printf("First Init Lower Left  wbwalk.fkik.WBIK_Q : %f  %f  %f  %f  %f  %f \n",wbwalk.fkik.WBIK_Q[13]*R2D,wbwalk.fkik.WBIK_Q[14]*R2D,wbwalk.fkik.WBIK_Q[15]*R2D,wbwalk.fkik.WBIK_Q[16]*R2D,wbwalk.fkik.WBIK_Q[17]*R2D,wbwalk.fkik.WBIK_Q[18]*R2D);

    usleep(100*1000);
    printf("COM : %f  %f  %f \n",wbwalk.fkik.desired.pCOM_3x1[0],wbwalk.fkik.desired.pCOM_3x1[1],wbwalk.fkik.desired.pCOM_3x1[2]);
    printf("qPel : %f  %f  %f  %f \n",wbwalk.fkik.desired.qPEL_4x1[0],wbwalk.fkik.desired.qPEL_4x1[1],wbwalk.fkik.desired.qPEL_4x1[2],wbwalk.fkik.desired.qPEL_4x1[3]);
    printf("pRF : %f  %f  %f \n",wbwalk.fkik.desired.pRF_3x1[0],wbwalk.fkik.desired.pRF_3x1[1],wbwalk.fkik.desired.pRF_3x1[2]);
    printf("qRF : %f  %f  %f  %f \n",wbwalk.fkik.desired.qRF_4x1[0],wbwalk.fkik.desired.qRF_4x1[1],wbwalk.fkik.desired.qRF_4x1[2],wbwalk.fkik.desired.qRF_4x1[3]);
    printf("pLF : %f  %f  %f \n",wbwalk.fkik.desired.pLF_3x1[0],wbwalk.fkik.desired.pLF_3x1[1],wbwalk.fkik.desired.pLF_3x1[2]);
    printf("qLF : %f  %f  %f  %f \n",wbwalk.fkik.desired.qLF_4x1[0],wbwalk.fkik.desired.qLF_4x1[1],wbwalk.fkik.desired.qLF_4x1[2],wbwalk.fkik.desired.qLF_4x1[3]);
    printf("pRH : %f  %f  %f \n",wbwalk.fkik.desired.pRH_3x1[0],wbwalk.fkik.desired.pRH_3x1[1],wbwalk.fkik.desired.pRH_3x1[2]);
    printf("qRH : %f  %f  %f  %f \n",wbwalk.fkik.desired.qRH_4x1[0],wbwalk.fkik.desired.qRH_4x1[1],wbwalk.fkik.desired.qRH_4x1[2],wbwalk.fkik.desired.qRH_4x1[3]);
    printf("pLH : %f  %f  %f \n",wbwalk.fkik.desired.pLH_3x1[0],wbwalk.fkik.desired.pLH_3x1[1],wbwalk.fkik.desired.pLH_3x1[2]);
    printf("qLH : %f  %f  %f  %f \n",wbwalk.fkik.desired.qLH_4x1[0],wbwalk.fkik.desired.qLH_4x1[1],wbwalk.fkik.desired.qLH_4x1[2],wbwalk.fkik.desired.qLH_4x1[3]);

    get_WBIK_Q_from_RefAngleCurrent();
    printf("First Init Upper Right wbwalk.fkik.WBIK_Q : %f  %f  %f  %f  %f  %f %f \n",wbwalk.fkik.WBIK_Q[idRSP]*R2D,wbwalk.fkik.WBIK_Q[idRSR]*R2D,wbwalk.fkik.WBIK_Q[idRSY]*R2D,wbwalk.fkik.WBIK_Q[idREB]*R2D,wbwalk.fkik.WBIK_Q[idRWY]*R2D,wbwalk.fkik.WBIK_Q[idRWP]*R2D,wbwalk.fkik.WBIK_Q[idRWY2]*R2D);

    //----------------------------------- Reset COM to 0
    wbwalk.fkik.WBIK_Q[idQ0] = 1;
    wbwalk.fkik.WBIK_Q[idQ1] = 0;
    wbwalk.fkik.WBIK_Q[idQ2] = 0;
    wbwalk.fkik.WBIK_Q[idQ3] = 0;

    // PELVIS Position Reset
    kine_drc_hubo4.FK_COM_Global(wbwalk.fkik.WBIK_Q,wbwalk.fkik.FK.pCOM_3x1);
    printf("First Initialize FK com = %f,%f,%f,wbwalk.fkik.WBIK_Q[idXYZ] = %f,%f,%f\n",wbwalk.fkik.FK.pCOM_3x1[0],wbwalk.fkik.FK.pCOM_3x1[1],wbwalk.fkik.FK.pCOM_3x1[2],wbwalk.fkik.WBIK_Q[idX],wbwalk.fkik.WBIK_Q[idY],wbwalk.fkik.WBIK_Q[idZ]);

    wbwalk.fkik.WBIK_Q[idX] = wbwalk.fkik.WBIK_Q[idX] - wbwalk.fkik.FK.pCOM_3x1[0];//reset to 0;
    wbwalk.fkik.WBIK_Q[idY] = wbwalk.fkik.WBIK_Q[idY] - wbwalk.fkik.FK.pCOM_3x1[1];//reset to 0;
    wbwalk.fkik.WBIK_Q[idZ] = wbwalk.fkik.WBIK_Q[idZ] - wbwalk.fkik.FK.pCOM_3x1[2] + userData->WalkReadyCOM[2];// + fsm->AddComInfos[0][2];//0;

    kine_drc_hubo4.FK_RightFoot_Global(wbwalk.fkik.WBIK_Q,wbwalk.fkik.FK.pRF_3x1,  wbwalk.fkik.FK.qRF_4x1);
    kine_drc_hubo4.FK_LeftFoot_Global(wbwalk.fkik.WBIK_Q,wbwalk.fkik.FK.pLF_3x1,  wbwalk.fkik.FK.qLF_4x1);
    kine_drc_hubo4.FK_RightHand_Local(wbwalk.fkik.WBIK_Q,2,wbwalk.fkik.FK.pRH_3x1,wbwalk.fkik.FK.qRH_4x1,wbwalk.fkik.FK.REB_ang);
    kine_drc_hubo4.FK_LeftHand_Local(wbwalk.fkik.WBIK_Q,2,wbwalk.fkik.FK.pLH_3x1,wbwalk.fkik.FK.qLH_4x1,wbwalk.fkik.FK.LEB_ang);
    kine_drc_hubo4.FK_COM_Global(wbwalk.fkik.WBIK_Q,wbwalk.fkik.FK.pCOM_3x1);

    printf("222Fisrt Iniitalize  FK com = %f,%f,%f,wbwalk.fkik.WBIK_Q[idXYZ] = %f,%f,%f\n",wbwalk.fkik.FK.pCOM_3x1[0],wbwalk.fkik.FK.pCOM_3x1[1],wbwalk.fkik.FK.pCOM_3x1[2],wbwalk.fkik.WBIK_Q[idX],wbwalk.fkik.WBIK_Q[idY],wbwalk.fkik.WBIK_Q[idZ]);

    printf("FK right pos  %f, %f, %f \n",wbwalk.fkik.FK.pRF_3x1[0],wbwalk.fkik.FK.pRF_3x1[1],wbwalk.fkik.FK.pRF_3x1[2]);
    printf("FK right ori  %f, %f, %f \n",wbwalk.fkik.FK.qRF_4x1[0],wbwalk.fkik.FK.qRF_4x1[1],wbwalk.fkik.FK.qRF_4x1[2],wbwalk.fkik.FK.qRF_4x1[3]);

    printf("FK left  pos  %f, %f, %f \n",wbwalk.fkik.FK.pLF_3x1[0],wbwalk.fkik.FK.pLF_3x1[1],wbwalk.fkik.FK.pLF_3x1[2]);
    printf("FK left  ori  %f, %f, %f \n",wbwalk.fkik.FK.qLF_4x1[0],wbwalk.fkik.FK.qLF_4x1[1],wbwalk.fkik.FK.qLF_4x1[2],wbwalk.fkik.FK.qLF_4x1[3]);

}

void WBIK_PARA_CHANGE(){
    // Tuned value with the head
    kine_drc_hubo4.C_Torso[0] = 0.000941-0.065;
    // weight of Torso is increased ( Torso + Head)
    kine_drc_hubo4.m_Torso = 24.98723 + 0.0; // kg
    kine_drc_hubo4.m_RightWrist = 4.5;
    kine_drc_hubo4.m_LeftWrist = 4.5;
    kine_drc_hubo4.m_Pelvis = 11.867886 +2.;
    kine_drc_hubo4.C_Torso[1] =0;
    kine_drc_hubo4.L_FOOT = 0.113;
    kine_drc_hubo4.iter_limit = 100;
    kine_drc_hubo4.converge_criterium = 1e-6;

}

void Walking_initialize()
{

    printf("##################### Walking Initialize!!!!!!!!!!!!!!!!!!! \n");
    FINAL_TIMER = 0.;
    Controller_initialize();

    get_WBIK_Q_from_RefAngleCurrent();

    if(FLAG_UBIK == false)
    {
        double PelYaw,PelRoll,PelPitch,temp1_qPel_4x1[4],temp2_qPel_4x1[4],temp3_qPel_4x1[4],temp4_qPel_4x1[4],temp5_qPel_4x1[4];
        PelYaw = 0*D2R;
        PelRoll = 0*D2R;
        PelPitch = 0*D2R;

        qtRZ(PelYaw, temp1_qPel_4x1);
        qtRX(PelRoll, temp2_qPel_4x1);
        qtRY(PelPitch, temp3_qPel_4x1);

        QTcross(temp1_qPel_4x1,temp2_qPel_4x1,temp4_qPel_4x1);
        QTcross(temp4_qPel_4x1,temp3_qPel_4x1,temp5_qPel_4x1);

        wbwalk.fkik.WBIK_Q[idQ0] = temp5_qPel_4x1[0];//1;
        wbwalk.fkik.WBIK_Q[idQ1] = temp5_qPel_4x1[1];//0;
        wbwalk.fkik.WBIK_Q[idQ2] = temp5_qPel_4x1[2];//0;
        wbwalk.fkik.WBIK_Q[idQ3] = temp5_qPel_4x1[3];//0;
    }


    // PELVIS Position Reset

    printf("Walking Init Upper Right wbwalk.fkik.WBIK_Q : %f  %f  %f  %f  %f  %f %f \n",wbwalk.fkik.WBIK_Q[idRSP]*R2D,wbwalk.fkik.WBIK_Q[idRSR]*R2D,wbwalk.fkik.WBIK_Q[idRSY]*R2D,wbwalk.fkik.WBIK_Q[idREB]*R2D,wbwalk.fkik.WBIK_Q[idRWY]*R2D,wbwalk.fkik.WBIK_Q[idRWP]*R2D,wbwalk.fkik.WBIK_Q[idRWY2]*R2D);
    kine_drc_hubo4.FK_COM_Global(wbwalk.fkik.WBIK_Q,wbwalk.fkik.FK.pCOM_3x1);
    kine_drc_hubo4.FK_RightFoot_Global(wbwalk.fkik.WBIK_Q,wbwalk.fkik.FK.pRF_3x1,  wbwalk.fkik.FK.qRF_4x1);
    kine_drc_hubo4.FK_LeftFoot_Global(wbwalk.fkik.WBIK_Q,wbwalk.fkik.FK.pLF_3x1,  wbwalk.fkik.FK.qLF_4x1);
    kine_drc_hubo4.FK_RightHand_Local(wbwalk.fkik.WBIK_Q,2,wbwalk.fkik.FK.pRH_3x1,wbwalk.fkik.FK.qRH_4x1,wbwalk.fkik.FK.REB_ang);
    kine_drc_hubo4.FK_LeftHand_Local(wbwalk.fkik.WBIK_Q,2,wbwalk.fkik.FK.pLH_3x1,wbwalk.fkik.FK.qLH_4x1,wbwalk.fkik.FK.LEB_ang);

    printf("FK3 RH = %f, %f, %f, LH = %f, %f, %f\n",wbwalk.fkik.FK.pRH_3x1[0],wbwalk.fkik.FK.pRH_3x1[1],wbwalk.fkik.FK.pRH_3x1[2],wbwalk.fkik.FK.pLH_3x1[0],wbwalk.fkik.FK.pLH_3x1[1],wbwalk.fkik.FK.pLH_3x1[2]);
    printf("FK3 com = %f,%f,%f,PEL = %f,%f,%f,RF = %f,%f,%f,LF = %f,%f,%f\n",wbwalk.fkik.FK.pCOM_3x1[0],wbwalk.fkik.FK.pCOM_3x1[1],wbwalk.fkik.FK.pCOM_3x1[2],wbwalk.fkik.WBIK_Q[idX],wbwalk.fkik.WBIK_Q[idY],wbwalk.fkik.WBIK_Q[idZ],wbwalk.fkik.FK.pRF_3x1[0],wbwalk.fkik.FK.pRF_3x1[1],wbwalk.fkik.FK.pRF_3x1[2],wbwalk.fkik.FK.pLF_3x1[0],wbwalk.fkik.FK.pLF_3x1[1],wbwalk.fkik.FK.pLF_3x1[2]);

    printf("init_WBIK_pCOM : (%f,%f,%f),init_wbwalk.fkik.WBIK_Q : (%f,%f,%f)\n",init_WBIK_pCOM[0],init_WBIK_pCOM[1],init_WBIK_pCOM[2],init_WBIK_Q[0],init_WBIK_Q[1],init_WBIK_Q[2]);

    wbwalk.fkik.WBIK_Q[idX] = wbwalk.fkik.WBIK_Q[idX] - wbwalk.fkik.FK.pCOM_3x1[0];//reset to 0;
    wbwalk.fkik.WBIK_Q[idY] = wbwalk.fkik.WBIK_Q[idY] - wbwalk.fkik.FK.pCOM_3x1[1];//reset to 0;
    wbwalk.fkik.WBIK_Q[idZ] = wbwalk.fkik.WBIK_Q[idZ] - wbwalk.fkik.FK.pCOM_3x1[2] + userData->WalkReadyCOM[2];


    printf("========================\n");
    printf("PEL : %f,%f,%f,%f,%f,%f,%f\n",wbwalk.fkik.WBIK_Q[idX],wbwalk.fkik.WBIK_Q[idY],wbwalk.fkik.WBIK_Q[idZ],wbwalk.fkik.WBIK_Q[idQ0],wbwalk.fkik.WBIK_Q[idQ1],wbwalk.fkik.WBIK_Q[idQ2],wbwalk.fkik.WBIK_Q[idQ3]);
    printf("RLEG : %f,%f,%f,%f,%f,%f\n",wbwalk.fkik.WBIK_Q[idRHY],wbwalk.fkik.WBIK_Q[idRHR],wbwalk.fkik.WBIK_Q[idRHP],wbwalk.fkik.WBIK_Q[idRKN],wbwalk.fkik.WBIK_Q[idRAP],wbwalk.fkik.WBIK_Q[idRAR]);
    printf("LLEG : %f,%f,%f,%f,%f,%f\n",wbwalk.fkik.WBIK_Q[idLHY],wbwalk.fkik.WBIK_Q[idLHR],wbwalk.fkik.WBIK_Q[idLHP],wbwalk.fkik.WBIK_Q[idLKN],wbwalk.fkik.WBIK_Q[idLAP],wbwalk.fkik.WBIK_Q[idLAR]);

    printf("RARM : %f,%f,%f,%f,%f,%f,%f\n",wbwalk.fkik.WBIK_Q[idRSP],wbwalk.fkik.WBIK_Q[idRSR],wbwalk.fkik.WBIK_Q[idRSY],wbwalk.fkik.WBIK_Q[idREB],wbwalk.fkik.WBIK_Q[idRWP],wbwalk.fkik.WBIK_Q[idRWY],wbwalk.fkik.WBIK_Q[idRWY2]);
    printf("LARM : %f,%f,%f,%f,%f,%f,%f\n",wbwalk.fkik.WBIK_Q[idLSP],wbwalk.fkik.WBIK_Q[idLSR],wbwalk.fkik.WBIK_Q[idLSY],wbwalk.fkik.WBIK_Q[idLEB],wbwalk.fkik.WBIK_Q[idLWP],wbwalk.fkik.WBIK_Q[idLWY],wbwalk.fkik.WBIK_Q[idLWY2]);
    printf("========================\n");


    kine_drc_hubo4.FK_RightFoot_Global(wbwalk.fkik.WBIK_Q,wbwalk.fkik.FK.pRF_3x1,  wbwalk.fkik.FK.qRF_4x1);
    kine_drc_hubo4.FK_LeftFoot_Global(wbwalk.fkik.WBIK_Q,wbwalk.fkik.FK.pLF_3x1,  wbwalk.fkik.FK.qLF_4x1);
    kine_drc_hubo4.FK_RightHand_Local(wbwalk.fkik.WBIK_Q,2,wbwalk.fkik.FK.pRH_3x1,wbwalk.fkik.FK.qRH_4x1,wbwalk.fkik.FK.REB_ang);
    kine_drc_hubo4.FK_LeftHand_Local(wbwalk.fkik.WBIK_Q,2,wbwalk.fkik.FK.pLH_3x1,wbwalk.fkik.FK.qLH_4x1,wbwalk.fkik.FK.LEB_ang);
    kine_drc_hubo4.FK_COM_Global(wbwalk.fkik.WBIK_Q,wbwalk.fkik.FK.pCOM_3x1);

    printf("[[[[[[[[[[[[[[FK]]]]]]]]]]]]]]]]\n");
    printf("com = %f,%f,%f,PEL = %f,%f,%f,\nRF = %f,%f,%f,LF = %f,%f,%f\nRH = %f, %f, %f\nLH = %f, %f, %f\n",
           wbwalk.fkik.FK.pCOM_3x1[0],wbwalk.fkik.FK.pCOM_3x1[1],wbwalk.fkik.FK.pCOM_3x1[2],
            wbwalk.fkik.WBIK_Q[idX],wbwalk.fkik.WBIK_Q[idY],wbwalk.fkik.WBIK_Q[idZ],
            wbwalk.fkik.FK.pRF_3x1[0],wbwalk.fkik.FK.pRF_3x1[1],wbwalk.fkik.FK.pRF_3x1[2],
            wbwalk.fkik.FK.pLF_3x1[0],wbwalk.fkik.FK.pLF_3x1[1],wbwalk.fkik.FK.pLF_3x1[2],
            wbwalk.fkik.FK.pRH_3x1[0],wbwalk.fkik.FK.pRH_3x1[1],wbwalk.fkik.FK.pRH_3x1[2],
            wbwalk.fkik.FK.pLH_3x1[0],wbwalk.fkik.FK.pLH_3x1[1],wbwalk.fkik.FK.pLH_3x1[2]);


    //Set Foot print Initial value


    QT2YPR(wbwalk.fkik.FK.qRF_4x1,FK_RFoot_yaw,FK_RFoot_pitch,FK_RFoot_roll);

    QT2YPR(wbwalk.fkik.FK.qLF_4x1,FK_LFoot_yaw,FK_LFoot_pitch,FK_LFoot_roll);


    Init_Right_Leg = GLOBAL_Z_RF = GLOBAL_Z_RF_last[0] = wbwalk.fkik.FK.pRF_3x1[2];//fsm->RightInfos[0][2];
    Init_Left_Leg  = GLOBAL_Z_LF = GLOBAL_Z_LF_last[0] = wbwalk.fkik.FK.pLF_3x1[2];//fsm->LeftInfos[0][2];



        wbwalk.fkik.desired.pRF_3x1[0] =wbwalk.fkik.FK.pRF_3x1[0];
        wbwalk.fkik.desired.pRF_3x1[1] =wbwalk.fkik.FK.pRF_3x1[1];
        wbwalk.fkik.desired.pRF_3x1[2] =wbwalk.fkik.FK.pRF_3x1[2];

        wbwalk.fkik.desired.pLF_3x1[0] =wbwalk.fkik.FK.pLF_3x1[0];
        wbwalk.fkik.desired.pLF_3x1[1] =wbwalk.fkik.FK.pLF_3x1[1];
        wbwalk.fkik.desired.pLF_3x1[2] =wbwalk.fkik.FK.pLF_3x1[2];

        wbwalk.fkik.desired.pCOM_3x1[0] = wbwalk.fkik.FK.pCOM_3x1[0];
        wbwalk.fkik.desired.pCOM_3x1[1] = wbwalk.fkik.FK.pCOM_3x1[1];
        wbwalk.fkik.desired.pCOM_3x1[2] = wbwalk.fkik.FK.pCOM_3x1[2];

    //controller reset
    kirkZMPCon_XP2(0,0,0);
    kirkZMPCon_YP2(0,0,0);
    Del_PC_X_DSP_XZMP_CON = 0;
    Del_PC_Y_DSP_YZMP_CON = 0;

    Del_PC_X_SSP_XZMP_CON = 0;
    Del_PC_Y_SSP_YZMP_CON = 0;

    I_ZMP_CON_X = 0.0f;
    I_ZMP_CON_Y = 0.0f;

}

void Controller_initialize()
{
//    wbwalk.fkik.desired.pRF_3x1[0] =0;
//    wbwalk.fkik.desired.pRF_3x1[1] =0;
//    wbwalk.fkik.desired.pRF_3x1[2] =0;

//    wbwalk.fkik.desired.pLF_3x1[0] =0;
//    wbwalk.fkik.desired.pLF_3x1[1] =0;
//    wbwalk.fkik.desired.pLF_3x1[2] =0;


    X_ZMP = 0;
    Y_ZMP = 0;
    X_ZMP_Local = 0;
    Y_ZMP_Local = 0;
    X_ZMP_Global = 0;
    Y_ZMP_Global = 0;
    X_ZMP_n = 0;
    Y_ZMP_n = 0;
    X_ZMP_LPF = 0;
    Y_ZMP_LPF = 0;
    CNT_final_gain_DSP_ZMP_CON = 0;
    CNT_final_gain_SSP_ZMP_CON = 0;


    // Gyro feedback;
    y_i_1 = 0;
    y_i_11= 0;
    u_i_1 = 0;
    u_i_11 = 0;
    NotchFilter_GyroRollControlInput(0,0);
    NotchFilter_GyroPitchControlInput(0,0);
    NotchFilter_GyroRollVel(0,0);
    NotchFilter_GyroPitchVel(0,0);
    GLOBAL_Xori_RF_last = 0;
    GLOBAL_Xori_LF_last = 0;
    GLOBAL_Yori_RF_last = 0;
    GLOBAL_Yori_LF_last = 0;

    GLOBAL_Xori_RF2_last = 0;
    GLOBAL_Xori_LF2_last = 0;
    GLOBAL_Yori_RF2_last = 0;

    GLOBAL_Yori_LF2_last = 0;

    U_Gain = 0.;
    GLOBAL_Xori_RF = 0.;
    GLOBAL_Xori_LF = 0.;
    GLOBAL_Yori_RF = 0.;
    GLOBAL_Yori_LF = 0.;






    // ankle torque control;
    RDPitch = RMYC(2,2,0,0.,0.,0./1000.0,0./1000.0,wbwalk.fkik.desired.pLF_3x1[0],wbwalk.fkik.desired.pRF_3x1[0],wbwalk.fkik.desired.pLF_3x1[1],wbwalk.fkik.desired.pRF_3x1[1],0.,0.);
    LDPitch = LMYC(2,2,0,0.,0.,0./1000.0,0./1000.0,wbwalk.fkik.desired.pLF_3x1[0],wbwalk.fkik.desired.pRF_3x1[0],wbwalk.fkik.desired.pLF_3x1[1],wbwalk.fkik.desired.pRF_3x1[1],0.,0.);
    RDPitch =0.;
    LDPitch =0.;
    RDRoll = RMXC(2,2,0,0.,0.,0./1000.0,0./1000.0,wbwalk.fkik.desired.pLF_3x1[0],wbwalk.fkik.desired.pRF_3x1[0],wbwalk.fkik.desired.pLF_3x1[1],wbwalk.fkik.desired.pRF_3x1[1],0.,0.);
    LDRoll = LMXC(2,2,0,0.,0.,0./1000.0,0./1000.0,wbwalk.fkik.desired.pLF_3x1[0],wbwalk.fkik.desired.pRF_3x1[0],wbwalk.fkik.desired.pLF_3x1[1],wbwalk.fkik.desired.pRF_3x1[1],0.,0.);
    RDRoll =0.;
    LDRoll =0.;

    RDPitch2 = RMYC2(2,2,0,0.,0.,0./1000.0,0./1000.0,wbwalk.fkik.desired.pLF_3x1[0],wbwalk.fkik.desired.pRF_3x1[0],wbwalk.fkik.desired.pLF_3x1[1],wbwalk.fkik.desired.pRF_3x1[1],0.,0.);
    LDPitch2= LMYC2(2,2,0,0.,0.,0./1000.0,0./1000.0,wbwalk.fkik.desired.pLF_3x1[0],wbwalk.fkik.desired.pRF_3x1[0],wbwalk.fkik.desired.pLF_3x1[1],wbwalk.fkik.desired.pRF_3x1[1],0.,0.);
    RDPitch2 =0.;
    LDPitch2 =0.;


    RDRoll2 = RMXC2(2,2,0,0.,0.,0./1000.0,0./1000.0,wbwalk.fkik.desired.pLF_3x1[0],wbwalk.fkik.desired.pRF_3x1[0],wbwalk.fkik.desired.pLF_3x1[1],wbwalk.fkik.desired.pRF_3x1[1],0.,0.);
    LDRoll2= LMXC2(2,2,0,0.,0.,0./1000.0,0./1000.0,wbwalk.fkik.desired.pLF_3x1[0],wbwalk.fkik.desired.pRF_3x1[0],wbwalk.fkik.desired.pLF_3x1[1],wbwalk.fkik.desired.pRF_3x1[1],0.,0.);
    RDRoll2 =0.;
    LDRoll2 =0.;


}

/****************************** 9. Functions ********************************/
/* GainOverride */
void Upperbody_Gain_Lock()
{

    MCenableFrictionCompensation(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[LSP].canch,JOINT_INFO[LSP].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[LSR].canch,JOINT_INFO[LSR].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[LSY].canch,JOINT_INFO[LSY].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[LEB].canch,JOINT_INFO[LEB].bno, SW_MODE_COMPLEMENTARY);

    MCenableFrictionCompensation(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[RSP].canch,JOINT_INFO[RSP].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[RSY].canch,JOINT_INFO[RSY].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[RSR].canch,JOINT_INFO[RSR].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[REB].canch,JOINT_INFO[REB].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, DISABLE);

            MCJointGainOverride(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, 0,500); //--LSP
            MCJointGainOverride(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, 0,500); //--LSR
            MCJointGainOverride(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, 0,500); //--LSY
            MCJointGainOverride(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, 0,500); //--LEB

            MCJointGainOverride(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, 0,500); //--RSP
            MCJointGainOverride(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, 0,500); //--RSR
            MCJointGainOverride(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, 0,500); //--RSY
            MCJointGainOverride(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, 0,500); //--REB


}
void Upperbody_Gain_Override()
{
    ZeroGainLeftArm();
    ZeroGainRightArm();
}
int ZeroGainLeftArm(){

    MCsetFrictionParameter(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, 1000, 10, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, 50, 10);
    usleep(5000);

    MCsetFrictionParameter(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, 1000, 7, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, 50, 10);
    usleep(5000);

    MCsetFrictionParameter(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, 1000, 9, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, 20, 10);
    usleep(5000);
    MCsetFrictionParameter(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, 1000, 5, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, 40, 10);
    usleep(5000);



    cout<<"Zero gain LeftArm!"<<endl;
    return 0;
}
int ZeroGainRightArm(){

    MCsetFrictionParameter(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, 1000, 8, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, 50, 10);
    usleep(5000);

    MCsetFrictionParameter(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, 1000, 7, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, 50, 10);
    usleep(5000);

    MCsetFrictionParameter(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, 1000, 9, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, 20, 10);
    usleep(5000);
    MCsetFrictionParameter(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, 1000, 5, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, 40, 10);
    usleep(5000);

    cout<<"Zero Gain Right Arm!"<<endl;

    return 0;
}

/* FeedForward Motion Control */
void Compensator_deflection(int state)
{
    static double p0[3] = {0.,0.,0.},pf[3] = {0.0,0.,0.},ref[3]={0.,};
    double comp_angle = 1.0; // deg
    double sagging_time = 0.2;
    if(state == SSP_RF)
    {

        if(window[0].timer.current<sagging_time)
        {
            p0[0] = 0.;
            pf[0] = comp_angle;
            Fifth(window[0].timer.current,sagging_time,p0,pf,ref);
            deflection_comp_LAR = ref[0];

        }else if(window[0].timer.current > sagging_time + 0.4 && window[0].timer.current < sagging_time + 0.6)
        {
            p0[0] = comp_angle;
            pf[0] = 0.;
            Fifth(window[0].timer.current-sagging_time - 0.4,sagging_time,p0,pf,ref);
            deflection_comp_LAR = ref[0];
        }

    }else if(state == SSP_LF)
    {

        if(window[0].timer.current<sagging_time)
        {
            p0[0] = 0.;
            pf[0] = -comp_angle;
            Fifth(window[0].timer.current,sagging_time,p0,pf,ref);
            deflection_comp_RAR = ref[0];

        }else if(window[0].timer.current > sagging_time + 0.4 && window[0].timer.current < sagging_time + 0.6)
        {
            p0[0] = -comp_angle;
            pf[0] = 0.;
            Fifth(window[0].timer.current - sagging_time - 0.4,sagging_time,p0,pf,ref);
            deflection_comp_RAR = ref[0];
        }
    }
}

void JW_INV_MODEL(double Pattern1,double Pattern1_d,double Pattern2,double Pattern2_d)
{
    if(pv_Index == 1){
        JW_InvPattern_U[0]=Pattern1;
        JW_InvPattern_U[1]=Pattern2;

        JW_InvPattern_U_I[0] = 0.;
        JW_InvPattern_U_I[1] = 0.;

        JW_InvPattern_Y_old[0] = Pattern1;
        JW_InvPattern_Y_old[1] = Pattern1_d;

        JW_InvPattern_X_old[0] = Pattern2;
        JW_InvPattern_X_old[1] = Pattern2_d;
        printf("Invmodel reset!!!\n");
    }
        JW_InvPattern_Klqr[0]=30.6386;//20.6386f;//2.3166;// 30.6386;//315.2293f;
        JW_InvPattern_Klqr[1]=0.7508;//0.7508f;//0.1868;//0.7508;//2.4780f;

        JW_InvPattern_U_I[0] +=0.1*(Pattern1-JW_InvPattern_Y_old[0]);
        JW_InvPattern_U[0]   = JW_InvPattern_Klqr[0]*(Pattern1-JW_InvPattern_Y_old[0])+JW_InvPattern_Klqr[1]*(Pattern1_d-JW_InvPattern_Y_old[1])+JW_InvPattern_U_I[0];

        JW_InvPattern_U_I[1] +=0.1*(Pattern2-JW_InvPattern_X_old[0]);
        JW_InvPattern_U[1]   = JW_InvPattern_Klqr[0]*(Pattern2-JW_InvPattern_X_old[0])+JW_InvPattern_Klqr[1]*(Pattern2_d-JW_InvPattern_X_old[1])+JW_InvPattern_U_I[1];


        JW_InvPattern_A[0][0] =0.f;
        JW_InvPattern_A[0][1] =1.0f;
        JW_InvPattern_A[1][0] =-JW_InvPattern_k/JW_InvPattern_m;
        JW_InvPattern_A[1][1] =-JW_InvPattern_c/JW_InvPattern_m;

        JW_InvPattern_A_X[0][0] =0.f;
        JW_InvPattern_A_X[0][1] =1.0f;
        JW_InvPattern_A_X[1][0] =-JW_InvPattern_k_X/JW_InvPattern_m;
        JW_InvPattern_A_X[1][1] =-JW_InvPattern_c_X/JW_InvPattern_m;

        JW_InvPattern_l = sqrt((userData->WalkReadyCOM[2])*(userData->WalkReadyCOM[2]) + Pattern1*Pattern1);

        if(pv_Index == 1){

            Y_inv = Pattern1;
            Y_inv_d = Pattern1_d;
            theta = atan2(Pattern1,(userData->WalkReadyCOM[2]));
            theta_d = 0;
            U_I[0] = 0.0f;
            Y_inv_old = Pattern1;
        }
        U_I[0] +=.1*(Pattern1-Y_inv);
        U[0]   = JW_InvPattern_Klqr[0]*(Pattern1 - Y_inv) + JW_InvPattern_Klqr[1]*(Pattern1_d - Y_inv_d) + U_I[0];

        theta_ref = atan2(U[0],(userData->WalkReadyCOM[2]));
        if(pv_Index == 1){
            theta_ref =-(((9.81/JW_InvPattern_l*sin(theta))/(1/JW_InvPattern_m/(JW_InvPattern_l*JW_InvPattern_l))-JW_InvPattern_c*(theta_d))/JW_InvPattern_k - theta) ;//*(*(theta-theta_ref)+);

        }
        theta_dd = 9.81/JW_InvPattern_l*sin(theta)-1/JW_InvPattern_m/(JW_InvPattern_l*JW_InvPattern_l)*(JW_InvPattern_k*(theta-theta_ref)+JW_InvPattern_c*(theta_d));
        theta_d = theta_d + theta_dd*DEL_T;
        theta   = theta + theta_d*DEL_T;

        Y_inv_old = Y_inv;
        Y_inv=(userData->WalkReadyCOM[2])*tan(theta);
        Y_inv_d = (Y_inv - Y_inv_old)/DEL_T;
        JW_InvPattern_B[0] = 0.f;
        JW_InvPattern_B[1] = JW_InvPattern_k/JW_InvPattern_m;

        JW_InvPattern_B_X[0] = 0.f;
        JW_InvPattern_B_X[1] = JW_InvPattern_k_X/JW_InvPattern_m;

        JW_InvPattern_Y_d[0] = JW_InvPattern_A[0][0]*JW_InvPattern_Y_old[0] + JW_InvPattern_A[0][1]*JW_InvPattern_Y_old[1] + JW_InvPattern_B[0]*JW_InvPattern_U[0];
        JW_InvPattern_Y_d[1] = JW_InvPattern_A[1][0]*JW_InvPattern_Y_old[0] + JW_InvPattern_A[1][1]*JW_InvPattern_Y_old[1] + JW_InvPattern_B[1]*JW_InvPattern_U[0];

        JW_InvPattern_X_d[0] = JW_InvPattern_A_X[0][0]*JW_InvPattern_X_old[0] + JW_InvPattern_A_X[0][1]*JW_InvPattern_X_old[1] + JW_InvPattern_B_X[0]*JW_InvPattern_U[1];
        JW_InvPattern_X_d[1] = JW_InvPattern_A_X[1][0]*JW_InvPattern_X_old[0] + JW_InvPattern_A_X[1][1]*JW_InvPattern_X_old[1] + JW_InvPattern_B_X[1]*JW_InvPattern_U[1];

        JW_InvPattern_Y[0] = JW_InvPattern_Y_old[0] + JW_InvPattern_Y_d[0]*DEL_T;
        JW_InvPattern_Y[1] = JW_InvPattern_Y_old[1] + JW_InvPattern_Y_d[1]*DEL_T;

        JW_InvPattern_X[0] = JW_InvPattern_X_old[0] + JW_InvPattern_X_d[0]*DEL_T;
        JW_InvPattern_X[1] = JW_InvPattern_X_old[1] + JW_InvPattern_X_d[1]*DEL_T;

        JW_InvPattern_Y_old[0] = JW_InvPattern_Y[0];
        JW_InvPattern_Y_old[1] = JW_InvPattern_Y[1];

        JW_InvPattern_X_old[0] = JW_InvPattern_X[0];
        JW_InvPattern_X_old[1] = JW_InvPattern_X[1];

        double Global[3],Local[3];
        Global[0] = JW_InvPattern_U[1];
        Global[1] = 0;
        Global[2] = 0;
        Global2Local2(Global,Local);
        JW_InvPattern_U_n[1] = Local[0];
}

void get_WBIK_Q_from_RefAngleCurrent()
{
    jCon->RefreshToCurrentReference();
    for(int i=RHY; i<=LAR; i++) {
       wbwalk.fkik.WBIK_Q[i+7] = jCon->Joints[i]->RefAngleCurrent*D2R;
    }
    wbwalk.fkik.WBIK_Q[idWST] = jCon->GetJointRefAngle(WST)*D2R;
    wbwalk.fkik.WBIK_Q[idRSP] = jCon->GetJointRefAngle(RSP)*D2R;
    wbwalk.fkik.WBIK_Q[idRSR] = (jCon->GetJointRefAngle(RSR)+OFFSET_RSR)*D2R;
    printf("RefRSR = %f, %f\n",wbwalk.fkik.WBIK_Q[idRSR], jCon->GetJointRefAngle(RSR));
    wbwalk.fkik.WBIK_Q[idRSY] = jCon->GetJointRefAngle(RSY)*D2R;
    wbwalk.fkik.WBIK_Q[idREB] = (jCon->GetJointRefAngle(REB)+OFFSET_ELB)*D2R;
    wbwalk.fkik.WBIK_Q[idRWY] = jCon->GetJointRefAngle(RWY)*D2R;
    wbwalk.fkik.WBIK_Q[idRWP] = jCon->GetJointRefAngle(RWP)*D2R;
    wbwalk.fkik.WBIK_Q[idRWY2] = jCon->GetJointRefAngle(RWY2)*D2R;

    wbwalk.fkik.WBIK_Q[idLSP] = jCon->GetJointRefAngle(LSP)*D2R;
    wbwalk.fkik.WBIK_Q[idLSR] = (jCon->GetJointRefAngle(LSR)+OFFSET_LSR)*D2R;
    wbwalk.fkik.WBIK_Q[idLSY] = jCon->GetJointRefAngle(LSY)*D2R;
    wbwalk.fkik.WBIK_Q[idLEB] = (jCon->GetJointRefAngle(LEB)+OFFSET_ELB)*D2R;
    wbwalk.fkik.WBIK_Q[idLWY] = jCon->GetJointRefAngle(LWY)*D2R;
    wbwalk.fkik.WBIK_Q[idLWP] = jCon->GetJointRefAngle(LWP)*D2R;
    wbwalk.fkik.WBIK_Q[idLWY2] = jCon->GetJointRefAngle(LWY2)*D2R;

    wbwalk.fkik.Qub[idRSR] = wbwalk.fkik.WBIK_Q[idRSR];
    wbwalk.fkik.Qub[idRSP] = wbwalk.fkik.WBIK_Q[idRSP];
    wbwalk.fkik.Qub[idRSY] = wbwalk.fkik.WBIK_Q[idRSY];
    wbwalk.fkik.Qub[idREB] = wbwalk.fkik.WBIK_Q[idREB];
    wbwalk.fkik.Qub[idRWY] = wbwalk.fkik.WBIK_Q[idRWY];
    wbwalk.fkik.Qub[idRWP] = wbwalk.fkik.WBIK_Q[idRWP];
    wbwalk.fkik.Qub[idRWY] = wbwalk.fkik.WBIK_Q[idRWY];
    wbwalk.fkik.Qub[idRWY2] = wbwalk.fkik.WBIK_Q[idRWY2];

    wbwalk.fkik.Qub[idLSR] = wbwalk.fkik.WBIK_Q[idLSR];
    wbwalk.fkik.Qub[idLSP] = wbwalk.fkik.WBIK_Q[idLSP];
    wbwalk.fkik.Qub[idLSY] = wbwalk.fkik.WBIK_Q[idLSY];
    wbwalk.fkik.Qub[idLEB] = wbwalk.fkik.WBIK_Q[idLEB];
    wbwalk.fkik.Qub[idLWY] = wbwalk.fkik.WBIK_Q[idLWY];
    wbwalk.fkik.Qub[idLWP] = wbwalk.fkik.WBIK_Q[idLWP];
    wbwalk.fkik.Qub[idLWY] = wbwalk.fkik.WBIK_Q[idLWY];
    wbwalk.fkik.Qub[idLWY2] = wbwalk.fkik.WBIK_Q[idLWY2];

    wbwalk.fkik.Qub[idWST] = wbwalk.fkik.WBIK_Q[idWST];

}

double BasicTrajectory(double count, int mode)
{
    if(count > 1.) count = 1.;
    else if(count < 0.) count = 0.;

    if(mode == MODE_ONEtoZERO) return 1./2.*(cos(count*PI) + 1.);
    else return 1./2.*(1. - cos(count*PI));
}
double temp1[3];
void Global2Local(double _Global[],double _Local[])
{


    // Foot Center in Global Coord.
    pCenter[0] = (wbwalk.fkik.desired.pRF_3x1[0] + wbwalk.fkik.desired.pLF_3x1[0])/2.;
    pCenter[1] = (wbwalk.fkik.desired.pRF_3x1[1] + wbwalk.fkik.desired.pLF_3x1[1])/2.;
    pCenter[2] = (wbwalk.fkik.desired.pRF_3x1[2] + wbwalk.fkik.desired.pLF_3x1[2])/2.;

    qtRZ((window[0].right_foot_ref.yaw*D2R + window[0].left_foot_ref.yaw*D2R)/2.,qCenter);

    diff_vv(_Global,3,pCenter,temp1); // _Global - pCenter

    qCenter_bar[0] =  qCenter[0];
    qCenter_bar[1] = -qCenter[1];
    qCenter_bar[2] = -qCenter[2];
    qCenter_bar[3] = -qCenter[3];

    QTtransform(qCenter_bar, temp1, _Local);
}
void Global2Local2(double _Global[],double _Local[])
{
    double pCenter[3],qCenter[4],qCenter_bar[4];
    double temp1[3];

    // Foot Center in Global Coord.
    pCenter[0] = (wbwalk.fkik.desired.pRF_3x1[0] + wbwalk.fkik.desired.pLF_3x1[0])/2.;
    pCenter[1] = (wbwalk.fkik.desired.pRF_3x1[1] + wbwalk.fkik.desired.pLF_3x1[1])/2.;
    pCenter[2] = (wbwalk.fkik.desired.pRF_3x1[2] + wbwalk.fkik.desired.pLF_3x1[2])/2.;

qtRZ((window[0].right_foot_ref.yaw*D2R + window[0].left_foot_ref.yaw*D2R)/2.,qCenter);

    diff_vv(_Global,3,pCenter,temp1); // _Global - pCenter

    qCenter_bar[0] = qCenter[0];
    qCenter_bar[1] = -qCenter[1];
    qCenter_bar[2] = -qCenter[2];
    qCenter_bar[3] = -qCenter[3];

    QTtransform(qCenter_bar, temp1, _Local);
}
void Local2Global(double _Local[],double _Global[])
{
    double pCenter[3],qCenter[4];
    double temp1[3];

    // Foot Center in Global Coord.
    pCenter[0] = (wbwalk.fkik.desired.pRF_3x1[0] + wbwalk.fkik.desired.pLF_3x1[0])/2.;
    pCenter[1] = (wbwalk.fkik.desired.pRF_3x1[1] + wbwalk.fkik.desired.pLF_3x1[1])/2.;
    pCenter[2] = (wbwalk.fkik.desired.pRF_3x1[2] + wbwalk.fkik.desired.pLF_3x1[2])/2.;

    qtRZ((window[0].right_foot_ref.yaw*D2R + window[0].left_foot_ref.yaw*D2R)/2.,qCenter);

    QTtransform(qCenter, _Local, temp1);

    sum_vv(temp1,3,pCenter,_Global); // zmp - pCenter
}

/* QP Formulation */
void PreComputeQP()
{
    // MPC Precomputing for Fast calculation

      MPC_A[0][0] = 1.0;      MPC_A[0][1] = MPC_T;    MPC_A[0][2] = MPC_T*MPC_T/2.0;
      MPC_A[1][0] = 0.0;      MPC_A[1][1] = 1.0;      MPC_A[1][2] = MPC_T;
      MPC_A[2][0] = 0.0;      MPC_A[2][1] = 0.0;      MPC_A[2][2] = 1.0;

      MPC_B[0] = MPC_T*MPC_T*MPC_T/6.0;  MPC_B[1] = MPC_T*MPC_T/2.0;  MPC_B[2] = MPC_T;

      MPC_C[0] = 1.0;  MPC_C[1] = 0.;  MPC_C[2] = -MPC_h/MPC_g;

      // Precomputing
      for(int i=0;i<MPC_time;i++)
      {
          Pps[i][0] = 1.0;    Pps[i][1] = MPC_T*((double)(i+1));    Pps[i][2] = MPC_T*MPC_T*((double)(i+1))*((double)(i+1))/2.0;
          Pvs[i][0] = 0.0;    Pvs[i][1] = 1.0;    Pvs[i][2] = MPC_T*((double)(i+1));
          Pzs[i][0] = 1.0;    Pzs[i][1] = MPC_T*((double)(i+1));    Pzs[i][2] = MPC_T*MPC_T*((double)(i+1))*((double)(i+1))/2.0-MPC_h/MPC_g;

          for(int j = 0;j<i+1;j++)
          {
              Ppu[i][j] = (1.0 + 3.0*(((double)(i+1))-((double)(j+1))) + 3.0*(((double)(i+1))-((double)(j+1)))*(((double)(i+1))-((double)(j+1))))*MPC_T*MPC_T*MPC_T/6.0;


              Pvu[i][j] = (1.0 + 2.0*(((double)(i+1))-((double)(j+1))))*MPC_T*MPC_T/2.0;

              Pzu[i][j] = (1.0 + 3.0*(((double)(i+1))-((double)(j+1))) + 3.0*(((double)(i+1))-((double)(j+1)))*(((double)(i+1))-((double)(j+1))))*MPC_T*MPC_T*MPC_T/6.0 - MPC_T*MPC_h/MPC_g;

          }
      }

      printf("==================================================================\n");
      printf("--Pps : %f    %f  %f \n",Pps[0][0],Pps[0][1],Pps[0][2]);
      printf("--Pps : %f    %f  %f \n",Pps[1][0],Pps[1][1],Pps[1][2]);
      printf("--Pps : %f    %f  %f \n",Pps[14][0],Pps[14][1],Pps[14][2]);
      printf("==================================================================\n");

      printf("--Pvs : %f    %f  %f \n",Pvs[0][0],Pvs[0][1],Pvs[0][2]);
      printf("--Pvs : %f    %f  %f \n",Pvs[1][0],Pvs[1][1],Pvs[1][2]);
      printf("--Pvs : %f    %f  %f \n",Pvs[14][0],Pvs[14][1],Pvs[14][2]);
      printf("==================================================================\n");
      printf("--Pzs : %f    %f  %f \n",Pzs[0][0],Pzs[0][1],Pzs[0][2]);
      printf("--Pzs : %f    %f  %f \n",Pzs[1][0],Pzs[1][1],Pzs[1][2]);
      printf("--Pzs : %f    %f  %f \n",Pzs[14][0],Pzs[14][1],Pzs[14][2]);

      InvMatrix(MPC_time,(double*)Pzu,(double*)Pzu_inv);

      for(int i=0; i<MPC_time;i++){
          for(int j=0;j<MPC_time;j++){
              Pzu_inv_trans[j][i] = Pzu_inv[i][j];
              Pvu_trans[j][i] = Pvu[i][j];
          }
      }


      mat15by15x15by15(Pzu_inv_trans,Pzu_inv,Pzu_invxtrans);

      mat15by15x15by15(Pzu_inv_trans,Pvu_trans,temp_mat);
      mat15by15x15by15(temp_mat,Pvu,temp_mat2);
      mat15by15x15by15(temp_mat2,Pzu_inv,temp_mat);

      for(int i = 0;i<MPC_time;i++){
          for(int j = 0;j<MPC_time;j++){
              if(i == j){
                  temp_mat2[i][j] = MPC_beta;
              }
          }
      }

      for(int i=0;i<MPC_time;i++){
          for(int j=0;j<MPC_time;j++){
              MPC_Q[i][j] = MPC_gamma*Pzu_invxtrans[i][j] + MPC_alpha*temp_mat[i][j] + temp_mat2[i][j];
          }
      }


      for(int i=MPC_time;i<MPC_time + MPC_m;i++){
          for(int j=MPC_time;j<MPC_time+ MPC_m;j++){
              if(i == j){
                  MPC_Q[i][j] = MPC_mu*1.0;
              }
          }
      }



      // MPC_pk

      mat15by15x15by15(Pvu,Pzu_inv,temp_mat);
      mat15by15x15by3(temp_mat,Pzs,t_mat);
      mat15by3minus15by3(Pvs,t_mat,t_mat2);

      mat15by15x15by15(Pzu_inv_trans,Pvu_trans,temp_mat2);
      mat15by15x15by3(temp_mat2,t_mat2,t_mat3);



      mat15by15x15by15(Pzu_inv_trans,Pzu_inv,temp_mat);
      mat15by15x15by3(temp_mat,Pzs,t_mat);



      for(int i=0;i<MPC_time;i++)
      {
          for(int j=0;j<3;j++)
          {
              t_mat3[i][j] = t_mat3[i][j]*MPC_alpha;
              t_mat[i][j] = t_mat[i][j]*MPC_gamma;
          }
      }

      mat15by3minus15by3(t_mat3,t_mat,MPC_pc);


      printf("==================================================================\n");
      printf("--pc : %f       %f      %f \n",MPC_pc[0][0],MPC_pc[0][1],MPC_pc[0][2]);
      printf("--Pc : %f       %f      %f \n",MPC_pc[1][0],MPC_pc[1][1],MPC_pc[1][2]);
      printf("--Pc : %f       %f      %f \n",MPC_pc[14][0],MPC_pc[14][1],MPC_pc[14][2]);



      printf("==================================================================\n");
      printf("--MPC_Q : %f    %f  %f    %f    %f  %f    %f    %f  %f    %f    %f  %f \n",MPC_Q[0][0],MPC_Q[0][1],MPC_Q[0][2],MPC_Q[0][3],MPC_Q[0][4],MPC_Q[0][5],MPC_Q[0][6],MPC_Q[0][7],MPC_Q[0][8],MPC_Q[0][9],MPC_Q[0][10],MPC_Q[0][11]);
      printf("--MPC_Q : %f    %f  %f    %f    %f  %f    %f    %f  %f    %f    %f  %f \n",MPC_Q[1][0],MPC_Q[1][1],MPC_Q[1][2],MPC_Q[1][3],MPC_Q[1][4],MPC_Q[1][5],MPC_Q[1][6],MPC_Q[1][7],MPC_Q[1][8],MPC_Q[1][9],MPC_Q[1][10],MPC_Q[1][11]);
      printf("--MPC_Q : %f    %f  %f    %f    %f  %f    %f    %f  %f    %f    %f  %f \n",MPC_Q[14][0],MPC_Q[14][1],MPC_Q[14][2],MPC_Q[14][3],MPC_Q[14][4],MPC_Q[14][5],MPC_Q[14][6],MPC_Q[14][7],MPC_Q[14][8],MPC_Q[14][9],MPC_Q[14][10],MPC_Q[14][11]);


      //--Ci

      // MPC_C1
      for(int i = 0; i<MPC_time;i++){
          for(int j = 0; j<MPC_time;j++){
                  if(i == j){
                      MPC_Ci[i][j] = 1.;
                  }else{
                      MPC_Ci[i][j] = 0.;
                  }
          }
      }

      for(int i = MPC_time; i<MPC_time*2;i++){
          for(int j = 0; j<MPC_time;j++){
                  if(i-MPC_time == j){
                      MPC_Ci[i][j] = -1.;
                  }else{
                      MPC_Ci[i][j] = 0.;
                  }
          }
      }


      for(int oo=0;oo<30;oo++)
      {
          printf("%d--MPC_Ci : %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",oo,MPC_Ci[oo][0],MPC_Ci[oo][1],MPC_Ci[oo][2],MPC_Ci[oo][3],MPC_Ci[oo][4],MPC_Ci[oo][5],MPC_Ci[oo][6],MPC_Ci[oo][7],MPC_Ci[oo][8],MPC_Ci[oo][9],MPC_Ci[oo][10],MPC_Ci[oo][11],MPC_Ci[oo][12],MPC_Ci[oo][13],MPC_Ci[oo][14]);
      }

}
void save()
{
    if(Save_Index < ROW)
    {
            Save_Data[0][Save_Index] = window[0].zmp.x;
            Save_Data[1][Save_Index] = window[1].zmp.x;
            Save_Data[2][Save_Index] = window[299].zmp.x;
            Save_Data[3][Save_Index] = window[0].zmp.y;
            Save_Data[4][Save_Index] = window[1].zmp.y;
            Save_Data[5][Save_Index] = window[299].zmp.y;
            Save_Data[6][Save_Index] = window[0].state;
            Save_Data[7][Save_Index] = window[0].right_foot_ref.x;
            Save_Data[8][Save_Index] = window[0].right_foot_ref.y;
            Save_Data[9][Save_Index] = window[0].right_foot_ref.z;
            Save_Data[10][Save_Index] = window[0].left_foot_ref.x;
            Save_Data[11][Save_Index] = window[0].left_foot_ref.y;
            Save_Data[12][Save_Index] = window[0].left_foot_ref.z;

            Save_Data[13][Save_Index] = window[0].left_foot_ref.yaw;
            Save_Data[14][Save_Index] = window[0].right_foot_ref.yaw;

            Save_Data[15][Save_Index] = Pel_Yaw;

            Save_Data[16][Save_Index] = GLOBAL_X_LIPM;
            Save_Data[17][Save_Index] = GLOBAL_Y_LIPM;
            Save_Data[18][Save_Index] = wbwalk.fkik.desired.pCOM_3x1[0];
            Save_Data[19][Save_Index] = wbwalk.fkik.desired.pCOM_3x1[1];
            Save_Data[20][Save_Index] = ALPHA;
            Save_Data[21][Save_Index] = window[0].timer.current;
            Save_Data[22][Save_Index] = window[0].timer.total;

            Save_Data[23][Save_Index] = deflection_comp_LAR;
            Save_Data[24][Save_Index] = deflection_comp_RAR;

            Save_Data[25][Save_Index] = sharedData->FT[RAFT].Fz;
            Save_Data[26][Save_Index] = sharedData->FT[LAFT].Fz;

            Save_Data[27][Save_Index] = sharedData->FOG.Roll;
            Save_Data[28][Save_Index] = sharedData->FOG.RollVel;

            Save_Data[29][Save_Index] = sharedData->FOG.Pitch;
            Save_Data[30][Save_Index] = sharedData->FOG.PitchVel;

            Save_Data[31][Save_Index] = GLOBAL_Z_RF;
            Save_Data[32][Save_Index] = GLOBAL_Z_LF;

            Save_Data[32][Save_Index] = Del_PC_X_DSP_XZMP_CON;
            Save_Data[33][Save_Index] = Del_PC_Y_DSP_YZMP_CON;

            Save_Data[34][Save_Index] = I_ZMP_CON_X;
            Save_Data[35][Save_Index] = I_ZMP_CON_Y;

            Save_Data[36][Save_Index] = GLOBAL_Xori_RF;
            Save_Data[37][Save_Index] = GLOBAL_Xori_LF;

            Save_Data[38][Save_Index] = GLOBAL_Yori_RF;
            Save_Data[39][Save_Index] = GLOBAL_Yori_LF;

            Save_Data[40][Save_Index] = U_Gain;
            Save_Data[41][Save_Index] = U_Gain_DSP;

            Save_Data[42][Save_Index] = wbwalk.fkik.desired.pRF_3x1[2];
            Save_Data[43][Save_Index] = wbwalk.fkik.desired.pLF_3x1[2];

            Save_Data[44][Save_Index] = GLOBAL_Z_RF;
            Save_Data[45][Save_Index] = GLOBAL_Z_LF;//Zctrl2;

            Save_Data[46][Save_Index] = Zctrl;
            Save_Data[47][Save_Index] = Zctrl2;

            Save_Data[48][Save_Index] = EarlyLandingFlag[RIGHT];
            Save_Data[49][Save_Index] = EarlyLandingFlag[LEFT];

            Save_Data[50][Save_Index] = X_ZMP_REF_Global;
            Save_Data[51][Save_Index] = Y_ZMP_REF_Global;
            Save_Data[52][Save_Index] = X_ZMP_Global;
            Save_Data[53][Save_Index] = Y_ZMP_Global;

            Save_Data[54][Save_Index] = GLOBAL_Xori_RF_n;
            Save_Data[55][Save_Index] = GLOBAL_Xori_LF_n;

            Save_Data[56][Save_Index] = AnkleControl1;
            Save_Data[57][Save_Index] = AnkleControl2;

            Save_Data[58][Save_Index] = Add_FootTask[RIGHT][Zdir];
            Save_Data[59][Save_Index] = AnkleControl2;

            Save_Data[60][Save_Index] = X_ZMP_REF_Local;
            Save_Data[61][Save_Index] = Y_ZMP_REF_Local;

            Save_Data[62][Save_Index] = X_ZMP_Local;
            Save_Data[63][Save_Index] = Y_ZMP_Local;


            Save_Data[64][Save_Index] = Estimated_Orientation[0]*R2D;
            Save_Data[65][Save_Index] = Estimated_Orientation[1]*R2D;//Y_ZMP_Local;


            Save_Data[66][Save_Index] = sharedData->IMU[0].AccX;
            Save_Data[67][Save_Index] = sharedData->IMU[0].AccY;//Estmated_Orientation[1]*D2R;//Y_ZMP_Local;

            Save_Data[68][Save_Index] = HPF_Estimated_Orientation[0]*R2D;
            Save_Data[69][Save_Index] = HPF_Estimated_Orientation[1]*R2D;

            Save_Data[70][Save_Index] = Comp_Orientation[0];
            Save_Data[71][Save_Index] = Comp_Orientation[1];


            Save_Data[72][Save_Index] = RDPitch;
            Save_Data[73][Save_Index] = LDPitch;//Comp_Orientation[1];

            Save_Data[74][Save_Index] = RDRoll;
            Save_Data[75][Save_Index] = LDRoll;//Comp_Orientation[1];

            Save_Data[76][Save_Index] = Add_Leg_Recovery[RIGHT][Zdir];
            Save_Data[77][Save_Index] = Add_Leg_Recovery[LEFT][Zdir];//Comp_Orientation[1];

            Save_Data[78][Save_Index] = temp_debug[0];
            Save_Data[79][Save_Index] = temp_debug[1];


            Save_Data[80][Save_Index] = temp_debug[5];
            Save_Data[81][Save_Index] = temp_debug[6];


            Save_Data[82][Save_Index] = GLOBAL_X_LIPM_d;
            Save_Data[83][Save_Index] = GLOBAL_Y_LIPM_d;//temp_debug[6];


            Save_Data[84][Save_Index] = CONT_X;
            Save_Data[85][Save_Index] = CONT_Y;//GLOBAL_Y_LIPM_d;//temp_debug[6];


            Save_Data[86][Save_Index] = U[0];
            Save_Data[87][Save_Index] = U_Gain;//GLOBAL_Y_LIPM_d;//temp_debug[6];

            Save_Data[88][Save_Index] = target_foot[0].footprint.lfoot[0];
            Save_Data[89][Save_Index] = target_foot[0].footprint.lfoot[1];//U_Gain;//GLOBAL_Y_LIPM_d;//temp_debug[6];
            Save_Data[90][Save_Index] = target_foot[0].footprint.lori[0];//U[0];

            Save_Data[91][Save_Index] = target_foot[0].footprint.rfoot[0];//U_Gain;//GLOBAL_Y_LIPM_d;//temp_debug[6];
            Save_Data[92][Save_Index] = target_foot[0].footprint.rfoot[1];//U[0];
            Save_Data[93][Save_Index] = target_foot[0].footprint.rori[0];//U_Gain;//GLOBAL_Y_LIPM_d;//temp_debug[6];


            Save_Data[94][Save_Index] = sharedData->IMU[CIMU].Pitch;//U[0];
            Save_Data[95][Save_Index] = sharedData->IMU[CIMU].PitchVel;//U_Gain;//GLOBAL_Y_LIPM_d;//temp_debug[6];


            Save_Data[96][Save_Index] = sharedData->IMU[CIMU].Roll;//U[0];
            Save_Data[97][Save_Index] = sharedData->IMU[CIMU].RollVel;//U_Gain;//GLOBAL_Y_LIPM_d;//temp_debug[6];


            Save_Data[98][Save_Index] = wbwalk.fkik.desired.pRF_3x1[0];
            Save_Data[99][Save_Index] = wbwalk.fkik.desired.pRF_3x1[1];

            Save_Data[100][Save_Index] = wbwalk.fkik.desired.pLF_3x1[0];
            Save_Data[101][Save_Index] = wbwalk.fkik.desired.pLF_3x1[1];

            Save_Data[102][Save_Index] = wbwalk.fkik.desired.pCOM_3x1[0];
            Save_Data[103][Save_Index] = wbwalk.fkik.desired.pCOM_3x1[1];

            Save_Data[104][Save_Index] = wbwalk.fkik.WBIK_Q[idRHP];

            Save_Data[105][Save_Index] = X_ZMP_Local;
            Save_Data[106][Save_Index] = Y_ZMP_Local;
            Save_Data[107][Save_Index] = Del_PC_X_DSP_XZMP_CON;
            Save_Data[108][Save_Index] = Del_PC_Y_DSP_YZMP_CON;
            Save_Data[109][Save_Index] = (- 0.001*Del_PC_X_DSP_XZMP_CON + I_ZMP_CON_X*1.);
            Save_Data[110][Save_Index] = (- 0.001*Del_PC_Y_DSP_YZMP_CON + I_ZMP_CON_Y*0.);
            Save_Data[111][Save_Index] = Save_Index;

            Save_Data[112][Save_Index] = F_RF_Global[1];
            Save_Data[113][Save_Index] = F_LF_Global[1];
            Save_Data[114][Save_Index] = M_RF_Global[1];
            Save_Data[115][Save_Index] = M_LF_Global[1];
            Save_Data[116][Save_Index] = pCenter[0];
            Save_Data[117][Save_Index] = pCenter[1];


            Save_Data[118][Save_Index] = wbwalk.fkik.desired.pCOM_3x1[0];
            Save_Data[119][Save_Index] = wbwalk.fkik.desired.pCOM_3x1[1];

            Save_Data[120][Save_Index] = (- 0.001*Del_PC_X_DSP_XZMP_CON + I_ZMP_CON_X*1.);
            Save_Data[121][Save_Index] = (- 0.001*Del_PC_Y_DSP_YZMP_CON + I_ZMP_CON_Y*0.);

            Save_Data[122][Save_Index] = GLOBAL_X_LIPM_n;

            Save_Data[123][Save_Index] = G_DSP_X;

            Save_Data[124][Save_Index] = MODE_compliance;
            Save_Data[125][Save_Index] = Del_PC_X_DSP_XZMP_CON;
            Save_Data[126][Save_Index] = I_ZMP_CON_X;
            Save_Data[127][Save_Index] = GLOBAL_Y_LIPM_n;
            Save_Data[128][Save_Index] = U0_Gain;
            Save_Data[129][Save_Index] = Del_PC_Y_DSP_YZMP_CON;
            Save_Data[130][Save_Index] = G_DSP_Y;

            Save_Data[131][Save_Index] = sharedData->FT[0].Fz;
            Save_Data[132][Save_Index] = sharedData->FT[1].Fz;

            Save_Data[133][Save_Index] = sharedData->ENCODER[MC_ID_CH_Pairs[RHP].id][MC_ID_CH_Pairs[RHP].ch].CurrentReference;
            Save_Data[134][Save_Index] = sharedData->ENCODER[MC_ID_CH_Pairs[LHP].id][MC_ID_CH_Pairs[LHP].ch].CurrentReference;
            Save_Data[135][Save_Index] = sharedData->ENCODER[MC_ID_CH_Pairs[RAP].id][MC_ID_CH_Pairs[RAP].ch].CurrentReference;
            Save_Data[136][Save_Index] = sharedData->ENCODER[MC_ID_CH_Pairs[LAP].id][MC_ID_CH_Pairs[LAP].ch].CurrentReference;

            Save_Data[137][Save_Index] = sharedData->ENCODER[MC_ID_CH_Pairs[RHP].id][MC_ID_CH_Pairs[RHP].ch].CurrentPosition;
            Save_Data[138][Save_Index] = sharedData->ENCODER[MC_ID_CH_Pairs[LHP].id][MC_ID_CH_Pairs[LHP].ch].CurrentPosition;
            Save_Data[139][Save_Index] = sharedData->ENCODER[MC_ID_CH_Pairs[RAP].id][MC_ID_CH_Pairs[RAP].ch].CurrentPosition;
            Save_Data[140][Save_Index] = sharedData->ENCODER[MC_ID_CH_Pairs[LAP].id][MC_ID_CH_Pairs[LAP].ch].CurrentPosition;

            Save_Data[141][Save_Index] = ZMP_FeedBack_ONOFF;

            Save_Data[142][Save_Index] = GLOBAL_X_RF;
            Save_Data[143][Save_Index] = GLOBAL_Y_RF;

            Save_Data[144][Save_Index] = pCenter[0];
            Save_Data[145][Save_Index] = pCenter[1];
            Save_Data[146][Save_Index] = pCenter[2];

            Save_Data[147][Save_Index] = qCenter[0];
            Save_Data[148][Save_Index] = qCenter[1];
            Save_Data[149][Save_Index] = qCenter[2];
            Save_Data[150][Save_Index] = qCenter[3];

            Save_Data[151][Save_Index] = temp1[0];
            Save_Data[152][Save_Index] = temp1[1];
            Save_Data[153][Save_Index] = temp1[2];

            Save_Data[154][Save_Index] = wbwalk.fkik.FWRefAngleCurrent[RAP];
            Save_Data[155][Save_Index] = wbwalk.fkik.FWRefAngleCurrent[RAR];
            Save_Data[156][Save_Index] = wbwalk.fkik.FWRefAngleCurrent[LAP];
            Save_Data[157][Save_Index] = wbwalk.fkik.FWRefAngleCurrent[LAR];

            Save_Data[158][Save_Index] = wbwalk.fkik.WBIK_Q[RAP+7];
            Save_Data[159][Save_Index] = wbwalk.fkik.WBIK_Q[RAR+7];
            Save_Data[160][Save_Index] = wbwalk.fkik.WBIK_Q[LAP+7];
            Save_Data[161][Save_Index] = wbwalk.fkik.WBIK_Q[LAR+7];

            Save_Data[162][Save_Index] = GLOBAL_Xori_RF;
            Save_Data[163][Save_Index] = GLOBAL_Yori_RF;

            Save_Data[164][Save_Index] = deflection_comp_RAR;
            Save_Data[165][Save_Index] = RDRoll;
            Save_Data[166][Save_Index] = RDPitch;

            Save_Data[167][Save_Index] = GLOBAL_Xori_LF;
            Save_Data[168][Save_Index] = GLOBAL_Yori_LF;

            Save_Data[169][Save_Index] = deflection_comp_LAR;
            Save_Data[170][Save_Index] = LDRoll;
            Save_Data[171][Save_Index] = LDPitch;

            Save_Data[172][Save_Index] = wbwalk.fkik.desired_hat.pRF_3x1[Xdir];
            Save_Data[173][Save_Index] = wbwalk.fkik.desired_hat.pRF_3x1[Ydir];
            Save_Data[174][Save_Index] = wbwalk.fkik.desired_hat.pRF_3x1[Zdir];

            Save_Data[175][Save_Index] = wbwalk.fkik.desired_hat.pLF_3x1[Xdir];
            Save_Data[176][Save_Index] = wbwalk.fkik.desired_hat.pLF_3x1[Ydir];
            Save_Data[177][Save_Index] = wbwalk.fkik.desired_hat.pLF_3x1[Zdir];

            Save_Data[178][Save_Index] = wbwalk.fkik.desired.pCOM_3x1[2];
            Save_Data[179][Save_Index] = wbwalk.fkik.desired.pPCz;
            Save_Data[180][Save_Index] = wbwalk.fkik.desired_hat.pCOM_3x1[2];
            Save_Data[181][Save_Index] = Local[0];

            Save_Data[182][Save_Index] = wbwalk.fkik.FK.pCOM_3x1[0];
            Save_Data[183][Save_Index] = wbwalk.fkik.FK.pCOM_3x1[1];

            Save_Data[184][Save_Index] = wbwalk.fkik.desired.pRH_3x1[0];
            Save_Data[185][Save_Index] = wbwalk.fkik.desired.pRH_3x1[1];
            Save_Data[186][Save_Index] = wbwalk.fkik.desired.pRH_3x1[2];

            Save_Data[187][Save_Index] = wbwalk.fkik.FK.pRH_3x1[0];
            Save_Data[188][Save_Index] = wbwalk.fkik.FK.pRH_3x1[1];
            Save_Data[189][Save_Index] = wbwalk.fkik.FK.pRH_3x1[2];

            Save_Data[190][Save_Index] = wbwalk.pRH[0].reference;
            Save_Data[191][Save_Index] = wbwalk.pRH[1].reference;
            Save_Data[192][Save_Index] = wbwalk.pRH[2].reference;

            Save_Data[193][Save_Index] = wbwalk.pRH[0].current;
            Save_Data[194][Save_Index] = wbwalk.pRH[1].current;
            Save_Data[195][Save_Index] = wbwalk.pRH[2].current;

            Save_Data[196][Save_Index] = wbwalk.pRH[0].currenttime;
            Save_Data[197][Save_Index] = wbwalk.pRH[1].currenttime;
            Save_Data[198][Save_Index] = wbwalk.pRH[2].currenttime;

            Save_Data[199][Save_Index] = wbwalk.pRH[0].target;
            Save_Data[200][Save_Index] = wbwalk.pRH[1].target;
            Save_Data[201][Save_Index] = wbwalk.pRH[2].target;

            Save_Data[202][Save_Index] = wbwalk.fkik.desired.pLH_3x1[0];
            Save_Data[203][Save_Index] = wbwalk.fkik.desired.pLH_3x1[1];
            Save_Data[204][Save_Index] = wbwalk.fkik.desired.pLH_3x1[2];

            Save_Data[205][Save_Index] = wbwalk.fkik.FK.pLH_3x1[0];
            Save_Data[206][Save_Index] = wbwalk.fkik.FK.pLH_3x1[1];
            Save_Data[207][Save_Index] = wbwalk.fkik.FK.pLH_3x1[2];

            Save_Data[208][Save_Index] = wbwalk.pLH[0].reference;
            Save_Data[209][Save_Index] = wbwalk.pLH[1].reference;
            Save_Data[210][Save_Index] = wbwalk.pLH[2].reference;

            Save_Data[211][Save_Index] = wbwalk.pLH[0].current;
            Save_Data[212][Save_Index] = wbwalk.pLH[1].current;
            Save_Data[213][Save_Index] = wbwalk.pLH[2].current;

            Save_Data[214][Save_Index] = wbwalk.pLH[0].currenttime;
            Save_Data[215][Save_Index] = wbwalk.pLH[1].currenttime;
            Save_Data[216][Save_Index] = wbwalk.pLH[2].currenttime;

            Save_Data[217][Save_Index] = wbwalk.pLH[0].target;
            Save_Data[218][Save_Index] = wbwalk.pLH[1].target;
            Save_Data[219][Save_Index] = wbwalk.pLH[2].target;

            Save_Data[220][Save_Index] = wbwalk.fkik.WBIK_Q[idRSP];
            Save_Data[221][Save_Index] = wbwalk.fkik.WBIK_Q[idRSY];
            Save_Data[222][Save_Index] = wbwalk.fkik.WBIK_Q[idRSR];
            Save_Data[223][Save_Index] = wbwalk.fkik.WBIK_Q[idREB];
            Save_Data[224][Save_Index] = wbwalk.fkik.WBIK_Q[idRWY];
            Save_Data[225][Save_Index] = wbwalk.fkik.WBIK_Q[idRWP];
            Save_Data[226][Save_Index] =wbwalk.fkik.WBIK_Q[idRWY2];

            Save_Data[227][Save_Index] = sharedData->WBIK_Q[idRSP];
            Save_Data[228][Save_Index] = sharedData->WBIK_Q[idRSY];
            Save_Data[229][Save_Index] = sharedData->WBIK_Q[idRSR];
            Save_Data[230][Save_Index] = sharedData->WBIK_Q[idREB];
            Save_Data[231][Save_Index] = sharedData->WBIK_Q[idRWY];
            Save_Data[232][Save_Index] = sharedData->WBIK_Q[idRWP];
            Save_Data[233][Save_Index] =sharedData->WBIK_Q[idRWY2];



            Save_Data[234][Save_Index] = sharedData->FT[2].Fz;
            Save_Data[235][Save_Index] = LPF_RH_Fz;
            Save_Data[236][Save_Index] = PosX[0];
            Save_Data[237][Save_Index] = Measured_Force_Z[0];
            Save_Data[238][Save_Index] = Measured_Force_Z[0];


            Save_Data[239][Save_Index] = sharedData->ENCODER[MC_ID_CH_Pairs[RSR].id][MC_ID_CH_Pairs[RSR].ch].CurrentReference;
            Save_Data[240][Save_Index] = sharedData->ENCODER[MC_ID_CH_Pairs[RSP].id][MC_ID_CH_Pairs[RSP].ch].CurrentReference;
            Save_Data[241][Save_Index] = jCon->Joints[RSR]->RefAngleCurrent;
            Save_Data[242][Save_Index] = (wbwalk.fkik.WBIK_Q[idRSR])*R2D-OFFSET_RSR;
            Save_Data[243][Save_Index] = wbwalk.fkik.FK.REB_ang;
            Save_Data[244][Save_Index] = wbwalk.fkik.desired.REB_ang;


            Save_Index++;

            if(Save_Index >= ROW) Save_Index = 0;


    }
}

