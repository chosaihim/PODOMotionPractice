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
    sprintf(__AL_NAME, "ApproachBox");
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
        case APPROACHBOX_AL_TEST:
        {
            FILE_LOG(logSUCCESS) << "Command 999 received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();
            jCon->SetMoveJoint(WST, 30.0, 2000.0, MOVE_ABSOLUTE);
            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case APPROACHBOX_WALK_START:
        {
            if(continouse_walking_flag == true && walk_flag == true)
            {
                FILE_LOG(logSUCCESS) << "Command LIFT_WALK_START..";

                STEP_LENGTH = userData->G2M.StepLength;
                STEP_ANGLE  = userData->G2M.StepAngle;
                STEP_OFFSET  = userData->G2M.StepOffset;
                STEP_STOP = sharedData->COMMAND[PODO_NO].USER_PARA_INT[0];

                walk_start_flag = true;
                continouse_walking_flag = false;
            }
            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case APPROACHBOX_WALK_READY:
        {
            if(walk_flag == 0){
                FILE_LOG(logSUCCESS) << "Command APPROACHBOX_WALK_READY..";

                Upperbody_Gain_Override();

                sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0]=1;//fog zero
                sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_SENSOR_FOG_NULL;

                pv_Index = 0;
                Walking_initialize();

                /* Foot Print Generation Test
                 * (dir, step_num, step_length, step_angle, step_offset, LPEL2PEL) */
                FPG_TEST(APPROACHBOX_FORWARD_WALKING,10,0.001,0.,0.21,kine_drc_hubo4.L_PEL2PEL);
                usleep(200*1000);

                walk_flag = 1;

                continouse_walking_flag = true;

                jCon->RefreshToCurrentReference();
                jCon->SetAllMotionOwner();

                printf("while right des x: %f  y:%f   z:%f  \n",des_pRF_3x1[0],des_pRF_3x1[1],des_pRF_3x1[2]);
                printf("while left  des x: %f  y:%f   z:%f  \n",des_pLF_3x1[0],des_pLF_3x1[1],des_pLF_3x1[2]);
                printf("while COM x: %f  y: %f   z:%f \n",des_pCOM_3x1[0],des_pCOM_3x1[1],des_pCOM_3x1[2]);
                printf("while Right WBIK_Q : %f  %f  %f  %f  %f  %f \n",WBIK_Q[7],WBIK_Q[8],WBIK_Q[9],WBIK_Q[10],WBIK_Q[11],WBIK_Q[12]);
                printf("while Left  WBIK_Q : %f  %f  %f  %f  %f  %f \n",WBIK_Q[13],WBIK_Q[14],WBIK_Q[15],WBIK_Q[16],WBIK_Q[17],WBIK_Q[18]);
                printf("while right global x: %f  y:%f   z:%f  \n",GLOBAL_X_RF,GLOBAL_Y_RF,GLOBAL_Z_RF);
                printf("while left  global x: %f  y:%f   z:%f  \n",GLOBAL_X_LF,GLOBAL_Y_LF,GLOBAL_Z_LF);

            }
            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case APPROACHBOX_FORWARD_WALKING:
        {
            if(walk_flag == 0){
                FILE_LOG(logSUCCESS) << "Command Forward Walking received..";

                Upperbody_Gain_Override();
                sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0]=1;//fog zero
                sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_SENSOR_FOG_NULL;
                pv_Index = 0;

                /* walking initialize */
                Walking_initialize();

                /* Foot Print Generation Test */
                FPG_TEST(APPROACHBOX_FORWARD_WALKING,userData->G2M.StepNum,userData->G2M.StepLength,userData->G2M.StepAngle,userData->G2M.StepOffset,kine_drc_hubo4.L_PEL2PEL);

                usleep(200*1000);
                walk_flag = 1;

                sharedData->LaserLength=3333;

                continouse_walking_flag = false;
                jCon->RefreshToCurrentReference();
                jCon->SetAllMotionOwner();
            }
            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case APPROACHBOX_RIGHT_WALKING:
        {
            FILE_LOG(logSUCCESS) << "Command RIGHT Walking received..";
            Upperbody_Gain_Override();
            pv_Index = 0;

            // walking initialize
            Walking_initialize();

            // Foot Print Generation Test
            FPG_TEST(APPROACHBOX_RIGHT_WALKING,userData->G2M.StepNum,userData->G2M.StepLength,userData->G2M.StepAngle,userData->G2M.StepOffset,kine_drc_hubo4.L_PEL2PEL);

            usleep(200*1000);
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();
            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;

            walk_flag = 1;
            continouse_walking_flag = 0;
            break;
        }
        case APPROACHBOX_LEFT_WALKING:
        {
            FILE_LOG(logSUCCESS) << "Command LEFT Walking received..";
            Upperbody_Gain_Override();

            pv_Index = 0;

            // walking initialize
            Walking_initialize();

            // Foot Print Generation Test
            FPG_TEST(APPROACHBOX_LEFT_WALKING,userData->G2M.StepNum,userData->G2M.StepLength,userData->G2M.StepAngle,userData->G2M.StepOffset,kine_drc_hubo4.L_PEL2PEL);

            usleep(200*1000);

            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();
            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;

            walk_flag = 1;
            continouse_walking_flag = 0;
            break;
        }
        case APPROACHBOX_CW_WALKING:
        {
            FILE_LOG(logSUCCESS) << "Command CW Walking received..";
            Upperbody_Gain_Override();
            pv_Index = 0;

            // walking initialize
            Walking_initialize();

            // Foot Print Generation Test
            FPG_TEST(APPROACHBOX_CW_WALKING,userData->G2M.StepNum,userData->G2M.StepLength,userData->G2M.StepAngle,userData->G2M.StepOffset,kine_drc_hubo4.L_PEL2PEL);

            usleep(200*1000);
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();
            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;

            walk_flag = 1;
            continouse_walking_flag = 0;
            break;
        }
        case APPROACHBOX_CCW_WALKING:
        {
            FILE_LOG(logSUCCESS) << "Command CCW Walking received..";
            Upperbody_Gain_Override();

            pv_Index = 0;
            // walking initialize
            Walking_initialize();

            // Foot Print Generation Test
            FPG_TEST(APPROACHBOX_CCW_WALKING,userData->G2M.StepNum,userData->G2M.StepLength,userData->G2M.StepAngle,userData->G2M.StepOffset,kine_drc_hubo4.L_PEL2PEL);

            usleep(200*1000);
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();
            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;

            walk_flag = 1;
            continouse_walking_flag = 0;
            break;
        }
        case APPROACHBOX_WALK_STOP:
        {
            if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 0)//MODE_NORMAL
            {
                FILE_LOG(logSUCCESS) << "Command APPROACHBOX_WALK_STOP..(MODE_NORMAL)";
                stop_flag = true;
            } else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 1)//MODE_LIFTBOX
            {
                FILE_LOG(logSUCCESS) << "Command APPROACHBOX_WALK_STOP..(MODE_LIFTBOX)";
                walkstop_liftbox_flag = true;
            } else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == 2)//MODE_DOOR
            {
                FILE_LOG(logSUCCESS) << "Command APPROACHBOX_WALK_STOP..(MODE_DOOR)";
                walkstop_door_flag = true;
            }
            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case APPROACHBOX_DATA_SAVE:
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
            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case APPROACHBOX_COMPLIANCE_START:
        {
           FILE_LOG(logSUCCESS) << "Arm Compliance control start..\n";
           ShutDownAllFlag();
           OnOff_compliance = true;
           walk_flag = true;
           jCon->RefreshToCurrentReference();
           jCon->SetAllMotionOwner();

           sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
           break;
        }
        case APPROACHBOX_COMPLIANCE_STOP:
        {
           FILE_LOG(logSUCCESS) << "Arm Compliance control stop..\n";
           OnOff_compliance = false;
           ShutDownAllFlag();
           sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
           break;
        }
        case APPROACHBOX_PUSH_DOOR:
        {
           FILE_LOG(logSUCCESS) << "Push Door start..\n";
           FLAG_pushdoor = true;
           sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
           break;
        }
        case SINGLELOG_WALK:
        {
            if(walk_flag == 0){
                FILE_LOG(logSUCCESS) << "Single Log Walking Start!!";

                Upperbody_Gain_Override();
                sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0]=1;//fog zero
                sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_SENSOR_FOG_NULL;
                pv_Index = 0;

                /* walking initialize */
                Walking_initialize();

                /* Foot Print Generation Test */
                //FPG_TEST(APPROACHBOX_FORWARD_WALKING,userData->G2M.StepNum,userData->G2M.StepLength,0.0,0.25,kine_drc_hubo4.L_PEL2PEL);
                FPG_SINGLELOG(userData->G2M.StepNum,userData->G2M.StepLength,kine_drc_hubo4.L_PEL2PEL);
                usleep(200*1000);
                walk_flag = 1;
                FLAG_SingleLog = true;

                jCon->RefreshToCurrentReference();
                jCon->SetAllMotionOwner();
            }
            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case APPROACHBOX_REALWALK:
        {
            if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == REALWALK_WALK_START)
            {
                if(walk_flag == 0 && sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 0)
                {
                    FILE_LOG(logSUCCESS) << "RealWalk Ready..";
                    Upperbody_Gain_Override();
                    sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0]=1;//fog zero
                    sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_SENSOR_FOG_NULL;
                    pv_Index = 0;

                    /* walking initialize */
                    Walking_initialize();

                    FPG_TEST(APPROACHBOX_FORWARD_WALKING,5,0.001,0.,0.21,kine_drc_hubo4.L_PEL2PEL);

                    walk_flag = true;
                    continouse_walking_flag = true;

                    jCon->RefreshToCurrentReference();
                    jCon->SetAllMotionOwner();
                }
                else if(sharedData->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 1)
                {
                    FILE_LOG(logSUCCESS) << "RealWalk Start..!!";
                    RSTEP_LENGTH = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                    continouse_walking_flag = false;
                    Command_RealWalk = REALWALK_WALK_START;
                }
            }else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == REALWALK_SINGLELOG_START)
            {
                FILE_LOG(logSUCCESS) << "SingleLog Start..!!";
                RSTEP_NUM = sharedData->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
                Command_RealWalk = REALWALK_SINGLELOG_START;
                //FLAG_SingleLog = false;
            }else if(sharedData->COMMAND[PODO_NO].USER_PARA_INT[0] == REALWALK_STOP)
            {
                Command_RealWalk = REALWALK_STOP;
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

/****************************** 2. TaskThread *******************************/
void RBTaskThread(void *)
{
    while(__IS_WORKING)
    {
        if(walk_flag == true)
        {

            if(Command_RealWalk != REALWALK_NO_ACT)
            {
                double StepLen_Normal = RSTEP_LENGTH;
                double StepLen_SingleLog = 0.35;
                double StepNum_SingleLog = RSTEP_NUM;
                double StepWidth_SingleLog = 0.05;
                RSTEP_OFFSET = 0.21 + 0.05*fabs(RSTEP_ANGLE/10.0);

                //

                int right_left = 0;
                int moving_leg;
                double rl = 0.;

                if(last_moving_leg == MOVING_RIGHT)
                {
                    moving_leg = MOVING_LEFT;
                    right_left  = 1;
                } else
                {
                    right_left  = 0;
                    moving_leg = MOVING_RIGHT;
                }

                _footprint_info dummyfoot;
                while(pull_short_foot(dummyfoot)){
                    ;
                }


                switch(Command_RealWalk)
                {
                case REALWALK_WALK_START:
                {
                    _footprint_info newfoot;
                    for(int i=0; i<5; i++)
                    {

                        /* Change direction */
                        if(right_left == 1){
                            rl = 1;
                        }else{
                            rl = -1;
                        }

                        if(FLAG_SingleLog==true && FLAG_BacktoNormalWalk != -1)
                        {
                            /* Make Newfoot print adding STEP_LENGTH, STEP_ANGLE*/
                            if(moving_leg == MOVING_RIGHT){
                                newfoot.footprint.rori[0] = last_short_foot.footprint.lori[0] + RSTEP_ANGLE;
                                newfoot.footprint.rori[1] = last_short_foot.footprint.rori[1];
                                newfoot.footprint.rori[2] = last_short_foot.footprint.rori[2];

                                newfoot.footprint.lori[0] = last_short_foot.footprint.lori[0];
                                newfoot.footprint.lori[1] = last_short_foot.footprint.lori[1];
                                newfoot.footprint.lori[2] = last_short_foot.footprint.lori[2];

                                newfoot.footprint.rfoot[0] = last_short_foot.footprint.lfoot[0] + StepLen_SingleLog*cos(newfoot.footprint.rori[0]*D2R) - RSTEP_OFFSET*sin(newfoot.footprint.rori[0]*D2R)*rl;
                                if(FLAG_BacktoNormalWalk == 5)
                                    newfoot.footprint.rfoot[0] = last_short_foot.footprint.lfoot[0] + RSTEP_LENGTH*cos(newfoot.footprint.rori[0]*D2R) - RSTEP_OFFSET*sin(newfoot.footprint.rori[0]*D2R)*rl;

                                newfoot.footprint.rfoot[1] = -0.105;//last_short_foot.footprint.lfoot[1] + RSTEP_LENGTH*sin(newfoot.footprint.rori[0]*D2R) + RSTEP_OFFSET*cos(newfoot.footprint.rori[0]*D2R)*rl;
                                newfoot.footprint.rfoot[2] = last_short_foot.footprint.rfoot[2];

                                newfoot.footprint.lfoot[0] = last_short_foot.footprint.lfoot[0];
                                newfoot.footprint.lfoot[1] = last_short_foot.footprint.lfoot[1];
                                newfoot.footprint.lfoot[2] = last_short_foot.footprint.lfoot[2];

                                moving_leg = MOVING_LEFT;
                                //FLAG_BacktoNormalWalk++;
                            }else
                            {
                                newfoot.footprint.rori[0] = last_short_foot.footprint.rori[0];
                                newfoot.footprint.rori[1] = last_short_foot.footprint.rori[1];
                                newfoot.footprint.rori[2] = last_short_foot.footprint.rori[2];

                                newfoot.footprint.lori[0] = last_short_foot.footprint.rori[0] + RSTEP_ANGLE;
                                newfoot.footprint.lori[1] = last_short_foot.footprint.lori[1];
                                newfoot.footprint.lori[2] = last_short_foot.footprint.lori[2];

                                newfoot.footprint.rfoot[0] = last_short_foot.footprint.rfoot[0];
                                newfoot.footprint.rfoot[1] = last_short_foot.footprint.rfoot[1];
                                newfoot.footprint.rfoot[2] = last_short_foot.footprint.rfoot[2];

                                newfoot.footprint.lfoot[0] = last_short_foot.footprint.rfoot[0] + StepLen_SingleLog*cos(newfoot.footprint.lori[0]*D2R) - RSTEP_OFFSET*sin(newfoot.footprint.lori[0]*D2R)*rl;
                                if(FLAG_BacktoNormalWalk == 5)
                                    newfoot.footprint.lfoot[0] = last_short_foot.footprint.rfoot[0] + RSTEP_LENGTH*cos(newfoot.footprint.lori[0]*D2R) - RSTEP_OFFSET*sin(newfoot.footprint.lori[0]*D2R)*rl;

                                newfoot.footprint.lfoot[1] = 0.105;//last_short_foot.footprint.rfoot[1] + RSTEP_LENGTH*sin(newfoot.footprint.lori[0]*D2R) + RSTEP_OFFSET*cos(newfoot.footprint.lori[0]*D2R)*rl;
                                newfoot.footprint.lfoot[2] = last_short_foot.footprint.lfoot[2];

                                moving_leg = MOVING_RIGHT;
                                //FLAG_BacktoNormalWalk++;
                            }
                            newfoot.mode = WALKING_CHANGE;
                            newfoot.time.dsp_time = 1.0;
                            newfoot.time.ssp_time = 1.0;
                            if(FLAG_BacktoNormalWalk == 2)
                            {
                                //newfoot.mode = WALKING_NORMAL;
                                FLAG_SingleLog = false;
                                FLAG_BacktoNormalWalk = -1;
                            }
                        } else
                        {
                            /* Make Newfoot print adding STEP_LENGTH, STEP_ANGLE*/
                            if(moving_leg == MOVING_RIGHT){
                                newfoot.footprint.rori[0] = last_short_foot.footprint.lori[0] + RSTEP_ANGLE;
                                newfoot.footprint.rori[1] = last_short_foot.footprint.rori[1];
                                newfoot.footprint.rori[2] = last_short_foot.footprint.rori[2];

                                newfoot.footprint.lori[0] = last_short_foot.footprint.lori[0];
                                newfoot.footprint.lori[1] = last_short_foot.footprint.lori[1];
                                newfoot.footprint.lori[2] = last_short_foot.footprint.lori[2];

                                newfoot.footprint.rfoot[0] = last_short_foot.footprint.lfoot[0] + RSTEP_LENGTH*cos(newfoot.footprint.rori[0]*D2R) - RSTEP_OFFSET*sin(newfoot.footprint.rori[0]*D2R)*rl;
                                newfoot.footprint.rfoot[1] = last_short_foot.footprint.lfoot[1] + RSTEP_LENGTH*sin(newfoot.footprint.rori[0]*D2R) + RSTEP_OFFSET*cos(newfoot.footprint.rori[0]*D2R)*rl;
                                newfoot.footprint.rfoot[2] = last_short_foot.footprint.rfoot[2];

                                newfoot.footprint.lfoot[0] = last_short_foot.footprint.lfoot[0];
                                newfoot.footprint.lfoot[1] = last_short_foot.footprint.lfoot[1];
                                newfoot.footprint.lfoot[2] = last_short_foot.footprint.lfoot[2];

                                moving_leg = MOVING_LEFT;
                            }else
                            {
                                newfoot.footprint.rori[0] = last_short_foot.footprint.rori[0];
                                newfoot.footprint.rori[1] = last_short_foot.footprint.rori[1];
                                newfoot.footprint.rori[2] = last_short_foot.footprint.rori[2];

                                newfoot.footprint.lori[0] = last_short_foot.footprint.rori[0] + RSTEP_ANGLE;
                                newfoot.footprint.lori[1] = last_short_foot.footprint.lori[1];
                                newfoot.footprint.lori[2] = last_short_foot.footprint.lori[2];

                                newfoot.footprint.rfoot[0] = last_short_foot.footprint.rfoot[0];
                                newfoot.footprint.rfoot[1] = last_short_foot.footprint.rfoot[1];
                                newfoot.footprint.rfoot[2] = last_short_foot.footprint.rfoot[2];

                                newfoot.footprint.lfoot[0] = last_short_foot.footprint.rfoot[0] + RSTEP_LENGTH*cos(newfoot.footprint.lori[0]*D2R) - RSTEP_OFFSET*sin(newfoot.footprint.lori[0]*D2R)*rl;
                                newfoot.footprint.lfoot[1] = last_short_foot.footprint.rfoot[1] + RSTEP_LENGTH*sin(newfoot.footprint.lori[0]*D2R) + RSTEP_OFFSET*cos(newfoot.footprint.lori[0]*D2R)*rl;
                                newfoot.footprint.lfoot[2] = last_short_foot.footprint.lfoot[2];

                                moving_leg = MOVING_RIGHT;
                            }
                            newfoot.mode = WALKING_NORMAL;
                            newfoot.time.dsp_time = 0.1;
                            newfoot.time.ssp_time = 0.8;
                        }
                        push_short_foot(newfoot);
                        right_left ^= 1;
                    }
                    break;
                }

                case REALWALK_SINGLELOG_START:
                {
                    FLAG_SingleLog = true;
                    _footprint_info newfoot;
                    for(int i=0; i<RSTEP_NUM; i++)
                    {//WHy 5???

                        /* Change direction */
                        if(right_left == 1){
                            rl = 1;
                        }else{
                            rl = -1;
                        }

                        /* Make Newfoot print adding STEP_LENGTH, STEP_ANGLE*/
                        if(moving_leg == MOVING_RIGHT){
                            newfoot.footprint.rori[0] = last_short_foot.footprint.lori[0] + RSTEP_ANGLE;
                            newfoot.footprint.rori[1] = last_short_foot.footprint.rori[1];
                            newfoot.footprint.rori[2] = last_short_foot.footprint.rori[2];

                            newfoot.footprint.lori[0] = last_short_foot.footprint.lori[0];
                            newfoot.footprint.lori[1] = last_short_foot.footprint.lori[1];
                            newfoot.footprint.lori[2] = last_short_foot.footprint.lori[2];

                            newfoot.footprint.rfoot[0] = last_short_foot.footprint.lfoot[0] + StepLen_SingleLog;//*cos(newfoot.footprint.rori[0]*D2R) - RSTEP_OFFSET*sin(newfoot.footprint.rori[0]*D2R)*rl;
                            newfoot.footprint.rfoot[1] = -StepWidth_SingleLog/2.;//*sin(newfoot.footprint.rori[0]*D2R)/2. + RSTEP_OFFSET*cos(newfoot.footprint.rori[0]*D2R)*rl;
                            newfoot.footprint.rfoot[2] = last_short_foot.footprint.rfoot[2];

                            newfoot.footprint.lfoot[0] = last_short_foot.footprint.lfoot[0];
                            newfoot.footprint.lfoot[1] = last_short_foot.footprint.lfoot[1];
                            newfoot.footprint.lfoot[2] = last_short_foot.footprint.lfoot[2];

                            moving_leg = MOVING_LEFT;
                        }else
                        {
                            newfoot.footprint.rori[0] = last_short_foot.footprint.rori[0];
                            newfoot.footprint.rori[1] = last_short_foot.footprint.rori[1];
                            newfoot.footprint.rori[2] = last_short_foot.footprint.rori[2];

                            newfoot.footprint.lori[0] = last_short_foot.footprint.rori[0] + RSTEP_ANGLE;
                            newfoot.footprint.lori[1] = last_short_foot.footprint.lori[1];
                            newfoot.footprint.lori[2] = last_short_foot.footprint.lori[2];

                            newfoot.footprint.rfoot[0] = last_short_foot.footprint.rfoot[0];
                            newfoot.footprint.rfoot[1] = last_short_foot.footprint.rfoot[1];
                            newfoot.footprint.rfoot[2] = last_short_foot.footprint.rfoot[2];

                            newfoot.footprint.lfoot[0] = last_short_foot.footprint.rfoot[0] + StepLen_SingleLog;//*cos(newfoot.footprint.lori[0]*D2R) - RSTEP_OFFSET*sin(newfoot.footprint.lori[0]*D2R)*rl;
                            newfoot.footprint.lfoot[1] = StepWidth_SingleLog/2.;//*sin(newfoot.footprint.lori[0]*D2R) + RSTEP_OFFSET*cos(newfoot.footprint.lori[0]*D2R)*rl;
                            newfoot.footprint.lfoot[2] = last_short_foot.footprint.lfoot[2];

                            moving_leg = MOVING_RIGHT;
                        }

                        newfoot.time.dsp_time = 1.0;
                        newfoot.time.ssp_time = 1.0;

                        newfoot.mode = WALKING_SINGLELOG;
                        push_short_foot(newfoot);
                        right_left ^= 1;
                    }
                    //Command_RealWalk = REALWALK_WALK_START;
                    break;
                }
                case REALWALK_SINGLELOG:
                {
                    break;
                }
                case REALWALK_STOP:
                {
                    FILE_LOG(logERROR) << "STOP CONTINUOUS WALKING!!";
                    walk_start_flag = false;
                    while(pull_short_foot(dummyfoot))
                    {;}

                    make_last_footprint();
//                    _footprint_info newfoot;

//                    /* Make Newfoot print adding STEP_LENGTH, STEP_ANGLE*/
/*//                    if(moving_leg == MOVING_RIGHT){
//                        newfoot.footprint.rori[0] = last_short_foot.footprint.lori[0];
//                        newfoot.footprint.rori[1] = last_short_foot.footprint.rori[1];
//                        newfoot.footprint.rori[2] = last_short_foot.footprint.rori[2];

//                        newfoot.footprint.lori[0] = last_short_foot.footprint.lori[0];
//                        newfoot.footprint.lori[1] = last_short_foot.footprint.lori[1];
//                        newfoot.footprint.lori[2] = last_short_foot.footprint.lori[2];

//                        newfoot.footprint.rfoot[0] = last_short_foot.footprint.lfoot[0];//*cos(newfoot.footprint.rori[0]*D2R) - RSTEP_OFFSET*sin(newfoot.footprint.rori[0]*D2R)*rl;
//                        newfoot.footprint.rfoot[1] = last_short_foot.footprint.lfoot[1] - RSTEP_OFFSET*cos(newfoot.footprint.rori[0]*D2R);
//                        newfoot.footprint.rfoot[2] = last_short_foot.footprint.rfoot[2];

//                        newfoot.footprint.lfoot[0] = last_short_foot.footprint.lfoot[0];
//                        newfoot.footprint.lfoot[1] = last_short_foot.footprint.lfoot[1];
//                        newfoot.footprint.lfoot[2] = last_short_foot.footprint.lfoot[2];

//                        moving_leg = MOVING_LEFT;
//                    }else
//                    {
//                        newfoot.footprint.rori[0] = last_short_foot.footprint.rori[0];
//                        newfoot.footprint.rori[1] = last_short_foot.footprint.rori[1];
//                        newfoot.footprint.rori[2] = last_short_foot.footprint.rori[2];

//                        newfoot.footprint.lori[0] = last_short_foot.footprint.rori[0];
//                        newfoot.footprint.lori[1] = last_short_foot.footprint.lori[1];
//                        newfoot.footprint.lori[2] = last_short_foot.footprint.lori[2];

//                        newfoot.footprint.rfoot[0] = last_short_foot.footprint.rfoot[0];
//                        newfoot.footprint.rfoot[1] = last_short_foot.footprint.rfoot[1];
//                        newfoot.footprint.rfoot[2] = last_short_foot.footprint.rfoot[2];

//                        newfoot.footprint.lfoot[0] = last_short_foot.footprint.rfoot[0];//*cos(newfoot.footprint.lori[0]*D2R) - RSTEP_OFFSET*sin(newfoot.footprint.lori[0]*D2R)*rl;
//                        newfoot.footprint.lfoot[1] = last_short_foot.footprint.rfoot[1] + RSTEP_OFFSET*cos(newfoot.footprint.lori[0]*D2R);
//                        newfoot.footprint.lfoot[2] = last_short_foot.footprint.lfoot[2];

//                        moving_leg = MOVING_RIGHT;
//                    }

//                    newfoot.time.dsp_time = 1.0;
//                    newfoot.time.ssp_time = 1.0;

//                    newfoot.mode = WALKING_NORMAL;
//                    push_short_foot(newfoot);
//                    right_left ^= 1;

//                    if(moving_leg == MOVING_RIGHT){
//                        newfoot.footprint.rori[0] = last_short_foot.footprint.rori[0];
//                        newfoot.footprint.rori[1] = last_short_foot.footprint.rori[1];
//                        newfoot.footprint.rori[2] = last_short_foot.footprint.rori[2];

//                        newfoot.footprint.lori[0] = last_short_foot.footprint.lori[0];
//                        newfoot.footprint.lori[1] = last_short_foot.footprint.lori[1];
//                        newfoot.footprint.lori[2] = last_short_foot.footprint.lori[2];

//                        newfoot.footprint.rfoot[0] = last_short_foot.footprint.rfoot[0];//*cos(newfoot.footprint.rori[0]*D2R) - RSTEP_OFFSET*sin(newfoot.footprint.rori[0]*D2R)*rl;
//                        newfoot.footprint.rfoot[1] = last_short_foot.footprint.rfoot[1];
//                        newfoot.footprint.rfoot[2] = last_short_foot.footprint.rfoot[2];

//                        newfoot.footprint.lfoot[0] = last_short_foot.footprint.lfoot[0];
//                        newfoot.footprint.lfoot[1] = last_short_foot.footprint.lfoot[1];
//                        newfoot.footprint.lfoot[2] = last_short_foot.footprint.lfoot[2];

//                        moving_leg = MOVING_LEFT;
//                    }else
//                    {
//                        newfoot.footprint.rori[0] = last_short_foot.footprint.rori[0];
//                        newfoot.footprint.rori[1] = last_short_foot.footprint.rori[1];
//                        newfoot.footprint.rori[2] = last_short_foot.footprint.rori[2];

//                        newfoot.footprint.lori[0] = last_short_foot.footprint.lori[0];
//                        newfoot.footprint.lori[1] = last_short_foot.footprint.lori[1];
//                        newfoot.footprint.lori[2] = last_short_foot.footprint.lori[2];

//                        newfoot.footprint.rfoot[0] = last_short_foot.footprint.rfoot[0];
//                        newfoot.footprint.rfoot[1] = last_short_foot.footprint.rfoot[1];
//                        newfoot.footprint.rfoot[2] = last_short_foot.footprint.rfoot[2];

//                        newfoot.footprint.lfoot[0] = last_short_foot.footprint.lfoot[0];//*cos(newfoot.footprint.lori[0]*D2R) - RSTEP_OFFSET*sin(newfoot.footprint.lori[0]*D2R)*rl;
//                        newfoot.footprint.lfoot[1] = last_short_foot.footprint.lfoot[1];
//                        newfoot.footprint.lfoot[2] = last_short_foot.footprint.lfoot[2];

//                        moving_leg = MOVING_RIGHT;
//                    }

//                    newfoot.time.dsp_time = 1.0;
//                    newfoot.time.ssp_time = 1.0;

//                    newfoot.mode = WALKING_NORMAL;
//                    push_short_foot(newfoot);

                    //push_short_foot(last_short_foot);
*/                    stop_flag = false;
                    Command_RealWalk = REALWALK_NO_ACT;
                    break;
                }
                }

            }

            if(walk_start_flag == true)
            {
                float velX = 0.001;
                float velTh = 0.0;
                /* set velX,velTh according to pos x,y value obtained by vision
                 * (yame v1) */
                //printf("posx = %f, posy = %f",sharedData->V2M.pos[0],sharedData->V2M.pos[1]);
                if(sharedData->V2M.pos[0] > 1.)
                {
                    //printf("Go");
                    velX = 0.07;
                }
                else
                {
                    //printf("Stop");
                    velX = 0.001;
                }

                if(sharedData->V2M.pos[1] > 1.)// Box is on the right side of the robot.
                {
                    //printf("right\n");
                    velTh = -5.0;
                }
                else if(sharedData->V2M.pos[1] < -1.)// left side
                {
                    //printf("left\n");
                    velTh = +5.0;
                }
                else// front
                {
                    //printf("front\n");
                    velTh = 0.0;
                }

                STEP_LENGTH = velX;
                STEP_ANGLE = velTh;
                STEP_OFFSET = 0.21 + 0.05*fabs(velTh/10.0);

                /*
                 * Empty short_foot[]
                 * All of the footsteps that were in the queue are disappear,
                 * But the 3 steps of target_foot[] will remain.
                 */
                _footprint_info dummyfoot;
                while(pull_short_foot(dummyfoot)){
                    ;
                }

                int right_left = 0;
                int moving_leg;
                double rl = 0.;

                if(last_moving_leg == MOVING_RIGHT)
                {
                    moving_leg = MOVING_LEFT;
                    right_left  = 1;
                } else
                {
                    right_left  = 0;
                    moving_leg = MOVING_RIGHT;
                }

                _footprint_info newfoot;

                for(int i=0; i<5; i++)
                {//WHy 5???

                    /* Change direction */
                    if(right_left == 1){
                        rl = 1;
                    }else{
                        rl = -1;
                    }

                    /* Make Newfoot print adding STEP_LENGTH, STEP_ANGLE*/
                    if(moving_leg == MOVING_RIGHT){
                        newfoot.footprint.rori[0] = last_short_foot.footprint.lori[0] + STEP_ANGLE;
                        newfoot.footprint.rori[1] = last_short_foot.footprint.rori[1];
                        newfoot.footprint.rori[2] = last_short_foot.footprint.rori[2];

                        newfoot.footprint.lori[0] = last_short_foot.footprint.lori[0];
                        newfoot.footprint.lori[1] = last_short_foot.footprint.lori[1];
                        newfoot.footprint.lori[2] = last_short_foot.footprint.lori[2];

                        newfoot.footprint.rfoot[0] = last_short_foot.footprint.lfoot[0] + STEP_LENGTH*cos(newfoot.footprint.rori[0]*D2R) - STEP_OFFSET*sin(newfoot.footprint.rori[0]*D2R)*rl;
                        newfoot.footprint.rfoot[1] = last_short_foot.footprint.lfoot[1] + STEP_LENGTH*sin(newfoot.footprint.rori[0]*D2R) + STEP_OFFSET*cos(newfoot.footprint.rori[0]*D2R)*rl;
                        newfoot.footprint.rfoot[2] = last_short_foot.footprint.rfoot[2];

                        newfoot.footprint.lfoot[0] = last_short_foot.footprint.lfoot[0];
                        newfoot.footprint.lfoot[1] = last_short_foot.footprint.lfoot[1];
                        newfoot.footprint.lfoot[2] = last_short_foot.footprint.lfoot[2];

                        moving_leg = MOVING_LEFT;
                    }else
                    {
                        newfoot.footprint.rori[0] = last_short_foot.footprint.rori[0];
                        newfoot.footprint.rori[1] = last_short_foot.footprint.rori[1];
                        newfoot.footprint.rori[2] = last_short_foot.footprint.rori[2];

                        newfoot.footprint.lori[0] = last_short_foot.footprint.rori[0] + STEP_ANGLE;
                        newfoot.footprint.lori[1] = last_short_foot.footprint.lori[1];
                        newfoot.footprint.lori[2] = last_short_foot.footprint.lori[2];

                        newfoot.footprint.rfoot[0] = last_short_foot.footprint.rfoot[0];
                        newfoot.footprint.rfoot[1] = last_short_foot.footprint.rfoot[1];
                        newfoot.footprint.rfoot[2] = last_short_foot.footprint.rfoot[2];

                        newfoot.footprint.lfoot[0] = last_short_foot.footprint.rfoot[0] + STEP_LENGTH*cos(newfoot.footprint.lori[0]*D2R) - STEP_OFFSET*sin(newfoot.footprint.lori[0]*D2R)*rl;
                        newfoot.footprint.lfoot[1] = last_short_foot.footprint.rfoot[1] + STEP_LENGTH*sin(newfoot.footprint.lori[0]*D2R) + STEP_OFFSET*cos(newfoot.footprint.lori[0]*D2R)*rl;
                        newfoot.footprint.lfoot[2] = last_short_foot.footprint.lfoot[2];

                        moving_leg = MOVING_RIGHT;
                    }

                    newfoot.time.dsp_time = 0.1;
                    newfoot.time.ssp_time = 0.8;

                    if(stop_flag == true)
                    {
                        FILE_LOG(logERROR) << "STOP CONTINUOUS WALKING!!";
                        walk_start_flag = false;
                        while(pull_short_foot(dummyfoot))
                        {;}
                        push_short_foot(last_short_foot);
                        stop_flag = false;
                        break;
                    } else if(walkstop_liftbox_flag == true)
                    {
                        FILE_LOG(logERROR) << "STOP APPROACH WALKING!! (MODE_LIFTBOX)";
                        walk_start_flag = false;
                        break;
                    } else if(walkstop_door_flag == true)
                    {
                        FILE_LOG(logERROR) << "STOP APPROACH WALKING!! (MODE_DOOR)";
                        walk_start_flag = false;
                        break;
                    } else
                    {
                        push_short_foot(newfoot);
                        right_left ^= 1;
                    }
                }
            }

            if(walkstop_liftbox_flag == true)
            {
                FILE_LOG(logERROR) << "APPROACH DONE!!";
                _footprint_info dummyfoot;
                while(pull_short_foot(dummyfoot))
                {;}

                make_last_liftbox_footprint();

                printf("shut down approach\n");
                walkstop_liftbox_flag = false;
            }

            if(walkstop_door_flag == true)
            {
                FILE_LOG(logERROR) << "APPROACH DONE!!";
                _footprint_info dummyfoot;
                while(pull_short_foot(dummyfoot))
                {;}

                make_last_door_footprint();

                printf("shut down approach\n");
                walkstop_door_flag = false;
            }

            if(OnOff_compliance == true)
            {
                //printf("Compliance on!\n");
                StartComplianceControl();
                //ZMP_FeedBack_ONOFF = true;
                ZMPControl();
            }

            if(FLAG_pushdoor == true)
            {
                StartPushDoor();
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
            WBIK();

            /* save data */
            save();
            SendDatatoGUI();

        } else
        {

            static double cnt111 = 0.;
            cnt111 = cnt111+1;

            if(cnt111>0.5)
            {
                sharedData->LaserLength=6666;
            }

                if(LandingState == FINAL)
                {
                    printf("******************* Walking Is Finishied!!!");
                    Walking_initialize();

                }
                LandingState = END;
        }

        jCon->MoveAllJoint();
        rt_task_suspend(&rtTaskCon);
    }
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
    userData->M2G.Des_posX = PosX;
    userData->M2G.Des_velX = VelX;
    userData->M2G.LPF_Fz = LPF_RH_Fz;
    userData->M2G.LPF_Fx = LPF_RH_Fx;
    userData->M2G.LPF_Fy = LPF_RH_Fy;
    userData->M2G.curZMP[0] = Y_ZMP_Local;
    userData->M2G.curZMP[1] = zmp_Local[1]*1000.;
    //userData->M2G.curZMP[2] = Local_bar[1]*1000.;
}
void ShutDownAllFlag()
{
    //WB_FLAG = false;
    OnOff_compliance = false;

    /*Init Variables*/
    PosX = PosY = PosZ = VelX = VelY = VelZ = 0.;
    //ZMPControlX = ZMPControlY = Del_PC_X_DSP_XZMP_CON = Del_PC_Y_DSP_YZMP_CON = I_ZMP_CON_X = 0.;
}
void StartComplianceControl()
{
    /* Fz LowPassFilter */
    float LPF_Gain_X = 0.05;
    float LPF_Gain_Y = 0.05;
    float LPF_Gain_Z = 0.1;


    if(sharedData->FT[3].Fz > 5. || sharedData->FT[3].Fz < -5.)
    {
        LPF_RH_Fz = sharedData->FT[3].Fz;
    } else
    {
        LPF_RH_Fz = 0.;
    }
    if(sharedData->FT[3].Fx > 3. || sharedData->FT[3].Fx < -3.)
    {
        LPF_RH_Fx = sharedData->FT[3].Fx;
    } else
    {
        LPF_RH_Fx = 0.;
    }
    if(sharedData->FT[3].Fy > 0.5 || sharedData->FT[3].Fy < -0.5)
    {
        LPF_RH_Fy = sharedData->FT[3].Fy;

    } else
    {
        LPF_RH_Fy = 0.;
    }

    LPF_RH_Fz = sharedData->FT[3].Fz;
    LPF_RH_Fx = sharedData->FT[3].Fx;
    LPF_RH_Fy = sharedData->FT[3].Fy;

    LPF_RH_Fz = LPF_Gain_Z*LPF_RH_Fz + (1-LPF_Gain_Z)*Before_RH_Fz;
    LPF_RH_Fx = LPF_Gain_X*LPF_RH_Fx + (1-LPF_Gain_X)*Before_RH_Fx;
    LPF_RH_Fy = LPF_Gain_Y*LPF_RH_Fy + (1-LPF_Gain_Y)*Before_RH_Fy;

    Before_RH_Fz = LPF_RH_Fz;
    Before_RH_Fx = LPF_RH_Fx;
    Before_RH_Fy = LPF_RH_Fy;



    /* RHand Compliance control */
    float Compliance_Gain_X = 0.01;
    float Resilence_Gain_X = 5.5;

    float Compliance_Gain_Y = 0.01;
    float Resilence_Gain_Y = 5.5;

    float Compliance_Gain_Z = 0.005;
    float Resilence_Gain_Z = 2.5;

    Desired_Force = 0;
    Measured_Force_Z = LPF_RH_Fz;
    Measured_Force_Y = LPF_RH_Fy;
    Measured_Force_X = LPF_RH_Fx;

    VelX = Compliance_Gain_X*(Desired_Force - Measured_Force_Z) - Resilence_Gain_X*PosX;
    PosX += 0.005*VelX;

    VelY = Compliance_Gain_Y*(Desired_Force + Measured_Force_Y) - Resilence_Gain_Y*PosY;
    PosY += 0.005*VelY;

    VelZ = Compliance_Gain_Z*(Desired_Force + Measured_Force_X) - Resilence_Gain_Z*PosZ;
    PosZ += 0.005*VelZ;

    if(PosX > 0.3)
    {
        PosX = 0.3;
    }else if(PosX <-0.3)
    {
        PosX = -0.3;
    }

    if(PosY > 0.07)
    {
        PosY = 0.07;
    }else if(PosY <-0.07)
    {
        PosY = -0.07;
    }

    if(PosZ > 0.07)
    {
        PosZ = 0.07;
    }else if(PosZ <-0.07)
    {
        PosZ = -0.07;
    }
}

void StartPushDoor()
{
    double _sTime = 3.0;
    double _xArm = 0.4;

}

void ZMPControl()
{
    get_zmp2();
    Kirk_Control();
    zmp_Local[0] = -0.001*Del_PC_X_DSP_XZMP_CON;
    zmp_Local[1] = -0.001*Del_PC_Y_DSP_YZMP_CON;
    zmp_Local[2] = 0;

    //Local2Global(zmp_Local,Global);

    ZMPControlX = zmp_Local[0];
    ZMPControlY = zmp_Local[1];
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
void WBIK()
{
    double temp1des_qPEL_4x1[4],temp2des_qPEL_4x1[4],temp3des_qPEL_4x1[4],temp4des_qPEL_4x1[4];
    double temp1des_qRF_4x1[4],temp2des_qRF_4x1[4],temp3des_qRF_4x1[4],temp4des_qRF_4x1[4],temp5des_qRF_4x1[4];
    double temp1des_qLF_4x1[4],temp2des_qLF_4x1[4],temp3des_qLF_4x1[4],temp4des_qLF_4x1[4],temp5des_qLF_4x1[4];
    double RightYaw,RightRoll,RightPitch,LeftYaw,LeftRoll,LeftPitch;

    double Global[3],Local[3];


    // Task Space Command

    if(pv_Index ==1)
    {
        // ankle torque control;

        RDPitch = RMYC(2,2,0,0.,0.,0./1000.0,0./1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],0.,0.);
        LDPitch = LMYC(2,2,0,0.,0.,0./1000.0,0./1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],0.,0.);
        RDPitch =0.;
        LDPitch =0.;
        RDRoll = RMXC(2,2,0,0.,0.,0./1000.0,0./1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],0.,0.);
        LDRoll = LMXC(2,2,0,0.,0.,0./1000.0,0./1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],0.,0.);
        RDRoll =0.;
        LDRoll =0.;

        RDPitch2 = RMYC2(2,2,0,0.,0.,0./1000.0,0./1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],0.,0.);
        LDPitch2= LMYC2(2,2,0,0.,0.,0./1000.0,0./1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],0.,0.);
        RDPitch2 =0.;
        LDPitch2 =0.;

        RDRoll2 = RMXC2(2,2,0,0.,0.,0./1000.0,0./1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],0.,0.);
        LDRoll2= LMXC2(2,2,0,0.,0.,0./1000.0,0./1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],0.,0.);
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
    QTcross(temp3des_qPEL_4x1,temp4des_qPEL_4x1,des_qPEL_4x1);



    //------------- CoM
    Local[0] = GLOBAL_X_LIPM_n + G_DSP_X*(- 0.001*Del_PC_X_DSP_XZMP_CON*G_DSP_X + I_ZMP_CON_X*1.)*ZMP_FeedBack_ONOFF;// + ZMPControlX*OnOff_compliance;
    if(Inv_ONOFF == 0){
        U_Gain = 0;
    }
    CONT_Y = Local[1] =  (GLOBAL_Y_LIPM_n)*(U0_Gain)+ (-0.001*Del_PC_Y_DSP_YZMP_CON*G_DSP_Y - 0.001*Del_PC_Y_SSP_YZMP_CON*0+ I_ZMP_CON_Y*0.)*ZMP_FeedBack_ONOFF;// + ZMPControlY*OnOff_compliance;

    Local[2]=0;
    Local2Global(Local,Global);

    des_pCOM_3x1_hat[Xdir] = Global[0];
    des_pCOM_3x1_hat[Ydir] = Global[1];
    des_pCOM_3x1_hat[Zdir] = userData->WalkReadyCOM[Zdir];// + GLOBAL_Z_LIPM;// - (fsm->AddRightFootInfos[0][2] + fsm->AddLeftFootInfos[0][2])*0.7;

    double RotX[9],RotY[9],RotZ[9],RotYX[9],TorsoOri[3],TorsoOri_n[3];
    TorsoOri[0] = 0.*D2R;
    TorsoOri[1] = 0.*D2R;
    TorsoOri[2] = 0.;

    RZ((window[0].right_foot_ref.yaw*D2R + window[0].left_foot_ref.yaw*D2R)/2.,RotZ);
    mult_mv(RotZ,3,3,TorsoOri,TorsoOri_n);

    RX(TorsoOri_n[0],RotX);
    RY(TorsoOri_n[1],RotY);

    mult_mm(RotY,3,3,RotX,3,RotYX);

    mult_mv(RotYX,3,3,des_pCOM_3x1_hat,des_pCOM_3x1);
    //---------------

    // ------------ RF
    des_pRF_3x1_hat[Xdir] = GLOBAL_X_RF;
    des_pRF_3x1_hat[Ydir] = GLOBAL_Y_RF;
    des_pRF_3x1_hat[Zdir] = GLOBAL_Z_RF;// + Zctrl*0.5*impONOFF+ Zctrl2*0.5*impONOFF2+ Add_FootTask[RIGHT][Zdir]*Leg_Length_FeedBack_ONOFF + Add_Leg_Recovery[RIGHT][Zdir]*Leg_Length_Recover_ONOFF;

    mult_mv(RotYX,3,3,des_pRF_3x1_hat,des_pRF_3x1);

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

    des_qRF_4x1[0] = temp5des_qRF_4x1[0];
    des_qRF_4x1[1] = temp5des_qRF_4x1[1];
    des_qRF_4x1[2] = temp5des_qRF_4x1[2];
    des_qRF_4x1[3] = temp5des_qRF_4x1[3];

    // ------------ LF
    des_pLF_3x1_hat[Xdir] = GLOBAL_X_LF;
    des_pLF_3x1_hat[Ydir] = GLOBAL_Y_LF;
    des_pLF_3x1_hat[Zdir] = GLOBAL_Z_LF;// - Zctrl*0.5*impONOFF- Zctrl2*0.5*impONOFF2+ Add_FootTask[LEFT][Zdir]*Leg_Length_FeedBack_ONOFF + Add_Leg_Recovery[LEFT][Zdir]*Leg_Length_Recover_ONOFF;

    mult_mv(RotYX,3,3,des_pLF_3x1_hat,des_pLF_3x1);

    // ------------ LF Orietation
    LeftYaw   = window[0].left_foot_ref.yaw*D2R;
    LeftPitch = window[0].left_foot_ref.pitch*D2R;
    LeftRoll  = window[0].left_foot_ref.roll*D2R;

    qtRZ(LeftYaw, temp1des_qLF_4x1);
    qtRY(LeftPitch, temp4des_qLF_4x1);
    qtRX(LeftRoll, temp2des_qLF_4x1);

    QTcross(temp1des_qLF_4x1,temp4des_qLF_4x1,temp3des_qLF_4x1);
    QTcross(temp3des_qLF_4x1,temp2des_qLF_4x1,temp5des_qLF_4x1);

    des_qLF_4x1[0] = temp5des_qLF_4x1[0];
    des_qLF_4x1[1] = temp5des_qLF_4x1[1];
    des_qLF_4x1[2] = temp5des_qLF_4x1[2];
    des_qLF_4x1[3] = temp5des_qLF_4x1[3];


    if(pv_Index <=1)
    {
        printf("%d WBIK*********** right x: %f  y:%f   z:%f  \n",pv_Index, des_pRF_3x1[0],des_pRF_3x1[1],des_pRF_3x1[2]);
        printf("%d WBIK*********** left  x: %f  y:%f   z:%f  \n",pv_Index,des_pLF_3x1[0],des_pLF_3x1[1],des_pLF_3x1[2]);
        printf("%d WBIK COM x: %f  y: %f   z:%f \n",pv_Index,des_pCOM_3x1[0],des_pCOM_3x1[1],des_pCOM_3x1[2]);
        printf("%d  Right WBIK_Q : %f  %f  %f  %f  %f  %f \n",pv_Index,WBIK_Q[7],WBIK_Q[8],WBIK_Q[9],WBIK_Q[10],WBIK_Q[11],WBIK_Q[12]);
        printf("%d  Left  WBIK_Q : %f  %f  %f  %f  %f  %f \n",pv_Index,WBIK_Q[13],WBIK_Q[14],WBIK_Q[15],WBIK_Q[16],WBIK_Q[17],WBIK_Q[18]);
        printf("%d  WBIK Pelvis :  %f   %f   %f \n",pv_Index,WBIK_Q[0],WBIK_Q[1],WBIK_Q[2]);
    }
    // 1 . PELVIS Orientation
    Pel_Yaw = (window[0].right_foot_ref.yaw + window[0].left_foot_ref.yaw)*D2R/2.;
    qtRZ(Pel_Yaw, temp1des_qPEL_4x1);
    qtRX(1.0*0*D2R, temp2des_qPEL_4x1);
    qtRY(1.0*0*D2R, temp4des_qPEL_4x1);

    QTcross(temp1des_qPEL_4x1,temp2des_qPEL_4x1,temp3des_qPEL_4x1);
    QTcross(temp3des_qPEL_4x1,temp4des_qPEL_4x1,des_qPEL_4x1);

    // 2. Right Foot Orientation
    RightYaw = window[0].right_foot_ref.yaw*D2R;
    RightPitch = window[0].right_foot_ref.pitch*D2R;
    RightRoll  = window[0].right_foot_ref.roll*D2R;

    qtRZ(RightYaw, temp1des_qRF_4x1);
    qtRY(RightPitch, temp4des_qRF_4x1);
    qtRX(RightRoll, temp2des_qRF_4x1);

    QTcross(temp1des_qRF_4x1,temp4des_qRF_4x1,temp3des_qRF_4x1);
    QTcross(temp3des_qRF_4x1,temp2des_qRF_4x1,temp5des_qRF_4x1);

    des_qRF_4x1[0] = temp5des_qRF_4x1[0];
    des_qRF_4x1[1] = temp5des_qRF_4x1[1];
    des_qRF_4x1[2] = temp5des_qRF_4x1[2];
    des_qRF_4x1[3] = temp5des_qRF_4x1[3];

    // 6 . LF Orietation
    LeftYaw = window[0].left_foot_ref.yaw*D2R ;
    LeftPitch = window[0].left_foot_ref.pitch*D2R ;
    LeftRoll = window[0].left_foot_ref.roll*D2R ;

    qtRZ(LeftYaw, temp1des_qLF_4x1);
    qtRY(LeftPitch, temp4des_qLF_4x1);
    qtRX(LeftRoll, temp2des_qLF_4x1);

    QTcross(temp1des_qLF_4x1,temp4des_qLF_4x1,temp3des_qLF_4x1);
    QTcross(temp3des_qLF_4x1,temp2des_qLF_4x1,temp5des_qLF_4x1);

    des_qLF_4x1[0] = temp5des_qLF_4x1[0];
    des_qLF_4x1[1] = temp5des_qLF_4x1[1];
    des_qLF_4x1[2] = temp5des_qLF_4x1[2];
    des_qLF_4x1[3] = temp5des_qLF_4x1[3];

    memcpy(WBIK_Q0,WBIK_Q,34*sizeof(double));


    kine_drc_hubo4.IK_LowerBody_Global(WBIK_Q0,Qub,des_pCOM_3x1, des_qPEL_4x1, des_pRF_3x1, des_qRF_4x1, des_pLF_3x1, des_qLF_4x1,WBIK_Q);

    // Ankle Joint control using gyro

    GLOBAL_Xori_RF_n = GLOBAL_Xori_RF*cos(WBIK_Q[idRHY]) + GLOBAL_Yori_RF*sin(WBIK_Q[idRHY]);
    GLOBAL_Yori_RF_n =-GLOBAL_Xori_RF*sin(WBIK_Q[idRHY]) + GLOBAL_Yori_RF*cos(WBIK_Q[idRHY]);

    GLOBAL_Xori_LF_n = GLOBAL_Xori_LF*cos(WBIK_Q[idLHY]) + GLOBAL_Yori_LF*sin(WBIK_Q[idLHY]);
    GLOBAL_Yori_LF_n =-GLOBAL_Xori_LF*sin(WBIK_Q[idLHY]) + GLOBAL_Yori_LF*cos(WBIK_Q[idLHY]);


    for(int i=0; i<=LAR; i++)
    {
        FWRefAngleCurrent[i] = WBIK_Q[i+7]*R2D;

        FWRefAngleCurrent[RAR] = WBIK_Q[RAR+7]*R2D + GLOBAL_Xori_RF*Gyro_Ankle_FeedBack_ONOFF + deflection_comp_RAR*Sagging_Comp_ONOFF - RDRoll*ssp_torque_ONOFF;
        FWRefAngleCurrent[RAP] = WBIK_Q[RAP+7]*R2D + GLOBAL_Yori_RF*Gyro_Ankle_FeedBack_ONOFF - RDPitch*ssp_torque_ONOFF;

        FWRefAngleCurrent[LAR] = WBIK_Q[LAR+7]*R2D + GLOBAL_Xori_LF*Gyro_Ankle_FeedBack_ONOFF + deflection_comp_LAR*Sagging_Comp_ONOFF - LDRoll*ssp_torque_ONOFF;
        FWRefAngleCurrent[LAP] = WBIK_Q[LAP+7]*R2D + GLOBAL_Yori_LF*Gyro_Ankle_FeedBack_ONOFF - LDPitch*ssp_torque_ONOFF;

        jCon->Joints[i]->RefAngleCurrent = FWRefAngleCurrent[i];
    }

    if(OnOff_compliance == true)
    {

    } else if(window[0].state == STATE_EMPTY)
    {
        FINAL_TIMER  = FINAL_TIMER + 0.005;

        if(FINAL_TIMER > 3.0)
        {
            printf("Walking is finished and Walkflag is set to 0 \n");
            walk_flag = 0;
            continouse_walking_flag = false;
            walk_start_flag = false;
            walkstop_liftbox_flag = false;
            stop_flag = false;
            FLAG_SingleLog = false;
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


void approach_last()
{
    make_last_liftbox_footprint();
}

void make_last_liftbox_footprint()
{
    int right_left = 0;
    int moving_leg;
    static int cnt = 0;
    cnt++;

    if(last_moving_leg == MOVING_RIGHT)
    {
        moving_leg = MOVING_LEFT;
        right_left  = 1;
    } else
    {
        right_left  = 0;
        moving_leg = MOVING_RIGHT;
    }

    _footprint_info dummyfoot;
    while(pull_short_foot(dummyfoot)){
        ;
    }
    _footprint_info newfoot;

    STEP_LENGTH = 0.1;
    STEP_ANGLE = 0.0;
    STEP_OFFSET = 0.21 + 0.05*fabs(STEP_ANGLE/10.0);

    for(int i=0;i<3;i++)
    {
        if(i==0)
        {
            if(right_left == 0){
                newfoot.footprint.rori[0] = last_short_foot.footprint.lori[0];
                newfoot.footprint.rori[1] = last_short_foot.footprint.rori[1];
                newfoot.footprint.rori[2] = last_short_foot.footprint.rori[2];

                newfoot.footprint.lori[0] = last_short_foot.footprint.lori[0];
                newfoot.footprint.lori[1] = last_short_foot.footprint.lori[1];
                newfoot.footprint.lori[2] = last_short_foot.footprint.lori[2];

                newfoot.footprint.rfoot[0] = last_short_foot.footprint.lfoot[0];
                newfoot.footprint.rfoot[1] = last_short_foot.footprint.lfoot[1] - STEP_LENGTH - STEP_OFFSET;
                newfoot.footprint.rfoot[2] = last_short_foot.footprint.rfoot[2];

                newfoot.footprint.lfoot[0] = last_short_foot.footprint.lfoot[0];
                newfoot.footprint.lfoot[1] = last_short_foot.footprint.lfoot[1];
                newfoot.footprint.lfoot[2] = last_short_foot.footprint.lfoot[2];

                moving_leg = MOVING_LEFT;
            }else
            {
                newfoot.footprint.rori[0] = last_short_foot.footprint.rori[0];
                newfoot.footprint.rori[1] = last_short_foot.footprint.rori[1];
                newfoot.footprint.rori[2] = last_short_foot.footprint.rori[2];

                newfoot.footprint.lori[0] = last_short_foot.footprint.rori[0];
                newfoot.footprint.lori[1] = last_short_foot.footprint.lori[1];
                newfoot.footprint.lori[2] = last_short_foot.footprint.lori[2];

                newfoot.footprint.rfoot[0] = last_short_foot.footprint.rfoot[0];
                newfoot.footprint.rfoot[1] = last_short_foot.footprint.rfoot[1];
                newfoot.footprint.rfoot[2] = last_short_foot.footprint.rfoot[2];

                newfoot.footprint.lfoot[0] = last_short_foot.footprint.rfoot[0];
                newfoot.footprint.lfoot[1] = last_short_foot.footprint.rfoot[1] + STEP_LENGTH + STEP_OFFSET;
                newfoot.footprint.lfoot[2] = last_short_foot.footprint.lfoot[2];

                moving_leg = MOVING_RIGHT;
            }
        }
        if(i==2)
        {
            if(right_left == 0){
                newfoot.footprint.rori[0] = newfoot.footprint.rori[0];
                newfoot.footprint.rori[1] = newfoot.footprint.rori[1];
                newfoot.footprint.rori[2] = newfoot.footprint.rori[2];

                newfoot.footprint.lori[0] = newfoot.footprint.lori[0];
                newfoot.footprint.lori[1] = newfoot.footprint.lori[1];
                newfoot.footprint.lori[2] = newfoot.footprint.lori[2];

                newfoot.footprint.rfoot[0] = newfoot.footprint.rfoot[0];
                newfoot.footprint.rfoot[1] = newfoot.footprint.rfoot[1];
                newfoot.footprint.rfoot[2] = newfoot.footprint.rfoot[2];

                newfoot.footprint.lfoot[0] = newfoot.footprint.lfoot[0];
                newfoot.footprint.lfoot[1] = newfoot.footprint.lfoot[1];
                newfoot.footprint.lfoot[2] = newfoot.footprint.lfoot[2];

                moving_leg = MOVING_LEFT;
            }else
            {
                newfoot.footprint.rori[0] = newfoot.footprint.rori[0];
                newfoot.footprint.rori[1] = newfoot.footprint.rori[1];
                newfoot.footprint.rori[2] = newfoot.footprint.rori[2];

                newfoot.footprint.lori[0] = newfoot.footprint.lori[0];
                newfoot.footprint.lori[1] = newfoot.footprint.lori[1];
                newfoot.footprint.lori[2] = newfoot.footprint.lori[2];

                newfoot.footprint.rfoot[0] = newfoot.footprint.rfoot[0];
                newfoot.footprint.rfoot[1] = newfoot.footprint.rfoot[1];
                newfoot.footprint.rfoot[2] = newfoot.footprint.rfoot[2];

                newfoot.footprint.lfoot[0] = newfoot.footprint.lfoot[0];
                newfoot.footprint.lfoot[1] = newfoot.footprint.lfoot[1];
                newfoot.footprint.lfoot[2] = newfoot.footprint.lfoot[2];

                moving_leg = MOVING_RIGHT;
            }

        } else
        {
            if(right_left == 0){
                newfoot.footprint.rori[0] = newfoot.footprint.lori[0];
                newfoot.footprint.rori[1] = newfoot.footprint.rori[1];
                newfoot.footprint.rori[2] = newfoot.footprint.rori[2];

                newfoot.footprint.lori[0] = newfoot.footprint.lori[0];
                newfoot.footprint.lori[1] = newfoot.footprint.lori[1];
                newfoot.footprint.lori[2] = newfoot.footprint.lori[2];

                newfoot.footprint.rfoot[0] = newfoot.footprint.lfoot[0];
                newfoot.footprint.rfoot[1] = newfoot.footprint.rfoot[1] - STEP_LENGTH;
                newfoot.footprint.rfoot[2] = newfoot.footprint.rfoot[2];

                newfoot.footprint.lfoot[0] = newfoot.footprint.lfoot[0];
                newfoot.footprint.lfoot[1] = newfoot.footprint.lfoot[1];
                newfoot.footprint.lfoot[2] = newfoot.footprint.lfoot[2];

                moving_leg = MOVING_LEFT;
            }else
            {
                newfoot.footprint.rori[0] = newfoot.footprint.rori[0];
                newfoot.footprint.rori[1] = newfoot.footprint.rori[1];
                newfoot.footprint.rori[2] = newfoot.footprint.rori[2];

                newfoot.footprint.lori[0] = newfoot.footprint.rori[0];
                newfoot.footprint.lori[1] = newfoot.footprint.lori[1];
                newfoot.footprint.lori[2] = newfoot.footprint.lori[2];

                newfoot.footprint.rfoot[0] = newfoot.footprint.rfoot[0];
                newfoot.footprint.rfoot[1] = newfoot.footprint.rfoot[1];
                newfoot.footprint.rfoot[2] = newfoot.footprint.rfoot[2];

                newfoot.footprint.lfoot[0] = newfoot.footprint.rfoot[0];
                newfoot.footprint.lfoot[1] = newfoot.footprint.lfoot[1] + STEP_LENGTH;
                newfoot.footprint.lfoot[2] = newfoot.footprint.lfoot[2];

                moving_leg = MOVING_RIGHT;
            }
        }

        newfoot.time.dsp_time = 0.1;
        newfoot.time.ssp_time = 0.8;

        push_short_foot(newfoot);
        printf("make approach foot print %dth \n",cnt);
        printf("moving_leg = %d\n",moving_leg);
        printf("nextfoot = R:[%f,%f,%f], L:[%f,%f,%f]\n",newfoot.footprint.rfoot[0],newfoot.footprint.rfoot[1],newfoot.footprint.rfoot[2],
                newfoot.footprint.lfoot[0],newfoot.footprint.lfoot[1],newfoot.footprint.lfoot[2]);
        right_left ^= 1;
    }
}


void make_last_footprint()
{
    int right_left = 0;
    int moving_leg;
    static int cnt = 0;
    cnt++;

    if(last_moving_leg == MOVING_RIGHT)
    {
        moving_leg = MOVING_LEFT;
        right_left  = 1;
    } else
    {
        right_left  = 0;
        moving_leg = MOVING_RIGHT;
    }

    _footprint_info dummyfoot;
    while(pull_short_foot(dummyfoot)){
        ;
    }
    _footprint_info newfoot;

    for(int i=0;i<3;i++)
    {
        if(i==0)
        {
            if(right_left == 0){
                newfoot.footprint.rori[0] = last_short_foot.footprint.lori[0];
                newfoot.footprint.rori[1] = last_short_foot.footprint.rori[1];
                newfoot.footprint.rori[2] = last_short_foot.footprint.rori[2];

                newfoot.footprint.lori[0] = last_short_foot.footprint.lori[0];
                newfoot.footprint.lori[1] = last_short_foot.footprint.lori[1];
                newfoot.footprint.lori[2] = last_short_foot.footprint.lori[2];

                newfoot.footprint.rfoot[0] = last_short_foot.footprint.lfoot[0];
                newfoot.footprint.rfoot[1] = last_short_foot.footprint.lfoot[1]- RSTEP_OFFSET;
                newfoot.footprint.rfoot[2] = last_short_foot.footprint.rfoot[2];

                newfoot.footprint.lfoot[0] = last_short_foot.footprint.lfoot[0];
                newfoot.footprint.lfoot[1] = last_short_foot.footprint.lfoot[1];
                newfoot.footprint.lfoot[2] = last_short_foot.footprint.lfoot[2];

                moving_leg = MOVING_LEFT;
            }else
            {
                newfoot.footprint.rori[0] = last_short_foot.footprint.rori[0];
                newfoot.footprint.rori[1] = last_short_foot.footprint.rori[1];
                newfoot.footprint.rori[2] = last_short_foot.footprint.rori[2];

                newfoot.footprint.lori[0] = last_short_foot.footprint.rori[0];
                newfoot.footprint.lori[1] = last_short_foot.footprint.lori[1];
                newfoot.footprint.lori[2] = last_short_foot.footprint.lori[2];

                newfoot.footprint.rfoot[0] = last_short_foot.footprint.rfoot[0];
                newfoot.footprint.rfoot[1] = last_short_foot.footprint.rfoot[1];
                newfoot.footprint.rfoot[2] = last_short_foot.footprint.rfoot[2];

                newfoot.footprint.lfoot[0] = last_short_foot.footprint.rfoot[0];
                newfoot.footprint.lfoot[1] = last_short_foot.footprint.rfoot[1]+ RSTEP_OFFSET;
                newfoot.footprint.lfoot[2] = last_short_foot.footprint.lfoot[2];

                moving_leg = MOVING_RIGHT;
            }
        }
        if(i==2)
        {
            if(right_left == 0){
                newfoot.footprint.rori[0] = newfoot.footprint.rori[0];
                newfoot.footprint.rori[1] = newfoot.footprint.rori[1];
                newfoot.footprint.rori[2] = newfoot.footprint.rori[2];

                newfoot.footprint.lori[0] = newfoot.footprint.lori[0];
                newfoot.footprint.lori[1] = newfoot.footprint.lori[1];
                newfoot.footprint.lori[2] = newfoot.footprint.lori[2];

                newfoot.footprint.rfoot[0] = newfoot.footprint.rfoot[0];
                newfoot.footprint.rfoot[1] = newfoot.footprint.rfoot[1];
                newfoot.footprint.rfoot[2] = newfoot.footprint.rfoot[2];

                newfoot.footprint.lfoot[0] = newfoot.footprint.lfoot[0];
                newfoot.footprint.lfoot[1] = newfoot.footprint.lfoot[1];
                newfoot.footprint.lfoot[2] = newfoot.footprint.lfoot[2];

                moving_leg = MOVING_LEFT;
            }else
            {
                newfoot.footprint.rori[0] = newfoot.footprint.rori[0];
                newfoot.footprint.rori[1] = newfoot.footprint.rori[1];
                newfoot.footprint.rori[2] = newfoot.footprint.rori[2];

                newfoot.footprint.lori[0] = newfoot.footprint.lori[0];
                newfoot.footprint.lori[1] = newfoot.footprint.lori[1];
                newfoot.footprint.lori[2] = newfoot.footprint.lori[2];

                newfoot.footprint.rfoot[0] = newfoot.footprint.rfoot[0];
                newfoot.footprint.rfoot[1] = newfoot.footprint.rfoot[1];
                newfoot.footprint.rfoot[2] = newfoot.footprint.rfoot[2];

                newfoot.footprint.lfoot[0] = newfoot.footprint.lfoot[0];
                newfoot.footprint.lfoot[1] = newfoot.footprint.lfoot[1];
                newfoot.footprint.lfoot[2] = newfoot.footprint.lfoot[2];

                moving_leg = MOVING_RIGHT;
            }

        } else
        {
            if(right_left == 0){
                newfoot.footprint.rori[0] = newfoot.footprint.lori[0];
                newfoot.footprint.rori[1] = newfoot.footprint.rori[1];
                newfoot.footprint.rori[2] = newfoot.footprint.rori[2];

                newfoot.footprint.lori[0] = newfoot.footprint.lori[0];
                newfoot.footprint.lori[1] = newfoot.footprint.lori[1];
                newfoot.footprint.lori[2] = newfoot.footprint.lori[2];

                newfoot.footprint.rfoot[0] = newfoot.footprint.lfoot[0];
                newfoot.footprint.rfoot[1] = newfoot.footprint.rfoot[1];
                newfoot.footprint.rfoot[2] = newfoot.footprint.rfoot[2];

                newfoot.footprint.lfoot[0] = newfoot.footprint.lfoot[0];
                newfoot.footprint.lfoot[1] = newfoot.footprint.lfoot[1];
                newfoot.footprint.lfoot[2] = newfoot.footprint.lfoot[2];

                moving_leg = MOVING_LEFT;
            }else
            {
                newfoot.footprint.rori[0] = newfoot.footprint.rori[0];
                newfoot.footprint.rori[1] = newfoot.footprint.rori[1];
                newfoot.footprint.rori[2] = newfoot.footprint.rori[2];

                newfoot.footprint.lori[0] = newfoot.footprint.rori[0];
                newfoot.footprint.lori[1] = newfoot.footprint.lori[1];
                newfoot.footprint.lori[2] = newfoot.footprint.lori[2];

                newfoot.footprint.rfoot[0] = newfoot.footprint.rfoot[0];
                newfoot.footprint.rfoot[1] = newfoot.footprint.rfoot[1];
                newfoot.footprint.rfoot[2] = newfoot.footprint.rfoot[2];

                newfoot.footprint.lfoot[0] = newfoot.footprint.rfoot[0];
                newfoot.footprint.lfoot[1] = newfoot.footprint.lfoot[1];
                newfoot.footprint.lfoot[2] = newfoot.footprint.lfoot[2];

                moving_leg = MOVING_RIGHT;
            }
        }

        newfoot.time.dsp_time = 0.1;
        newfoot.time.ssp_time = 0.8;

        push_short_foot(newfoot);
        printf("make approach foot print %dth \n",cnt);
        printf("moving_leg = %d\n",moving_leg);
        printf("nextfoot = R:[%f,%f,%f], L:[%f,%f,%f]\n",newfoot.footprint.rfoot[0],newfoot.footprint.rfoot[1],newfoot.footprint.rfoot[2],
                newfoot.footprint.lfoot[0],newfoot.footprint.lfoot[1],newfoot.footprint.lfoot[2]);
        right_left ^= 1;
    }
}

void make_last_door_footprint()
{
    int right_left = 0;
    int moving_leg;
    static int cnt = 0;
    cnt++;

    if(last_moving_leg == MOVING_RIGHT)
    {
        moving_leg = MOVING_LEFT;
        right_left  = 1;
    } else
    {
        right_left  = 0;
        moving_leg = MOVING_RIGHT;
    }

    _footprint_info dummyfoot;
    while(pull_short_foot(dummyfoot)){
        ;
    }
    _footprint_info newfoot;

    STEP_LENGTH = 0.2;
    STEP_ANGLE = 0.0;
    STEP_OFFSET = 0.21 + 0.05*fabs(STEP_ANGLE/10.0);

    for(int i=0;i<2;i++)
    {
        if(i==0)
        {
            if(right_left == 0){
                newfoot.footprint.rori[0] = last_short_foot.footprint.lori[0];
                newfoot.footprint.rori[1] = last_short_foot.footprint.rori[1];
                newfoot.footprint.rori[2] = last_short_foot.footprint.rori[2];

                newfoot.footprint.lori[0] = last_short_foot.footprint.lori[0];
                newfoot.footprint.lori[1] = last_short_foot.footprint.lori[1];
                newfoot.footprint.lori[2] = last_short_foot.footprint.lori[2];

                newfoot.footprint.rfoot[0] = last_short_foot.footprint.lfoot[0] + STEP_LENGTH;
                newfoot.footprint.rfoot[1] = last_short_foot.footprint.lfoot[1] - STEP_OFFSET;
                newfoot.footprint.rfoot[2] = last_short_foot.footprint.rfoot[2];

                newfoot.footprint.lfoot[0] = last_short_foot.footprint.lfoot[0];
                newfoot.footprint.lfoot[1] = last_short_foot.footprint.lfoot[1];
                newfoot.footprint.lfoot[2] = last_short_foot.footprint.lfoot[2];

                moving_leg = MOVING_LEFT;
            }else
            {
                newfoot.footprint.rori[0] = last_short_foot.footprint.rori[0];
                newfoot.footprint.rori[1] = last_short_foot.footprint.rori[1];
                newfoot.footprint.rori[2] = last_short_foot.footprint.rori[2];

                newfoot.footprint.lori[0] = last_short_foot.footprint.rori[0];
                newfoot.footprint.lori[1] = last_short_foot.footprint.lori[1];
                newfoot.footprint.lori[2] = last_short_foot.footprint.lori[2];

                newfoot.footprint.rfoot[0] = last_short_foot.footprint.rfoot[0];
                newfoot.footprint.rfoot[1] = last_short_foot.footprint.rfoot[1];
                newfoot.footprint.rfoot[2] = last_short_foot.footprint.rfoot[2];

                newfoot.footprint.lfoot[0] = last_short_foot.footprint.rfoot[0] + STEP_LENGTH;
                newfoot.footprint.lfoot[1] = last_short_foot.footprint.rfoot[1] + STEP_OFFSET;
                newfoot.footprint.lfoot[2] = last_short_foot.footprint.lfoot[2];

                moving_leg = MOVING_RIGHT;
            }
        } else if(i==1)
        {
            if(right_left == 0){
                newfoot.footprint.rori[0] = newfoot.footprint.rori[0];
                newfoot.footprint.rori[1] = newfoot.footprint.rori[1];
                newfoot.footprint.rori[2] = newfoot.footprint.rori[2];

                newfoot.footprint.lori[0] = newfoot.footprint.lori[0];
                newfoot.footprint.lori[1] = newfoot.footprint.lori[1];
                newfoot.footprint.lori[2] = newfoot.footprint.lori[2];

                newfoot.footprint.rfoot[0] = newfoot.footprint.rfoot[0];
                newfoot.footprint.rfoot[1] = newfoot.footprint.rfoot[1];
                newfoot.footprint.rfoot[2] = newfoot.footprint.rfoot[2];

                newfoot.footprint.lfoot[0] = newfoot.footprint.lfoot[0];
                newfoot.footprint.lfoot[1] = newfoot.footprint.lfoot[1];
                newfoot.footprint.lfoot[2] = newfoot.footprint.lfoot[2];

                moving_leg = MOVING_LEFT;
            }else
            {
                newfoot.footprint.rori[0] = newfoot.footprint.rori[0];
                newfoot.footprint.rori[1] = newfoot.footprint.rori[1];
                newfoot.footprint.rori[2] = newfoot.footprint.rori[2];

                newfoot.footprint.lori[0] = newfoot.footprint.lori[0];
                newfoot.footprint.lori[1] = newfoot.footprint.lori[1];
                newfoot.footprint.lori[2] = newfoot.footprint.lori[2];

                newfoot.footprint.rfoot[0] = newfoot.footprint.rfoot[0];
                newfoot.footprint.rfoot[1] = newfoot.footprint.rfoot[1];
                newfoot.footprint.rfoot[2] = newfoot.footprint.rfoot[2];

                newfoot.footprint.lfoot[0] = newfoot.footprint.lfoot[0];
                newfoot.footprint.lfoot[1] = newfoot.footprint.lfoot[1];
                newfoot.footprint.lfoot[2] = newfoot.footprint.lfoot[2];

                moving_leg = MOVING_RIGHT;
            }

        }
        newfoot.time.dsp_time = 0.1;
        newfoot.time.ssp_time = 0.8;

        push_short_foot(newfoot);
        printf("make approach door foot print %dth \n",cnt);
        printf("moving_leg = %d\n",moving_leg);
        printf("nextfoot = R:[%f,%f,%f], L:[%f,%f,%f]\n",newfoot.footprint.rfoot[0],newfoot.footprint.rfoot[1],newfoot.footprint.rfoot[2],
                newfoot.footprint.lfoot[0],newfoot.footprint.lfoot[1],newfoot.footprint.lfoot[2]);
        right_left ^= 1;
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

    state_global_pelvis[0] = WBIK_Q[0];
    state_global_pelvis[1] = WBIK_Q[1];
    state_global_pelvis[2] = WBIK_Q[2];

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
        pCenter[0] = (des_pRF_3x1[0] + des_pLF_3x1[0])/2.;
        pCenter[1] = (des_pRF_3x1[1] + des_pLF_3x1[1])/2.;
        pCenter[2] = (des_pRF_3x1[2] + des_pLF_3x1[2])/2.;

        qtRZ((window[0].right_foot_ref.yaw*D2R + window[0].left_foot_ref.yaw*D2R)/2.,qCenter);


        if(sharedData->FT[RAFT].Fz + sharedData->FT[LAFT].Fz > 50.)
        {
            M_LF[0] =  sharedData->FT[LAFT].Mx;
            M_LF[1] =  sharedData->FT[LAFT].My;
            M_LF[2] =  sharedData->FT[LAFT].Mz;

            QTtransform(des_qLF_4x1,M_LF,M_LF_Global);

            M_RF[0] =  sharedData->FT[RAFT].Mx;
            M_RF[1] =  sharedData->FT[RAFT].My;
            M_RF[2] =  sharedData->FT[RAFT].Mz;

            QTtransform(des_qRF_4x1,M_RF,M_RF_Global);

            F_LF[0] = sharedData->FT[LAFT].Fx;
            F_LF[1] = sharedData->FT[LAFT].Fy;
            F_LF[2] = sharedData->FT[LAFT].Fz;

            QTtransform(des_qLF_4x1,F_LF,F_LF_Global);

            F_RF[0] = sharedData->FT[RAFT].Fx;
            F_RF[1] = sharedData->FT[RAFT].Fy;
            F_RF[2] = sharedData->FT[RAFT].Fz;

            QTtransform(des_qRF_4x1,F_RF,F_RF_Global);

            double temp1[3],temp2[3],temp3[3],temp4[3];

            diff_vv(des_pRF_3x1,3,pCenter,temp1);// (despRF - pCenter)
            diff_vv(des_pLF_3x1,3,pCenter,temp2);// (despLF - pCenter)

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
        kine_drc_hubo4.FK_LeftFoot_Local(WBIK_Q,des_pLF_3x1_n,qLF);
        kine_drc_hubo4.FK_RightFoot_Local(WBIK_Q,des_pRF_3x1_n,qRF);

        des_pLF_3x1_n[2] =  des_pLF_3x1[2];
        des_pRF_3x1_n[2] =  des_pRF_3x1[2];

        convert_euler(des_pRF_3x1_n, des_pLF_3x1_n, sharedData->IMU[0].AccX*D2R*cos(-yaw_angle)- sharedData->IMU[0].AccY*D2R*sin(-yaw_angle), ( sharedData->IMU[0].AccX*D2R)*sin(-yaw_angle)+ sharedData->IMU[0].AccY*D2R*cos(-yaw_angle), 0,BTW_FOOT_Angle_roll, BTW_FOOT_Angle_pitch, BTW_FOOT_Angle_yaw,BTW_FOOT_qPEL_comp_4x1);

        Add_FootTask[RIGHT][Zdir] = HUBO2ZMPInitLegLength(0., BTW_FOOT_Angle_roll*R2D, 1);
        Add_FootTask[LEFT][Zdir] = -Add_FootTask[RIGHT][Zdir];

    }
    else if(window[0].state == DSP_FINAL)
    {

        kine_drc_hubo4.FK_LeftFoot_Local(WBIK_Q,des_pLF_3x1_n,qLF);
        kine_drc_hubo4.FK_RightFoot_Local(WBIK_Q,des_pRF_3x1_n,qRF);

        des_pLF_3x1_n[2] =  des_pLF_3x1[2];
        des_pRF_3x1_n[2] =  des_pRF_3x1[2];

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

    WBIK_Q0[idRHY] = 0.;
    WBIK_Q0[idRHR] = -2.78*D2R;
    WBIK_Q0[idRHP] = -43.9*D2R;
    WBIK_Q0[idRKN] = 80.58*D2R; //77.4*D2R;
    WBIK_Q0[idRAP] = -36.68*D2R;
    WBIK_Q0[idRAR] = 2.78*D2R;

    WBIK_Q0[idLHY] = 0.;
    WBIK_Q0[idLHR] = 2.78*D2R;
    WBIK_Q0[idLHP] = -43.9*D2R;
    WBIK_Q0[idLKN] = 80.58*D2R;//77.4*D2R;
    WBIK_Q0[idLAP] = -36.68*D2R;
    WBIK_Q0[idLAR] = -2.78*D2R;
    WBIK_Q0[idRSP] = -5.*D2R;
    WBIK_Q0[idLSP] = -5.*D2R;

    WBIK_Q0[idRSR] = 10.*D2R;
    WBIK_Q0[idLSR] = -10.*D2R;

    WBIK_Q0[idRSY] = 0.*D2R;
    WBIK_Q0[idLSY] = 0.*D2R;

    WBIK_Q0[idREB] = -130.*D2R;
    WBIK_Q0[idLEB] = -130.*D2R;

    WBIK_Q0[idRWY] = 0.*D2R;
    WBIK_Q0[idLWY] = 0.*D2R;

    WBIK_Q0[idRWP] = 20.*D2R;
    WBIK_Q0[idLWP] = 20.*D2R;
    WBIK_Q0[idRWY2] = 0;//20.*D2R;
    WBIK_Q0[idLWY2] = 0;//20.*D2R;
    WBIK_Q0[idWST] = 0;//-180*D2R;//20.*D2R;

    WBIK_PARA_CHANGE();

    cout << "IK Version: " << kine_drc_hubo4.get_version() << endl;

    des_pCOM_3x1[0] = userData->WalkReadyCOM[0] = 0.0;//0.0237f;
    des_pCOM_3x1[1] = userData->WalkReadyCOM[1] = 0.0;
    des_pCOM_3x1[2] = userData->WalkReadyCOM[2] = 0.77;// + 0.11;//(61.8kg:0.74)//59//0.8;//71kg

    des_qPEL_4x1[0] = 1.;
    des_qPEL_4x1[1] = 0.;
    des_qPEL_4x1[2] = 0.;
    des_qPEL_4x1[3] = 0.;

    des_pRF_3x1[0] = 0.;
    des_pRF_3x1[1] = -kine_drc_hubo4.L_PEL2PEL/2.;//-0.13;//-kine_drc_hubo4.L_PEL2PEL/2;//-0.135;//
    des_pRF_3x1[2] = 0.;

    des_qRF_4x1[0] = 1.;
    des_qRF_4x1[1] = 0.;
    des_qRF_4x1[2] = 0.;
    des_qRF_4x1[3] = 0.;

    des_pLF_3x1[0] = 0.;
    des_pLF_3x1[1] = kine_drc_hubo4.L_PEL2PEL/2.;//0.13;//kine_drc_hubo4.L_PEL2PEL/2;//0.135;//
    des_pLF_3x1[2] = 0.;

    des_qLF_4x1[0] = 1.;
    des_qLF_4x1[1] = 0.;
    des_qLF_4x1[2] = 0.;
    des_qLF_4x1[3] = 0.;

    get_WBIK_Q_from_RefAngleCurrent();

    printf("First Init Right WBIK_Q : %f  %f  %f  %f  %f  %f \n",WBIK_Q[7],WBIK_Q[8],WBIK_Q[9],WBIK_Q[10],WBIK_Q[11],WBIK_Q[12]);
    printf("First Init Left  WBIK_Q : %f  %f  %f  %f  %f  %f \n",WBIK_Q[13],WBIK_Q[14],WBIK_Q[15],WBIK_Q[16],WBIK_Q[17],WBIK_Q[18]);

    printf("COM : %f  %f  %f \n",des_pCOM_3x1[0],des_pCOM_3x1[1],des_pCOM_3x1[2]);
    printf("qPel : %f  %f  %f  %f \n",des_qPEL_4x1[0],des_qPEL_4x1[1],des_qPEL_4x1[2],des_qPEL_4x1[3]);
    printf("pRF : %f  %f  %f \n",des_pRF_3x1[0],des_pRF_3x1[1],des_pRF_3x1[2]);
    printf("qRF : %f  %f  %f  %f \n",des_qRF_4x1[0],des_qRF_4x1[1],des_qRF_4x1[2],des_qRF_4x1[3]);
    printf("pLF : %f  %f  %f \n",des_pLF_3x1[0],des_pLF_3x1[1],des_pLF_3x1[2]);
    printf("qLF : %f  %f  %f  %f \n",des_qLF_4x1[0],des_qLF_4x1[1],des_qLF_4x1[2],des_qLF_4x1[3]);

    kine_drc_hubo4.IK_LowerBody_Global(WBIK_Q0,Qub,des_pCOM_3x1, des_qPEL_4x1, des_pRF_3x1, des_qRF_4x1, des_pLF_3x1, des_qLF_4x1,WBIK_Q);

    printf("First Init Right WBIK_Q : %f  %f  %f  %f  %f  %f \n",WBIK_Q[7],WBIK_Q[8],WBIK_Q[9],WBIK_Q[10],WBIK_Q[11],WBIK_Q[12]);
    printf("First Init Left  WBIK_Q : %f  %f  %f  %f  %f  %f \n",WBIK_Q[13],WBIK_Q[14],WBIK_Q[15],WBIK_Q[16],WBIK_Q[17],WBIK_Q[18]);


    get_WBIK_Q_from_RefAngleCurrent();

    //----------------------------------- Reset COM to 0
    WBIK_Q[idQ0] = 1;
    WBIK_Q[idQ1] = 0;
    WBIK_Q[idQ2] = 0;
    WBIK_Q[idQ3] = 0;

    // PELVIS Position Reset
    kine_drc_hubo4.FK_COM_Global(WBIK_Q,FK_pCOM_3x1);
    printf("First Initialize FK com = %f,%f,%f,WBIK_Q[idXYZ] = %f,%f,%f\n",FK_pCOM_3x1[0],FK_pCOM_3x1[1],FK_pCOM_3x1[2],WBIK_Q[idX],WBIK_Q[idY],WBIK_Q[idZ]);

    // F.K

    WBIK_Q[idX] = WBIK_Q[idX] - FK_pCOM_3x1[0];//reset to 0;
    WBIK_Q[idY] = WBIK_Q[idY] - FK_pCOM_3x1[1];//reset to 0;
    WBIK_Q[idZ] = WBIK_Q[idZ] - FK_pCOM_3x1[2] + userData->WalkReadyCOM[2];// + fsm->AddComInfos[0][2];//0;

    kine_drc_hubo4.FK_RightFoot_Global(WBIK_Q,FK_pRFoot_3x1,  FK_qRFoot_4x1);
    kine_drc_hubo4.FK_LeftFoot_Global(WBIK_Q,FK_pLFoot_3x1,  FK_qLFoot_4x1);
    kine_drc_hubo4.FK_COM_Global(WBIK_Q,FK_pCOM_3x1);

    printf("222Fisrt Iniitalize  FK com = %f,%f,%f,WBIK_Q[idXYZ] = %f,%f,%f\n",FK_pCOM_3x1[0],FK_pCOM_3x1[1],FK_pCOM_3x1[2],WBIK_Q[idX],WBIK_Q[idY],WBIK_Q[idZ]);

    printf("FK right pos  %f, %f, %f \n",FK_pRFoot_3x1[0],FK_pRFoot_3x1[1],FK_pRFoot_3x1[2]);
    printf("FK right ori  %f, %f, %f \n",FK_qRFoot_4x1[0],FK_qRFoot_4x1[1],FK_qRFoot_4x1[2],FK_qRFoot_4x1[3]);

    printf("FK left  pos  %f, %f, %f \n",FK_pLFoot_3x1[0],FK_pLFoot_3x1[1],FK_pLFoot_3x1[2]);
    printf("FK left  ori  %f, %f, %f \n",FK_qLFoot_4x1[0],FK_qLFoot_4x1[1],FK_qLFoot_4x1[2],FK_qLFoot_4x1[3]);

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

    double PelYaw,PelRoll,PelPitch,temp1_qPel_4x1[4],temp2_qPel_4x1[4],temp3_qPel_4x1[4],temp4_qPel_4x1[4],temp5_qPel_4x1[4];
    PelYaw = 0*D2R;
    PelRoll = 0*D2R;
    PelPitch = 0*D2R;

    qtRZ(PelYaw, temp1_qPel_4x1);
    qtRX(PelRoll, temp2_qPel_4x1);
    qtRY(PelPitch, temp3_qPel_4x1);

    QTcross(temp1_qPel_4x1,temp2_qPel_4x1,temp4_qPel_4x1);
    QTcross(temp4_qPel_4x1,temp3_qPel_4x1,temp5_qPel_4x1);

    WBIK_Q[idQ0] = temp5_qPel_4x1[0];//1;
    WBIK_Q[idQ1] = temp5_qPel_4x1[1];//0;
    WBIK_Q[idQ2] = temp5_qPel_4x1[2];//0;
    WBIK_Q[idQ3] = temp5_qPel_4x1[3];//0;

    // PELVIS Position Reset
    kine_drc_hubo4.FK_COM_Global(WBIK_Q,FK_pCOM_3x1);
    kine_drc_hubo4.FK_RightFoot_Global(WBIK_Q,FK_pRFoot_3x1,  FK_qRFoot_4x1);
    kine_drc_hubo4.FK_LeftFoot_Global(WBIK_Q,FK_pLFoot_3x1,  FK_qLFoot_4x1);
    printf("FK3 com = %f,%f,%f,PEL = %f,%f,%f,RF = %f,%f,%f,LF = %f,%f,%f\n",FK_pCOM_3x1[0],FK_pCOM_3x1[1],FK_pCOM_3x1[2],WBIK_Q[idX],WBIK_Q[idY],WBIK_Q[idZ],FK_pRFoot_3x1[0],FK_pRFoot_3x1[1],FK_pRFoot_3x1[2],FK_pLFoot_3x1[0],FK_pLFoot_3x1[1],FK_pLFoot_3x1[2]);

    printf("init_WBIK_pCOM : (%f,%f,%f),init_WBIK_Q : (%f,%f,%f)\n",init_WBIK_pCOM[0],init_WBIK_pCOM[1],init_WBIK_pCOM[2],init_WBIK_Q[0],init_WBIK_Q[1],init_WBIK_Q[2]);


    WBIK_Q[idX] = WBIK_Q[idX] - FK_pCOM_3x1[0];//reset to 0;
    WBIK_Q[idY] = WBIK_Q[idY] - FK_pCOM_3x1[1];//reset to 0;
    WBIK_Q[idZ] = WBIK_Q[idZ] - FK_pCOM_3x1[2] + userData->WalkReadyCOM[2];


    printf("========================\n");
    printf("PEL : %f,%f,%f,%f,%f,%f,%f\n",WBIK_Q[idX],WBIK_Q[idY],WBIK_Q[idZ],WBIK_Q[idQ0],WBIK_Q[idQ1],WBIK_Q[idQ2],WBIK_Q[idQ3]);
    printf("RLEG : %f,%f,%f,%f,%f,%f\n",WBIK_Q[idRHY],WBIK_Q[idRHR],WBIK_Q[idRHP],WBIK_Q[idRKN],WBIK_Q[idRAP],WBIK_Q[idRAR]);
    printf("LLEG : %f,%f,%f,%f,%f,%f\n",WBIK_Q[idLHY],WBIK_Q[idLHR],WBIK_Q[idLHP],WBIK_Q[idLKN],WBIK_Q[idLAP],WBIK_Q[idLAR]);

    printf("RARM : %f,%f,%f,%f,%f,%f,%f\n",WBIK_Q[idRSP],WBIK_Q[idRSR],WBIK_Q[idRSY],WBIK_Q[idREB],WBIK_Q[idRWP],WBIK_Q[idRWY],WBIK_Q[idRWY2]);
    printf("LARM : %f,%f,%f,%f,%f,%f,%f\n",WBIK_Q[idLSP],WBIK_Q[idLSR],WBIK_Q[idLSY],WBIK_Q[idLEB],WBIK_Q[idLWP],WBIK_Q[idLWY],WBIK_Q[idLWY2]);
    printf("========================\n");


    kine_drc_hubo4.FK_RightFoot_Global(WBIK_Q,FK_pRFoot_3x1,  FK_qRFoot_4x1);
    kine_drc_hubo4.FK_LeftFoot_Global(WBIK_Q,FK_pLFoot_3x1,  FK_qLFoot_4x1);
    kine_drc_hubo4.FK_COM_Global(WBIK_Q,FK_pCOM_3x1);
    printf("FK com = %f,%f,%f,PEL = %f,%f,%f,\nRF = %f,%f,%f,LF = %f,%f,%f\n",FK_pCOM_3x1[0],FK_pCOM_3x1[1],FK_pCOM_3x1[2],WBIK_Q[idX],WBIK_Q[idY],WBIK_Q[idZ],FK_pRFoot_3x1[0],FK_pRFoot_3x1[1],FK_pRFoot_3x1[2],FK_pLFoot_3x1[0],FK_pLFoot_3x1[1],FK_pLFoot_3x1[2]);


    //Set Foot print Initial value


    QT2YPR(FK_qRFoot_4x1,FK_RFoot_yaw,FK_RFoot_pitch,FK_RFoot_roll);

    QT2YPR(FK_qLFoot_4x1,FK_LFoot_yaw,FK_LFoot_pitch,FK_LFoot_roll);


    Init_Right_Leg = GLOBAL_Z_RF = GLOBAL_Z_RF_last[0] = FK_pRFoot_3x1[2];//fsm->RightInfos[0][2];
    Init_Left_Leg  = GLOBAL_Z_LF = GLOBAL_Z_LF_last[0] = FK_pLFoot_3x1[2];//fsm->LeftInfos[0][2];



        des_pRF_3x1[0] =FK_pRFoot_3x1[0];
        des_pRF_3x1[1] =FK_pRFoot_3x1[1];
        des_pRF_3x1[2] =FK_pRFoot_3x1[2];

        des_pLF_3x1[0] =FK_pLFoot_3x1[0];
        des_pLF_3x1[1] =FK_pLFoot_3x1[1];
        des_pLF_3x1[2] =FK_pLFoot_3x1[2];

        des_pCOM_3x1[0] = FK_pCOM_3x1[0];
        des_pCOM_3x1[1] = FK_pCOM_3x1[1];
        des_pCOM_3x1[2] = FK_pCOM_3x1[2];

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
    des_pRF_3x1[0] =0;
    des_pRF_3x1[1] =0;
    des_pRF_3x1[2] =0;

    des_pLF_3x1[0] =0;
    des_pLF_3x1[1] =0;
    des_pLF_3x1[2] =0;


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
    RDPitch = RMYC(2,2,0,0.,0.,0./1000.0,0./1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],0.,0.);
    LDPitch = LMYC(2,2,0,0.,0.,0./1000.0,0./1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],0.,0.);
    RDPitch =0.;
    LDPitch =0.;
    RDRoll = RMXC(2,2,0,0.,0.,0./1000.0,0./1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],0.,0.);
    LDRoll = LMXC(2,2,0,0.,0.,0./1000.0,0./1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],0.,0.);
    RDRoll =0.;
    LDRoll =0.;

    RDPitch2 = RMYC2(2,2,0,0.,0.,0./1000.0,0./1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],0.,0.);
    LDPitch2= LMYC2(2,2,0,0.,0.,0./1000.0,0./1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],0.,0.);
    RDPitch2 =0.;
    LDPitch2 =0.;


    RDRoll2 = RMXC2(2,2,0,0.,0.,0./1000.0,0./1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],0.,0.);
    LDRoll2= LMXC2(2,2,0,0.,0.,0./1000.0,0./1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],0.,0.);
    RDRoll2 =0.;
    LDRoll2 =0.;


}

void Walking_initialize_1st()
{

    get_WBIK_Q_from_RefAngleCurrent();

    WBIK_Q[idQ0] = 1;
    WBIK_Q[idQ1] = 0;
    WBIK_Q[idQ2] = 0;
    WBIK_Q[idQ3] = 0;

    // PELVIS Position Reset
    kine_drc_hubo4.FK_COM_Global(WBIK_Q,FK_pCOM_3x1);
    printf("FK com = %f,%f,%f,WBIK_Q[idXYZ] = %f,%f,%f\n",FK_pCOM_3x1[0],FK_pCOM_3x1[1],FK_pCOM_3x1[2],WBIK_Q[idX],WBIK_Q[idY],WBIK_Q[idZ]);

    init_WBIK_pCOM[0] = FK_pCOM_3x1[0];
    init_WBIK_pCOM[1] = FK_pCOM_3x1[1];
    init_WBIK_pCOM[2] = FK_pCOM_3x1[2];

    init_WBIK_Q[0] = WBIK_Q[idX];
    init_WBIK_Q[1] = WBIK_Q[idY];
    init_WBIK_Q[2] = WBIK_Q[idZ];

    WBIK_Q[idX] = WBIK_Q[idX] - FK_pCOM_3x1[0];//reset to 0;
    WBIK_Q[idY] = WBIK_Q[idY] - FK_pCOM_3x1[1];//reset to 0;
    WBIK_Q[idZ] = WBIK_Q[idZ] - FK_pCOM_3x1[2] + userData->WalkReadyCOM[2];// + fsm->AddComInfos[0][2];//0;

    kine_drc_hubo4.FK_RightFoot_Global(WBIK_Q,FK_pRFoot_3x1,  FK_qRFoot_4x1);
    kine_drc_hubo4.FK_LeftFoot_Global(WBIK_Q,FK_pLFoot_3x1,  FK_qLFoot_4x1);
    kine_drc_hubo4.FK_COM_Global(WBIK_Q,FK_pCOM_3x1);

    printf("FK com = %f,%f,%f,WBIK_Q[idXYZ] = %f,%f,%f\n",FK_pCOM_3x1[0],FK_pCOM_3x1[1],FK_pCOM_3x1[2],WBIK_Q[idX],WBIK_Q[idY],WBIK_Q[idZ]);


    printf("FK_pRFoot_3x1[0] = %f, FK_pRFoot_3x1[1] = %f, FK_pRFoot_3x1[2] = %f,FK_pLFoot_3x1[0] = %f, FK_pLFoot_3x1[1] = %f, FK_pLFoot_3x1[2] = %f\n",FK_pRFoot_3x1[0],FK_pRFoot_3x1[1],FK_pRFoot_3x1[2],FK_pLFoot_3x1[0],FK_pLFoot_3x1[1],FK_pLFoot_3x1[2]);

    GLOBAL_Z_RF = 0;//FK_pRFoot_3x1[2];//fsm->RightInfos[0][2];
    GLOBAL_Z_LF = 0;//FK_pLFoot_3x1[2];//fsm->LeftInfos[0][2];
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
       WBIK_Q[i+7] = jCon->Joints[i]->RefAngleCurrent*D2R;

    }
    WBIK_Q[idWST] = jCon->GetJointRefAngle(WST)*D2R;
    WBIK_Q[idRSP] = jCon->GetJointRefAngle(RSP)*D2R;
    WBIK_Q[idRSR] = (jCon->GetJointRefAngle(RSR)+OFFSET_RSR)*D2R;
    WBIK_Q[idRSY] = jCon->GetJointRefAngle(RSY)*D2R;
    WBIK_Q[idREB] = (jCon->GetJointRefAngle(REB)+OFFSET_ELB)*D2R;
    WBIK_Q[idRWY] = jCon->GetJointRefAngle(RWY)*D2R;
    WBIK_Q[idRWP] = jCon->GetJointRefAngle(RWP)*D2R;
    WBIK_Q[idRWY2] = jCon->GetJointRefAngle(RWY2)*D2R;

    WBIK_Q[idLSP] = jCon->GetJointRefAngle(LSP)*D2R;
    WBIK_Q[idLSR] = (jCon->GetJointRefAngle(LSR)+OFFSET_LSR)*D2R;
    WBIK_Q[idLSY] = jCon->GetJointRefAngle(LSY)*D2R;
    WBIK_Q[idLEB] = (jCon->GetJointRefAngle(LEB)+OFFSET_ELB)*D2R;
    WBIK_Q[idLWY] = jCon->GetJointRefAngle(LWY)*D2R;
    WBIK_Q[idLWP] = jCon->GetJointRefAngle(LWP)*D2R;
    WBIK_Q[idLWY2] = jCon->GetJointRefAngle(LWY2)*D2R;

    Qub[idRSR] = WBIK_Q[idRSR];
    Qub[idRSP] = WBIK_Q[idRSP];
    Qub[idRSY] = WBIK_Q[idRSY];
    Qub[idREB] = WBIK_Q[idREB];
    Qub[idRWY] = WBIK_Q[idRWY];
    Qub[idRWP] = WBIK_Q[idRWP];
    Qub[idRWY] = WBIK_Q[idRWY];
    Qub[idRWY2] = WBIK_Q[idRWY2];

    Qub[idLSR] = WBIK_Q[idLSR];
    Qub[idLSP] = WBIK_Q[idLSP];
    Qub[idLSY] = WBIK_Q[idLSY];
    Qub[idLEB] = WBIK_Q[idLEB];
    Qub[idLWY] = WBIK_Q[idLWY];
    Qub[idLWP] = WBIK_Q[idLWP];
    Qub[idLWY] = WBIK_Q[idLWY];
    Qub[idLWY2] = WBIK_Q[idLWY2];

    Qub[idWST] = WBIK_Q[idWST];

}

double BasicTrajectory(double count, int mode)
{
    if(count > 1.) count = 1.;
    else if(count < 0.) count = 0.;

    if(mode == MODE_ONEtoZERO) return 1./2.*(cos(count*PI) + 1.);
    else return 1./2.*(1. - cos(count*PI));
}


//double pCenter[3],qCenter[4],qCenter_bar[4];
double temp1[3];
void Global2Local(double _Global[],double _Local[])
{


    // Foot Center in Global Coord.
    pCenter[0] = (des_pRF_3x1[0] + des_pLF_3x1[0])/2.;
    pCenter[1] = (des_pRF_3x1[1] + des_pLF_3x1[1])/2.;
    pCenter[2] = (des_pRF_3x1[2] + des_pLF_3x1[2])/2.;

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
    pCenter[0] = (des_pRF_3x1[0] + des_pLF_3x1[0])/2.;
    pCenter[1] = (des_pRF_3x1[1] + des_pLF_3x1[1])/2.;
    pCenter[2] = (des_pRF_3x1[2] + des_pLF_3x1[2])/2.;

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
    pCenter[0] = (des_pRF_3x1[0] + des_pLF_3x1[0])/2.;
    pCenter[1] = (des_pRF_3x1[1] + des_pLF_3x1[1])/2.;
    pCenter[2] = (des_pRF_3x1[2] + des_pLF_3x1[2])/2.;

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
            Save_Data[18][Save_Index] = des_pCOM_3x1[0];
            Save_Data[19][Save_Index] = des_pCOM_3x1[1];
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

            Save_Data[42][Save_Index] = des_pRF_3x1[2];
            Save_Data[43][Save_Index] = des_pLF_3x1[2];

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


            Save_Data[98][Save_Index] = des_pRF_3x1[0];
            Save_Data[99][Save_Index] = des_pRF_3x1[1];

            Save_Data[100][Save_Index] = des_pLF_3x1[0];
            Save_Data[101][Save_Index] = des_pLF_3x1[1];

            Save_Data[102][Save_Index] = des_pCOM_3x1[0];
            Save_Data[103][Save_Index] = des_pCOM_3x1[1];

            Save_Data[104][Save_Index] = WBIK_Q[idRHP];

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


            Save_Data[118][Save_Index] = des_pCOM_3x1[0];
            Save_Data[119][Save_Index] = des_pCOM_3x1[1];

            Save_Data[120][Save_Index] = (- 0.001*Del_PC_X_DSP_XZMP_CON + I_ZMP_CON_X*1.);
            Save_Data[121][Save_Index] = (- 0.001*Del_PC_Y_DSP_YZMP_CON + I_ZMP_CON_Y*0.);

            Save_Data[122][Save_Index] = GLOBAL_X_LIPM_n;

            Save_Data[123][Save_Index] = G_DSP_X;

            Save_Data[124][Save_Index] = OnOff_compliance;
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


            Save_Data[154][Save_Index] = FWRefAngleCurrent[RAP];
            Save_Data[155][Save_Index] = FWRefAngleCurrent[RAR];
            Save_Data[156][Save_Index] = FWRefAngleCurrent[LAP];
            Save_Data[157][Save_Index] = FWRefAngleCurrent[LAR];

            Save_Data[158][Save_Index] = WBIK_Q[RAP+7];
            Save_Data[159][Save_Index] = WBIK_Q[RAR+7];
            Save_Data[160][Save_Index] = WBIK_Q[LAP+7];
            Save_Data[161][Save_Index] = WBIK_Q[LAR+7];

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

            Save_Data[172][Save_Index] = des_pRF_3x1_hat[Xdir];
            Save_Data[173][Save_Index] = des_pRF_3x1_hat[Ydir];
            Save_Data[174][Save_Index] = des_pRF_3x1_hat[Zdir];

            Save_Data[175][Save_Index] = des_pLF_3x1_hat[Xdir];
            Save_Data[176][Save_Index] = des_pLF_3x1_hat[Ydir];
            Save_Data[177][Save_Index] = des_pLF_3x1_hat[Zdir];

            Save_Index++;

            if(Save_Index >= ROW) Save_Index = 0;


    }
}


