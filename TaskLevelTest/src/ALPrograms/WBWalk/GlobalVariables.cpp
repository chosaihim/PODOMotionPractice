


/**************************** Variables Definitions (Basic) ***********************/
/* Define */
    int         __IS_WORKING = false;
    int         __IS_GAZEBO = false;

    double      FINAL_TIMER = 0.;
    int         PODO_NO = -1;

/**************************** 3. Variables Definitions (Trajectory) ******************/
/* Global Variable For Task Trajecotry (CoM, Foot, Pelvis ori...etc) */
    double des_pCOM_3x1[3]={0,},des_pCOM_3x1_hat[3]={0,},des_pCOM_3x1_LPF[3]={0,}, des_pPCz, des_qPEL_4x1[4]={1,0,0,0}, des_pRF_3x1[3], des_pRF_3x1_hat[3], des_qRF_4x1[4]={1,0,0,0}, des_qRF_4x1_hat[4]={1,0,0,0}, des_pLF_3x1[3],des_pLF_3x1_hat[3], des_qLF_4x1[4]={1,0,0,0}, des_qLF_4x1_hat[4]={1,0,0,0};
    double des_pRH_3x1[3]={0,},des_pLH_3x1[3]={0,},des_qRH_4x1[4]={0,},des_qLH_4x1[4]={0,},des_RElb_ang=0,des_LElb_ang=0,RH_ref_frame=2,LH_ref_frame=2,des_wst_ang=0;
    double FK_pCOM_3x1[3]={0,},FK_pRFoot_3x1[3]={0,},FK_qRFoot_4x1[4]={1,0,0,0},FK_pLFoot_3x1[3]={0,},FK_qLFoot_4x1[4]={1,0,0,0};
    double FK_pRHand_3x1[3]={0,},FK_pLHand_3x1[3]={0,},FK_qRHand_4x1[4]={1,0,0,0},FK_qLHand_4x1[4]={1,0,0,0};
    double FK_REB_ang=0, FK_LEB_ang=0, FK_WST_ang=0;
    double init_WBIK_pCOM[3] = {0,},init_WBIK_Q[3] = {0,};
    double FK_RFoot_yaw = 0.,FK_RFoot_pitch= 0.,FK_RFoot_roll= 0.,FK_LFoot_yaw = 0.,FK_LFoot_pitch= 0.,FK_LFoot_roll= 0.;

    double GLOBAL_Z_LF_last_earlylanding = 0.0f,GLOBAL_Z_RF_last_earlylanding=0.0f;
    double GLOBAL_X_LIPM = 0.0f,GLOBAL_X_LF = 0.0f,GLOBAL_X_RF = 0.0f,GLOBAL_X_LIPM_d = 0.0f;
    double GLOBAL_X_LIPM_n = 0.0f,GLOBAL_X_LF_n = 0.0f,GLOBAL_X_RF_n = 0.0f,GLOBAL_X_LIPM_d_n = 0.0f;
    double GLOBAL_Y_LIPM_n = 0.0f,GLOBAL_Y_LF_n = 0.0f,GLOBAL_Y_RF_n = 0.0f,GLOBAL_Y_LIPM_d_n = 0.0f;
    double GLOBAL_X_LIPM_last =0.0f,GLOBAL_ZMP_REF_X_last=0.0f;
    double GLOBAL_X_LF_last = 0.0f,GLOBAL_X_RF_last = 0.0f;
    double GLOBAL_Y_LF = 0.0f,GLOBAL_Y_RF = 0.0f;

    double GLOBAL_ZMP_REF_Y=0.0f,GLOBAL_ZMP_REF_X=0.0f,GLOBAL_ZMP_REF_X_local=0.0f;

    double GLOBAL_ZMP_REF_Y_n=0.0f,GLOBAL_ZMP_REF_X_n=0.0f;
    double GLOBAL_Y_LIPM = 0.0f,GLOBAL_Y_LIPM_d = 0.0f;
    double GLOBAL_Z_LIPM = 0.0f,GLOBAL_Z_LIPM2 = 0.0f,GLOBAL_Z_LF = 0.0f,GLOBAL_Z_RF = 0.0f,GLOBAL_Z_LIPM_last=0.0f;
    double GLOBAL_Z_LF_last[3] = {0.0f,},GLOBAL_Z_RF_last[3]={0.0f,};
    double GLOBAL_Z_LF_goal[3] = {0.0f,},GLOBAL_Z_RF_goal[3]={0.0f,};
    double GLOBAL_Z_LF_last2 = 0.0f,GLOBAL_Z_RF_last2=0.0f;
    double Init_Right_Leg = 0.,Init_Left_Leg = 0.;
    double Pel_Yaw=0;

/* Global Variable For Joint Trajecotry (Joint angle) */
    double WBIK_Q[34] = {0.,},Qub[34]={0.,},WBIK_Q0[34] = {0.,};
    double FWRefAngleCurrent[NO_OF_JOINTS] = {0.,};

/* Ugain */
    double U_Gain  = 0.;
    double U0_Gain  = 0.8;
    double U3_Gain = 0.;
    double U_Gain_DSP = 1.;
    double U0_Gain_KI  = 0.,U0_Gain_KI_last = 0.;
    double U0_Gain_Goal_KI  = 0.,U0_Gain_Goal_KI_last = 0.;
    double G_DSP_X = 1.,G_DSP_Y=1.,G_DSP_X_last = 1.,G_DSP_Y_last=1.;

/* inverse model variable */
    double JW_InvPattern_l,JW_InvPattern_l2;
    double JW_InvPattern_Klqr[2]={0.f,0.f};
    double JW_InvPattern_U[2]  = {0.f,0.f};
    double JW_InvPattern_U_n[2]  = {0.f,0.f};
    double JW_InvPattern_U_I[2]  = {0.f,0.f};
    double JW_InvPattern_A[2][2] ={{0.f,0.f},{0.f,0.f}};
    double JW_InvPattern_A_X[2][2] ={{0.f,0.f},{0.f,0.f}};
    double JW_InvPattern_B[2] = {0.f,0.f};
    double JW_InvPattern_B_X[2] = {0.f,0.f};

    double JW_InvPattern_k=5550.f;
    double JW_InvPattern_k_X=9000.f;
    double JW_InvPattern_c=60.f;
    double JW_InvPattern_c_X=80.f;
    double JW_InvPattern_m=80.f;

    double JW_InvPattern_X_d[2] ={0.f,0.f};
    double JW_InvPattern_X[2] ={0.f,0.f};
    double JW_InvPattern_X_old[2] ={0.f,0.f};

    double JW_InvPattern_Y_d[2] ={0.f,0.f};
    double JW_InvPattern_Y[2] ={0.f,0.f};
    double JW_InvPattern_Y_old[2] ={0.f,0.f};

    double Y_inv=0.,Y_inv_d=0.,Y_inv_old=0.,theta_ref=0.,theta_dd=0.,theta_d=0.,theta=0.;
    double Y_inv2=0.,Y_inv_d2=0.,Y_inv_old2=0.,theta2_ref=0.,theta2_dd=0.,theta2_d=0.,theta2=0.;
    double U[2]={0.f,0.f},U_I[2]={0.f,0.f};

/* Variable for pattern generator */
    int pv_Index = 0;
    double pv_Gd[301],pv_Gx[4],pv_Gi[2];
    double pv_A[3][3],pv_B[3][1],pv_C[1][3];
    double CONT_X,CONT_Y,CONT_X_n,CONT_Y_n;
    double _temp_debug_data[50]={0.0f,},temp_debug[20]={0.0f,};

/* Variable for MPC */
    double MPC_alpha = 150.0,MPC_beta = 100000.0,MPC_gamma = 1.0,MPC_mu = 1000000.0;
    double Pps[15][3] = {{0.,},},Pvs[15][3] = {{0.,},},Pzs[15][3] = {{0.,},};
    double Ppu[15][15] = {{0.,},}, Pvu[15][15] = {{0.,},}, Pzu[15][15] = {{0.,},}, Pzu_T[15][15] = {{0.,},}, PzuPzu_T[15][15] = {{0.,},};
    double Pzu_invxtrans[15][15]= {{0.,},},Pzu_inv[15][15]= {{0.,},},MPC_pc[15][3]= {{0.,},},MPC_D[30][15]= {{0.,},},Pvu_trans[15][15]= {{0.,},},Pzu_inv_trans[15][15]= {{0.,},},temp_mat[15][15]= {{0.,},},temp_mat3[15][15]= {{0.,},},temp_mat2[15][15]= {{0.,},},t_mat[15][3]= {{0.,},},t_mat2[15][3]= {{0.,},},t_mat3[15][3]= {{0.,},};
    double MPC_h = 0.77,MPC_g = 9.81,MPC_T = 0.1;
    double MPC_ysep = 0.15,MPC_ystr = 0.35,MPC_sp[2] = {0.07,0.04};
    int MPC_time = 15,MPC_m = 3;
    double MPC_B[3] = {0.0,},MPC_C[3] = {0.,},MPC_A[3][3] = {{0.0,},},MPC_Q[18][18] = {{0.0,},},MPC_Ci[32][18]={{0.,},},MPC_ci[32]={0.,};

    double M_LF[3],M_RF[3],M_LF_Global[3],M_RF_Global[3],F_LF[3],F_RF[3],F_LF_Global[3],F_RF_Global[3];
    double pCenter[3],qCenter[4],qCenter_bar[4];
    double zmp[3],zmp_local[3],zmp_ref_local[3];

    /* Debug variables (temp) */
        double First_FKcom[3];
        double Second_FKcom[3];
        double First_FKRH[3];
        double First_FKLH[3];

        double Init_FKcom[3];
        double Init_FKRH[3];
        double Init_FKLH[3];
        double Init2_FKcom[3];
        double Init2_FKRH[3];
        double Init2_FKLH[3];

        double FKcom[3];
        double FK0com[3];
        double FKrh[3];


/**************************** 4. Variables Definitions (Controller) ******************/
/* Stabilizer Variable */
    double  AnkleControl1=0,AnkleControl2=0;
    double  FOGRollVel_LPF=0,FOGPitchVel_LPF=0;
    double  FOGRollVel_NF=0,FOGPitchVel_NF=0;
    double  FOGRollVel_NF2=0,FOGPitchVel_NF2=0;
    double  AngleRoll =0.0 ,AngleVel =0.0 ,AngleRoll_last =0.0;
    double  LPF_AngleRoll =0.0 ,LPF_AngleVel =0.0 ,LPF_AngleRoll_last =0.0;
    double  den_a1,den_a2,den_a3,den_a4,den_a5,num_b1,num_b2,num_b3,num_b4,num_b5;
    double  den_a11,den_a21,den_a31,den_a41,den_a51,num_b11,num_b21,num_b31,num_b41,num_b51;
    double  u_i_4 = 0,u_i_3 = 0, u_i_2 = 0, u_i_1 = 0, u_i = 0;
    double  y_i_4 = 0,y_i_3 = 0, y_i_2 = 0, y_i_1 = 0, y_i = 0;
    double  u_i_41 = 0, u_i_31 = 0, u_i_21 = 0, u_i_11 = 0, u_i1 = 0;
    double  y_i_41 = 0, y_i_31 = 0, y_i_21 = 0, y_i_11 = 0, y_i1 = 0;

/* Kirk Variable */
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

/* Reactive Control variable */
    double ALPHA = 0.,RDF=400,LDF = 400;
    double Zctrl=0.,Zctrl2;
    double RDPitch=0.,RDPitch2=0.,RDRoll=0.,LDPitch=0.,LDPitch2=0.,LDRoll=0.,LDRoll2=0.,RDRoll2=0.;

/* Gyro feedback variable */
    double GLOBAL_Xori_LF = 0.,GLOBAL_Yori_LF = 0.,GLOBAL_Zori_LF = 0.;
    double GLOBAL_Xori_LF_n = 0.,GLOBAL_Yori_LF_n = 0.,GLOBAL_Zori_LF_n = 0.;
    double sum_GLOBAL_Xori_LF_n = 0.,sum_GLOBAL_Yori_LF_n = 0.;
    double ave_GLOBAL_Xori_LF_n = 0.,ave_GLOBAL_Yori_LF_n = 0.;
    double GLOBAL_Xori_LF2 = 0.,GLOBAL_Yori_LF2 = 0.,GLOBAL_Zori_LF2 = 0.;
    double GLOBAL_Xori_LF_last = 0.,GLOBAL_Yori_LF_last = 0.,GLOBAL_Zori_LF_last = 0.;
    double GLOBAL_Xori_LF_last2 = 0.,GLOBAL_Yori_LF_last2 = 0.,GLOBAL_Zori_LF_last2 = 0.;
    double GLOBAL_Xori_LF2_last = 0.,GLOBAL_Yori_LF2_last = 0.,GLOBAL_Zori_LF2_last = 0.;
    double GLOBAL_Xori_RF = 0.,GLOBAL_Yori_RF = 0.,GLOBAL_Zori_RF = 0.;
    double GLOBAL_Xori_RF_n = 0.,GLOBAL_Yori_RF_n = 0.,GLOBAL_Zori_RF_n = 0.;
    double sum_GLOBAL_Xori_RF_n = 0.,sum_GLOBAL_Yori_RF_n = 0.;
    double ave_GLOBAL_Xori_RF_n = 0.,ave_GLOBAL_Yori_RF_n = 0.;
    double GLOBAL_Xori_RF2 = 0.,GLOBAL_Yori_RF2 = 0.,GLOBAL_Zori_RF2 = 0.;
    double GLOBAL_Xori_RF_last = 0.,GLOBAL_Yori_RF_last = 0.,GLOBAL_Zori_RF_last = 0.;
    double GLOBAL_Xori_RF_last2 = 0.,GLOBAL_Yori_RF_last2 = 0.,GLOBAL_Zori_RF_last2 = 0.;
    double GLOBAL_Xori_RF2_last = 0.,GLOBAL_Yori_RF2_last = 0.,GLOBAL_Zori_RF2_last = 0.;
    double Foot_gainLF=0,Foot_gainRF=0;

/* Landing Control variable */
    int EarlyLandingFlag[2]={0,};
    int LandingState = FINAL;
    int Pre_LandingState =-1;

/* leg length control */
    double Add_FootTask[2][3]={{0.,},},Add_Leg_Recovery[2][3]={{0.,},};
    double BTW_FOOT_Angle=0,BTW_FOOT_Angle_roll=0,BTW_FOOT_Angle_pitch =0,BTW_FOOT_Angle_yaw=0,BTW_FOOT_qPEL_comp_4x1[4]={1,0,0,0};
    double BTW_PEL_angle_roll = 0,BTW_PEL_angle_pitch = 0,BTW_PEL_angle_yaw =0;
    double BTW_PEL_angle_roll_vel = 0,BTW_PEL_angle_pitch_vel = 0,BTW_PEL_angle_yaw_vel =0;


/* deflection compensator variable */
    double deflection_comp_RAR=0.,deflection_comp_LAR=0.;

/* pelvis yaw control */
    double AngularMomentumComp(double velx,double torso_yaw,int sign, int reset);
    double Yaw_Gain = 2.15f,Yaw_Gain2 = 0.0f;
    double Yaw_min = -41.0f,Yaw_max = 41.0f;
    double yaw_angle = 0,yaw_angle_last = 0;

    double Global[3],Local[3];


/**************************** 3. Functions Definitions *******************************/
    double Estimated_Orientation[3]={0.,},Old_Estimated_Orientation[3]={0.,},HPF_Estimated_Orientation[3]={0.,},Old_HPF_Estimated_Orientation[3]={0.,};
    double Comp_Orientation[3]={0.,};
    #define D2R 3.141592/180.0
    #define R2D 180.0/3.141592
