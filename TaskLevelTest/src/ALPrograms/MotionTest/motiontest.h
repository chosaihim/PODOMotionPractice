#ifndef MOTIONTEST_H
#define MOTIONTEST_H

/*
 * MOTIONTEST_H.h
 * This contains Definitions of variables and functions.
 * Based on approachbox AL
 */

#include "BasicFiles/BasicSetting.h"
#include "BasicFiles/BasicMath.h"
#include "../../share/Headers/commandlist.h"
#include "solver/solver.h"
#include "StateMachine.h"
#include "ManualCAN.h"
#include "../../../share/Headers/kine_drc_hubo4.h"
#include "../../../share/Headers/kine_drc_hubo2.h"


/**************************** 0. OnOff Switch ****************************************/
    double impONOFF = 0.0;
    double impONOFF2 = 0.0;
    double ssp_torque_ONOFF = 0.0;
    double dsp_torque_ONOFF = 0.0;
    double Gyro_Ankle_FeedBack_ONOFF = 1.0;
    double ZMP_FeedBack_ONOFF = 1.0;

    double Leg_Length_FeedBack_ONOFF = 0.;
    double Leg_Length_Recover_ONOFF = 0.;
    double EarlyLanding_ONOFF = 1.;
    double Sagging_Comp_ONOFF = 1.;
    double Inv_ONOFF =1.;

    double     OnOff_compliance = false;
    int     FLAG_pushdoor = false;
    int     FLAG_SingleLog = false;
    int     FLAG_BacktoNormalWalk = 0;

/**************************** 1. Structs Definitions *********************************/
/* Shared Memory */
    pRBCORE_SHM             sharedData;
    pUSER_SHM               userData;

/* Basic Control */
    JointControlClass       *jCon;

/* Basic Trajectory */
    KINE_DRC_HUBO4          kine_drc_hubo4;
    CKINE_DRC_HUBO2         kine_drc_hubo2;

extern _footprint_info    last_short_foot;

/* BasicTrajectory mode */
enum{MODE_ZEROtoONE = 0, MODE_ONEtoZERO};

/**************************** 2. Variables Definitions (Basic) ***********************/
/* Define */
    int         __IS_WORKING = false;
    int         __IS_GAZEBO = false;
    const double    OFFSET_ELB = -20.0;
    const double    OFFSET_RSR = -15.0;
    const double    OFFSET_LSR = 15.0;
    double      FINAL_TIMER = 0.;
    int         PODO_NO = -1;
    const int   SW_MODE_COMPLEMENTARY = 0x00;
    const int   SW_MODE_NON_COMPLEMENTARY = 0x01;

    extern int                last_moving_leg;

/* Walking Variable */
    double      STEP_LENGTH = 0.001;
    double      STEP_ANGLE = 0.;
    double      STEP_OFFSET = 0.;
    int         STEP_STOP = 0;


    double      RSTEP_LENGTH = 0.001;
    double      RSTEP_NUM = 0.;
    double      RSTEP_ANGLE = 0.;
    double      RSTEP_OFFSET = 0.;
    int         RSTEP_STOP = 0;
/* selection mat */
    void UKSEL(int sampling_tic,double V[15],double U[15][3],double IND[2]);

/* GUI Command variable */
    int walk_flag = 0;
    int walk_start_flag = false;
    int walkstop_liftbox_flag = false;
    int walkstop_door_flag = false;
    int test_cnt = 0;
    int continouse_walking_flag = 0;// 1: walking with continous footprintf comming from joystic
    int stop_flag = false;
    int Command_RealWalk = REALWALK_NO_ACT;

    /* Data Save Variable */
    #define ROW 20000
    #define COL 200
    int     Save_Index;
    double  Save_Data[COL][ROW];
    FILE *fp;
    FILE *fp2;
    FILE *fp3;
    FILE *fp4;

    Vars vars;
    Params params;
    Workspace work;
    Settings settings;


/**************************** 3. Variables Definitions (Trajectory) ******************/
/* Global Variable For Task Trajecotry (CoM, Foot, Pelvis ori...etc) */
    double des_pCOM_3x1[3]={0,},des_pCOM_3x1_hat[3]={0,},des_pCOM_3x1_LPF[3]={0,}, des_pPCz, des_qPEL_4x1[4]={1,0,0,0}, des_pRF_3x1[3], des_pRF_3x1_hat[3], des_qRF_4x1[4]={1,0,0,0}, des_qRF_4x1_hat[4]={1,0,0,0}, des_pLF_3x1[3],des_pLF_3x1_hat[3], des_qLF_4x1[4]={1,0,0,0}, des_qLF_4x1_hat[4]={1,0,0,0};
    double des_pRH_3x1[3]={0,},des_pLH_3x1[3]={0,},des_qRH_4x1[4]={0,},des_qLH_4x1[4]={0,},des_RElb_ang=0,des_LElb_ang=0,RH_ref_frame=0,LH_ref_frame=0,des_pWST=0;
    double FK_pCOM_3x1[3]={0,},FK_pRFoot_3x1[3]={0,},FK_qRFoot_4x1[4]={1,0,0,0},FK_pLFoot_3x1[3]={0,},FK_qLFoot_4x1[4]={1,0,0,0};
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

    const double DEL_T = 0.005;

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

    // pelvis yaw control
    double AngularMomentumComp(double velx,double torso_yaw,int sign, int reset);
    double Yaw_Gain = 2.15f,Yaw_Gain2 = 0.0f;
    double Yaw_min = -41.0f,Yaw_max = 41.0f;
    double yaw_angle = 0,yaw_angle_last = 0;

/* Sensor */
    float LPF_RH_Fz = 0, LPF_RH_Fx = 0, LPF_RH_Fy = 0;
    float Before_RH_Fz = 0, Before_RH_Fx = 0, Before_RH_Fy = 0;
    float Desired_Force=0;
    float Measured_Force_X=0;
    float Measured_Force_Y=0;
    float Measured_Force_Z=0;
    float VelX, VelY, VelZ;
    float PosX, PosY, PosZ;
    double ZMPControlX;
    double ZMPControlY;
    double zmp_Local[3];
/**************************** 3. Functions Definitions *******************************/
/* Basic Function */
    void get_WBIK_Q_from_RefAngleCurrent();
    void First_Initialize();
    void Walking_initialize();
    void Walking_initialize_1st();
    void PreComputeQP();
    void FPG_TEST(int dir,int step_num,double step_length,double step_angle,double step_offset,double LPEL2PEL);
    void FPG_TEST2(int dir,int step_num,double step_length,double step_angle,double step_offset,double LPEL2PEL);
    void FPG_SINGLELOG(int dir,int step_num,double step_length,double LPEL2PEL);
    void FPG_JOYSTIC(int dir,int step_num,double step_length,double step_angle,double step_offset,double LPEL2PEL);
    void WBIK_PARA_CHANGE();
    void Fifth(double t1,double tf,double p0[3],double pf[3],double ref[3]);

    void Global2Local(double _Global[],double _Local[]);
    void Global2Local2(double _Global[],double _Local[]);
    void Local2Global(double _Local[],double _Global[]);

    double BasicTrajectory(double count, int mode);


/* gain override */
    void Upperbody_Gain_Override();
    void Upperbody_Gain_Lock();
    int ZeroGainRightArm();
    int ZeroGainLeftArm();

/* Motion control Function */
    void WMG();
    void PreviewControl(double h,int N,double old_xk[2][3],double old_sum_error[2],double ZMP_REF[2][600],double cp_ref[2][300],double next_sum_error[2],double next_xk[2][3]);
    void FPC_WMG();
    void WBIK();
    void FPG_JOYSTIC(double vel,double omega,double LPEL2PEL);
    void make_last_liftbox_footprint();
    void make_last_footprint();
    void make_last_door_footprint();
    void approach_last();

/* Stabilizer Function */
    void Controller();
    void Controller_initialize();
    void Leg_Length_Control();
    void RecoverLegLength();
    void Gyro_Feedback();
    void LandingControl(int cur_time,int state,double rForce,double lForce);
    void ReactiveControl(int state,int state2,int state3);
    double HUBO2ZMPInitLegLength(double _ref, double _force, int _zero);
    double RecoverRightLegLength(double _ref, double _force, int _zero);
    double RecoverLeftLegLength(double _ref, double _force, int _zero);
    double RightDesForce(int state,double ref_zmp_x,double ref_zmp_y,double LFoot_x,double RFoot_x,double LFoot_y,double RFoot_y);
    double LeftDesForce(int state,double ref_zmp_x,double ref_zmp_y,double LFoot_x,double RFoot_x,double LFoot_y,double RFoot_y);
    double RMXC(int state,int state2,int state3,double ref_zmp_x,double ref_zmp_y,double zmp_x,double zmp_y,double LFoot_x,double RFoot_x,double LFoot_y,double RFoot_y,double FTmx,double Qd0);
    double LMXC(int state,int state2,int state3,double ref_zmp_x,double ref_zmp_y,double zmp_x,double zmp_y,double LFoot_x,double RFoot_x,double LFoot_y,double RFoot_y,double FTmx,double Qd0);
    double RMYC(int state,int state2,int state3,double ref_zmp_x,double ref_zmp_y,double zmp_x,double zmp_y,double LFoot_x,double RFoot_x,double LFoot_y,double RFoot_y,double FTmx,double Qd0);
    double LMYC(int state,int state2,int state3,double ref_zmp_x,double ref_zmp_y,double zmp_x,double zmp_y,double LFoot_x,double RFoot_x,double LFoot_y,double RFoot_y,double FTmx,double Qd0);
    double RMYC2(int state,int state2,int state3,double ref_zmp_x,double ref_zmp_y,double zmp_x,double zmp_y,double LFoot_x,double RFoot_x,double LFoot_y,double RFoot_y,double FTmx,double Qd0);
    double LMYC2(int state,int state2,int state3,double ref_zmp_x,double ref_zmp_y,double zmp_x,double zmp_y,double LFoot_x,double RFoot_x,double LFoot_y,double RFoot_y,double FTmx,double Qd0);
    double RMXC2(int state,int state2,int state3,double ref_zmp_x,double ref_zmp_y,double zmp_x,double zmp_y,double LFoot_x,double RFoot_x,double LFoot_y,double RFoot_y,double FTmx,double Qd0);
    double LMXC2(int state,int state2,int state3,double ref_zmp_x,double ref_zmp_y,double zmp_x,double zmp_y,double LFoot_x,double RFoot_x,double LFoot_y,double RFoot_y,double FTmx,double Qd0);

    double FootForceControl(int state,double dRforce,double dLforce,double mRforce,double mLforce,int reset,double Qd0);
    double FootForceControl2(int state,double dRforce,double dLforce,double mRforce,double mLforce,int reset,double Qd0);

    double ZMPControllerX(double desX,double desY,double desXdot,double desYdot,double desPx,double desPy,double x,double x_dot,double px,double y,double y_dot,double py);
    double ZMPControllerY(double desX,double desY,double desXdot,double desYdot,double desPx,double desPy,double x,double x_dot,double px,double y,double y_dot,double py);

    double NotchFilter_GyroRollControlInput(double _input, int _reset);
    double NotchFilter_GyroPitchControlInput(double _input, int _reset);
    double NotchFilter_GyroRollVel(double _input, int _reset);
    double NotchFilter_GyroPitchVel(double _input, int _reset);

/* feed forward control */
    void Compensator_deflection(int state);

/* Kirk Controllers */
    void Kirk_Control();
    void Kirk_Control_ssp();

/* DSP ZMP Controller */
    double  kirkZMPCon_XP2(double u, double ZMP, int zero);
    double  kirkZMPCon_YP2(double u, double ZMP, int zero);
    void get_zmp();
    void get_zmp2();
    void ZMP_intergral_control();

/* inverse model */
    void JW_INV_MODEL(double Pattern1,double Pattern1_d,double Pattern2,double Pattern2_d);


/* state estimation */
    void ACCtoWorldFrame(double euler[3],double a[3], double new_a[3]);
    void State_Estimator(double p,double q, double r, double ax, double ay, double orientation[3]);


/* Vision */
    void GoalSetwithVision();

/* MissionDoor */
    void StartComplianceControl();
    void ShutDownAllFlag();
    void SendDatatoGUI();
    void StartPushDoor();
    void ZMPControl();


#endif // MOTIONTEST_H
