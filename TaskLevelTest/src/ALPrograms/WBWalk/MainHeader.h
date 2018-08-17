/*
 * MainHeader.h
 * This contains Definitions of variables and functions.
 * Based on MPCWalking's main.cpp
 */

#ifndef MAINHEADER_H
#define MAINHEADER_H

#include "GlobalVariables.h"

/********************************** enum ************************************/
enum{
    CART_NO_ACT = 0, APPROACH_HANDLE, GRASP_HANDLE, WALKING, QUIT_HANDLE
};

enum{
    GRIPPER_STOP = 0, GRIPPER_OPEN, GRIPPER_CLOSE
};

enum{
    WALK_STOP = 0, WALK_FORWARD, WALK_BACKWARD, WALK_RIGHT, WALK_LEFT, WALK_CW, WALK_CCW
};

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

double  MODE_compliance = OFF;
int     Command_CART = CART_NO_ACT;
int     Command_GRIPPER = GRIPPER_STOP;


/**************************** Variables Definitions (Basic) ***********************/
/* Define */
    int             __IS_WORKING;
    int             __IS_GAZEBO ;
    const double    OFFSET_ELB = -20.0;
    const double    OFFSET_RSR = -15.0;
    const double    OFFSET_LSR = 15.0;
    double          FINAL_TIMER;
    int             PODO_NO;
    const int       SW_MODE_COMPLEMENTARY = 0x00;
    const int       SW_MODE_NON_COMPLEMENTARY = 0x01;

    int      last_moving_leg;


/* selection mat */
    void UKSEL(int sampling_tic,double V[15],double U[15][3],double IND[2]);

/* Global Variable For Task Trajecotry (CoM, Foot, Pelvis ori...etc) */
    double init_WBIK_pCOM[3] ,init_WBIK_Q[3] ;
    double FK_RFoot_yaw ,FK_RFoot_pitch,FK_RFoot_roll,FK_LFoot_yaw ,FK_LFoot_pitch,FK_LFoot_roll;

    double GLOBAL_Z_LF_last_earlylanding ,GLOBAL_Z_RF_last_earlylanding;
    double GLOBAL_X_LIPM ,GLOBAL_X_LF ,GLOBAL_X_RF ,GLOBAL_X_LIPM_d ;
    double GLOBAL_X_LIPM_n ,GLOBAL_X_LF_n ,GLOBAL_X_RF_n ,GLOBAL_X_LIPM_d_n ;
    double GLOBAL_Y_LIPM_n ,GLOBAL_Y_LF_n ,GLOBAL_Y_RF_n ,GLOBAL_Y_LIPM_d_n ;
    double GLOBAL_X_LIPM_last ,GLOBAL_ZMP_REF_X_last;
    double GLOBAL_X_LF_last ,GLOBAL_X_RF_last ;
    double GLOBAL_Y_LF ,GLOBAL_Y_RF ;

    double GLOBAL_ZMP_REF_Y,GLOBAL_ZMP_REF_X,GLOBAL_ZMP_REF_X_local;

    double GLOBAL_ZMP_REF_Y_n,GLOBAL_ZMP_REF_X_n;
    double GLOBAL_Y_LIPM ,GLOBAL_Y_LIPM_d ;
    double GLOBAL_Z_LIPM ,GLOBAL_Z_LIPM2 ,GLOBAL_Z_LF ,GLOBAL_Z_RF ,GLOBAL_Z_LIPM_last;
    double GLOBAL_Z_LF_last[3] ,GLOBAL_Z_RF_last[3];
    double GLOBAL_Z_LF_goal[3] ,GLOBAL_Z_RF_goal[3];
    double GLOBAL_Z_LF_last2 ,GLOBAL_Z_RF_last2;
    double Init_Right_Leg ,Init_Left_Leg ;
    double Pel_Yaw;


/* GUI Command variable */
    int walk_flag = 0;
    int walk_start_flag = false;
    int test_cnt = 0;
    int continouse_walking_flag = 0;// 1: walking with continous footprintf comming from joystic
    int stop_flag = false;

    Vars vars;
    Params params;
    Workspace work;
    Settings settings;

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
/* Data Save Variable */
    #define ROW 20000
    #define COL 300
    int     Save_Index;
    double  Save_Data[COL][ROW];
    FILE *fp;
    FILE *fp2;
    FILE *fp3;
    FILE *fp4;


/* Ugain */
    double U_Gain  ;
    double U0_Gain;
    double U3_Gain ;
    double U_Gain_DSP;
    double U0_Gain_KI  ,U0_Gain_KI_last ;
    double U0_Gain_Goal_KI  ,U0_Gain_Goal_KI_last ;
    double G_DSP_X ,G_DSP_Y,G_DSP_X_last ,G_DSP_Y_last;

/* inverse model variable */
    double JW_InvPattern_l,JW_InvPattern_l2;
    double JW_InvPattern_Klqr[2];
    double JW_InvPattern_U[2]   ;
    double JW_InvPattern_U_n[2] ;
    double JW_InvPattern_U_I[2] ;
    double JW_InvPattern_A[2][2];
    double JW_InvPattern_A_X[2][2];
    double JW_InvPattern_B[2];
    double JW_InvPattern_B_X[2];

    double JW_InvPattern_k;
    double JW_InvPattern_k_X;
    double JW_InvPattern_c;
    double JW_InvPattern_c_X;
    double JW_InvPattern_m;

    double JW_InvPattern_X_d[2];
    double JW_InvPattern_X[2];
    double JW_InvPattern_X_old[2];

    double JW_InvPattern_Y_d[2] ;
    double JW_InvPattern_Y[2] ;
    double JW_InvPattern_Y_old[2] ;

    double Y_inv,Y_inv_d,Y_inv_old,theta_ref,theta_dd,theta_d,theta;
    double Y_inv2,Y_inv_d2,Y_inv_old2,theta2_ref,theta2_dd,theta2_d,theta2;
    double U[2],U_I[2];

/* Variable for pattern generator */
    int pv_Index ;
    double pv_Gd[301],pv_Gx[4],pv_Gi[2];
    double pv_A[3][3],pv_B[3][1],pv_C[1][3];
    double CONT_X,CONT_Y,CONT_X_n,CONT_Y_n;
    double _temp_debug_data[50],temp_debug[20];

/* Variable for MPC */
    double MPC_alpha,MPC_beta ,MPC_gamma ,MPC_mu;
    double Pps[15][3] ,Pvs[15][3] ,Pzs[15][3] ;
    double Ppu[15][15] , Pvu[15][15] , Pzu[15][15] , Pzu_T[15][15] , PzuPzu_T[15][15] ;
    double Pzu_invxtrans[15][15],Pzu_inv[15][15],MPC_pc[15][3],MPC_D[30][15],Pvu_trans[15][15],Pzu_inv_trans[15][15],temp_mat[15][15],temp_mat3[15][15],temp_mat2[15][15],t_mat[15][3],t_mat2[15][3],t_mat3[15][3];
    double MPC_h ,MPC_g ,MPC_T ;
    double MPC_ysep,MPC_ystr,MPC_sp[2];
    int MPC_time,MPC_m;
    double MPC_B[3] ,MPC_C[3] ,MPC_A[3][3],MPC_Q[18][18],MPC_Ci[32][18],MPC_ci[32];

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
    double  AnkleControl1,AnkleControl2;
    double  FOGRollVel_LPF,FOGPitchVel_LPF;
    double  FOGRollVel_NF,FOGPitchVel_NF;
    double  FOGRollVel_NF2,FOGPitchVel_NF2;
    double  AngleRoll  ,AngleVel  ,AngleRoll_last ;
    double  LPF_AngleRoll  ,LPF_AngleVel  ,LPF_AngleRoll_last ;
    double  den_a1,den_a2,den_a3,den_a4,den_a5,num_b1,num_b2,num_b3,num_b4,num_b5;
    double  den_a11,den_a21,den_a31,den_a41,den_a51,num_b11,num_b21,num_b31,num_b41,num_b51;
    double  u_i_4 ,u_i_3 , u_i_2 , u_i_1 , u_i ;
    double  y_i_4 ,y_i_3 , y_i_2 , y_i_1 , y_i ;
    double  u_i_41 , u_i_31 , u_i_21 , u_i_11 , u_i1 ;
    double  y_i_41 , y_i_31 , y_i_21 , y_i_11 , y_i1 ;

/* Kirk Variable */
    double  X_ZMP_Local,Y_ZMP_Local,X_ZMP_Global,Y_ZMP_Global,X_ZMP_REF_Local,Y_ZMP_REF_Local,X_ZMP_REF_Global,Y_ZMP_REF_Global;
    double  X_ZMP_IMU , Y_ZMP_IMU ;
    double  X_ZMP_IMU_n , Y_ZMP_IMU_n ;
    double  X_ZMP , Y_ZMP , X_ZMP_LF , Y_ZMP_LF , Old_X_ZMP_LF , Old_Y_ZMP_LF ;
    double  X_ZMP_n , Y_ZMP_n ;
    double  X_ZMP_LPF , Y_ZMP_LPF ;
    double  final_gain_DSP_ZMP_CON , final_gain_SSP_ZMP_CON ;
    double  Del_PC_X_DSP_XZMP_CON , Del_PC_Y_DSP_YZMP_CON , Old_Del_PC_X_DSP_XZMP_CON , Old_Del_PC_Y_DSP_YZMP_CON ,
            Del_PC_X_SSP_XZMP_CON , Del_PC_Y_SSP_YZMP_CON , Old_Del_PC_X_SSP_XZMP_CON , Old_Del_PC_Y_SSP_YZMP_CON ;
    double  LPF_Del_PC_X_DSP_XZMP_CON , LPF_Del_PC_Y_DSP_YZMP_CON ;
    double  Del_PC_X_DSP_XZMP_CON_n ,Del_PC_Y_DSP_YZMP_CON_n ;
    double  LPF_Del_PC_X_SSP_XZMP_CON , LPF_Del_PC_Y_SSP_YZMP_CON ;
    double  Old_Del_PC_X_DSP_XZMP_CON2 ;
    double  Old_Del_PC_Y_DSP_YZMP_CON2 ;
    double  Old_Del_PC_X_SSP_XZMP_CON_2 , Old_Del_PC_Y_SSP_YZMP_CON_2 ;
    unsigned int CNT_final_gain_DSP_ZMP_CON ,  CNT_final_gain_SSP_ZMP_CON ;
    unsigned int CNT_SSP_ZMP_CON ;

    double I_ZMP_CON_X,I_ZMP_CON_Y;
    double I_ZMP_CON_X_last,I_ZMP_CON_Y_last;
    double Old_I_ZMP_CON_X,Old_I_ZMP_CON_Y;

    const double DEL_T = 0.005;

/* Reactive Control variable */
    double ALPHA ,RDF,LDF ;
    double Zctrl,Zctrl2;
    double RDPitch,RDPitch2,RDRoll,LDPitch,LDPitch2,LDRoll,LDRoll2,RDRoll2;

/* Gyro feedback variable */
    double GLOBAL_Xori_LF ,GLOBAL_Yori_LF ,GLOBAL_Zori_LF ;
    double GLOBAL_Xori_LF_n ,GLOBAL_Yori_LF_n ,GLOBAL_Zori_LF_n ;
    double sum_GLOBAL_Xori_LF_n ,sum_GLOBAL_Yori_LF_n ;
    double ave_GLOBAL_Xori_LF_n ,ave_GLOBAL_Yori_LF_n ;
    double GLOBAL_Xori_LF2 ,GLOBAL_Yori_LF2 ,GLOBAL_Zori_LF2 ;
    double GLOBAL_Xori_LF_last ,GLOBAL_Yori_LF_last ,GLOBAL_Zori_LF_last ;
    double GLOBAL_Xori_LF_last2 ,GLOBAL_Yori_LF_last2 ,GLOBAL_Zori_LF_last2 ;
    double GLOBAL_Xori_LF2_last ,GLOBAL_Yori_LF2_last ,GLOBAL_Zori_LF2_last ;
    double GLOBAL_Xori_RF ,GLOBAL_Yori_RF ,GLOBAL_Zori_RF ;
    double GLOBAL_Xori_RF_n ,GLOBAL_Yori_RF_n ,GLOBAL_Zori_RF_n ;
    double sum_GLOBAL_Xori_RF_n ,sum_GLOBAL_Yori_RF_n ;
    double ave_GLOBAL_Xori_RF_n ,ave_GLOBAL_Yori_RF_n ;
    double GLOBAL_Xori_RF2 ,GLOBAL_Yori_RF2 ,GLOBAL_Zori_RF2 ;
    double GLOBAL_Xori_RF_last ,GLOBAL_Yori_RF_last ,GLOBAL_Zori_RF_last ;
    double GLOBAL_Xori_RF_last2 ,GLOBAL_Yori_RF_last2 ,GLOBAL_Zori_RF_last2 ;
    double GLOBAL_Xori_RF2_last ,GLOBAL_Yori_RF2_last ,GLOBAL_Zori_RF2_last ;
    double Foot_gainLF,Foot_gainRF;

/* Landing Control variable */
    int EarlyLandingFlag[2];
    int LandingState;
    int Pre_LandingState ;

/* leg length control */
    double Add_FootTask[2][3],Add_Leg_Recovery[2][3];
    double BTW_FOOT_Angle,BTW_FOOT_Angle_roll,BTW_FOOT_Angle_pitch ,BTW_FOOT_Angle_yaw,BTW_FOOT_qPEL_comp_4x1[4];
    double BTW_PEL_angle_roll ,BTW_PEL_angle_pitch ,BTW_PEL_angle_yaw ;
    double BTW_PEL_angle_roll_vel ,BTW_PEL_angle_pitch_vel ,BTW_PEL_angle_yaw_vel ;


/* deflection compensator variable */
    double deflection_comp_RAR,deflection_comp_LAR;

/* pelvis yaw control */
    double AngularMomentumComp(double velx,double torso_yaw,int sign, int reset);
    double Yaw_Gain,Yaw_Gain2 ;
    double Yaw_min,Yaw_max;
    double yaw_angle ,yaw_angle_last ;

    double Global[3],Local[3];
    double Estimated_Orientation[3],Old_Estimated_Orientation[3],HPF_Estimated_Orientation[3],Old_HPF_Estimated_Orientation[3];
    double Comp_Orientation[3];
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

    int convert_euler_FT(double _pre_state,double pRF_3x1[], double pLF_3x1[],double qRF_4x1[], double qLF_4x1[], double control_result_RF[],double control_result_LF[]);
    int convert_euler(double pRF_3x1[], double pLF_3x1[], double euler_global_x, double euler_global_y, double euler_global_z,
                      double &euler_stance_x, double &euler_stance_y, double &euler_stance_z, double qPEL_comp_4x1[]);
    double PitchRoll_Ori_Integral(double _ref, double _angle, int _zero);
    void mat30by30x30by30(double a[30][30],double b[30][30],double out[30][30]);
    void mat30by30x30by1(double a[30][30],double b[30],double out[30]);
    void mat30by3x3by1(double a[30][3],double b[3],double out[30]);
    void mat15by3minus15by3(double a[15][3],double b[15][3],double out[15][3]);
    void mat15by15x15by3(double a[15][15],double b[15][3],double out[15][3]);
    void mat15by15x15by15(double a[15][15],double b[15][15],double out[15][15]);
    void mat15by15x15by1(double a[15][15],double b[15],double out[15]);
    void mat15by3x3by1(double a[15][3],double b[3],double out[15]);
    void mat3by3x3by1(double a[3][3],double b[3],double out[3]);
    void mat4by4minus4by4(double a[4][4],double b[4][4],double out[4][4]);
    void mat4by4plus4by4(double a[4][4],double b[4][4],double out[4][4]);
    void mat4by4x4by4(double a[4][4],double b[4][4],double out[4][4]);

    void Kalman(double A[4][4],double z[4],double out[3]);
    void EulerToQt(double ang[3],double z[4]);
    int InvMatrix(int n, double* A, double* b);



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

/* MissionDoor */
    void StartComplianceControl();
    void ShutDownAllFlag();
    void SendDatatoGUI();
    void ZMPControl();

/* Cart */
    void GripperTH();
    void CartTH();




#endif // MAINHEADER_H
