

#include "BasicFiles/BasicSetting.h"
#include "solver/solver.h"

#include "StateMachine.h"
#include "JoyStick/joystickclass.h"
#include "ManualCAN.h"

#include "../../../share/Headers/ik_math4.h"
#include "../../../share/Headers/kine_drc_hubo4.h"


double impONOFF = 0.0;
double impONOFF2 = 0.0;
double ssp_torque_ONOFF = 0.0;
double dsp_torque_ONOFF = 0.0;
double Gyro_Ankle_FeedBack_ONOFF = 1.0;
double ZMP_FeedBack_ONOFF = 1.0;
//double Ankle_Moment_FeedBack_ONOFF = 1.;

double Leg_Length_FeedBack_ONOFF = 0.;
double Leg_Length_Recover_ONOFF = 0.;
double EarlyLanding_ONOFF = 1.;
//double LateLanding_ONOFF =1.;
double Sagging_Comp_ONOFF = 1.;
double Inv_ONOFF =1.;

double FINAL_TIMER = 0.;




// Basic --------
pRBCORE_SHM             sharedData;
pUSER_SHM               userData;
JointControlClass       *jCon;


int     __IS_WORKING = false;
int     __IS_GAZEBO = false;
int     PODO_NO = -1;

//enum CONTACT_STATE{
//    DSP_INIT_RF = 0,
//    DSP_INIT_LF,
//    DSP_RF,
//    SSP_RF,
//    DSP_LF,
//    SSP_LF,
//    DSP_FINAL
//};

// JOY STICK COMMAND

double JOY_STICK_STEP_LENGTH = 0.;
double JOY_STICK_STEP_ANGLE  = 0.;
double JOY_STICK_STEP_OFFSET = 0.;
int JOY_STICK_STEP_STOP = 0;


enum MPC_COMMAND
{
    MPC_NO_ACT = 100,
    MPC_FORWARD_WALKING,
    MPC_RIGHT_WALKING,
    MPC_LEFT_WALKING,
    MPC_CCW_WALKING,
    MPC_CW_WALKING,
    MPC_JOY_STICK_WALKING_START,
    MPC_JOY_STICK_WALKING_PARA_CHANGE,
    MPC_JOY_STICK_WALKING_STOP,
    MPC_LIM_WALKING,
    MPC_DATA_SAVE,
    MPC_JOY_STICK_WALK_READY,
    MPC_JOY_STICK_WALK_START
};
enum{
    Xdir = 0,
    Ydir,
    Zdir
};
enum{
    RIGHT = 0,
    LEFT
};
enum FTNAME{
    RAFT = 0,
    LAFT
};
enum IMUNAME{
    CIMU = 0
};
enum{
    NOLANDING = 0,
    RSSP,
    LSSP,
    DSP,
    FINAL,
    END
};

const double    OFFSET_ELB = -20.0;
const double    OFFSET_RSR = -15.0;
const double    OFFSET_LSR = 15.0;



KINE_DRC_HUBO4 kine_drc_hubo4;

//********************** Stabilizer variable
// Stabilizer variable
double AnkleControl1=0,AnkleControl2=0;
double FOGRollVel_LPF=0,FOGPitchVel_LPF=0;
double FOGRollVel_NF=0,FOGPitchVel_NF=0;
double FOGRollVel_NF2=0,FOGPitchVel_NF2=0;
double AngleRoll =0.0 ,AngleVel =0.0 ,AngleRoll_last =0.0;
double LPF_AngleRoll =0.0 ,LPF_AngleVel =0.0 ,LPF_AngleRoll_last =0.0;
double den_a1,den_a2,den_a3,den_a4,den_a5,num_b1,num_b2,num_b3,num_b4,num_b5;
double den_a11,den_a21,den_a31,den_a41,den_a51,num_b11,num_b21,num_b31,num_b41,num_b51;
double u_i_4 = 0,u_i_3 = 0, u_i_2 = 0, u_i_1 = 0, u_i = 0;
double y_i_4 = 0,y_i_3 = 0, y_i_2 = 0, y_i_1 = 0, y_i = 0;
double u_i_41 = 0, u_i_31 = 0, u_i_21 = 0, u_i_11 = 0, u_i1 = 0;
double y_i_41 = 0, y_i_31 = 0, y_i_21 = 0, y_i_11 = 0, y_i1 = 0;

//Kirk variables
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

// Ugain
double U_Gain  = 0.;
double U0_Gain  = 0.8;
double U3_Gain = 0.;
double U_Gain_DSP = 1.;
double U0_Gain_KI  = 0.,U0_Gain_KI_last = 0.;
double U0_Gain_Goal_KI  = 0.,U0_Gain_Goal_KI_last = 0.;
double G_DSP_X = 1.,G_DSP_Y=1.,G_DSP_X_last = 1.,G_DSP_Y_last=1.;


// Reactive Control variable
double ALPHA = 0.,RDF=400,LDF = 400;
double Zctrl=0.,Zctrl2;
double RDPitch=0.,RDPitch2=0.,RDRoll=0.,LDPitch=0.,LDPitch2=0.,LDRoll=0.,LDRoll2=0.,RDRoll2=0.;


// Gyro feedback variable
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



// Landing Control variable
int EarlyLandingFlag[2]={0,};

int LandingState = FINAL;
int Pre_LandingState =-1;

// leg length control
double Add_FootTask[2][3]={{0.,},},Add_Leg_Recovery[2][3]={{0.,},};


//
double BTW_YAW,BTW_ROLL,BTW_PITCH;
double BTW_FOOT_Angle=0,BTW_FOOT_Angle_roll=0,BTW_FOOT_Angle_pitch =0,BTW_FOOT_Angle_yaw=0,BTW_FOOT_qPEL_comp_4x1[4]={1,0,0,0};
double BTW_PEL_angle_roll = 0,BTW_PEL_angle_pitch = 0,BTW_PEL_angle_yaw =0;
double BTW_PEL_angle_roll_vel = 0,BTW_PEL_angle_pitch_vel = 0,BTW_PEL_angle_yaw_vel =0;

// deflection compensator variable
double deflection_comp_RAR=0.,deflection_comp_LAR=0.;

// inverse model variable
double JW_InvPattern_l,JW_InvPattern_l2;
double JW_InvPattern_Klqr[2]={0.f,0.f};
double JW_InvPattern_U[2]  = {0.f,0.f};
double JW_InvPattern_U_n[2]  = {0.f,0.f};
double JW_InvPattern_U_I[2]  = {0.f,0.f};
double JW_InvPattern_A[2][2] ={{0.f,0.f},{0.f,0.f}};
double JW_InvPattern_A_X[2][2] ={{0.f,0.f},{0.f,0.f}};
double JW_InvPattern_B[2] = {0.f,0.f};
double JW_InvPattern_B_X[2] = {0.f,0.f};

double JW_InvPattern_k=5550.f;//5160.f;//5550.f;
double JW_InvPattern_k_X=9000.f;//5550.f;
double JW_InvPattern_c=60.f;
double JW_InvPattern_c_X=80.f;
double JW_InvPattern_m=80.f;//69.f;//62.f;//68.f;//

double JW_InvPattern_X_d[2] ={0.f,0.f};
double JW_InvPattern_X[2] ={0.f,0.f};
double JW_InvPattern_X_old[2] ={0.f,0.f};

double JW_InvPattern_Y_d[2] ={0.f,0.f};
double JW_InvPattern_Y[2] ={0.f,0.f};
double JW_InvPattern_Y_old[2] ={0.f,0.f};

double Y_inv=0.,Y_inv_d=0.,Y_inv_old=0.,theta_ref=0.,theta_dd=0.,theta_d=0.,theta=0.;
double Y_inv2=0.,Y_inv_d2=0.,Y_inv_old2=0.,theta2_ref=0.,theta2_dd=0.,theta2_d=0.,theta2=0.;
double U[2]={0.f,0.f},U_I[2]={0.f,0.f};



// -------------------------- gain override
void Upperbody_Gain_Override();
void Upperbody_Gain_Lock();
int ZeroGainRightArm();
int ZeroGainLeftArm();

const int SW_MODE_COMPLEMENTARY = 0x00;
const int SW_MODE_NON_COMPLEMENTARY = 0x01;

// ************************* Variable Declare
// Global Variable For Task Trajecotry (CoM, Foot, Pelvis ori...etc)
double des_pCOM_3x1[3]={0,},des_pCOM_3x1_hat[3]={0,},des_pCOM_3x1_LPF[3]={0,}, des_pPCz, des_qPEL_4x1[4]={1,0,0,0}, des_pRF_3x1[3], des_pRF_3x1_hat[3], des_qRF_4x1[4]={1,0,0,0}, des_qRF_4x1_hat[4]={1,0,0,0}, des_pLF_3x1[3],des_pLF_3x1_hat[3], des_qLF_4x1[4]={1,0,0,0}, des_qLF_4x1_hat[4]={1,0,0,0};
double FK_pCOM_3x1[3]={0,},FK_pRFoot_3x1[3]={0,},FK_qRFoot_4x1[4]={1,0,0,0},FK_pLFoot_3x1[3]={0,},FK_qLFoot_4x1[4]={1,0,0,0};
double init_WBIK_pCOM[3] = {0,},init_WBIK_Q[3] = {0,};
double FK_RFoot_yaw = 0.,FK_RFoot_pitch= 0.,FK_RFoot_roll= 0.,FK_LFoot_yaw = 0.,FK_LFoot_pitch= 0.,FK_LFoot_roll= 0.;

//double GLOBAL_Y_LIPM = 0.0f,GLOBAL_Y_LIPM_d = 0.0f;
//double GLOBAL_X_LIPM = 0.0f,GLOBAL_Z_LIPM = 0.0f,GLOBAL_X_LF = 0.0f,GLOBAL_X_RF = 0.0f,GLOBAL_X_LIPM_d = 0.0f, GLOBAL_Y_LF = 0.0f, GLOBAL_Z_LF = 0.0f, GLOBAL_Z_RF = 0.0f,GLOBAL_Y_RF = 0.0f;
//double GLOBAL_X_LIPM_n = 0.0f,GLOBAL_X_LF_n = 0.0f,GLOBAL_X_RF_n = 0.0f,GLOBAL_X_LIPM_d_n = 0.0f;


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


// Global Variable For Joint Trajecotry (Joint angle)
double WBIK_Q[34] = {0.,},Qub[34]={0.,},WBIK_Q0[34] = {0.,};
double FWRefAngleCurrent[NO_OF_JOINTS] = {0.,};


// pelvis yaw control
double AngularMomentumComp(double velx,double torso_yaw,int sign, int reset);
double Yaw_Gain = 2.15f,Yaw_Gain2 = 0.0f;
double Yaw_min = -41.0f,Yaw_max = 41.0f;
double yaw_angle = 0,yaw_angle_last = 0;



// Variable for pattern generator
int pv_Index = 0;
double pv_Gd[301],pv_Gx[4],pv_Gi[2];
double pv_A[3][3],pv_B[3][1],pv_C[1][3];
double CONT_X,CONT_Y,CONT_X_n,CONT_Y_n;

double _temp_debug_data[50]={0.0f,},temp_debug[20]={0.0f,};

// Variable for MPC
double MPC_alpha = 150.0,MPC_beta = 100000.0,MPC_gamma = 1.0,MPC_mu = 1000000.0;
double Pps[15][3] = {{0.,},},Pvs[15][3] = {{0.,},},Pzs[15][3] = {{0.,},};
double Ppu[15][15] = {{0.,},}, Pvu[15][15] = {{0.,},}, Pzu[15][15] = {{0.,},}, Pzu_T[15][15] = {{0.,},}, PzuPzu_T[15][15] = {{0.,},};
double Pzu_invxtrans[15][15]= {{0.,},},Pzu_inv[15][15]= {{0.,},},MPC_pc[15][3]= {{0.,},},MPC_D[30][15]= {{0.,},},Pvu_trans[15][15]= {{0.,},},Pzu_inv_trans[15][15]= {{0.,},},temp_mat[15][15]= {{0.,},},temp_mat3[15][15]= {{0.,},},temp_mat2[15][15]= {{0.,},},t_mat[15][3]= {{0.,},},t_mat2[15][3]= {{0.,},},t_mat3[15][3]= {{0.,},};
double MPC_h = 0.77,MPC_g = 9.81,MPC_T = 0.1;
double MPC_ysep = 0.15,MPC_ystr = 0.35,MPC_sp[2] = {0.07,0.04};
int MPC_time = 15,MPC_m = 3;
double MPC_B[3] = {0.0,},MPC_C[3] = {0.,},MPC_A[3][3] = {{0.0,},},MPC_Q[18][18] = {{0.0,},},MPC_Ci[32][18]={{0.,},},MPC_ci[32]={0.,};
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

// selection mat
void UKSEL(int sampling_tic,double V[15],double U[15][3],double IND[2]);

//*****************  Function Declare
// Basic Function
void get_WBIK_Q_from_RefAngleCurrent();

void First_Initialize();
void Walking_initialize();
void Walking_initialize_1st();
void PreComputeQP();
void FPG_TEST(int dir,int step_num,double step_length,double step_angle,double step_offset,double LPEL2PEL);
void FPG_TEST2(int dir,int step_num,double step_length,double step_angle,double step_offset,double LPEL2PEL);
void FPG_JOYSTIC(int dir,int step_num,double step_length,double step_angle,double step_offset,double LPEL2PEL);
void WBIK_PARA_CHANGE();
void Fifth(double t1,double tf,double p0[3],double pf[3],double ref[3]);
int InvMatrix(int n, double* A, double* b);


void Global2Local(double _Global[],double _Local[]);
void Global2Local2(double _Global[],double _Local[]);
void Local2Global(double _Local[],double _Global[]);
int convert_euler_FT(double _pre_state,double pRF_3x1[], double pLF_3x1[],double qRF_4x1[], double qLF_4x1[], double control_result_RF[],double control_result_LF[]);

int convert_euler(double pRF_3x1[], double pLF_3x1[], double euler_global_x, double euler_global_y, double euler_global_z,
                  double &euler_stance_x, double &euler_stance_y, double &euler_stance_z, double qPEL_comp_4x1[]);

double PitchRoll_Ori_Integral(double _ref, double _angle, int _zero);




// Motion control Function
void WMG();
void PreviewControl(double h,int N,double old_xk[2][3],double old_sum_error[2],double ZMP_REF[2][600],double cp_ref[2][300],double next_sum_error[2],double next_xk[2][3]);
void FPC_WMG();
void WBIK();
void FPG_JOYSTIC(double vel,double omega,double LPEL2PEL);

// Stabilizer Function
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

// feed forward control
void Compensator_deflection(int state);

// Kirk Controllers
void Kirk_Control();
void Kirk_Control_ssp();
// DSP ZMP Controller
double  kirkZMPCon_XP2(double u, double ZMP, int zero);
double  kirkZMPCon_YP2(double u, double ZMP, int zero);
void get_zmp();
void get_zmp2();
void ZMP_intergral_control();

// inverse model
void JW_INV_MODEL(double Pattern1,double Pattern1_d,double Pattern2,double Pattern2_d);


// state estimation
void EulerAccel(double ax,double ay, double out[3]);
void EulerToQt(double ang[3],double z[4]);
void ACCtoWorldFrame(double euler[3],double a[3], double new_a[3]);
void Kalman(double A[4][4],double z[4],double out[3]);
void State_Estimator(double p,double q, double r, double ax, double ay, double orientation[3]);

double Estimated_Orientation[3]={0.,},Old_Estimated_Orientation[3]={0.,},HPF_Estimated_Orientation[3]={0.,},Old_HPF_Estimated_Orientation[3]={0.,};
double Comp_Orientation[3]={0.,};



// GUI Command variable
int walk_flag = 0;
int test_cnt = 0;
int continouse_walking_flag = 0;// 1: walking with continous footprintf comming from joystic

// Data Save Variable
//FILE *fp;
//#define ROW_data_debug 20000
//#define COL_data_debug 100
//double   SaveData[COL_data_debug][ROW_data_debug];
// Debugging data
#define ROW 20000
#define COL 100
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


RBJoystick  *rbjoy;
int __JOY_COMMAND_MODE = false;
extern _footprint_info    last_short_foot;
extern int                last_moving_leg;

int main(int argc, char *argv[])
{
    // Termination signALTutorialal ---------------------------------
    signal(SIGTERM, CatchSignals);   // "kill" from shell
    signal(SIGINT,  CatchSignals);    // Ctrl-c
    signal(SIGHUP,  CatchSignals);    // shell termination
    signal(SIGKILL, CatchSignals);
    signal(SIGSEGV, CatchSignals);

    // Block memory swapping ------------------------------
    mlockall(MCL_CURRENT|MCL_FUTURE);

    // Setting the AL Name <-- Must be a unique name!!
    sprintf(__AL_NAME, "MPCWalking");


    CheckArguments(argc, argv);

    if(PODO_NO == -1){
        FILE_LOG(logERROR) << "Please check the AL Number";
        FILE_LOG(logERROR) << "Terminate this AL..";
        return 0;
    }

    //************* Initialize RBCore -----------------------------------
    if(RBInitialize() == false)
        __IS_WORKING = false;

    rbjoy = new RBJoystick();
    rbjoy->ConnectJoy();

    // Change WBIK Parameter



    //************* Get F.K
    printf("******* Excute First Initialize ******* \n");

    First_Initialize();

    //    Walking_initialize_1st();
    printf("******* First Initialize complete ******* \n");

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

//    // Foot Print Generation Test
//    FPG_TEST();

    while(__IS_WORKING){
        usleep(100*1000);

        if(rbjoy->JoyAxis[5]>32000 && rbjoy->JoyAxis[5] < 33000 && walk_flag == false)
        {
            // RT
            // Put dummy foot prints in the buffur
            // walking on place
            printf("joystic command : %d \n",rbjoy->JoyAxis[5]);
            printf("joystic command : %d \n",rbjoy->JoyAxis[5]);

            sharedData->COMMAND[PODO_NO].USER_COMMAND = MPC_JOY_STICK_WALK_READY;
//            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            printf("Walk on place!!!!        \n");

        }

        if(rbjoy->JoyButton[5] >0 && walk_flag == true)
        {
            // RB joy stick walking start. you can give a command
            printf("joystic command : %d \n",rbjoy->JoyButton[5]);
            printf("joystic command : %d \n",rbjoy->JoyButton[5]);
            sharedData->COMMAND[PODO_NO].USER_COMMAND = MPC_JOY_STICK_WALK_START;

            // RB
//            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
        }


        switch(sharedData->COMMAND[PODO_NO].USER_COMMAND){
        case MPC_LIM_WALKING:
            FILE_LOG(logSUCCESS) << "Command LIM_WALKING received..";

            pv_Index = 0;

            Walking_initialize();

            _footprint_info tempFoot;

            tempFoot.footprint.rfoot[0] = FK_pRFoot_3x1[0];//0.0;
            tempFoot.footprint.rfoot[1] = FK_pRFoot_3x1[1];//-0.105;
            tempFoot.footprint.rfoot[2] = FK_pRFoot_3x1[2];//0.0;

            tempFoot.footprint.rori[0] = FK_RFoot_yaw*R2D;//FK_pRFoot_3x1[0];//0.0;
            tempFoot.footprint.rori[1] = FK_RFoot_roll*R2D;//FK_pRFoot_3x1[1];//-0.105;
            tempFoot.footprint.rori[2] = FK_RFoot_pitch*R2D;//FK_pRFoot_3x1[2];//0.0;

            tempFoot.footprint.lfoot[0] = FK_pLFoot_3x1[0];//0.0;
            tempFoot.footprint.lfoot[1] = FK_pLFoot_3x1[1];//0.105;
            tempFoot.footprint.lfoot[2] = FK_pLFoot_3x1[2];

            tempFoot.footprint.lori[0] = FK_LFoot_yaw*R2D;//FK_pLFoot_3x1[0];//0.0;
            tempFoot.footprint.lori[1] = FK_LFoot_roll*R2D;//FK_pLFoot_3x1[1];//0.105;
            tempFoot.footprint.lori[2] = FK_LFoot_pitch*R2D;//FK_pLFoot_3x1[2];


            printf(">>>>>>>>>> Foot Print Initial Value <<<<<<<<<<< \n");
            printf("right x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.rfoot[0],tempFoot.footprint.rfoot[1],tempFoot.footprint.rfoot[2],tempFoot.footprint.rori[0],tempFoot.footprint.rori[1],tempFoot.footprint.rori[2]);
            printf("left  x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.lfoot[0],tempFoot.footprint.lfoot[1],tempFoot.footprint.lfoot[2],tempFoot.footprint.lori[0],tempFoot.footprint.lori[1],tempFoot.footprint.lori[2]);

            tempFoot.time.dsp_time = 0.0;
            tempFoot.time.ssp_time = 0.0;
            tempFoot.info = FOOTINFO_NO;//FOOTINFO_FIRST_STEP;

        //    push_short_foot(tempFoot);
            memcpy(&prev_foot, &tempFoot, sizeof(_footprint_info));
            memcpy(&last_short_foot, &prev_foot, sizeof(_footprint_info));

            printf("while last short foot right x: %f  y:%f   z:%f  \n",last_short_foot.footprint.rfoot[0],last_short_foot.footprint.rfoot[1],last_short_foot.footprint.rfoot[2]);
            printf("whilelast short foot left  x: %f  y:%f   z:%f  \n",last_short_foot.footprint.lfoot[0],last_short_foot.footprint.lfoot[1],last_short_foot.footprint.lfoot[2]);

            usleep(200*1000);

            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            __JOY_COMMAND_MODE = true;

            printf("while right des x: %f  y:%f   z:%f  \n",des_pRF_3x1[0],des_pRF_3x1[1],des_pRF_3x1[2]);
            printf("while left  des x: %f  y:%f   z:%f  \n",des_pLF_3x1[0],des_pLF_3x1[1],des_pLF_3x1[2]);
            printf("while COM x: %f  y: %f   z:%f \n",des_pCOM_3x1[0],des_pCOM_3x1[1],des_pCOM_3x1[2]);
            printf("while Right WBIK_Q : %f  %f  %f  %f  %f  %f \n",WBIK_Q[7],WBIK_Q[8],WBIK_Q[9],WBIK_Q[10],WBIK_Q[11],WBIK_Q[12]);
            printf("while Left  WBIK_Q : %f  %f  %f  %f  %f  %f \n",WBIK_Q[13],WBIK_Q[14],WBIK_Q[15],WBIK_Q[16],WBIK_Q[17],WBIK_Q[18]);
            printf("while right global x: %f  y:%f   z:%f  \n",GLOBAL_X_RF,GLOBAL_Y_RF,GLOBAL_Z_RF);
            printf("while left  global x: %f  y:%f   z:%f  \n",GLOBAL_X_LF,GLOBAL_Y_LF,GLOBAL_Z_LF);

            walk_flag = 1;

            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;


        case MPC_JOY_STICK_WALKING_START:

            FILE_LOG(logSUCCESS) << "Command JOY STICK received..";

            JOY_STICK_STEP_LENGTH = userData->G2M.StepLength;
            JOY_STICK_STEP_ANGLE  = userData->G2M.StepAngle;
            JOY_STICK_STEP_OFFSET  = userData->G2M.StepOffset;
            JOY_STICK_STEP_STOP = sharedData->COMMAND[PODO_NO].USER_PARA_INT[0];

            __JOY_COMMAND_MODE = true;

            continouse_walking_flag = true;


            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        case MPC_JOY_STICK_WALK_START:

            if(continouse_walking_flag == true && walk_flag == true)
            {
                FILE_LOG(logSUCCESS) << "Command JOY STICK received..";
                FILE_LOG(logSUCCESS) << "Command JOY STICK received..";
                FILE_LOG(logSUCCESS) << "Command JOY STICK received..";

                JOY_STICK_STEP_LENGTH = userData->G2M.StepLength;
                JOY_STICK_STEP_ANGLE  = userData->G2M.StepAngle;
                JOY_STICK_STEP_OFFSET  = userData->G2M.StepOffset;
                JOY_STICK_STEP_STOP = sharedData->COMMAND[PODO_NO].USER_PARA_INT[0];

                __JOY_COMMAND_MODE = true;

                continouse_walking_flag = false;

            }
            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;

            break;


        case MPC_JOY_STICK_WALKING_PARA_CHANGE:
            FILE_LOG(logSUCCESS) << "Command Change parameter received..";

            JOY_STICK_STEP_LENGTH = userData->G2M.StepLength;
            JOY_STICK_STEP_ANGLE  = userData->G2M.StepAngle;
            JOY_STICK_STEP_OFFSET  = userData->G2M.StepOffset;

            JOY_STICK_STEP_STOP = sharedData->COMMAND[PODO_NO].USER_PARA_INT[0];
            // if joy stick step stop is 1, stop walking.

            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;

        case MPC_FORWARD_WALKING:
            if(walk_flag == 0){
                FILE_LOG(logSUCCESS) << "Command ForwardWalking received..";

                Upperbody_Gain_Override();

                sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0]=1;//fog zero
                sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_SENSOR_FOG_NULL;
    //            zero_window();
    //            zero_localfoot();
                pv_Index = 0;

                // walking initialize
                Walking_initialize();

                // Foot Print Generation Test
                FPG_TEST(MPC_FORWARD_WALKING,userData->G2M.StepNum,userData->G2M.StepLength,userData->G2M.StepAngle,userData->G2M.StepOffset,kine_drc_hubo4.L_PEL2PEL);

                usleep(200*1000);
                walk_flag = 1;

                continouse_walking_flag = 0;

                jCon->RefreshToCurrentReference();

                jCon->SetAllMotionOwner();
            }


            printf("while right des x: %f  y:%f   z:%f  \n",des_pRF_3x1[0],des_pRF_3x1[1],des_pRF_3x1[2]);
            printf("while left  des x: %f  y:%f   z:%f  \n",des_pLF_3x1[0],des_pLF_3x1[1],des_pLF_3x1[2]);
            printf("while COM x: %f  y: %f   z:%f \n",des_pCOM_3x1[0],des_pCOM_3x1[1],des_pCOM_3x1[2]);
            printf("while Right WBIK_Q : %f  %f  %f  %f  %f  %f \n",WBIK_Q[7],WBIK_Q[8],WBIK_Q[9],WBIK_Q[10],WBIK_Q[11],WBIK_Q[12]);
            printf("while Left  WBIK_Q : %f  %f  %f  %f  %f  %f \n",WBIK_Q[13],WBIK_Q[14],WBIK_Q[15],WBIK_Q[16],WBIK_Q[17],WBIK_Q[18]);
            printf("while right global x: %f  y:%f   z:%f  \n",GLOBAL_X_RF,GLOBAL_Y_RF,GLOBAL_Z_RF);
            printf("while left  global x: %f  y:%f   z:%f  \n",GLOBAL_X_LF,GLOBAL_Y_LF,GLOBAL_Z_LF);

            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;


            break;
        case MPC_JOY_STICK_WALK_READY:

            if(walk_flag == 0){
                FILE_LOG(logSUCCESS) << "Command JOY_STICK_WALK_READY..";
                FILE_LOG(logSUCCESS) << "Command JOY_STICK_WALK_READY..";
                FILE_LOG(logSUCCESS) << "Command JOY_STICK_WALK_READY..";

                Upperbody_Gain_Override();

                sharedData->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0]=1;//fog zero
                sharedData->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_SENSOR_FOG_NULL;
    //            zero_window();
    //            zero_localfoot();
                pv_Index = 0;

                // walking initialize
                Walking_initialize();

                // Foot Print Generation Test
                FPG_TEST(MPC_FORWARD_WALKING,10,0.001,0.,0.21,kine_drc_hubo4.L_PEL2PEL);

                usleep(200*1000);

                walk_flag = 1;

                continouse_walking_flag = true;

//                __JOY_COMMAND_MODE = true;

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

        case MPC_RIGHT_WALKING:
            FILE_LOG(logSUCCESS) << "Command RIGHT Walking received..";
            Upperbody_Gain_Override();
//            zero_window();

//            zero_localfoot();

            pv_Index = 0;

            // walking initialize
            Walking_initialize();

            // Foot Print Generation Test
            FPG_TEST(MPC_RIGHT_WALKING,userData->G2M.StepNum,userData->G2M.StepLength,userData->G2M.StepAngle,userData->G2M.StepOffset,kine_drc_hubo4.L_PEL2PEL);

            usleep(200*1000);

            jCon->RefreshToCurrentReference();

            jCon->SetAllMotionOwner();

            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;

            printf("*********** right x: %f  y:%f   z:%f  \n",des_pRF_3x1[0],des_pRF_3x1[1],des_pRF_3x1[2]);
            printf("*********** left  x: %f  y:%f   z:%f  \n",des_pLF_3x1[0],des_pLF_3x1[1],des_pLF_3x1[2]);
            printf("COM x: %f  y: %f   z:%f \n",des_pCOM_3x1[0],des_pCOM_3x1[1],des_pCOM_3x1[2]);
            printf("while Right WBIK_Q : %f  %f  %f  %f  %f  %f \n",WBIK_Q[7],WBIK_Q[8],WBIK_Q[9],WBIK_Q[10],WBIK_Q[11],WBIK_Q[12]);
            printf("while Left  WBIK_Q : %f  %f  %f  %f  %f  %f \n",WBIK_Q[13],WBIK_Q[14],WBIK_Q[15],WBIK_Q[16],WBIK_Q[17],WBIK_Q[18]);

            walk_flag = 1;
            continouse_walking_flag = 0;
            break;

        case MPC_LEFT_WALKING:
            FILE_LOG(logSUCCESS) << "Command LEFT Walking received..";
            Upperbody_Gain_Override();
//            zero_window();

//            zero_localfoot();

            pv_Index = 0;

            // walking initialize
            Walking_initialize();

            // Foot Print Generation Test
            FPG_TEST(MPC_LEFT_WALKING,userData->G2M.StepNum,userData->G2M.StepLength,userData->G2M.StepAngle,userData->G2M.StepOffset,kine_drc_hubo4.L_PEL2PEL);

            usleep(200*1000);

            jCon->RefreshToCurrentReference();

            jCon->SetAllMotionOwner();

            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;

            printf("*********** right x: %f  y:%f   z:%f  \n",des_pRF_3x1[0],des_pRF_3x1[1],des_pRF_3x1[2]);
            printf("*********** left  x: %f  y:%f   z:%f  \n",des_pLF_3x1[0],des_pLF_3x1[1],des_pLF_3x1[2]);
            printf("COM x: %f  y: %f   z:%f \n",des_pCOM_3x1[0],des_pCOM_3x1[1],des_pCOM_3x1[2]);
            printf("while Right WBIK_Q : %f  %f  %f  %f  %f  %f \n",WBIK_Q[7],WBIK_Q[8],WBIK_Q[9],WBIK_Q[10],WBIK_Q[11],WBIK_Q[12]);
            printf("while Left  WBIK_Q : %f  %f  %f  %f  %f  %f \n",WBIK_Q[13],WBIK_Q[14],WBIK_Q[15],WBIK_Q[16],WBIK_Q[17],WBIK_Q[18]);

            walk_flag = 1;
            continouse_walking_flag = 0;
            break;

        case MPC_CW_WALKING:
            FILE_LOG(logSUCCESS) << "Command CW Walking received..";
            Upperbody_Gain_Override();
//            zero_window();

//            zero_localfoot();

            pv_Index = 0;

            // walking initialize
            Walking_initialize();

            // Foot Print Generation Test
            FPG_TEST(MPC_CW_WALKING,userData->G2M.StepNum,userData->G2M.StepLength,userData->G2M.StepAngle,userData->G2M.StepOffset,kine_drc_hubo4.L_PEL2PEL);

            usleep(200*1000);

            jCon->RefreshToCurrentReference();

            jCon->SetAllMotionOwner();

            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;

            printf("*********** right x: %f  y:%f   z:%f  \n",des_pRF_3x1[0],des_pRF_3x1[1],des_pRF_3x1[2]);
            printf("*********** left  x: %f  y:%f   z:%f  \n",des_pLF_3x1[0],des_pLF_3x1[1],des_pLF_3x1[2]);
            printf("COM x: %f  y: %f   z:%f \n",des_pCOM_3x1[0],des_pCOM_3x1[1],des_pCOM_3x1[2]);
            printf("while Right WBIK_Q : %f  %f  %f  %f  %f  %f \n",WBIK_Q[7],WBIK_Q[8],WBIK_Q[9],WBIK_Q[10],WBIK_Q[11],WBIK_Q[12]);
            printf("while Left  WBIK_Q : %f  %f  %f  %f  %f  %f \n",WBIK_Q[13],WBIK_Q[14],WBIK_Q[15],WBIK_Q[16],WBIK_Q[17],WBIK_Q[18]);

            walk_flag = 1;
            continouse_walking_flag = 0;
            break;

        case MPC_CCW_WALKING:
            FILE_LOG(logSUCCESS) << "Command CCW Walking received..";
            Upperbody_Gain_Override();
//            zero_window();

//            zero_localfoot();

            pv_Index = 0;

            // walking initialize
            Walking_initialize();

            // Foot Print Generation Test
            FPG_TEST(MPC_CCW_WALKING,userData->G2M.StepNum,userData->G2M.StepLength,userData->G2M.StepAngle,userData->G2M.StepOffset,kine_drc_hubo4.L_PEL2PEL);

            usleep(200*1000);

            jCon->RefreshToCurrentReference();

            jCon->SetAllMotionOwner();

            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;

            printf("*********** right x: %f  y:%f   z:%f  \n",des_pRF_3x1[0],des_pRF_3x1[1],des_pRF_3x1[2]);
            printf("*********** left  x: %f  y:%f   z:%f  \n",des_pLF_3x1[0],des_pLF_3x1[1],des_pLF_3x1[2]);
            printf("COM x: %f  y: %f   z:%f \n",des_pCOM_3x1[0],des_pCOM_3x1[1],des_pCOM_3x1[2]);
            printf("while Right WBIK_Q : %f  %f  %f  %f  %f  %f \n",WBIK_Q[7],WBIK_Q[8],WBIK_Q[9],WBIK_Q[10],WBIK_Q[11],WBIK_Q[12]);
            printf("while Left  WBIK_Q : %f  %f  %f  %f  %f  %f \n",WBIK_Q[13],WBIK_Q[14],WBIK_Q[15],WBIK_Q[16],WBIK_Q[17],WBIK_Q[18]);

            walk_flag = 1;
            continouse_walking_flag = 0;
            break;

        case MPC_DATA_SAVE:
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

        case 999:
            FILE_LOG(logSUCCESS) << "Command 999 received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();
            jCon->SetMoveJoint(WST, 30.0, 2000.0, MOVE_ABSOLUTE);
            sharedData->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        default:
            break;
        }
    }



    FILE_LOG(logERROR) << "Process \"" << __AL_NAME << "\" is terminated" << endl;
    return 0;
}



#define JOY_MAX     32767.0

//==============================//
// Task Thread
//==============================//

void RBTaskThread(void *)
{
    printf("TASKTHREADDDDDDDDDDDDDDDDDDDDDDDDDDDDDD\n");
    while(__IS_WORKING)
    {

        printf("WALKFLAG is %d\n",walk_flag);
//      Walking Motion Generator
        if(walk_flag == 1)
        {
            printf("WALKFLAG\n");
            // joy cmd
            if(rbjoy->isConnected() && __JOY_COMMAND_MODE)
            {
                float velX = (float)(-rbjoy->JoyAxis[1]) / JOY_MAX * 0.1;
                float velTh = (float)(-rbjoy->JoyAxis[0]) / JOY_MAX * 10.0;

                if(fabs(velX) < 0.001){

                    velX = 0.001;
                }
                if(fabs(velTh) < 0.01)
                {
                    velTh = 0.0;
                }

                JOY_STICK_STEP_LENGTH = velX;
                JOY_STICK_STEP_ANGLE = velTh;
                JOY_STICK_STEP_OFFSET =  0.21 + 0.05 * fabs(velTh /10.0);

                _footprint_info dummyfoot;
                while(pull_short_foot(dummyfoot)){
                    ;
                }

                int right_left = 0;
                int moving_leg;
                double rl = 0.;

                if(last_moving_leg == MOVING_RIGHT){
                    moving_leg = MOVING_LEFT;
                    right_left  = 1;
                }else
                {
                    right_left  = 0;
                    moving_leg = MOVING_RIGHT;
                }

                _footprint_info newfoot;

                for(int i=0; i<5; i++)
                {
                    if(right_left == 1){
                        rl = 1;
                    }else{
                        rl = -1;
                    }
                    if(moving_leg == MOVING_RIGHT){
                        newfoot.footprint.rori[0] = last_short_foot.footprint.lori[0] + JOY_STICK_STEP_ANGLE;
                        newfoot.footprint.rori[1] = last_short_foot.footprint.rori[1];
                        newfoot.footprint.rori[2] = last_short_foot.footprint.rori[2];

                        newfoot.footprint.lori[0] = last_short_foot.footprint.lori[0];
                        newfoot.footprint.lori[1] = last_short_foot.footprint.lori[1];
                        newfoot.footprint.lori[2] = last_short_foot.footprint.lori[2];

                        newfoot.footprint.rfoot[0] = last_short_foot.footprint.lfoot[0] + JOY_STICK_STEP_LENGTH*cos(newfoot.footprint.rori[0]*D2R) - JOY_STICK_STEP_OFFSET*sin(newfoot.footprint.rori[0]*D2R)*rl;
                        newfoot.footprint.rfoot[1] = last_short_foot.footprint.lfoot[1] + JOY_STICK_STEP_LENGTH*sin(newfoot.footprint.rori[0]*D2R) + JOY_STICK_STEP_OFFSET*cos(newfoot.footprint.rori[0]*D2R)*rl;
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

                        newfoot.footprint.lori[0] = last_short_foot.footprint.rori[0] + JOY_STICK_STEP_ANGLE;
                        newfoot.footprint.lori[1] = last_short_foot.footprint.lori[1];
                        newfoot.footprint.lori[2] = last_short_foot.footprint.lori[2];

                        newfoot.footprint.rfoot[0] = last_short_foot.footprint.rfoot[0];
                        newfoot.footprint.rfoot[1] = last_short_foot.footprint.rfoot[1];
                        newfoot.footprint.rfoot[2] = last_short_foot.footprint.rfoot[2];

                        newfoot.footprint.lfoot[0] = last_short_foot.footprint.rfoot[0] + JOY_STICK_STEP_LENGTH*cos(newfoot.footprint.lori[0]*D2R) - JOY_STICK_STEP_OFFSET*sin(newfoot.footprint.lori[0]*D2R)*rl;
                        newfoot.footprint.lfoot[1] = last_short_foot.footprint.rfoot[1] + JOY_STICK_STEP_LENGTH*sin(newfoot.footprint.lori[0]*D2R) + JOY_STICK_STEP_OFFSET*cos(newfoot.footprint.lori[0]*D2R)*rl;
                        newfoot.footprint.lfoot[2] = last_short_foot.footprint.lfoot[2];

                        moving_leg = MOVING_RIGHT;
                    }

                    newfoot.time.dsp_time = 0.1;
                    newfoot.time.ssp_time = 0.8;

                    if(rbjoy->JoyButton[4]>0)
                    {
                        FILE_LOG(logERROR) << "STOP JOY WALKING!!";
                        __JOY_COMMAND_MODE  = false;
                        while(pull_short_foot(dummyfoot)){
                            ;
                        }
                        push_short_foot(last_short_foot);
                        break;
                    }else
                    {
                        push_short_foot(newfoot);
                        right_left ^= 1;
                    }
                }
            }

            State_Estimator(sharedData->FOG.RollVel,sharedData->FOG.PitchVel, sharedData->FOG.YawVel, sharedData->IMU[0].AccX, sharedData->IMU[0].AccY, Estimated_Orientation);

            update_window();

            get_zmp2();

            WMG();

            Controller();

//            JW_INV_MODEL(GLOBAL_Y_LIPM_n - (GLOBAL_Y_LF_n + GLOBAL_Y_RF_n)/2.0,GLOBAL_Y_LIPM_d_n,GLOBAL_X_LIPM,GLOBAL_X_LIPM_d);

            WBIK();

            save();
        }else
        {
                if(LandingState == FINAL)
                {
                    printf("******************* Walking Is Finishied!!!");
                    Walking_initialize();
                }
                LandingState = END;

//                continouse_walking_flag = false;
        }

        jCon->MoveAllJoint();
        rt_task_suspend(&rtTaskCon);
    }
}



//==============================//


//==============================//

// Flag Thread
//==============================//

void RBFlagThread(void *)
{
    rt_task_set_periodic(NULL, TM_NOW, 300*1000);        // 300 usec
    while(__IS_WORKING)
    {
//        State_Estimator(sharedData->FOG.RollVel,sharedData->FOG.PitchVel, sharedData->FOG.YawVel, sharedData->IMU[0].AccX, sharedData->IMU[0].AccY, Estmated_Orientation);
        rt_task_wait_period(NULL);
        if(HasAnyOwnership()){
            if(sharedData->SYNC_SIGNAL[PODO_NO] == true){
                jCon->JointUpdate();
                rt_task_resume(&rtTaskCon);
            }
        }
    }
}

//==============================//
// Foot Print Generator (Path Planning)
//==============================//
void FPG_JOYSTIC(double vel,double omega,double LPEL2PEL)
{
    //check whether the command is first.

    // Change Joystic Input to footprint
    int right_left = 0,test_short_foot_num = 3;
    double rl = 0., step_angle = 0. ,step_length =0.,dir=0,step_offset = LPEL2PEL;

    step_length =  vel/1.0;
    step_angle =  D2R * omega/1.0;
    printf("***********step length: %f     step angle: %f  \n",step_length,step_angle);
    // when the robot is rotating, first foot selection.
    if(step_angle < 0)
        dir = MPC_RIGHT_WALKING;
    else
        dir = MPC_LEFT_WALKING;

    if(dir == MPC_RIGHT_WALKING){
        right_left = 0;
    }else if(dir == MPC_LEFT_WALKING){
        right_left  = 1;
    }


    _footprint_info tempFoot;

    memcpy(&prev_foot, &tempFoot, sizeof(_footprint_info));

    tempFoot.info = FOOTINFO_NO;

    FILE_LOG(logSUCCESS) << "short_foot_index: " << ring_short_head << ", " << ring_short_tail;

    if(test_short_foot_num < 3){
        test_short_foot_num = 3;
    }


        if(right_left == 1)
        {
            //rotate left direction ,, left foot first.
            rl = 1;

        }else
        {
            rl = -1;
        }

    if(right_left == 0){
        tempFoot.footprint.rori[0] = tempFoot.footprint.lori[0] + step_angle*R2D;
        tempFoot.footprint.rori[1] = tempFoot.footprint.rori[1];
        tempFoot.footprint.rori[2] = tempFoot.footprint.rori[2];

        tempFoot.footprint.lori[0] = tempFoot.footprint.lori[0];
        tempFoot.footprint.lori[1] = tempFoot.footprint.lori[1];
        tempFoot.footprint.lori[2] = tempFoot.footprint.lori[2];

        tempFoot.footprint.rfoot[0] = tempFoot.footprint.lfoot[0] + step_length*cos(tempFoot.footprint.rori[0]*D2R) - step_offset*sin(tempFoot.footprint.rori[0]*D2R)*rl;
        tempFoot.footprint.rfoot[1] = tempFoot.footprint.lfoot[1] + step_length*sin(tempFoot.footprint.rori[0]*D2R) + step_offset*cos(tempFoot.footprint.rori[0]*D2R)*rl;
        tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

        tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
        tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
        tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
    }else
    {
        tempFoot.footprint.rori[0] = tempFoot.footprint.rori[0];
        tempFoot.footprint.rori[1] = tempFoot.footprint.rori[1];
        tempFoot.footprint.rori[2] = tempFoot.footprint.rori[2];

        tempFoot.footprint.lori[0] = tempFoot.footprint.rori[0] + step_angle*R2D;
        tempFoot.footprint.lori[1] = tempFoot.footprint.lori[1];
        tempFoot.footprint.lori[2] = tempFoot.footprint.lori[2];

        tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
        tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
        tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

        tempFoot.footprint.lfoot[0] = tempFoot.footprint.rfoot[0] + step_length*cos(tempFoot.footprint.lori[0]*D2R) - step_offset*sin(tempFoot.footprint.lori[0]*D2R)*rl;
        tempFoot.footprint.lfoot[1] = tempFoot.footprint.rfoot[1] + step_length*sin(tempFoot.footprint.lori[0]*D2R) + step_offset*cos(tempFoot.footprint.lori[0]*D2R)*rl;
        tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
    }



    printf("    step_length: %f   step_angle: %f \n",step_length,step_angle);

//    int temp_index = 0;
//    int temp_head = ring_short_head;

    //Making Foot print
    for(int i=0; i<test_short_foot_num; i++){
        if(right_left == 0){
            tempFoot.footprint.rfoot[0] = tempFoot.footprint.lfoot[0] + step_length;
            tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
            tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

            tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
            tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
            tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
        }else{
            tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
            tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
            tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

            tempFoot.footprint.lfoot[0] = tempFoot.footprint.rfoot[0] + step_length;
            tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
            tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
        }

        FILE_LOG(logERROR) << i << ": " << prev_foot.footprint.rfoot[0] << ", " << prev_foot.footprint.lfoot[0];
        tempFoot.time.dsp_time = 1.5;
        tempFoot.time.ssp_time = 0.8;

        push_short_foot(tempFoot);
        right_left ^= 1;

    }
}



void FPG_TEST(int dir,int step_num,double step_length,double step_angle,double step_offset,double LPEL2PEL)
{
    // example short footprint information =============================

    zero_window();

    zero_localfoot();

    if(step_angle>15.0)
    {
        step_angle = 15.0;
    }else if(step_angle<-15.0)
    {
        step_angle = -15.0;
    }

    step_angle = step_angle*D2R;

    double rl = 0;

    int right_left = 0;

    if(dir == MPC_RIGHT_WALKING){
        right_left = 0;
    }else if(dir == MPC_LEFT_WALKING){
        right_left  = 1;
    }

    printf("***********step angle: %f     step offset: %f  \n",step_angle,step_offset);

    _footprint_info dummyfoot;
    // remove old foot printf;
    while(pull_short_foot(dummyfoot)){
        ;
    }

    _footprint_info tempFoot;
//    _footprint_info prevfoot;


//    memcpy(&prevfoot, &last_short_foot, sizeof(_footprint_info));

    tempFoot.footprint.rfoot[0] = FK_pRFoot_3x1[0];//0.0;
    tempFoot.footprint.rfoot[1] = FK_pRFoot_3x1[1];//-0.105;
    tempFoot.footprint.rfoot[2] = FK_pRFoot_3x1[2];//0.0;

    tempFoot.footprint.rori[0] = FK_RFoot_yaw*R2D;//FK_pRFoot_3x1[0];//0.0;
    tempFoot.footprint.rori[1] = FK_RFoot_roll*R2D;//FK_pRFoot_3x1[1];//-0.105;
    tempFoot.footprint.rori[2] = FK_RFoot_pitch*R2D;//FK_pRFoot_3x1[2];//0.0;


    tempFoot.footprint.lfoot[0] = FK_pLFoot_3x1[0];//0.0;
    tempFoot.footprint.lfoot[1] = FK_pLFoot_3x1[1];//0.105;
    tempFoot.footprint.lfoot[2] = FK_pLFoot_3x1[2];

    tempFoot.footprint.lori[0] = FK_LFoot_yaw*R2D;//FK_pLFoot_3x1[0];//0.0;
    tempFoot.footprint.lori[1] = FK_LFoot_roll*R2D;//FK_pLFoot_3x1[1];//0.105;
    tempFoot.footprint.lori[2] = FK_LFoot_pitch*R2D;//FK_pLFoot_3x1[2];

    printf(">>>>>>>>>> Foot Print Initial Value <<<<<<<<<<< \n");
    printf("right x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.rfoot[0],tempFoot.footprint.rfoot[1],tempFoot.footprint.rfoot[2],tempFoot.footprint.rori[0],tempFoot.footprint.rori[1],tempFoot.footprint.rori[2]);
    printf("left  x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.lfoot[0],tempFoot.footprint.lfoot[1],tempFoot.footprint.lfoot[2],tempFoot.footprint.lori[0],tempFoot.footprint.lori[1],tempFoot.footprint.lori[2]);

    tempFoot.time.dsp_time = 0.0;
    tempFoot.time.ssp_time = 0.0;
    tempFoot.info = FOOTINFO_NO;//FOOTINFO_FIRST_STEP;

//    push_short_foot(tempFoot);
    memcpy(&prev_foot, &tempFoot, sizeof(_footprint_info));

    int test_short_foot_num = step_num;

    if(dir != MPC_FORWARD_WALKING)
    {
        if(step_num%2 == 0)
        {
            step_num = step_num+1;
        }

        test_short_foot_num = step_num;

        if(test_short_foot_num < 3)
        {
            test_short_foot_num = 3;
        }

    }


    if(step_offset<0.25)
    {
        step_offset = 0.25;
    }

    //printf("step_num : %d \n    step_length: %f   step_angle: %f \n",step_num,step_length,step_angle);

    int temp_index = 0;
    int temp_head = ring_short_head;

    printf("short foot num:%d     Rinf Short head : %d     Rinf Short tail : %d\n",test_short_foot_num,ring_short_head,ring_short_tail);

    if(dir == MPC_FORWARD_WALKING)
    {
        FILE_LOG(logSUCCESS) << "Forward Foot print Generation  ";

        for(int i=0; i<test_short_foot_num; i++){
            if(i == 0){
                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.lfoot[0] + step_length;
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.rfoot[0] + step_length;
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }
                //FILE_LOG(logERROR) << i << ": " << prev_foot.footprint.rfoot[0] << ", " << prev_foot.footprint.lfoot[0];
                tempFoot.time.dsp_time = 1.2;
                tempFoot.time.ssp_time = 0.8;

            }else if(i == test_short_foot_num-2){
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] =tempFoot.footprint.lfoot[0];
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.rfoot[0];
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }

                tempFoot.time.dsp_time = 0.1;
                tempFoot.time.ssp_time = 0.8;

            }else if(i == test_short_foot_num-1){
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }
                tempFoot.time.dsp_time = 1.5;
                tempFoot.time.ssp_time = 0.0;
            }else{
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.lfoot[0] + step_length;
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.rfoot[0] + step_length;
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }
                //FILE_LOG(logWARNING) << i << ": " << short_foot[temp_index].footprint.rfoot[0] << ", " << short_foot[temp_index].footprint.lfoot[0];
                tempFoot.time.dsp_time = 0.1;
                tempFoot.time.ssp_time = 0.8;
            }

            printf(">>>>>>>>>> Foot Print <<<<<<<<<<< \n");
            printf(">>>>>>>>>> Foot Print <<<<<<<<<<< \n");
            printf("right x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.rfoot[0],tempFoot.footprint.rfoot[1],tempFoot.footprint.rfoot[2],tempFoot.footprint.rori[0],tempFoot.footprint.rori[1],tempFoot.footprint.rori[2]);
            printf("left  x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.lfoot[0],tempFoot.footprint.lfoot[1],tempFoot.footprint.lfoot[2],tempFoot.footprint.lori[0],tempFoot.footprint.lori[1],tempFoot.footprint.lori[2]);



            push_short_foot(tempFoot);
            right_left ^= 1;


        }

    }else if(dir == MPC_LEFT_WALKING)
    {
        FILE_LOG(logSUCCESS) << "Left Foot print Generation";
        for(int i=0; i<test_short_foot_num; i++){
            if(i == 0){
                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1] + step_length;
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1] + step_length;
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }
                FILE_LOG(logERROR) << i << ": " << prev_foot.footprint.rfoot[0] << ", " << prev_foot.footprint.lfoot[0];
                tempFoot.time.dsp_time = 1.5;
                tempFoot.time.ssp_time = 0.8;

//            }else if(i == test_short_foot_num-2){
//                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

//                if(right_left == 0){
//                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
//                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
//                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

//                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
//                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
//                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
//                }else{
//                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
//                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
//                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

//                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
//                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
//                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
//                }

//                tempFoot.time.dsp_time = 0.1;
//                tempFoot.time.ssp_time = 0.8;

            }else if(i == test_short_foot_num-1)
            {
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }
                tempFoot.time.dsp_time = 1.5;
                tempFoot.time.ssp_time = 0.0;
            }else
            {
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1] + step_length;
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];


                }else{
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1] + step_length;
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }
                FILE_LOG(logWARNING) << i << ": " << short_foot[temp_index].footprint.rfoot[0] << ", " << short_foot[temp_index].footprint.lfoot[0];
                tempFoot.time.dsp_time = 0.1;
                tempFoot.time.ssp_time = 0.8;
            }

//            printf(">>>>>>>>>> Foot Print <<<<<<<<<<< \n");
//            printf(">>>>>>>>>> Foot Print <<<<<<<<<<< \n");
//            printf(">>>>>>>>>> Foot Print <<<<<<<<<<< \n");
//            printf("right x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.rfoot[0],tempFoot.footprint.rfoot[1],tempFoot.footprint.rfoot[2],tempFoot.footprint.rori[0],tempFoot.footprint.rori[1],tempFoot.footprint.rori[2]);
//            printf("left  x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.lfoot[0],tempFoot.footprint.lfoot[1],tempFoot.footprint.lfoot[2],tempFoot.footprint.lori[0],tempFoot.footprint.lori[1],tempFoot.footprint.lori[2]);

            push_short_foot(tempFoot);
            right_left ^= 1;


        }
    }else if(dir == MPC_RIGHT_WALKING)
    {
        FILE_LOG(logSUCCESS) << "Right Foot print Generation  ";
        for(int i=0; i<test_short_foot_num; i++){
            if(i == 0){
                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1] - step_length;
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1] - step_length;
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }
                FILE_LOG(logERROR) << i << ": " << prev_foot.footprint.rfoot[0] << ", " << prev_foot.footprint.lfoot[0];
                tempFoot.time.dsp_time = 1.5;
                tempFoot.time.ssp_time = 0.8;

            }else if(i == test_short_foot_num-1){
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }
                tempFoot.time.dsp_time = 1.5;
                tempFoot.time.ssp_time = 0.0;
            }else{
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1] - step_length;
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1] - step_length;
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }
                FILE_LOG(logWARNING) << i << ": " << short_foot[temp_index].footprint.rfoot[0] << ", " << short_foot[temp_index].footprint.lfoot[0];
                tempFoot.time.dsp_time = 0.1;
                tempFoot.time.ssp_time = 0.8;
            }

            printf(">>>>>>>>>> Foot Print <<<<<<<<<<< \n");
            printf(">>>>>>>>>> Foot Print <<<<<<<<<<< \n");
            printf(">>>>>>>>>> Foot Print <<<<<<<<<<< \n");
            printf("right x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.rfoot[0],tempFoot.footprint.rfoot[1],tempFoot.footprint.rfoot[2],tempFoot.footprint.rori[0],tempFoot.footprint.rori[1],tempFoot.footprint.rori[2]);
            printf("left  x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.lfoot[0],tempFoot.footprint.lfoot[1],tempFoot.footprint.lfoot[2],tempFoot.footprint.lori[0],tempFoot.footprint.lori[1],tempFoot.footprint.lori[2]);

            push_short_foot(tempFoot);
            right_left ^= 1;


        }
    }else if(dir == MPC_CCW_WALKING)
    {


        FILE_LOG(logSUCCESS) << "CCW Foot print Generation  ";

        right_left = 1;

        for(int i=0; i<test_short_foot_num; i++){

            if(right_left == 1)
            {
                //rotate left direction ,, left foot first.
                rl = 1;

            }else
            {
                rl = -1;
            }

            if(i == 0){
                if(right_left == 0){
                    tempFoot.footprint.rori[0] = tempFoot.footprint.lori[0] + step_angle*R2D;
                    tempFoot.footprint.rori[1] = tempFoot.footprint.rori[1];
                    tempFoot.footprint.rori[2] = tempFoot.footprint.rori[2];

                    tempFoot.footprint.lori[0] = tempFoot.footprint.lori[0];
                    tempFoot.footprint.lori[1] = tempFoot.footprint.lori[1];
                    tempFoot.footprint.lori[2] = tempFoot.footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.lfoot[0] + step_length*cos(tempFoot.footprint.rori[0]*D2R) - step_offset*sin(tempFoot.footprint.rori[0]*D2R)*rl;
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.lfoot[1] + step_length*sin(tempFoot.footprint.rori[0]*D2R) + step_offset*cos(tempFoot.footprint.rori[0]*D2R)*rl;
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }else
                {
                    tempFoot.footprint.rori[0] = tempFoot.footprint.rori[0];
                    tempFoot.footprint.rori[1] = tempFoot.footprint.rori[1];
                    tempFoot.footprint.rori[2] = tempFoot.footprint.rori[2];

                    tempFoot.footprint.lori[0] = tempFoot.footprint.rori[0] + step_angle*R2D;
                    tempFoot.footprint.lori[1] = tempFoot.footprint.lori[1];
                    tempFoot.footprint.lori[2] = tempFoot.footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.rfoot[0] + step_length*cos(tempFoot.footprint.lori[0]*D2R) - step_offset*sin(tempFoot.footprint.lori[0]*D2R)*rl;
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.rfoot[1] + step_length*sin(tempFoot.footprint.lori[0]*D2R) + step_offset*cos(tempFoot.footprint.lori[0]*D2R)*rl;
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }

                FILE_LOG(logERROR) << i << ": " << prev_foot.footprint.rfoot[0] << ", " << prev_foot.footprint.lfoot[0];
                tempFoot.time.dsp_time = 1.5;
                tempFoot.time.ssp_time = 0.8;



             }
            else if(i == test_short_foot_num-2){
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.lori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.lfoot[0] + LPEL2PEL*sin(tempFoot.footprint.lori[0]*D2R);
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.lfoot[1] - LPEL2PEL*cos(tempFoot.footprint.lori[0]*D2R);
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.rfoot[0] + LPEL2PEL*sin(tempFoot.footprint.rori[0]*D2R);
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.rfoot[1] + LPEL2PEL*cos(tempFoot.footprint.rori[0]*D2R);
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }

                tempFoot.time.dsp_time = 0.1;
                tempFoot.time.ssp_time = 0.8;

            }else if(i == test_short_foot_num-1){
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];//  + LPEL2PEL*sin(tempFoot.footprint.rori[0]*D2R);
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];//  - LPEL2PEL*cos(tempFoot.footprint.rori[0]*D2R);
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];//  + LPEL2PEL*sin(tempFoot.footprint.lori[0]*D2R);
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];//  + LPEL2PEL*cos(tempFoot.footprint.lori[0]*D2R);
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }

                tempFoot.time.dsp_time = 1.5;
                tempFoot.time.ssp_time = 0.0;

            }else{
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){   
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.lori[0] + step_angle*R2D;
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.lfoot[0] + step_length*cos(tempFoot.footprint.rori[0]*D2R) - step_offset*sin(tempFoot.footprint.rori[0]*D2R)*rl;
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.lfoot[1] + step_length*sin(tempFoot.footprint.rori[0]*D2R) + step_offset*cos(tempFoot.footprint.rori[0]*D2R)*rl;
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.rori[0] + step_angle*R2D;
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.rfoot[0] + step_length*cos(tempFoot.footprint.lori[0]*D2R) - step_offset*sin(tempFoot.footprint.lori[0]*D2R)*rl;
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.rfoot[1] + step_length*sin(tempFoot.footprint.lori[0]*D2R) + step_offset*cos(tempFoot.footprint.lori[0]*D2R)*rl;
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }

                FILE_LOG(logWARNING) << i << ": " << short_foot[temp_index].footprint.rfoot[0] << ", " << short_foot[temp_index].footprint.lfoot[0];

                tempFoot.time.dsp_time = 0.1;
                tempFoot.time.ssp_time = 0.8;
            }

            printf(">>>>>>>>>> %d Foot Print <<<<<<<<<<< \n",i);

            printf("right x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.rfoot[0],tempFoot.footprint.rfoot[1],tempFoot.footprint.rfoot[2],tempFoot.footprint.rori[0],tempFoot.footprint.rori[1],tempFoot.footprint.rori[2]);
            printf("left  x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.lfoot[0],tempFoot.footprint.lfoot[1],tempFoot.footprint.lfoot[2],tempFoot.footprint.lori[0],tempFoot.footprint.lori[1],tempFoot.footprint.lori[2]);

            push_short_foot(tempFoot);
            right_left ^= 1;
        }
    }else if(dir == MPC_CW_WALKING)
    {
        if(step_angle > 0)
        {
            step_angle  = -step_angle;
        }

        FILE_LOG(logSUCCESS) << "CW Foot print Generation  ";

        right_left = 0;

        for(int i=0; i<test_short_foot_num; i++){

            if(right_left == 1)
            {
                //rotate left direction ,, left foot first.
                rl = 1;
            }else
            {
                rl = -1;
            }

            if(i == 0){
                if(right_left == 0){
                    tempFoot.footprint.rori[0] = tempFoot.footprint.lori[0] + step_angle*R2D;
                    tempFoot.footprint.rori[1] = tempFoot.footprint.rori[1];
                    tempFoot.footprint.rori[2] = tempFoot.footprint.rori[2];

                    tempFoot.footprint.lori[0] = tempFoot.footprint.lori[0];
                    tempFoot.footprint.lori[1] = tempFoot.footprint.lori[1];
                    tempFoot.footprint.lori[2] = tempFoot.footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.lfoot[0] + step_length*cos(tempFoot.footprint.rori[0]*D2R) - step_offset*sin(tempFoot.footprint.rori[0]*D2R)*rl;
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.lfoot[1] + step_length*sin(tempFoot.footprint.rori[0]*D2R) + step_offset*cos(tempFoot.footprint.rori[0]*D2R)*rl;
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rori[0] = tempFoot.footprint.rori[0];
                    tempFoot.footprint.rori[1] = tempFoot.footprint.rori[1];
                    tempFoot.footprint.rori[2] = tempFoot.footprint.rori[2];

                    tempFoot.footprint.lori[0] = tempFoot.footprint.rori[0] + step_angle*R2D;
                    tempFoot.footprint.lori[1] = tempFoot.footprint.lori[1];
                    tempFoot.footprint.lori[2] = tempFoot.footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.rfoot[0] + step_length*cos(tempFoot.footprint.lori[0]*D2R) - step_offset*sin(tempFoot.footprint.lori[0]*D2R)*rl;
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.rfoot[1] + step_length*sin(tempFoot.footprint.lori[0]*D2R) + step_offset*cos(tempFoot.footprint.lori[0]*D2R)*rl;
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }
                FILE_LOG(logERROR) << i << ": " << prev_foot.footprint.rfoot[0] << ", " << prev_foot.footprint.lfoot[0];
                tempFoot.time.dsp_time = 1.5;
                tempFoot.time.ssp_time = 0.8;

//            }else if(i == test_short_foot_num-3){
//                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

//                printf("**********test_short_footnum -3 \n");
//                if(right_left == 0){
//                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.lori[0];
//                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
//                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

//                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.lori[0];
//                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
//                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

//                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.lfoot[0] + LPEL2PEL*sin(tempFoot.footprint.rori[0]*D2R);
//                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.lfoot[1] + LPEL2PEL*cos(tempFoot.footprint.rori[0]*D2R);
//                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

//                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
//                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
//                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];

//                    printf("rightfoot \n");
//                }else{
//                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.rori[0];
//                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
//                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

//                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.rori[0];
//                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
//                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

//                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
//                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
//                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

//                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.rfoot[0] - LPEL2PEL*sin(tempFoot.footprint.lori[0]*D2R);
//                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.rfoot[1] + LPEL2PEL*cos(tempFoot.footprint.lori[0]*D2R);
//                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
//                    printf("leftfoot \n");
//                }

//                tempFoot.time.dsp_time = 0.1;
//                tempFoot.time.ssp_time = 0.8;

            }else if(i == test_short_foot_num-2){
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;
                if(right_left == 0){
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];


//                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
//                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
//                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

//                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
//                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
//                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.lfoot[0] + LPEL2PEL*sin(tempFoot.footprint.lori[0]*D2R);
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.lfoot[1] + LPEL2PEL*cos(tempFoot.footprint.lori[0]*D2R);
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];

                }else{
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];


                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.rfoot[0] - LPEL2PEL*sin(tempFoot.footprint.rori[0]*D2R);
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.rfoot[1] + LPEL2PEL*cos(tempFoot.footprint.rori[0]*D2R);
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }

                tempFoot.time.dsp_time = 0.1;
                tempFoot.time.ssp_time = 0.8;

            }else if(i == test_short_foot_num-1){
                temp_index = (i + temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];


                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }
                tempFoot.time.dsp_time = 1.5;
                tempFoot.time.ssp_time = 0.0;
            }else{
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.lori[0] + step_angle*R2D;
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.lfoot[0] + step_length*cos(tempFoot.footprint.rori[0]*D2R) - step_offset*sin(tempFoot.footprint.rori[0]*D2R)*rl;
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.lfoot[1] + step_length*sin(tempFoot.footprint.rori[0]*D2R) + step_offset*cos(tempFoot.footprint.rori[0]*D2R)*rl;
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];

                }else{
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.rori[0] + step_angle*R2D;
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.rfoot[0] + step_length*cos(tempFoot.footprint.lori[0]*D2R) - step_offset*sin(tempFoot.footprint.lori[0]*D2R)*rl;
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.rfoot[1] + step_length*sin(tempFoot.footprint.lori[0]*D2R) + step_offset*cos(tempFoot.footprint.lori[0]*D2R)*rl;
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];

                }
                FILE_LOG(logWARNING) << i << ": " << short_foot[temp_index].footprint.rfoot[0] << ", " << short_foot[temp_index].footprint.lfoot[0];
                tempFoot.time.dsp_time = 0.1;
                tempFoot.time.ssp_time = 0.8;
            }

            printf(">>>>>>>>>> %d Foot Print <<<<<<<<<<< \n",i);
            printf("right x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.rfoot[0],tempFoot.footprint.rfoot[1],tempFoot.footprint.rfoot[2],tempFoot.footprint.rori[0],tempFoot.footprint.rori[1],tempFoot.footprint.rori[2]);
            printf("left  x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.lfoot[0],tempFoot.footprint.lfoot[1],tempFoot.footprint.lfoot[2],tempFoot.footprint.lori[0],tempFoot.footprint.lori[1],tempFoot.footprint.lori[2]);

            push_short_foot(tempFoot);
            right_left ^= 1;
        }
    }

//    for(int i=0; i<test_short_foot_num; i++){
//        pull_short_foot(tempFoot);
//        printf("%d RFP:  %f  %f  %f   %f  %f  %f\n",i,tempFoot.footprint.rfoot[0],tempFoot.footprint.rfoot[1],tempFoot.footprint.rfoot[2],tempFoot.footprint.lfoot[0],tempFoot.footprint.lfoot[1],tempFoot.footprint.lfoot[2]);
//        printf("%d ssp: %f  dsp: %f \n",i,tempFoot.time.ssp_time,tempFoot.time.dsp_time);
//    }
    // =================================================================


//    double U0[15] = {0.,},UK[15][3] = {{0.,},}, IND[2] = {0.,};

//    for(int timecnt = 0; timecnt < 200*2; timecnt++){
//        update_window();

//        save();

//        if(timecnt%20 == 0){
////            UKSEL(20,U0,UK,IND);
//            printf("%f [%d](%.2f, %.2f)):  %s   preFoot X: (%.3f, %.3f) ==> target Foot X:(%.3f, %.3f)  preFoot y: (%.3f, %.3f) ==> target Foot y:(%.3f, %.3f)  previnfo: %d  ZMP:(%.3f, %.3f)\n",
//                   timecnt/200.0,
//                   target_foot[0].info,
//                   target_foot[0].time.dsp_time,
//                   target_foot[0].time.ssp_time,
//                   STATE_NAME[window[0].state],
//                   prev_foot.footprint.rfoot[0],
//                   prev_foot.footprint.lfoot[0],
//                   target_foot[0].footprint.rfoot[0],
//                   target_foot[0].footprint.lfoot[0],

//                    prev_foot.footprint.rfoot[1],
//                    prev_foot.footprint.lfoot[1],
//                    target_foot[0].footprint.rfoot[1],
//                    target_foot[0].footprint.lfoot[1],
//                    prev_foot.info,

//                    window[0].zmp.x,
//                    window[0].zmp.y
//                    );

//            for(int k = 0;k<15;k++){
//            printf("U0: %f  UK: %f  %f  %f \n",U0[k],UK[k][0],UK[k][1],UK[k][2]);
//            }


//        }
//    }
}


void FPG_TEST2(int dir,int step_num,double step_length,double step_angle,double step_offset,double LPEL2PEL)
{
    // example short footprint information =============================



    if(step_angle>15.0)
    {
        step_angle = 15.0;
    }else if(step_angle<-15.0)
    {
        step_angle = -15.0;
    }

    step_angle = step_angle*D2R;

    double rl = 0;

    int right_left = 0;

    if(dir == MPC_RIGHT_WALKING){
        right_left = 0;
    }else if(dir == MPC_LEFT_WALKING){
        right_left  = 1;
    }

    printf("***********step angle: %f     step offset: %f  \n",step_angle,step_offset);

    _footprint_info dummyfoot;
    // remove old foot printf;
    while(pull_short_foot(dummyfoot)){
        ;
    }

    _footprint_info tempFoot;
//    _footprint_info prevfoot;


//    memcpy(&prevfoot, &last_short_foot, sizeof(_footprint_info));


    tempFoot.footprint.rfoot[0] = FK_pRFoot_3x1[0];//0.0;
    tempFoot.footprint.rfoot[1] = FK_pRFoot_3x1[1];//-0.105;
    tempFoot.footprint.rfoot[2] = FK_pRFoot_3x1[2];//0.0;

    tempFoot.footprint.rori[0] = FK_RFoot_yaw*R2D;//FK_pRFoot_3x1[0];//0.0;
    tempFoot.footprint.rori[1] = FK_RFoot_roll*R2D;//FK_pRFoot_3x1[1];//-0.105;
    tempFoot.footprint.rori[2] = FK_RFoot_pitch*R2D;//FK_pRFoot_3x1[2];//0.0;


    tempFoot.footprint.lfoot[0] = FK_pLFoot_3x1[0];//0.0;
    tempFoot.footprint.lfoot[1] = FK_pLFoot_3x1[1];//0.105;
    tempFoot.footprint.lfoot[2] = FK_pLFoot_3x1[2];

    tempFoot.footprint.lori[0] = FK_LFoot_yaw*R2D;//FK_pLFoot_3x1[0];//0.0;
    tempFoot.footprint.lori[1] = FK_LFoot_roll*R2D;//FK_pLFoot_3x1[1];//0.105;
    tempFoot.footprint.lori[2] = FK_LFoot_pitch*R2D;//FK_pLFoot_3x1[2];


    printf(">>>>>>>>>> Foot Print Initial Value <<<<<<<<<<< \n");
    printf("right x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.rfoot[0],tempFoot.footprint.rfoot[1],tempFoot.footprint.rfoot[2],tempFoot.footprint.rori[0],tempFoot.footprint.rori[1],tempFoot.footprint.rori[2]);
    printf("left  x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.lfoot[0],tempFoot.footprint.lfoot[1],tempFoot.footprint.lfoot[2],tempFoot.footprint.lori[0],tempFoot.footprint.lori[1],tempFoot.footprint.lori[2]);

    tempFoot.time.dsp_time = 0.0;
    tempFoot.time.ssp_time = 0.0;
    tempFoot.info = FOOTINFO_NO;//FOOTINFO_FIRST_STEP;

//    push_short_foot(tempFoot);
    memcpy(&prev_foot, &tempFoot, sizeof(_footprint_info));


    int test_short_foot_num = step_num;

    if(dir != MPC_FORWARD_WALKING)
    {
        if(step_num%2 == 0){
            step_num = step_num+1;
        }

        test_short_foot_num = step_num;

        if(test_short_foot_num ){
            test_short_foot_num = 3;
        }
    }


    if(step_offset<0.25)
    {
        step_offset = 0.25;
    }



    //printf("step_num : %d \n    step_length: %f   step_angle: %f \n",step_num,step_length,step_angle);

    int temp_index = 0;
    int temp_head = ring_short_head;

    printf("---------- Rinf Short head : %d     Rinf Short tail : %d\n",ring_short_head,ring_short_tail);

    if(dir == MPC_FORWARD_WALKING)
    {
        FILE_LOG(logSUCCESS) << "Forward Foot print Generation  ";

        for(int i=0; i<test_short_foot_num; i++){
            if(i == 0){
                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.lfoot[0] + step_length;
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.rfoot[0] + step_length;
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }
                //FILE_LOG(logERROR) << i << ": " << prev_foot.footprint.rfoot[0] << ", " << prev_foot.footprint.lfoot[0];
                tempFoot.time.dsp_time = 1.5;
                tempFoot.time.ssp_time = 0.8;

            }else if(i == test_short_foot_num-2){
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }

                tempFoot.time.dsp_time = 0.2;
                tempFoot.time.ssp_time = 0.8;

            }else if(i == test_short_foot_num-1){
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }

                tempFoot.time.dsp_time = 1.5;
                tempFoot.time.ssp_time = 0.0;

            }else{
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.lfoot[0] + step_length;
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.rfoot[0] + step_length;
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }
                //FILE_LOG(logWARNING) << i << ": " << short_foot[temp_index].footprint.rfoot[0] << ", " << short_foot[temp_index].footprint.lfoot[0];
                tempFoot.time.dsp_time = 0.2;
                tempFoot.time.ssp_time = 0.8;
            }

            printf(">>>>>>>>>> Foot Print <<<<<<<<<<< \n");
            printf(">>>>>>>>>> Foot Print <<<<<<<<<<< \n");
            printf("right x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.rfoot[0],tempFoot.footprint.rfoot[1],tempFoot.footprint.rfoot[2],tempFoot.footprint.rori[0],tempFoot.footprint.rori[1],tempFoot.footprint.rori[2]);
            printf("left  x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.lfoot[0],tempFoot.footprint.lfoot[1],tempFoot.footprint.lfoot[2],tempFoot.footprint.lori[0],tempFoot.footprint.lori[1],tempFoot.footprint.lori[2]);

            push_short_foot(tempFoot);
            right_left ^= 1;
        }

    }else if(dir == MPC_LEFT_WALKING)
    {
        FILE_LOG(logSUCCESS) << "Left Foot print Generation";
        for(int i=0; i<test_short_foot_num; i++){
            if(i == 0){
                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1] + step_length;
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1] + step_length;
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }
                FILE_LOG(logERROR) << i << ": " << prev_foot.footprint.rfoot[0] << ", " << prev_foot.footprint.lfoot[0];
                tempFoot.time.dsp_time = 1.5;
                tempFoot.time.ssp_time = 0.8;

            }else if(i == test_short_foot_num-1)
            {
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }
                tempFoot.time.dsp_time = 1.5;
                tempFoot.time.ssp_time = 0.0;
            }else
            {
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1] + step_length;
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];


                }else{
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1] + step_length;
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }
                FILE_LOG(logWARNING) << i << ": " << short_foot[temp_index].footprint.rfoot[0] << ", " << short_foot[temp_index].footprint.lfoot[0];
                tempFoot.time.dsp_time = 0.1;
                tempFoot.time.ssp_time = 0.8;
            }

//            printf(">>>>>>>>>> Foot Print <<<<<<<<<<< \n");
//            printf(">>>>>>>>>> Foot Print <<<<<<<<<<< \n");
//            printf(">>>>>>>>>> Foot Print <<<<<<<<<<< \n");
//            printf("right x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.rfoot[0],tempFoot.footprint.rfoot[1],tempFoot.footprint.rfoot[2],tempFoot.footprint.rori[0],tempFoot.footprint.rori[1],tempFoot.footprint.rori[2]);
//            printf("left  x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.lfoot[0],tempFoot.footprint.lfoot[1],tempFoot.footprint.lfoot[2],tempFoot.footprint.lori[0],tempFoot.footprint.lori[1],tempFoot.footprint.lori[2]);

            push_short_foot(tempFoot);
            right_left ^= 1;


        }
    }else if(dir == MPC_RIGHT_WALKING)
    {
        FILE_LOG(logSUCCESS) << "Right Foot print Generation  ";
        for(int i=0; i<test_short_foot_num; i++){
            if(i == 0){
                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1] - step_length;
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1] - step_length;
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }
                FILE_LOG(logERROR) << i << ": " << prev_foot.footprint.rfoot[0] << ", " << prev_foot.footprint.lfoot[0];
                tempFoot.time.dsp_time = 1.5;
                tempFoot.time.ssp_time = 0.8;

//            }else if(i == test_short_foot_num-2){
//                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

//                if(right_left == 0){
//                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
//                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
//                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

//                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
//                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
//                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];

//                }else{

//                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
//                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
//                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

//                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
//                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
//                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
//                }

//                tempFoot.time.dsp_time = 0.1;
//                tempFoot.time.ssp_time = 0.8;

            }else if(i == test_short_foot_num-1){
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }
                tempFoot.time.dsp_time = 1.5;
                tempFoot.time.ssp_time = 0.0;
            }else{
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1] - step_length;
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1] - step_length;
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }
                FILE_LOG(logWARNING) << i << ": " << short_foot[temp_index].footprint.rfoot[0] << ", " << short_foot[temp_index].footprint.lfoot[0];
                tempFoot.time.dsp_time = 0.1;
                tempFoot.time.ssp_time = 0.8;
            }

            printf(">>>>>>>>>> Foot Print <<<<<<<<<<< \n");
            printf(">>>>>>>>>> Foot Print <<<<<<<<<<< \n");
            printf(">>>>>>>>>> Foot Print <<<<<<<<<<< \n");
            printf("right x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.rfoot[0],tempFoot.footprint.rfoot[1],tempFoot.footprint.rfoot[2],tempFoot.footprint.rori[0],tempFoot.footprint.rori[1],tempFoot.footprint.rori[2]);
            printf("left  x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.lfoot[0],tempFoot.footprint.lfoot[1],tempFoot.footprint.lfoot[2],tempFoot.footprint.lori[0],tempFoot.footprint.lori[1],tempFoot.footprint.lori[2]);

            push_short_foot(tempFoot);
            right_left ^= 1;


        }
    }else if(dir == MPC_CCW_WALKING)
    {


        FILE_LOG(logSUCCESS) << "CCW Foot print Generation  ";

        right_left = 1;

        for(int i=0; i<test_short_foot_num; i++){

            if(right_left == 1)
            {
                //rotate left direction ,, left foot first.
                rl = 1;

            }else
            {
                rl = -1;
            }

            if(i == 0){
                if(right_left == 0){
                    tempFoot.footprint.rori[0] = tempFoot.footprint.lori[0] + step_angle*R2D;
                    tempFoot.footprint.rori[1] = tempFoot.footprint.rori[1];
                    tempFoot.footprint.rori[2] = tempFoot.footprint.rori[2];

                    tempFoot.footprint.lori[0] = tempFoot.footprint.lori[0];
                    tempFoot.footprint.lori[1] = tempFoot.footprint.lori[1];
                    tempFoot.footprint.lori[2] = tempFoot.footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.lfoot[0] + step_length*cos(tempFoot.footprint.rori[0]*D2R) - step_offset*sin(tempFoot.footprint.rori[0]*D2R)*rl;
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.lfoot[1] + step_length*sin(tempFoot.footprint.rori[0]*D2R) + step_offset*cos(tempFoot.footprint.rori[0]*D2R)*rl;
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }else
                {
                    tempFoot.footprint.rori[0] = tempFoot.footprint.rori[0];
                    tempFoot.footprint.rori[1] = tempFoot.footprint.rori[1];
                    tempFoot.footprint.rori[2] = tempFoot.footprint.rori[2];

                    tempFoot.footprint.lori[0] = tempFoot.footprint.rori[0] + step_angle*R2D;
                    tempFoot.footprint.lori[1] = tempFoot.footprint.lori[1];
                    tempFoot.footprint.lori[2] = tempFoot.footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.rfoot[0] + step_length*cos(tempFoot.footprint.lori[0]*D2R) - step_offset*sin(tempFoot.footprint.lori[0]*D2R)*rl;
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.rfoot[1] + step_length*sin(tempFoot.footprint.lori[0]*D2R) + step_offset*cos(tempFoot.footprint.lori[0]*D2R)*rl;
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }

                FILE_LOG(logERROR) << i << ": " << prev_foot.footprint.rfoot[0] << ", " << prev_foot.footprint.lfoot[0];
                tempFoot.time.dsp_time = 1.5;
                tempFoot.time.ssp_time = 0.8;

//            }else if(i == test_short_foot_num-3){
//                 temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;
//printf("**********test_short_footnum -3 \n");
//                 if(right_left == 0){
//                     tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.lori[0];
//                     tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
//                     tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

//                     tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.lori[0];
//                     tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
//                     tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

//                     tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.lfoot[0] + LPEL2PEL*sin(tempFoot.footprint.rori[0]*D2R);
//                     tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.lfoot[1] - LPEL2PEL*cos(tempFoot.footprint.rori[0]*D2R);
//                     tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

//                     tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
//                     tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
//                     tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];

//                 }else{
//                     tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.rori[0];
//                     tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
//                     tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

//                     tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.rori[0];
//                     tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
//                     tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

//                     tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.lfoot[0];
//                     tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.lfoot[1];
//                     tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

//                     tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.rfoot[0] + LPEL2PEL*sin(tempFoot.footprint.lori[0]*D2R);
//                     tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.rfoot[1] + LPEL2PEL*cos(tempFoot.footprint.lori[0]*D2R);
//                     tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
//                 }

//                 tempFoot.time.dsp_time = 0.1;
//                 tempFoot.time.ssp_time = 0.8;

             }
            else if(i == test_short_foot_num-2){
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.lori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.lfoot[0] + LPEL2PEL*sin(tempFoot.footprint.lori[0]*D2R);
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.lfoot[1] - LPEL2PEL*cos(tempFoot.footprint.lori[0]*D2R);
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.rfoot[0] + LPEL2PEL*sin(tempFoot.footprint.rori[0]*D2R);
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.rfoot[1] + LPEL2PEL*cos(tempFoot.footprint.rori[0]*D2R);
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }

                tempFoot.time.dsp_time = 0.1;
                tempFoot.time.ssp_time = 0.8;

            }else if(i == test_short_foot_num-1){
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];//  + LPEL2PEL*sin(tempFoot.footprint.rori[0]*D2R);
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];//  - LPEL2PEL*cos(tempFoot.footprint.rori[0]*D2R);
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];//  + LPEL2PEL*sin(tempFoot.footprint.lori[0]*D2R);
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];//  + LPEL2PEL*cos(tempFoot.footprint.lori[0]*D2R);
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }

                tempFoot.time.dsp_time = 1.5;
                tempFoot.time.ssp_time = 0.0;

            }else{
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.lori[0] + step_angle*R2D;
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.lfoot[0] + step_length*cos(tempFoot.footprint.rori[0]*D2R) - step_offset*sin(tempFoot.footprint.rori[0]*D2R)*rl;
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.lfoot[1] + step_length*sin(tempFoot.footprint.rori[0]*D2R) + step_offset*cos(tempFoot.footprint.rori[0]*D2R)*rl;
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.rori[0] + step_angle*R2D;
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.rfoot[0] + step_length*cos(tempFoot.footprint.lori[0]*D2R) - step_offset*sin(tempFoot.footprint.lori[0]*D2R)*rl;
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.rfoot[1] + step_length*sin(tempFoot.footprint.lori[0]*D2R) + step_offset*cos(tempFoot.footprint.lori[0]*D2R)*rl;
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }

                FILE_LOG(logWARNING) << i << ": " << short_foot[temp_index].footprint.rfoot[0] << ", " << short_foot[temp_index].footprint.lfoot[0];

                tempFoot.time.dsp_time = 0.1;
                tempFoot.time.ssp_time = 0.8;
            }

            printf(">>>>>>>>>> %d Foot Print <<<<<<<<<<< \n",i);

            printf("right x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.rfoot[0],tempFoot.footprint.rfoot[1],tempFoot.footprint.rfoot[2],tempFoot.footprint.rori[0],tempFoot.footprint.rori[1],tempFoot.footprint.rori[2]);
            printf("left  x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.lfoot[0],tempFoot.footprint.lfoot[1],tempFoot.footprint.lfoot[2],tempFoot.footprint.lori[0],tempFoot.footprint.lori[1],tempFoot.footprint.lori[2]);

            push_short_foot(tempFoot);
            right_left ^= 1;
        }
    }else if(dir == MPC_CW_WALKING)
    {
        if(step_angle > 0)
        {
            step_angle  = -step_angle;
        }

        FILE_LOG(logSUCCESS) << "CW Foot print Generation  ";

        right_left = 0;

        for(int i=0; i<test_short_foot_num; i++){

            if(right_left == 1)
            {
                //rotate left direction ,, left foot first.
                rl = 1;
            }else
            {
                rl = -1;
            }

            if(i == 0){
                if(right_left == 0){
                    tempFoot.footprint.rori[0] = tempFoot.footprint.lori[0] + step_angle*R2D;
                    tempFoot.footprint.rori[1] = tempFoot.footprint.rori[1];
                    tempFoot.footprint.rori[2] = tempFoot.footprint.rori[2];

                    tempFoot.footprint.lori[0] = tempFoot.footprint.lori[0];
                    tempFoot.footprint.lori[1] = tempFoot.footprint.lori[1];
                    tempFoot.footprint.lori[2] = tempFoot.footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.lfoot[0] + step_length*cos(tempFoot.footprint.rori[0]*D2R) - step_offset*sin(tempFoot.footprint.rori[0]*D2R)*rl;
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.lfoot[1] + step_length*sin(tempFoot.footprint.rori[0]*D2R) + step_offset*cos(tempFoot.footprint.rori[0]*D2R)*rl;
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rori[0] = tempFoot.footprint.rori[0];
                    tempFoot.footprint.rori[1] = tempFoot.footprint.rori[1];
                    tempFoot.footprint.rori[2] = tempFoot.footprint.rori[2];

                    tempFoot.footprint.lori[0] = tempFoot.footprint.rori[0] + step_angle*R2D;
                    tempFoot.footprint.lori[1] = tempFoot.footprint.lori[1];
                    tempFoot.footprint.lori[2] = tempFoot.footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = tempFoot.footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = tempFoot.footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = tempFoot.footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = tempFoot.footprint.rfoot[0] + step_length*cos(tempFoot.footprint.lori[0]*D2R) - step_offset*sin(tempFoot.footprint.lori[0]*D2R)*rl;
                    tempFoot.footprint.lfoot[1] = tempFoot.footprint.rfoot[1] + step_length*sin(tempFoot.footprint.lori[0]*D2R) + step_offset*cos(tempFoot.footprint.lori[0]*D2R)*rl;
                    tempFoot.footprint.lfoot[2] = tempFoot.footprint.lfoot[2];
                }
                FILE_LOG(logERROR) << i << ": " << prev_foot.footprint.rfoot[0] << ", " << prev_foot.footprint.lfoot[0];
                tempFoot.time.dsp_time = 1.5;
                tempFoot.time.ssp_time = 0.8;

//            }else if(i == test_short_foot_num-3){
//                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

//                printf("**********test_short_footnum -3 \n");
//                if(right_left == 0){
//                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.lori[0];
//                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
//                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

//                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.lori[0];
//                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
//                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

//                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.lfoot[0] + LPEL2PEL*sin(tempFoot.footprint.rori[0]*D2R);
//                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.lfoot[1] + LPEL2PEL*cos(tempFoot.footprint.rori[0]*D2R);
//                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

//                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
//                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
//                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];

//                    printf("rightfoot \n");
//                }else{
//                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.rori[0];
//                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
//                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

//                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.rori[0];
//                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
//                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

//                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
//                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
//                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

//                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.rfoot[0] - LPEL2PEL*sin(tempFoot.footprint.lori[0]*D2R);
//                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.rfoot[1] + LPEL2PEL*cos(tempFoot.footprint.lori[0]*D2R);
//                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
//                    printf("leftfoot \n");
//                }

//                tempFoot.time.dsp_time = 0.1;
//                tempFoot.time.ssp_time = 0.8;

            }else if(i == test_short_foot_num-2){
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;
                if(right_left == 0){
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];


//                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
//                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
//                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

//                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
//                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
//                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.lfoot[0] + LPEL2PEL*sin(tempFoot.footprint.lori[0]*D2R);
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.lfoot[1] + LPEL2PEL*cos(tempFoot.footprint.lori[0]*D2R);
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];

                }else{
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];


                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.rfoot[0] - LPEL2PEL*sin(tempFoot.footprint.rori[0]*D2R);
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.rfoot[1] + LPEL2PEL*cos(tempFoot.footprint.rori[0]*D2R);
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }

                tempFoot.time.dsp_time = 0.1;
                tempFoot.time.ssp_time = 0.8;

            }else if(i == test_short_foot_num-1){
                temp_index = (i + temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }else{
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];


                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];
                }
                tempFoot.time.dsp_time = 1.5;
                tempFoot.time.ssp_time = 0.0;
            }else{
                temp_index = (i+temp_head-1)%SHORT_FOOT_NUM;

                if(right_left == 0){
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.lori[0] + step_angle*R2D;
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.lori[0];
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.lfoot[0] + step_length*cos(tempFoot.footprint.rori[0]*D2R) - step_offset*sin(tempFoot.footprint.rori[0]*D2R)*rl;
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.lfoot[1] + step_length*sin(tempFoot.footprint.rori[0]*D2R) + step_offset*cos(tempFoot.footprint.rori[0]*D2R)*rl;
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.lfoot[0];
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.lfoot[1];
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];

                }else{
                    tempFoot.footprint.rori[0] = short_foot[temp_index].footprint.rori[0];
                    tempFoot.footprint.rori[1] = short_foot[temp_index].footprint.rori[1];
                    tempFoot.footprint.rori[2] = short_foot[temp_index].footprint.rori[2];

                    tempFoot.footprint.lori[0] = short_foot[temp_index].footprint.rori[0] + step_angle*R2D;
                    tempFoot.footprint.lori[1] = short_foot[temp_index].footprint.lori[1];
                    tempFoot.footprint.lori[2] = short_foot[temp_index].footprint.lori[2];

                    tempFoot.footprint.rfoot[0] = short_foot[temp_index].footprint.rfoot[0];
                    tempFoot.footprint.rfoot[1] = short_foot[temp_index].footprint.rfoot[1];
                    tempFoot.footprint.rfoot[2] = short_foot[temp_index].footprint.rfoot[2];

                    tempFoot.footprint.lfoot[0] = short_foot[temp_index].footprint.rfoot[0] + step_length*cos(tempFoot.footprint.lori[0]*D2R) - step_offset*sin(tempFoot.footprint.lori[0]*D2R)*rl;
                    tempFoot.footprint.lfoot[1] = short_foot[temp_index].footprint.rfoot[1] + step_length*sin(tempFoot.footprint.lori[0]*D2R) + step_offset*cos(tempFoot.footprint.lori[0]*D2R)*rl;
                    tempFoot.footprint.lfoot[2] = short_foot[temp_index].footprint.lfoot[2];

                }
                FILE_LOG(logWARNING) << i << ": " << short_foot[temp_index].footprint.rfoot[0] << ", " << short_foot[temp_index].footprint.lfoot[0];
                tempFoot.time.dsp_time = 0.1;
                tempFoot.time.ssp_time = 0.8;
            }

            printf(">>>>>>>>>> %d Foot Print <<<<<<<<<<< \n",i);
            printf("right x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.rfoot[0],tempFoot.footprint.rfoot[1],tempFoot.footprint.rfoot[2],tempFoot.footprint.rori[0],tempFoot.footprint.rori[1],tempFoot.footprint.rori[2]);
            printf("left  x: %f  y: %f  z: %f  yaw: %f  roll: %f  pitch: %f \n",tempFoot.footprint.lfoot[0],tempFoot.footprint.lfoot[1],tempFoot.footprint.lfoot[2],tempFoot.footprint.lori[0],tempFoot.footprint.lori[1],tempFoot.footprint.lori[2]);

            push_short_foot(tempFoot);
            right_left ^= 1;
        }
    }

//    for(int i=0; i<test_short_foot_num; i++){
//        pull_short_foot(tempFoot);
//        printf("%d RFP:  %f  %f  %f   %f  %f  %f\n",i,tempFoot.footprint.rfoot[0],tempFoot.footprint.rfoot[1],tempFoot.footprint.rfoot[2],tempFoot.footprint.lfoot[0],tempFoot.footprint.lfoot[1],tempFoot.footprint.lfoot[2]);
//        printf("%d ssp: %f  dsp: %f \n",i,tempFoot.time.ssp_time,tempFoot.time.dsp_time);
//    }
    // =================================================================


//    double U0[15] = {0.,},UK[15][3] = {{0.,},}, IND[2] = {0.,};

//    for(int timecnt = 0; timecnt < 200*2; timecnt++){
//        update_window();

//        save();

//        if(timecnt%20 == 0){
////            UKSEL(20,U0,UK,IND);
//            printf("%f [%d](%.2f, %.2f)):  %s   preFoot X: (%.3f, %.3f) ==> target Foot X:(%.3f, %.3f)  preFoot y: (%.3f, %.3f) ==> target Foot y:(%.3f, %.3f)  previnfo: %d  ZMP:(%.3f, %.3f)\n",
//                   timecnt/200.0,
//                   target_foot[0].info,
//                   target_foot[0].time.dsp_time,
//                   target_foot[0].time.ssp_time,
//                   STATE_NAME[window[0].state],
//                   prev_foot.footprint.rfoot[0],
//                   prev_foot.footprint.lfoot[0],
//                   target_foot[0].footprint.rfoot[0],
//                   target_foot[0].footprint.lfoot[0],

//                    prev_foot.footprint.rfoot[1],
//                    prev_foot.footprint.lfoot[1],
//                    target_foot[0].footprint.rfoot[1],
//                    target_foot[0].footprint.lfoot[1],
//                    prev_foot.info,

//                    window[0].zmp.x,
//                    window[0].zmp.y
//                    );

//            for(int k = 0;k<15;k++){
//            printf("U0: %f  UK: %f  %f  %f \n",U0[k],UK[k][0],UK[k][1],UK[k][2]);
//            }


//        }
//    }
}

//==============================//
// Walking Motion Generator ( CoM Trajectory and Foot Trajectory)
//==============================//
void WMG()
{
    int Preview_time = 15,cn = 20;
    double disturbance[2][3] = {{0,},},FOOT_REF[4] = {0.0,},temp_zmp[15]={0.0,},temp_state[3] = {0.0,},temp_vec[15] = {0.0,},FOOT_IND[2] = {1.,1.},optimal_U[15] = {0.,};
    static double pv_state[2][3] = {{0.0}}, pv_state_old[2][3] = {{0.0}};
    double UK[15][3]={{0.0,},},U0[15]={0.0,},MPC_pk[18] = {0.,},MPC_zmp[2]={0.0,};
    static double refx[3] = {0.,},refy[3] = {0.,},px0[3] = {0.,},py0[3] = {0.,},pxf[3] = {0.,},pyf[3] = {0.,},t1=0,t2=0,tf=0.1;

//    static int internal_cnt = 0;

    // initialize the parameter
    if(pv_Index == 0)
    {
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
        {
//            Zref[i] = 0.;
            temp_vec[i] = 0.;
            t1 = 0;
            t2 = 0;

        }
//        internal_cnt=0;
    }

//    if( window[0].state != STATE_EMPTY)
    {
        if(pv_Index == 700){
                disturbance[1][0] = 0.;//sharedData->FOGRollVel*0.77*(3.141592/180.0);
                disturbance[1][1] = 0.;//sharedData->FOGRollVel*0.77*(3.141592/180.0);
                disturbance[1][2] = 0.;

                disturbance[0][0] = 0.;//sharedData->FOGPitch*0.77*(3.141592/180.0)*0.5;
                disturbance[0][1] = 0.;//sharedData->FOGPitchVel*0.77*(3.141592/180.0);
                disturbance[0][2] = 0.;
        }


        if(pv_Index%cn == 0)
        {
            //void UKSEL(int sampling_tic,double V[15],double U[15][3],double &IND)

            UKSEL(20,U0,UK,FOOT_IND);

            t1 = 0;
            t2 = 0;
            for(int i=0;i<2;i++)
            {
                if(i==0)
                {
                    px0[0] = pv_state_old[i][0]+disturbance[i][0]; px0[1] = pv_state_old[i][1]+disturbance[i][1]; px0[2] = pv_state_old[i][2]+disturbance[i][2];
                }else
                {
                    py0[0] = pv_state_old[i][0]+disturbance[i][0]; py0[1] = pv_state_old[i][1]+disturbance[i][1]; py0[2] = pv_state_old[i][2]+disturbance[i][2];
                }

                temp_state[0] = pv_state_old[i][0]+disturbance[i][0]; temp_state[1] = pv_state_old[i][1]+disturbance[i][1]; temp_state[2] = pv_state_old[i][2]+disturbance[i][2];

                // MPC_pk
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

                for(int asdf = 0;asdf<15;asdf++){
                        MPC_pk[asdf] = temp_vec[asdf] - MPC_beta*temp_zmp[asdf];
                }

                //------Cix>=ci
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


                for(int s=0;s<15;s++){
                    for(int t=0;t<15;t++){
                        params.Q[s + t*15 ] = MPC_Q[s][t];
                    }
                }


                for(int s=0;s<15;s++){
                    params.c[s] = MPC_pk[s];
                }

                for(int s=0;s<30;s++){
                    for(int t=0;t<15;t++){
                        params.A[s + t*30] = MPC_Ci[s][t];
                    }
                }

                for(int s=0;s<30;s++){
                    params.b[s] = MPC_ci[s];
                }

                // solving optimization problem
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

                // ZMP Output from LIPM
                MPC_zmp[i] = MPC_C[0]*pv_state[i][0] + MPC_C[1]*pv_state[i][1]+ MPC_C[2]*pv_state[i][2];

                pv_state_old[i][0] = pv_state[i][0];
                pv_state_old[i][1] = pv_state[i][1];
                pv_state_old[i][2] = pv_state[i][2];

                if(i == 0){
                    temp_debug[0] = temp_zmp[0];
                    temp_debug[1] = MPC_zmp[i];
                    temp_debug[2] = pv_state[i][0];
                    temp_debug[3] = pv_state[i][1];
//                    temp_debug[4] = FOOT_VAR[0];
                }else
                {
                    temp_debug[5] = temp_zmp[0];
                    temp_debug[6] = MPC_zmp[i];
                    temp_debug[7] = pv_state[i][0];
                    temp_debug[8] = pv_state[i][1];
//                    temp_debug[9] = FOOT_VAR[0];
                }
            }

            temp_debug[10] = FOOT_REF[0];
            temp_debug[11] = FOOT_IND[0];

            pxf[0] = pv_state_old[0][0]; pxf[1] = pv_state_old[0][1]; pxf[2] = pv_state_old[0][2];
            pyf[0] = pv_state_old[1][0]; pyf[1] = pv_state_old[1][1]; pyf[2] = pv_state_old[1][2];

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
//        GLOBAL_Z_RF = window[0].right_foot_ref.z;

        GLOBAL_X_LF = window[0].left_foot_ref.x;
        GLOBAL_Y_LF = window[0].left_foot_ref.y;
//        GLOBAL_Z_LF = window[0].left_foot_ref.z;

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

}

void Global2Local(double _Global[],double _Local[])
{
    double pCenter[3],qCenter[4],qCenter_bar[4];
    double temp1[3];

    // Foot Center in Global Coord.
    pCenter[0] = (des_pRF_3x1[0] + des_pLF_3x1[0])/2.;
    pCenter[1] = (des_pRF_3x1[1] + des_pLF_3x1[1])/2.;
    pCenter[2] = (des_pRF_3x1[2] + des_pLF_3x1[2])/2.;

//        one_cos_orientation2(0.5,des_qRF_4x1,des_qLF_4x1,0,1,qCenter);
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

//        one_cos_orientation2(0.5,des_qRF_4x1,des_qLF_4x1,0,1,qCenter);
//    qtRZ((fsm->RightInfos[0][3]*D2R + fsm->LeftInfos[0][3]*D2R)/2.,qCenter);
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

//        one_cos_orientation2(0.5,des_qRF_4x1,des_qLF_4x1,0,1,qCenter);
//    qtRZ((fsm->RightInfos[0][3]*D2R + fsm->LeftInfos[0][3]*D2R)/2.,qCenter);

qtRZ((window[0].right_foot_ref.yaw*D2R + window[0].left_foot_ref.yaw*D2R)/2.,qCenter);

    QTtransform(qCenter, _Local, temp1);

    sum_vv(temp1,3,pCenter,_Global); // zmp - pCenter


}


void WBIK()
{
    double temp1des_qPEL_4x1[4],temp2des_qPEL_4x1[4],temp3des_qPEL_4x1[4],temp4des_qPEL_4x1[4];
    double temp1des_qRF_4x1[4],temp2des_qRF_4x1[4],temp3des_qRF_4x1[4],temp4des_qRF_4x1[4],temp5des_qRF_4x1[4];
    double temp1des_qLF_4x1[4],temp2des_qLF_4x1[4],temp3des_qLF_4x1[4],temp4des_qLF_4x1[4],temp5des_qLF_4x1[4];
    double RightYaw,RightRoll,RightPitch,LeftYaw,LeftRoll,LeftPitch;
//    static double RightPitch_Torso=0.,RightRoll_Torso=0.;
//    static double LeftPitch_Torso=0.,LeftRoll_Torso=0.;
//    static double RightPitch_Torso_last=0.,RightRoll_Torso_last=0.;
//    static double LeftPitch_Torso_last=0.,LeftRoll_Torso_last=0.;
//    static double gain_left=0.,gain_right=0.;
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
//    Local[0]= GLOBAL_X_LIPM_n + (-0.001*Del_PC_X_DSP_XZMP_CON + I_ZMP_CON_X*1.)*ZMP_FeedBack_ONOFF;
//    Local[1]= GLOBAL_Y_LIPM_n + (-0.001*Del_PC_Y_DSP_YZMP_CON + I_ZMP_CON_Y*1.)*ZMP_FeedBack_ONOFF;
    Local[0] = GLOBAL_X_LIPM_n + G_DSP_X*(- 0.001*Del_PC_X_DSP_XZMP_CON*G_DSP_X + I_ZMP_CON_X*0.)*ZMP_FeedBack_ONOFF;
    if(Inv_ONOFF == 0){
        U_Gain = 0;
    }
//    Local[1] = GLOBAL_Y_LIPM_n + (-0.001*Del_PC_Y_DSP_YZMP_CON + I_ZMP_CON_Y*1.)*ZMP_FeedBack_ONOFF;//(U[0]*(U0_Gain  + U0_Gain_KI*0) + (GLOBAL_Y_LF_n + GLOBAL_Y_RF_n)/2.0)*U_Gain + (GLOBAL_Y_LIPM_n)*(1-U_Gain)+ (-0.001*Del_PC_Y_DSP_YZMP_CON*G_DSP_Y*(1 - fabs(GLOBAL_X_RF_n - GLOBAL_X_LF_n)*0.3/0.4*0)-0.001*Del_PC_Y_SSP_YZMP_CON*0+ I_ZMP_CON_Y*1.)*ZMP_FeedBack_ONOFF;
//    CONT_Y = Local[1] = (GLOBAL_Y_LIPM_n)*U0_Gain+ (-0.001*Del_PC_Y_DSP_YZMP_CON + I_ZMP_CON_Y*0.)*ZMP_FeedBack_ONOFF;
CONT_Y = Local[1] =  (GLOBAL_Y_LIPM_n)*(U0_Gain)+ (-0.001*Del_PC_Y_DSP_YZMP_CON*G_DSP_Y - 0.001*Del_PC_Y_SSP_YZMP_CON*0+ I_ZMP_CON_Y*0.)*ZMP_FeedBack_ONOFF;

//    CONT_Y = Local[1] = (U[0]*(U0_Gain  + U0_Gain_KI*0) + (GLOBAL_Y_LF_n + GLOBAL_Y_RF_n)/2.0)*U_Gain + (GLOBAL_Y_LIPM_n)*(1-U_Gain)+ (-0.001*Del_PC_Y_DSP_YZMP_CON*G_DSP_Y*(1 - fabs(GLOBAL_X_RF_n - GLOBAL_X_LF_n)*0.3/0.4*0)-0.001*Del_PC_Y_SSP_YZMP_CON*0+ I_ZMP_CON_Y*0.)*ZMP_FeedBack_ONOFF;


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
    des_pRF_3x1_hat[Zdir] = GLOBAL_Z_RF + Zctrl*0.5*impONOFF+ Zctrl2*0.5*impONOFF2+ Add_FootTask[RIGHT][Zdir]*Leg_Length_FeedBack_ONOFF + Add_Leg_Recovery[RIGHT][Zdir]*Leg_Length_Recover_ONOFF;

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
//    QTcross(temp5des_qRF_4x1,qt_WBIK_Torq_RF,temp5des_qRF_4x1);

    des_qRF_4x1[0] = temp5des_qRF_4x1[0];
    des_qRF_4x1[1] = temp5des_qRF_4x1[1];
    des_qRF_4x1[2] = temp5des_qRF_4x1[2];
    des_qRF_4x1[3] = temp5des_qRF_4x1[3];

    // ------------ LF
    des_pLF_3x1_hat[Xdir] = GLOBAL_X_LF;
    des_pLF_3x1_hat[Ydir] = GLOBAL_Y_LF;
    des_pLF_3x1_hat[Zdir] = GLOBAL_Z_LF - Zctrl*0.5*impONOFF- Zctrl2*0.5*impONOFF2+ Add_FootTask[LEFT][Zdir]*Leg_Length_FeedBack_ONOFF + Add_Leg_Recovery[LEFT][Zdir]*Leg_Length_Recover_ONOFF;

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
//    QTcross(temp5des_qLF_4x1,qt_WBIK_Torq_LF,temp5des_qLF_4x1);

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
//    QTcross(temp5des_qRF_4x1,qt_WBIK_Torq_RF,temp5des_qRF_4x1);

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
//    QTcross(temp5des_qLF_4x1,qt_WBIK_Torq_LF,temp5des_qLF_4x1);

    des_qLF_4x1[0] = temp5des_qLF_4x1[0];
    des_qLF_4x1[1] = temp5des_qLF_4x1[1];
    des_qLF_4x1[2] = temp5des_qLF_4x1[2];
    des_qLF_4x1[3] = temp5des_qLF_4x1[3];

    memcpy(WBIK_Q0,WBIK_Q,34*sizeof(double));

    kine_drc_hubo4.IK_LowerBody_Global(WBIK_Q0,Qub,des_pCOM_3x1, des_qPEL_4x1, des_pRF_3x1, des_qRF_4x1, des_pLF_3x1, des_qLF_4x1,WBIK_Q);


    // Ankle Joint control using gyro
//    GLOBAL_Xori_RF_n = GLOBAL_Xori_RF*cos(WBIK_Q[idRHY]) + GLOBAL_Yori_RF*sin(WBIK_Q[idRHY]);
//    GLOBAL_Yori_RF_n =-GLOBAL_Xori_RF*sin(WBIK_Q[idRHY]) + GLOBAL_Yori_RF*cos(WBIK_Q[idRHY]);

//    GLOBAL_Xori_LF_n = GLOBAL_Xori_LF*cos(WBIK_Q[idLHY]) + GLOBAL_Yori_LF*sin(WBIK_Q[idLHY]);
//    GLOBAL_Yori_LF_n =-GLOBAL_Xori_LF*sin(WBIK_Q[idLHY]) + GLOBAL_Yori_LF*cos(WBIK_Q[idLHY]);

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

    if(window[0].state == STATE_EMPTY)
    {
        FINAL_TIMER  = FINAL_TIMER + 0.005;

        if(FINAL_TIMER > 300.0)
        {
            printf("Walking is finished and Walkflag is set to 0 \n");
            walk_flag = 0;
            continouse_walking_flag = false;
            __JOY_COMMAND_MODE = false;
            LandingState = FINAL;
        }
    }
}

void UKSEL(int sampling_tic,double V[15],double U[15][3],double IND[2])
{    
    //        code Jay.
    // dsp init 0
    // **** ssp LF 2
    // dsp LF to RF 4
    // **** ssp RF 5
    // dsp RF to LF 1
    // DSP RFLF 7
    // FINISH 8
//    // walking states =========
//    typedef enum{
//        DSP_INIT_RF = 0,
//        DSP_INIT_LF,  1
//        DSP_RF,       2
//        SSP_RF,       3
//        DSP_LF,       4
//        SSP_LF,       5
//        DSP_FINAL,    6
//        STATE_EMPTY   7
//    }WALKING_STATE;


    // Initialize every control period
    int flag1 = 0,flag2 = 0 ,Rflag1 = 0,Rflag2 = 0,Lflag1 = 0,Lflag2 = 0;
    static int before_ind = 1;

    for(int i = 0;i<3;i++){
        for(int j = 0;j<15;j++){
            U[j][i] = 0.;
            V[j] = 0;
        }
    }

    // DSP INIT
    if(window[0].state == DSP_INIT_RF){
        // Left foot = 1, right foot  = -1
        IND[0] = 1.;
        for(int i=0;i<=14;i++){
                if((window[i*sampling_tic].state == DSP_INIT_RF ) && flag1 == 0){
                    U[i][0] = 0;
                    V[i] = 1;
                }else if((window[i*sampling_tic].state == SSP_RF ) && flag2 == 0){
                    U[i][0] = 1;
                    V[i] = 0;
                    flag1 = 1;
                }else if((window[i*sampling_tic].state == DSP_LF || window[i*sampling_tic].state == SSP_LF) && flag1 == 1 ){
                    U[i][1] = 1;
                    V[i] = 0;
                    flag2 = 1;
                }else if((window[i*sampling_tic].state == DSP_RF || window[i*sampling_tic].state == SSP_RF) && flag2 == 1 ){
                    U[i][2] = 1;
                    V[i] = 0;
                }
        }

    // L DSP L SSP
    }else if(window[0].state == DSP_INIT_LF){
        // Left foot = 1, right foot  = -1
        IND[0] = -1.;
        for(int i=0;i<=14;i++){
                if((window[i*sampling_tic].state == DSP_INIT_LF) && flag1 == 0){
                    U[i][0] = 0;
                    V[i] = 1;
                }else if((window[i*sampling_tic].state == SSP_LF ) && flag2 == 0){
                    U[i][0] = 1;
                    V[i] = 0;
                    flag1 = 1;
                }else if((window[i*sampling_tic].state == DSP_RF || window[i*sampling_tic].state == SSP_RF) && flag1 == 1 ){
                    U[i][1] = 1;
                    V[i] = 0;
                }else if((window[i*sampling_tic].state == DSP_LF || window[i*sampling_tic].state == SSP_LF) && flag1 == 1 ){
                    U[i][2] = 1;
                    V[i] = 0;
                }
        }

    }else if(window[0].state == SSP_RF || window[0].state == DSP_RF){
        // right foot swing phase --- left foot supporting
        IND[0] = 1;
        before_ind = 1;

        IND[0] = IND[0] + 1;

        if(window[0 + sampling_tic].state == DSP_LF){
            IND[1] = 0;
        }

        Rflag1 = 0; Rflag2=0;
        for(int i=0;i<=14;i++){
                //L SSP 0
                if((window[i*sampling_tic].state == SSP_RF || window[i*sampling_tic].state == DSP_RF) && Lflag1 == 0){
                    U[i][0] = 0;
                    V[i] = 1;
                    Lflag1 = 0;
                    Lflag2 = 0;
                // R SSP 1
                }else if((window[i*sampling_tic].state == SSP_LF ||window[i*sampling_tic].state == DSP_LF) && Lflag2 == 0){
                    U[i][0] = 1;
                    V[i] = 0;
                    Lflag1 = 1;
                // L SSP 2
                }else if((window[i*sampling_tic].state == SSP_RF || window[i*sampling_tic].state == DSP_RF) && Lflag1 == 1){
                    U[i][1] = 1;
                    V[i] = 0;
                    Lflag2 = 1;
                // R SSP 3
                }else if((window[i*sampling_tic].state == SSP_LF ||window[i*sampling_tic].state == DSP_LF) && Lflag2 == 1){
                    U[i][2] = 1;
                    V[i] = 0;
                    Lflag1 = 0;
                }else if((window[i*sampling_tic].state == DSP_FINAL ||window[i*sampling_tic].state == STATE_EMPTY)){
                    U[i][0] = 0;
                    U[i][1] = 0;
                    U[i][2] = 0;
                    V[i] = 1;
                }
        }
    // R SSP
    }else if(window[0].state == SSP_LF || window[0].state == DSP_LF){
        // left swing phase
        IND[0] = -1.;
        before_ind = -1;
        IND[1] = IND[1] + 1;

        if(window[0 + sampling_tic].state == DSP_RF){
                IND[1] = 0;
        }

        for(int i=0;i<=14;i++){
                Lflag1 = 0; Lflag2=0;
                //R SSP 0
                if((window[i*sampling_tic].state == SSP_LF || window[i*sampling_tic].state == DSP_LF) && Rflag1 == 0){
                    U[i][0] = 0;
                    V[i] = 1;
                    Rflag1 = 0;
                    Rflag2 = 0;
//                    printf("R SSP DSP \n");
                // R SSP 1
                }else if((window[i*sampling_tic].state == SSP_RF || window[i*sampling_tic].state == DSP_RF) && Rflag2 == 0){
                    U[i][0] = 1;
                    V[i] = 0;
                    Rflag1 = 1;
                // L SSP 2
                }else if((window[i*sampling_tic].state == SSP_LF || window[i*sampling_tic].state == DSP_LF) && Rflag1 == 1){
                    U[i][1] = 1;
                    V[i] = 0;
                    Rflag2 = 1;
                // R SSP 3
                }else if((window[i*sampling_tic].state == SSP_RF || window[i*sampling_tic].state == DSP_RF ) && Rflag2 == 1){
                    U[i][2] = 1;
                    V[i] = 0;
                    Rflag1 = 0;
                }else if((window[i*sampling_tic].state == DSP_FINAL || window[i*sampling_tic].state == STATE_EMPTY )){
                    U[i][0] = 0;
                    U[i][1] = 0;
                    U[i][2] = 0;
                    V[i] = 1;
                }
        }
    }
}



//==============================//
// State Machin for Controller and WMG
//==============================//



void Controller(){

    double init_start_time = 0.6,init_final_time = 1.5;

    switch(window[0].state){
    case DSP_INIT_RF:
        LandingState = DSP;
        // For continous walking

//        if((fsm_state_timer[fsm->StateInfos[0][0]]>=1.8)&&(fsm_state_timer[fsm->StateInfos[0][0]]<=2.7))
//        {
//            U_Gain = 0.5*(1-cos(PI*(fsm_state_timer[fsm->StateInfos[0][0]]-1.8)/(0.9-DEL_T)));
//        }


        // gain
        if((window[0].timer.current>=init_start_time)&&(window[0].timer.current<=init_final_time))
        {
            U_Gain = 0.5*(1-cos(PI*(window[0].timer.current - init_start_time)/(init_final_time - init_start_time)));
        }

        if(window[0].timer.current<=0.7)
        {
            G_DSP_X = G_DSP_Y = U_Gain_DSP = 0.5*(1-cos(PI*(window[0].timer.current)/(0.7)));
        }
//            printf("------------------Ugain : %f \n",U_Gain_DSP);
        Leg_Length_Control();

        break;
    case DSP_INIT_LF:
        LandingState = DSP;
        // For continous walking

        if((window[0].timer.current>=init_start_time)&&(window[0].timer.current<=init_final_time))
        {
            U_Gain = 0.5*(1-cos(PI*(window[0].timer.current - init_start_time)/(init_final_time - init_start_time)));
        }
        if(window[0].timer.current<=0.7)
        {
            G_DSP_X = G_DSP_Y = U_Gain_DSP = 0.5*(1-cos(PI*(window[0].timer.current)/(0.7)));
        }

        Leg_Length_Control();
        // to stable posture


        break;
    case DSP_FINAL:
        if(LandingState == RSSP) Pre_LandingState = RSSP;
        if(LandingState == LSSP) Pre_LandingState = LSSP;
        LandingState = FINAL;
        // recover
        // 1. early landing foot position
        Upperbody_Gain_Lock();
        Leg_Length_Control();
        RecoverLegLength();

        break;
    case DSP_RF:
        if(LandingState == RSSP) Pre_LandingState = RSSP;
        if(LandingState == LSSP) Pre_LandingState = LSSP;
        LandingState = DSP;

        ReactiveControl(2,2,1);

//        RDPitch2 = RMYC2(1,1,1,GLOBAL_ZMP_REF_X_n,GLOBAL_ZMP_REF_Y_n,X_ZMP_n/1000.0,Y_ZMP_n/1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],sharedData->FTMy[RAFT],fsm->RightInfos[0][5]*D2R);
//        LDPitch2 = LMYC2(1,1,1,GLOBAL_ZMP_REF_X_n,GLOBAL_ZMP_REF_Y_n,X_ZMP_n/1000.0,Y_ZMP_n/1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],sharedData->FTMy[LAFT],fsm->LeftInfos[0][5]*D2R);

//        RDRoll2 = RMXC2(1,1,1,GLOBAL_ZMP_REF_X_n,GLOBAL_ZMP_REF_Y_n,X_ZMP_n/1000.0,Y_ZMP_n/1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],sharedData->FTMy[RAFT],fsm->RightInfos[0][5]*D2R);
//        LDRoll2 = LMXC2(1,1,1,GLOBAL_ZMP_REF_X_n,GLOBAL_ZMP_REF_Y_n,X_ZMP_n/1000.0,Y_ZMP_n/1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],sharedData->FTMy[LAFT],fsm->LeftInfos[0][5]*D2R);


        break;
    case DSP_LF:
        if(LandingState == RSSP) Pre_LandingState = RSSP;
        if(LandingState == LSSP) Pre_LandingState = LSSP;
        LandingState = DSP;
        ReactiveControl(2,2,1);

//        RDPitch2 = RMYC2(1,1,1,GLOBAL_ZMP_REF_X_n,GLOBAL_ZMP_REF_Y_n,X_ZMP_n/1000.0,Y_ZMP_n/1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],sharedData->FTMy[RAFT],fsm->RightInfos[0][5]*D2R);
//        LDPitch2 = LMYC2(1,1,1,GLOBAL_ZMP_REF_X_n,GLOBAL_ZMP_REF_Y_n,X_ZMP_n/1000.0,Y_ZMP_n/1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],sharedData->FTMy[LAFT],fsm->LeftInfos[0][5]*D2R);

//        RDRoll2 = RMXC2(1,1,1,GLOBAL_ZMP_REF_X_n,GLOBAL_ZMP_REF_Y_n,X_ZMP_n/1000.0,Y_ZMP_n/1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],sharedData->FTMy[RAFT],fsm->RightInfos[0][5]*D2R);
//        LDRoll2 = LMXC2(1,1,1,GLOBAL_ZMP_REF_X_n,GLOBAL_ZMP_REF_Y_n,X_ZMP_n/1000.0,Y_ZMP_n/1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],sharedData->FTMy[LAFT],fsm->LeftInfos[0][5]*D2R);



        break;
    case SSP_RF:

        //reset control RX_TC while RF swing
        if((window[0].timer.current >= 0.1) && (window[0].timer.current <= 0.5))
        {
            // Recovery
            ReactiveControl(0,2,1);

//            RDPitch2 = RMYC2(0,1,1,GLOBAL_ZMP_REF_X_n,GLOBAL_ZMP_REF_Y_n,X_ZMP_n/1000.0,Y_ZMP_n/1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],sharedData->FTMy[RAFT],fsm->RightInfos[0][5]*D2R);
//            LDPitch2 = LMYC2(2,2,1,GLOBAL_ZMP_REF_X_n,GLOBAL_ZMP_REF_Y_n,X_ZMP_n/1000.0,Y_ZMP_n/1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],sharedData->FTMy[LAFT],fsm->LeftInfos[0][5]*D2R);

//            RDRoll2 = RMXC2(0,1,1,GLOBAL_ZMP_REF_X_n,GLOBAL_ZMP_REF_Y_n,X_ZMP_n/1000.0,Y_ZMP_n/1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],sharedData->FTMy[RAFT],fsm->RightInfos[0][5]*D2R);
//            LDRoll2 = LMXC2(2,2,1,GLOBAL_ZMP_REF_X_n,GLOBAL_ZMP_REF_Y_n,X_ZMP_n/1000.0,Y_ZMP_n/1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],sharedData->FTMy[LAFT],fsm->LeftInfos[0][5]*D2R);

        }
        else if((window[0].timer.current > 0.6))
        {
            // Control
            ReactiveControl(1,2,1);
        }else
        {
            // no control
            ReactiveControl(2,2,1);
        }

        LandingState = RSSP;
        break;
    case SSP_LF:
        LandingState = LSSP;

        if((window[0].timer.current >= 0.1) && (window[0].timer.current <= 0.5))
        {
            // Recovery
            ReactiveControl(2,0,1);

//            RDPitch2 = RMYC2(2,2,1,GLOBAL_ZMP_REF_X_n,GLOBAL_ZMP_REF_Y_n,X_ZMP_n/1000.0,Y_ZMP_n/1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],sharedData->FTMy[RAFT],fsm->RightInfos[0][5]*D2R);
//            LDPitch2 = LMYC2(2,0,1,GLOBAL_ZMP_REF_X_n,GLOBAL_ZMP_REF_Y_n,X_ZMP_n/1000.0,Y_ZMP_n/1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],sharedData->FTMy[LAFT],fsm->LeftInfos[0][5]*D2R);

//            RDRoll2 = RMXC2(2,2,1,GLOBAL_ZMP_REF_X_n,GLOBAL_ZMP_REF_Y_n,X_ZMP_n/1000.0,Y_ZMP_n/1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],sharedData->FTMy[RAFT],fsm->RightInfos[0][5]*D2R);
//            LDRoll2 = LMXC2(2,0,1,GLOBAL_ZMP_REF_X_n,GLOBAL_ZMP_REF_Y_n,X_ZMP_n/1000.0,Y_ZMP_n/1000.0,des_pLF_3x1[0],des_pRF_3x1[0],des_pLF_3x1[1],des_pRF_3x1[1],sharedData->FTMy[LAFT],fsm->LeftInfos[0][5]*D2R);

        }
        else if((window[0].timer.current > 0.6))
        {
            // Control
            ReactiveControl(2,1,1);
        }else
        {
            // no control
            ReactiveControl(2,2,1);
        }


    case STATE_EMPTY:
//        if(window[0].timer.current > 1.0){
//            Upperbody_Gain_Lock();
//        }
        Leg_Length_Control();
        RecoverLegLength();

        break;
    default:
        break;
    }


    Compensator_deflection(window[0].state);
    Kirk_Control();
    LandingControl(window[0].timer.current,window[0].state,sharedData->FT[RAFT].Fz,sharedData->FT[LAFT].Fz);
    Gyro_Feedback();



}







//==============================//
// Stabilizer ( CoM and Reaction Force Control)
//==============================//
void ZMP_intergral_control()
{
//    I_ZMP_CON_X += -0.001*(0.001*X_ZMP_n - (GLOBAL_ZMP_REF_X_n ));//-X_ZMP_n_OFFSET_BP
//    I_ZMP_CON_Y += -0.001*(0.001*Y_ZMP_n - (GLOBAL_ZMP_REF_Y_n ));

    I_ZMP_CON_X += -0.001*(0.001*X_ZMP_Local - (X_ZMP_REF_Local ));//-X_ZMP_n_OFFSET_BP
    I_ZMP_CON_Y += -0.001*(0.001*Y_ZMP_Local - (Y_ZMP_REF_Local ));

    if(I_ZMP_CON_X > 0.04)I_ZMP_CON_X=0.04;
    else if(I_ZMP_CON_X < -0.04)I_ZMP_CON_X=-0.04;
    if(I_ZMP_CON_Y > 0.04)I_ZMP_CON_Y=0.04;
    else if(I_ZMP_CON_Y < -0.04)I_ZMP_CON_Y=-0.04;
//    if(_preview_flag == true && pv_Index == 1){
//        I_ZMP_CON_X = 0.0f;
//        I_ZMP_CON_Y = 0.0f;
//    }
}
//-----------------------------------------------------------------------------------------------//
double kirkZMPCon_XP2(double u, double ZMP, int zero)
{
    int i;
    const double A[2][2] = {{0.000000000000, 1.000000000000}, {-93.44,-1.0204}};
    const double B[2] = {0,153.5062};
    const double C[2] = {5.3672,0.0510};
    const double D = -7.6675;
    const double Kg[2] = {-0.1917,0.0976};
    const double Og[2] = {3.5220,1.4988};
//    const double A[2][2] = {{0.000000000000, 1.000000000000}, {-93.44,-0.8218}};
//    const double B[2] = {0,135.9192};
//    const double C[2] = {6.5750,0.0510};
//    const double D = -8.4295;
//    const double Kg[2] = {-0.2165,0.1117};
//    const double Og[2] = {2.9066,1.3227};
//    const double A[2][2] = {{0.000000000000, 1.000000000000}, {-121.509441687773,-0.686900801711364}};
//    const double B[2] = {0,171.905595852174};
//    const double C[2] = {8.31581491568207,0.0426004533905397};
//    const double D = -10.6613011739514;
//    const double Kg[2] = {-0.561351369682899,0.0541756604962248};
//    const double Og[2] = {1.87687421562180,-6.91634420265651};

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
// ----------------------------------------------------------------------------------- //
double kirkZMPCon_YP2(double u, double ZMP, int zero)
{
    int i;
//    const double A[2][2] = {{0.000000000000, 1.000000000000}, {-93.44,-1.2327}};
//    const double B[2] = {0,135.9192};
//    const double C[2] = {6.5750,0.0765};
//    const double D = -8.4295;
//    const double Kg[2] = {-0.0915,0.1234};
//    const double Og[2] = {2.8458,0.7336};
//    const double Kg[2] = {-0.4152,0.0792};
//

//    const double Og[2] = {2.8405,1.1907};
//

    const double A[2][2] = {{0.000000000000, 1.000000000000}, {-93.44,-1.5306}};
    const double B[2] = {0,153.5062};
    const double C[2] = {5.3672,0.0765};
    const double D = -7.6675;

    const double Kg[2] = {-0.0809742090339354,	0.107288107510564};
    const double Og[2] = {3.43082537995055,0.723663240519607};

//    const double A[2][2] = {{0.000000000000, 1.000000000000}, {-119.405421865731,-0.441579086814448}};
//    const double B[2] = {0,169.208134541865};
//    const double C[2] = {8.18532708084719,0.0273860057510612};
//    const double D = -10.4940090780092;
//    const double Kg[2] = {-0.552014961447503,0.0564891335695250};
//    const double Og[2] = {3.56807911031747,12.8724994839785};

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

   // y = Kg[0]*(atan2(GLOBAL_Y_LIPM_n,des_pCOM_3x1[Zdir])*1000-x_new[0]) + Kg[1]*(atan2(GLOBAL_Y_LIPM_d_n,des_pCOM_3x1[Zdir])*1000-x_new[1]);
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

//    AnkleControl1 = (sharedData->FOG.Roll*R2D*2.5f + sharedData->FOG.RollVel*6.);
//    AnkleControl2 = (sharedData->FOG.Pitch*R2D*2.3f + sharedData->FOG.PitchVel*4.);

//    AnkleControl1 = ((1.0+HPF_Estimated_Orientation[0]*R2D)*Estimated_Orientation[0]*R2D*1.5f + sharedData->FOG.RollVel*6.);
//    AnkleControl2 = (Estimated_Orientation[1]*R2D*1.3f + sharedData->FOG.PitchVel*4.);

//    AnkleControl1 = (HPF_Estimated_Orientation[0]*R2D*1.5f + sharedData->FOG.RollVel*7.);
//    AnkleControl2 = (HPF_Estimated_Orientation[1]*R2D*1.3f + sharedData->FOG.PitchVel*5.);


//        AnkleControl1 = (sharedData->FOG.Roll*2.5f*60.0 + sharedData->FOG.RollVel*7.);
//        AnkleControl2 = (sharedData->FOG.Pitch*2.3f*60.0 + sharedData->FOG.PitchVel*5.);


    AnkleControl1 = (sharedData->FOG.Roll*1.5f + sharedData->FOG.RollVel*D2R*3.);
    AnkleControl2 = (sharedData->FOG.Pitch*1.3f + sharedData->FOG.PitchVel*D2R*2.);



//    AnkleControl1 = GLOBAL_Xori_RF*cos(WBIK_Q[idRHY]) + GLOBAL_Yori_RF*sin(WBIK_Q[idRHY]);
//    GLOBAL_Yori_RF_n =-GLOBAL_Xori_RF*sin(WBIK_Q[idRHY]) + GLOBAL_Yori_RF*cos(WBIK_Q[idRHY]);

//    GLOBAL_Xori_LF_n = GLOBAL_Xori_LF*cos(WBIK_Q[idLHY]) + GLOBAL_Yori_LF*sin(WBIK_Q[idLHY]);
//    GLOBAL_Yori_LF_n =-GLOBAL_Xori_LF*sin(WBIK_Q[idLHY]) + GLOBAL_Yori_LF*cos(WBIK_Q[idLHY]);





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



//    FOGRollVel_NF = y_i;
//    FOGPitchVel_NF = y_i1;



//    FOGRollVel_NF = NotchFilter_GyroRollControlInput(y_i,1);
//    FOGPitchVel_NF = NotchFilter_GyroPitchControlInput(y_i1,1);


//    GLOBAL_Xori_RF = ALPHA * FOGRollVel_NF;
//    GLOBAL_Xori_LF = (1.0 - ALPHA) * FOGRollVel_NF;

//    GLOBAL_Yori_RF = ALPHA * FOGPitchVel_NF;
//    GLOBAL_Yori_LF = (1.0 - ALPHA) * FOGPitchVel_NF;




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
//    printf("Angle = %f,sume = %f,y = %f\n", _force,sume,y);

    return y;
}

double RecoverRightLegLength(double _ref, double _force, int _zero)
{
    static double y;
    static double sume = 0.;
    const double KI = 0.051;

    y = 0.0*(_ref - _force) + KI*sume;

    sume += _ref - _force;

//    if(sume > 0.08/KI) sume = .08/KI;
//    else if(sume < -0.08/KI) sume = -.08/KI;

    if(y > 0.08) y = 0.08;
    else if(y < -0.08) y = -0.08;

    if(_zero == 0) sume = 0.;
//    printf("Angle = %f,sume = %f,y = %f\n", _force,sume,y);

    return y;
}

double RecoverLeftLegLength(double _ref, double _force, int _zero)
{
    static double y;
    static double sume = 0.;
    const double KI = 0.051;

    y = 0.0*(_ref - _force) + KI*sume;

    sume += _ref - _force;

//    if(sume > 0.08/KI) sume = .08/KI;
//    else if(sume < -0.08/KI) sume = -.08/KI;

    if(y > 0.08) y = 0.08;
    else if(y < -0.08) y = -0.08;

    if(_zero == 0) sume = 0.;
//    printf("Angle = %f,sume = %f,y = %f\n", _force,sume,y);

    return y;
}



double NotchFilter_GyroRollControlInput(double _input, int _reset)
{
    //wo = (200/6Hz)/(200Hz/2) , bw = wo/35
    //    double a[3] = {1.0,-0.985259453380759,0.970518906761517};
    //    double b[3] = {0.985259453380759,-0.985259453380759,0.985259453380759};
    //wo = (200/6Hz)/(200Hz/2) , bw = wo/20
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
    //wo = (200/6Hz)/(200Hz/2) , bw = wo/35
    //    double a[3] = {1.0,-0.985259453380759,0.970518906761517};
    //    double b[3] = {0.985259453380759,-0.985259453380759,0.985259453380759};
    //wo = (200/6Hz)/(200Hz/2) , bw = wo/20
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
    //wo = (200/6Hz)/(200Hz/2) , bw = wo/35
    //    double a[3] = {1.0,-0.985259453380759,0.970518906761517};
    //    double b[3] = {0.985259453380759,-0.985259453380759,0.985259453380759};
    //wo = (200/6Hz)/(200Hz/2) , bw = wo/20
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
    //wo = (200/6Hz)/(200Hz/2) , bw = wo/35
    //    double a[3] = {1.0,-0.985259453380759,0.970518906761517};
    //    double b[3] = {0.985259453380759,-0.985259453380759,0.985259453380759};
    //wo = (200/6Hz)/(200Hz/2) , bw = wo/20
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
    ZMP_intergral_control();
    final_gain_DSP_ZMP_CON = 0.5*(1-cos(PI*CNT_final_gain_DSP_ZMP_CON/40));

    Del_PC_X_DSP_XZMP_CON = kirkZMPCon_XP2(-Del_PC_X_DSP_XZMP_CON, X_ZMP_Local - (X_ZMP_REF_Local)*1000 , 1);
    Del_PC_Y_DSP_YZMP_CON = kirkZMPCon_YP2(-Del_PC_Y_DSP_YZMP_CON, Y_ZMP_Local - (Y_ZMP_REF_Local)*1000 , 1);

    LPF_Del_PC_X_DSP_XZMP_CON = (1.f-2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f)*LPF_Del_PC_X_DSP_XZMP_CON + 2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f*Del_PC_X_DSP_XZMP_CON;
    LPF_Del_PC_Y_DSP_YZMP_CON = (1.f-2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f)*LPF_Del_PC_Y_DSP_YZMP_CON + 2.f*PI*3.0*(double)RT_TIMER_PERIOD_MS*0.001f*Del_PC_Y_DSP_YZMP_CON;

//    if(CNT_final_gain_DSP_ZMP_CON < 40) CNT_final_gain_DSP_ZMP_CON++;

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
//    RDesTorqueY = dTorque = (RFoot_x - ref_zmp_x)*RDF + (LFoot_x - ref_zmp_x)*LDF;



    dTorque = 0.;

    //measured
    torqueX = (-(RFoot_x - zmp_x)*RDF - (LFoot_x-zmp_x)*LDF);


//    RmTorqueX = mTorque = alpha*torqueX;
    if(state ==1)
    {
//        mTorque = RDF*(sharedData->FOGPitch*0.1 + sharedData->FOGPitchVel*.3);//(RFoot_x - ref_zmp_x)*RDF + (LFoot_x - ref_zmp_x)*LDF;
//        mTorque = RDF*(sharedData->FOG.Pitch*0.3 + sharedData->FOG.PitchVel*.7);//(RFoot_x - ref_zmp_x)*RDF + (LFoot_x - ref_zmp_x)*LDF;
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
//    RDesTorqueY = dTorque = (RFoot_x - ref_zmp_x)*RDF + (LFoot_x - ref_zmp_x)*LDF;
//    dTorque = 0.;//(RFoot_x - ref_zmp_x)*RDF + (LFoot_x - ref_zmp_x)*LDF;
//dTorque = - alpha*(sharedData->FOGPitch*10.0 + sharedData->FOGPitchVel*.01);
        dTorque = 0.;

    //measured
    torqueX = (-(RFoot_x - zmp_x)*RDF - (LFoot_x-zmp_x)*LDF);


    if(state2 ==1)
    {
//        mTorque = LDF*(sharedData->FOGPitch*0.3 + sharedData->FOGPitchVel*.7);//(RFoot_x - ref_zmp_x)*RDF + (LFoot_x - ref_zmp_x)*LDF;
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
//    RDesTorqueY = dTorque = (RFoot_x - ref_zmp_x)*RDF + (LFoot_x - ref_zmp_x)*LDF;



        dTorque = 0.;




    //measured
    torqueX = (-(RFoot_x - zmp_x)*RDF - (LFoot_x-zmp_x)*LDF);


//    RmTorqueX = mTorque = alpha*torqueX;
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
//    RDesTorqueY = dTorque = (RFoot_x - ref_zmp_x)*RDF + (LFoot_x - ref_zmp_x)*LDF;
//    dTorque = 0.;//(RFoot_x - ref_zmp_x)*RDF + (LFoot_x - ref_zmp_x)*LDF;
//dTorque = - alpha*(sharedData->FOGPitch*10.0 + sharedData->FOGPitchVel*.01);
        dTorque = 0.;

    //measured
    torqueX = (-(RFoot_x - zmp_x)*RDF - (LFoot_x-zmp_x)*LDF);


    if(state2 ==1)
    {
//        mTorque = LDF*(sharedData->FOGRoll*0.3 + sharedData->FOGRollVel*.7);//(RFoot_x - ref_zmp_x)*RDF + (LFoot_x - ref_zmp_x)*LDF;
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
        // ZMP 읽기 //
        double M_LF[3],M_RF[3],M_LF_Global[3],M_RF_Global[3],F_LF[3],F_RF[3],F_LF_Global[3],F_RF_Global[3];
        double pCenter[3],qCenter[4],qCenter_bar[4];
        double zmp[3],zmp_local[3],zmp_ref_local[3];

        // Foot Center in Global Coord.
        pCenter[0] = (des_pRF_3x1[0] + des_pLF_3x1[0])/2.;
        pCenter[1] = (des_pRF_3x1[1] + des_pLF_3x1[1])/2.;
        pCenter[2] = (des_pRF_3x1[2] + des_pLF_3x1[2])/2.;

//        one_cos_orientation2(0.5,des_qRF_4x1,des_qLF_4x1,0,1,qCenter);
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


//            X_ZMP_n =(1.f-2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f)*X_ZMP_n + 2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f*X_ZMP;
//            Y_ZMP_n =(1.f-2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f)*Y_ZMP_n + 2.f*PI*7.0*(double)RT_TIMER_PERIOD_MS*0.001f*Y_ZMP;
        }
}



void LandingControl(int cur_time,int state,double rForce,double lForce)
{
    static double temp_Z_RF[3]={0.,},temp_Z_LF[3]={0.};
    double Force_Threshold = 150.,target_time = 0.4;

    if(state == SSP_RF)
    {
//        printf("curtime: %f    target_time: %f \n",cur_time,target_time);

        // Recovery left supporting leg
        if(window[0].timer.current < 0.4){
            GLOBAL_Z_LF_goal[0] = window[0].left_foot_ref.z;    GLOBAL_Z_LF_goal[1] = 0.;   GLOBAL_Z_LF_goal[2] = 0.;
            Fifth( window[0].timer.current , 0.4, GLOBAL_Z_LF_last ,GLOBAL_Z_LF_goal  , temp_Z_LF);
//            printf("global Z lf last: %f     global Z lf goal: %f\n",GLOBAL_Z_LF_goal[0],GLOBAL_Z_LF_last[0]);
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
//            printf("^^^^^^^^time: %f  rforce : %f    threshold : %f  \n",window[0].timer.current,rForce,Force_Threshold);
            if(rForce > Force_Threshold && window[0].timer.current>= 0.65){
                if(EarlyLandingFlag[RIGHT] == 0)
                {
                    GLOBAL_Z_RF_last_earlylanding = GLOBAL_Z_RF;
                }

                EarlyLandingFlag[RIGHT] = 1;

//                printf("$$$$$$$$$$$$$$$$$$Right foot Early landing !!!!!!!!!!!!!!!!!!!!!!!!!!!!! \n");

            }

            if(EarlyLandingFlag[RIGHT] == 0){ // normal walking
                GLOBAL_Z_RF = window[0].right_foot_ref.z;
            }else{
                GLOBAL_Z_RF = GLOBAL_Z_RF_last_earlylanding;

            }
            GLOBAL_Z_RF_last[0] = GLOBAL_Z_RF;

        }

    }else if(state == SSP_LF){

//        printf("curtime: %f   target time: %f\n",cur_time,target_time);

        // Recovery right supporting leg up to left off phase
        if(window[0].timer.current < 0.4){

            GLOBAL_Z_RF_goal[0] = window[0].right_foot_ref.z;    GLOBAL_Z_RF_goal[1] = 0.;   GLOBAL_Z_RF_goal[2] = 0.;

            Fifth( window[0].timer.current , 0.4, GLOBAL_Z_RF_last , GLOBAL_Z_RF_goal , temp_Z_RF);
//            printf("global Z lf last: %f     global Z lf goal: %f\n",GLOBAL_Z_LF_goal[0],GLOBAL_Z_LF_last[0]);

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
//            printf("^^^^^^^^ lforce : %f    threshold : %f  \n",window[0].timer.current,lForce,Force_Threshold);
            if(lForce > Force_Threshold && window[0].timer.current>= 0.65){

                if(EarlyLandingFlag[LEFT] == 0){

                    GLOBAL_Z_LF_last_earlylanding = GLOBAL_Z_LF;
                }
                EarlyLandingFlag[LEFT] = 1;

//                printf("$$$$$$$$$$$$$$$$$$Left foot Early landing !!!!!!!!!!!!!!!!!!!!!!!!!!!!! \n");
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

//        convert_euler(des_pRF_3x1_n, des_pLF_3x1_n, sharedData->FOG.Roll*cos(-yaw_angle)-sharedData->FOG.Pitch*sin(-yaw_angle), sharedData->FOG.Roll*sin(-yaw_angle)+sharedData->FOG.Pitch*cos(-yaw_angle), 0,BTW_FOOT_Angle_roll, BTW_FOOT_Angle_pitch, BTW_FOOT_Angle_yaw,BTW_FOOT_qPEL_comp_4x1);

        convert_euler(des_pRF_3x1_n, des_pLF_3x1_n, sharedData->IMU[0].AccX*D2R*cos(-yaw_angle)- sharedData->IMU[0].AccY*D2R*sin(-yaw_angle), ( sharedData->IMU[0].AccX*D2R)*sin(-yaw_angle)+ sharedData->IMU[0].AccY*D2R*cos(-yaw_angle), 0,BTW_FOOT_Angle_roll, BTW_FOOT_Angle_pitch, BTW_FOOT_Angle_yaw,BTW_FOOT_qPEL_comp_4x1);

        Add_FootTask[RIGHT][Zdir] = HUBO2ZMPInitLegLength(0., BTW_FOOT_Angle_roll*R2D, 1);
        Add_FootTask[LEFT][Zdir] = -Add_FootTask[RIGHT][Zdir];

    }
    else if(window[0].state == DSP_FINAL)
    {
//        if(window[0].timer.current < 0.5)
//        {

            kine_drc_hubo4.FK_LeftFoot_Local(WBIK_Q,des_pLF_3x1_n,qLF);
            kine_drc_hubo4.FK_RightFoot_Local(WBIK_Q,des_pRF_3x1_n,qRF);

            des_pLF_3x1_n[2] =  des_pLF_3x1[2];
            des_pRF_3x1_n[2] =  des_pRF_3x1[2];

//            convert_euler(des_pRF_3x1_n, des_pLF_3x1_n, sharedData->FOG.Roll*D2R*cos(-yaw_angle*D2R)-sharedData->FOG.Pitch*D2R*sin(-yaw_angle*D2R), sharedData->FOG.Roll*D2R*sin(-yaw_angle*D2R)+sharedData->FOG.Pitch*D2R*cos(-yaw_angle*D2R), 0,BTW_FOOT_Angle_roll, BTW_FOOT_Angle_pitch, BTW_FOOT_Angle_yaw,BTW_FOOT_qPEL_comp_4x1);
            convert_euler(des_pRF_3x1_n, des_pLF_3x1_n, sharedData->IMU[0].AccX*D2R*cos(-yaw_angle)- sharedData->IMU[0].AccY*D2R*sin(-yaw_angle), ( sharedData->IMU[0].AccX*D2R)*sin(-yaw_angle)+ sharedData->IMU[0].AccY*D2R*cos(-yaw_angle), 0,BTW_FOOT_Angle_roll, BTW_FOOT_Angle_pitch, BTW_FOOT_Angle_yaw,BTW_FOOT_qPEL_comp_4x1);
            Add_FootTask[RIGHT][Zdir] = HUBO2ZMPInitLegLength(0., BTW_FOOT_Angle_roll*R2D, 1);
            Add_FootTask[LEFT][Zdir] = -Add_FootTask[RIGHT][Zdir];

//        }
//        else
//        {
//            TorsoRollAngle =  1*HUBO2TorsoInitConRoll(0.0, AngleRoll, 1);
//            TorsoPitchAngle = HUBO2TorsoInitConPitch(0.0, AnglePitch, 1);
//        }
    }else if(window[0].state == STATE_EMPTY || window[0].timer.current < 2.0)
    {
//        kine_drc_hubo4.FK_LeftFoot_Local(WBIK_Q,des_pLF_3x1_n,qLF);
//        kine_drc_hubo4.FK_RightFoot_Local(WBIK_Q,des_pRF_3x1_n,qRF);

//        des_pLF_3x1_n[2] =  des_pLF_3x1[2];
//        des_pRF_3x1_n[2] =  des_pRF_3x1[2];

//        convert_euler(des_pRF_3x1_n, des_pLF_3x1_n, sharedData->FOG.Roll*D2R*cos(-yaw_angle*D2R)-sharedData->FOG.Pitch*D2R*sin(-yaw_angle*D2R), sharedData->FOG.Roll*D2R*sin(-yaw_angle*D2R)+sharedData->FOG.Pitch*D2R*cos(-yaw_angle*D2R), 0,BTW_FOOT_Angle_roll, BTW_FOOT_Angle_pitch, BTW_FOOT_Angle_yaw,BTW_FOOT_qPEL_comp_4x1);

//        Add_FootTask[RIGHT][Zdir] = HUBO2ZMPInitLegLength(0., BTW_FOOT_Angle_roll*R2D, 1);
//        Add_FootTask[LEFT][Zdir] = -Add_FootTask[RIGHT][Zdir];
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
//        kine_drc_hubo4.FK_LeftFoot_Local(WBIK_Q,des_pLF_3x1_n,qLF);
//        kine_drc_hubo4.FK_RightFoot_Local(WBIK_Q,des_pRF_3x1_n,qRF);

//        des_pLF_3x1_n[2] =  des_pLF_3x1[2];
//        des_pRF_3x1_n[2] =  des_pRF_3x1[2];

//        Add_FootTask[RIGHT][Zdir] = HUBO2ZMPInitLegLength(0., BTW_FOOT_Angle_roll*R2D, 1);
//        Add_FootTask[LEFT][Zdir] = -Add_FootTask[RIGHT][Zdir];


    }
    else if(window[0].state == DSP_FINAL)
    {

//            kine_drc_hubo4.FK_LeftFoot_Local(WBIK_Q,des_pLF_3x1_n,qLF);
//            kine_drc_hubo4.FK_RightFoot_Local(WBIK_Q,des_pRF_3x1_n,qRF);

//            des_pLF_3x1_n[2] =  des_pLF_3x1[2];
//            des_pRF_3x1_n[2] =  des_pRF_3x1[2];

//            convert_euler(des_pRF_3x1_n, des_pLF_3x1_n, sharedData->FOG.Roll*D2R*cos(-yaw_angle*D2R)-sharedData->FOG.Pitch*D2R*sin(-yaw_angle*D2R), sharedData->FOG.Roll*D2R*sin(-yaw_angle*D2R)+sharedData->FOG.Pitch*D2R*cos(-yaw_angle*D2R), 0,BTW_FOOT_Angle_roll, BTW_FOOT_Angle_pitch, BTW_FOOT_Angle_yaw,BTW_FOOT_qPEL_comp_4x1);

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

//    if(state == SSP_RF || state == SSP_LF){
//        d  = gain*((mRforce+mLforce)/(dRforce + dLforce));
//    }else{
//        d  = gain*(mLforce/dLforce);
//    }
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


//LFootForceControl

double RFootForceControl(int state,double dRforce,double dLforce,double mRforce,double mLforce,int reset,double Qd0 )
{
    static double CL=0.0;
    static double d1 = 10000,d2 = 15000, m = 2.5,dt = 0.005,T = 0.2;
    static double Qd1=0.0,Qd2=0.0,Q[3]={0.0,};


//    if(state == SSP_RF){
//        d1  = d2*(mRforce/dRforce);
//    }else{

//    }

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

//    if(state == SSP_LF){
//        d1  = d2*(mLforce/dLforce);
//    }else{

//    }

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
//    static double d1 = 200,d2 = 28000, m = 2.5,dt = 0.005,T = 0.7;
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

//            Q[0] = ( -sharedData->FOG.Roll*0.5 - sharedData->FOG.RollVel*3.28 )/d1 - (1.0/T)*Q[1];

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
//    printf("alhpa : %f \n",alpha);
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
//    RDesTorqueY = dTorque = (RFoot_x - ref_zmp_x)*RDF + (LFoot_x - ref_zmp_x)*LDF;
    dTorque = 0.;//(RFoot_x - ref_zmp_x)*RDF + (LFoot_x - ref_zmp_x)*LDF;


    //measured
    torqueX = (-(RFoot_x - zmp_x)*RDF - (LFoot_x-zmp_x)*LDF);


//    RmTorqueX = mTorque = alpha*torqueX;

    mTorque = FTmx;//alpha*(sharedData->FOGPitch*4.0 + sharedData->FOGPitchVel*2.0);


    static double CL=0.0;
    static double d = 30, m = 1.5,dt = 0.005;
    static double Qd1=0.0,Qd2=0.0,Q[3]={0.0,},T = 0.4;

//    if(state == 1)
//    {
//        d = 100;//1125;
//        T = 0.4;
//    }else if(state2 == 0)
//    {
//        d = 500;
//        T = 0.1;
//    }


    if(state == 1)
    {
        d = 50.;//1125;
        T = 0.4;
//        d = 10;//1125;
//        T = 0.2;
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
//    RDesTorqueY = dTorque = (RFoot_x - ref_zmp_x)*RDF + (LFoot_x - ref_zmp_x)*LDF;
dTorque = 0.;//(RFoot_x - ref_zmp_x)*RDF + (LFoot_x - ref_zmp_x)*LDF;


    //measured
    torqueX = (-(RFoot_x - zmp_x)*RDF - (LFoot_x-zmp_x)*LDF);


//    RmTorqueX = mTorque = alpha*torqueX;

    mTorque = FTmx;//alpha*(sharedData->FOGPitch*4.0 + sharedData->FOGPitchVel*2.0);


    static double CL=0.0;
    static double d = 30, m = 1.5,dt = 0.005;
    static double Qd1=0.0,Qd2=0.0,Q[3]={0.0,},T = 0.4;

//    if(state2 == 1)
//    {
////        d = 30;//1125;
////        T = 0.4;
//        //hand tuning
//        d = 100;//1125;
//        T = 0.4;
//    }else if(state2 == 0)
//    {
//        d = 500;
//        T = 0.1;
//    }

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
//    RDesTorqueY = dTorque = (RFoot_x - ref_zmp_x)*RDF + (LFoot_x - ref_zmp_x)*LDF;
        dTorque = 0.;

    //measured
    torqueX = (-(RFoot_x - zmp_x)*RDF - (LFoot_x-zmp_x)*LDF);


//    RmTorqueX = mTorque = alpha*torqueX;
//    if(state ==3)
//    {
//        mTorque = -RDF*(sharedData->FOGPitch*0.01 + sharedData->FOGPitchVel*.1);//(RFoot_x - ref_zmp_x)*RDF + (LFoot_x - ref_zmp_x)*LDF;
//    }else
    {
        mTorque = FTmx;//alpha*(sharedData->FOGPitch*4.0 + sharedData->FOGPitchVel*2.0);
    }

    static double CL=0.0;
    static double d = 30, m = 1.5,dt = 0.005;
    static double Qd1=0.0,Qd2=0.0,Q[3]={0.0,},T = 0.4;

    if(state == 1)
    {
        d = 50.;//1125;
        T = 0.4;
//        d = 10;//1125;
//        T = 0.2;
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
//    RDesTorqueY = dTorque = (RFoot_x - ref_zmp_x)*RDF + (LFoot_x - ref_zmp_x)*LDF;
//    dTorque = 0.;//(RFoot_x - ref_zmp_x)*RDF + (LFoot_x - ref_zmp_x)*LDF;
//dTorque = - alpha*(sharedData->FOGPitch*10.0 + sharedData->FOGPitchVel*.01);
        dTorque = 0.;

    //measured
    torqueX = (-(RFoot_x - zmp_x)*RDF - (LFoot_x-zmp_x)*LDF);


//    RmTorqueX = mTorque = alpha*torqueX;
//    if(state2 ==3)
//    {
////        mTorque = LDF*(sharedData->FOGPitch*0.77 + sharedData->FOGPitchVel*.01);//(RFoot_x - ref_zmp_x)*RDF + (LFoot_x - ref_zmp_x)*LDF;
//        mTorque = -LDF*(sharedData->FOGPitch*0.01 + sharedData->FOGPitchVel*0.1);//(RFoot_x - ref_zmp_x)*RDF + (LFoot_x - ref_zmp_x)*LDF;
//    }else
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


//----------------------------------------------- GainOverride
void Upperbody_Gain_Lock()
{
//    RBBoardSetSwitchingMode(CAN2,JMC13, SW_MODE_COMPLEMENTARY);
//    RBBoardSetSwitchingMode(CAN2,JMC14, SW_MODE_COMPLEMENTARY);
//    RBBoardSetSwitchingMode(CAN2,JMC15, SW_MODE_COMPLEMENTARY);
//    RBenableFrictionCompensation(CAN2,JMC13,DISABLE,DISABLE);
//    RBenableFrictionCompensation(CAN2,JMC14,DISABLE,DISABLE);
//    RBenableFrictionCompensation(CAN2,JMC15,DISABLE,DISABLE);
//    RBJointGainOverride(CAN2,JMC13,1000,1000,1000);
//    RBJointGainOverride(CAN2,JMC14,1000,1000,1000);
//    RBJointGainOverride(CAN2,JMC15,1000,1000,1000);

//    RBBoardSetSwitchingMode(CAN3,JMC17, SW_MODE_COMPLEMENTARY);
//    RBBoardSetSwitchingMode(CAN3,JMC18, SW_MODE_COMPLEMENTARY);
//    RBBoardSetSwitchingMode(CAN3,JMC19, SW_MODE_COMPLEMENTARY);
//    RBenableFrictionCompensation(CAN3,JMC17,DISABLE,DISABLE);
//    RBenableFrictionCompensation(CAN3,JMC18,DISABLE,DISABLE);
//    RBenableFrictionCompensation(CAN3,JMC19,DISABLE,DISABLE);
//    RBJointGainOverride(CAN3,JMC17,1000,1000,1000);
//    RBJointGainOverride(CAN3,JMC18,1000,1000,1000);
//    RBJointGainOverride(CAN3,JMC19,1000,1000,1000);
    MCenableFrictionCompensation(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[LSP].canch,JOINT_INFO[LSP].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[LSR].canch,JOINT_INFO[LSR].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[LSY].canch,JOINT_INFO[LSY].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[LEB].canch,JOINT_INFO[LEB].bno, SW_MODE_COMPLEMENTARY);
//    MCenableFrictionCompensation(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, JOINT_INFO[LWY].mch, DISABLE);
//    MCBoardSetSwitchingMode(JOINT_INFO[LWY].canch,JOINT_INFO[LWY].bno, SW_MODE_COMPLEMENTARY);
//    MCenableFrictionCompensation(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, JOINT_INFO[LWP].mch, DISABLE);
//    MCBoardSetSwitchingMode(JOINT_INFO[LWP].canch,JOINT_INFO[LWP].bno, SW_MODE_COMPLEMENTARY);
//    MCenableFrictionCompensation(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, JOINT_INFO[LWY2].mch, DISABLE);
//    MCBoardSetSwitchingMode(JOINT_INFO[LWY2].canch,JOINT_INFO[LWY2].bno, SW_MODE_COMPLEMENTARY);

    MCenableFrictionCompensation(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[RSP].canch,JOINT_INFO[RSP].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[RSY].canch,JOINT_INFO[RSY].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[RSR].canch,JOINT_INFO[RSR].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, DISABLE);
    MCBoardSetSwitchingMode(JOINT_INFO[REB].canch,JOINT_INFO[REB].bno, SW_MODE_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, DISABLE);
//    MCBoardSetSwitchingMode(JOINT_INFO[RWY].canch,JOINT_INFO[RWY].bno, SW_MODE_COMPLEMENTARY);
//    MCenableFrictionCompensation(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, DISABLE);
//    MCBoardSetSwitchingMode(JOINT_INFO[RWP].canch,JOINT_INFO[RWP].bno, SW_MODE_COMPLEMENTARY);
//    MCenableFrictionCompensation(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, DISABLE);
//    MCBoardSetSwitchingMode(JOINT_INFO[RWY2].canch,JOINT_INFO[RWY2].bno, SW_MODE_COMPLEMENTARY);

            MCJointGainOverride(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, 0,500); //--LSP
            MCJointGainOverride(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, 0,500); //--LSR
            MCJointGainOverride(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch, 0,500); //--LSY
            MCJointGainOverride(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch, 0,500); //--LEB

//            MCJointGainOverride(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, JOINT_INFO[LWY].mch, 0,500); //---LWY
//            MCJointGainOverride(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, JOINT_INFO[LWP].mch, 0,500); //--LWP
//            MCJointGainOverride(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, JOINT_INFO[LWY2].mch, 0,500); //--LWY2

            MCJointGainOverride(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, 0,500); //--RSP
            MCJointGainOverride(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, 0,500); //--RSR
            MCJointGainOverride(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch, 0,500); //--RSY
            MCJointGainOverride(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch, 0,500); //--REB

//            MCJointGainOverride(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, 0,500); //---RWY
//            MCJointGainOverride(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, 0,500); //--RWP
//            MCJointGainOverride(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, 0,500); //--RWY2

}

void Upperbody_Gain_Override()
{
//    RBsetFrictionParameter(CAN2, JMC13, 60, 70, 10, 0);
//    RBsetFrictionParameter(CAN2, JMC14, 50, 70, 10, 0);
//    RBsetFrictionParameter(CAN2, JMC15, 60, 50, 70, 50);

//    RBsetFrictionParameter(CAN3, JMC17, 60, 80, 10, 0);
//    RBsetFrictionParameter(CAN3, JMC18, 50, 70, 10, 0);
//    RBsetFrictionParameter(CAN3, JMC19, 60, 55, 60, 42);

//    RBBoardSetSwitchingMode(CAN2,JMC13, SW_MODE_NON_COMPLEMENTARY);
//    RBBoardSetSwitchingMode(CAN2,JMC14, SW_MODE_NON_COMPLEMENTARY);
//    RBBoardSetSwitchingMode(CAN2,JMC15, SW_MODE_NON_COMPLEMENTARY);
//    RBenableFrictionCompensation(CAN2,JMC13,ENABLE,DISABLE);
//    RBenableFrictionCompensation(CAN2,JMC14,ENABLE,DISABLE);
//    RBenableFrictionCompensation(CAN2,JMC15,ENABLE,ENABLE);
//    RBJointGainOverride(CAN2,JMC13,1,1000,1000);
//    RBJointGainOverride(CAN2,JMC14,1,1000,1000);
//    RBJointGainOverride(CAN2,JMC15,1,1,1000);

//    RBBoardSetSwitchingMode(CAN3,JMC17, SW_MODE_NON_COMPLEMENTARY);
//    RBBoardSetSwitchingMode(CAN3,JMC18, SW_MODE_NON_COMPLEMENTARY);
//    RBBoardSetSwitchingMode(CAN3,JMC19, SW_MODE_NON_COMPLEMENTARY);
//    RBenableFrictionCompensation(CAN3,JMC17,ENABLE,DISABLE);
//    RBenableFrictionCompensation(CAN3,JMC18,ENABLE,DISABLE);
//    RBenableFrictionCompensation(CAN3,JMC19,ENABLE,ENABLE);
//    RBJointGainOverride(CAN3,JMC17,1,1000,1000);
//    RBJointGainOverride(CAN3,JMC18,1,1000,1000);
//    RBJointGainOverride(CAN3,JMC19,1,1,1000);
    ZeroGainLeftArm();
    ZeroGainRightArm();
}
int ZeroGainLeftArm(){
    //-- lsp
//    RBsetFrictionParameter(3, 17, 60, 80, 10,0);
//    RBBoardSetSwitchingMode(3,17, SW_MODE_NON_COMPLEMENTARY);
//    RBenableFrictionCompensation(3,17,ENABLE,DISABLE);
//    RBJointGainOverride(3,17,0,1000,1);
//    usleep(5000);
    MCsetFrictionParameter(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, 1000, 10, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, JOINT_INFO[LSP].mch, 50, 10);
    usleep(5000);

    //-- lsr
//    RBsetFrictionParameter(3, 18, 50, 70, 10,0);
//    RBBoardSetSwitchingMode(3,18, SW_MODE_NON_COMPLEMENTARY);
//    RBenableFrictionCompensation(3,18,ENABLE,DISABLE);
//    RBJointGainOverride(3,18,0,1000,1);
//    usleep(5000);
    MCsetFrictionParameter(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, 1000, 7, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, JOINT_INFO[LSR].mch, 50, 10);
    usleep(5000);

    //--- lsy, leb
//    RBsetFrictionParameter(3, 19, 60, 55, 60, 42);
//    RBBoardSetSwitchingMode(3,19, SW_MODE_NON_COMPLEMENTARY);
//    RBenableFrictionCompensation(3,19,ENABLE,ENABLE);
//    RBJointGainOverride(3,19,0,0,1);
//    usleep(5000);
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

//    //--- lwy, lwp
//    RBsetFrictionParameter(3, 20, 80, 100, 60,115);
//    RBBoardSetSwitchingMode(3,20, SW_MODE_NON_COMPLEMENTARY);
//    RBenableFrictionCompensation(3,20,ENABLE,ENABLE);
//    RBJointGainOverride(3,20,0,0,1);
//    usleep(5000);
//    MCsetFrictionParameter(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, JOINT_INFO[LWY].mch, 330, 5, 0);
//    MCBoardSetSwitchingMode(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, SW_MODE_NON_COMPLEMENTARY);
//    MCenableFrictionCompensation(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, JOINT_INFO[LWY].mch, ENABLE);
//    MCJointGainOverride(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, JOINT_INFO[LWY].mch, 100, 10);
//    usleep(5000);
//    MCsetFrictionParameter(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, JOINT_INFO[LWP].mch, 350, 5, 0);
//    MCBoardSetSwitchingMode(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, SW_MODE_NON_COMPLEMENTARY);
//    MCenableFrictionCompensation(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, JOINT_INFO[LWP].mch, ENABLE);
//    MCJointGainOverride(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, JOINT_INFO[LWP].mch, 100, 10);
//    usleep(5000);

//    //---- lwy2
//    //RBsetFrictionParameter(3, 37, 6, 120, 10,0);  // 11W motor(Iptime Hubo)
////    RBsetFrictionParameter(3, 37, 400, 50, 10,0);
////    RBenableFrictionCompensation(3,37,ENABLE,DISABLE);
////    RBJointGainOverride(3,37,0,1000,1);
//    MCsetFrictionParameter(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, JOINT_INFO[LWY2].mch, 280, 150, 0);
//    MCenableFrictionCompensation(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, JOINT_INFO[LWY2].mch, ENABLE);
//    MCJointGainOverride(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, JOINT_INFO[LWY2].mch, 100, 10);
//    usleep(5000);

//    RBJointOLCurrentCommand2ch(3, 17, 0, 0, 0x05);
//    RBJointOLCurrentCommand2ch(3, 18, 0, 0, 0x05);
//    RBJointOLCurrentCommand2ch(3, 19, 0, 0, 0x05);
//    RBJointOLCurrentCommand2ch(3, 20, 0, 0, 0x05);
//    RBJointOLCurrentCommand2ch(3, 37, 0, 0, 0x05);
//    MCJointPWMCommand2chHR(JOINT_INFO[LSP].canch, JOINT_INFO[LSP].bno, 4, 0, 0, 0);
//    MCJointPWMCommand2chHR(JOINT_INFO[LSR].canch, JOINT_INFO[LSR].bno, 4, 0, 0, 0);
//    MCJointPWMCommand2chHR(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, 4, 0, 4, 0);
//    MCJointPWMCommand2chHR(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, 4, 0, 4, 0);
//    MCJointPWMCommand2chHR(JOINT_INFO[LWY2].canch, JOINT_INFO[LWY2].bno, 4, 0, 0, 0);



    cout<<"Zero gain LeftArm!"<<endl;
    return 0;
}
int ZeroGainRightArm(){
    //-- Rsp
//    RBsetFrictionParameter(2, 13, 60, 70, 10,0);
//    RBBoardSetSwitchingMode(2, 13, SW_MODE_NON_COMPLEMENTARY);
//    RBenableFrictionCompensation(2, 13,ENABLE,DISABLE);
//    RBJointGainOverride(2, 13,0,1000,1);
//    usleep(5000);
    MCsetFrictionParameter(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, 1000, 8, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, JOINT_INFO[RSP].mch, 50, 10);
    usleep(5000);

    //-- Rsr
//    RBsetFrictionParameter(2, 14, 50, 70, 10,0);
//    RBBoardSetSwitchingMode(2, 14, SW_MODE_NON_COMPLEMENTARY);
//    RBenableFrictionCompensation(2, 14,ENABLE,DISABLE);
//    RBJointGainOverride(2, 14,0,1000,1);
//    usleep(5000);
    MCsetFrictionParameter(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, 1000, 7, 0);
    MCBoardSetSwitchingMode(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, SW_MODE_NON_COMPLEMENTARY);
    MCenableFrictionCompensation(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, ENABLE);
    MCJointGainOverride(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, JOINT_INFO[RSR].mch, 50, 10);
    usleep(5000);

    //--- Rsy, Reb
//    RBsetFrictionParameter(2, 15, 60, 50, 70, 50);
//    RBBoardSetSwitchingMode(2, 15, SW_MODE_NON_COMPLEMENTARY);
//    RBenableFrictionCompensation(2, 15,ENABLE,ENABLE);
//    RBJointGainOverride(2, 15,0,0,1);
//    usleep(5000);
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

    //--- Rwy, Rwp
//    RBsetFrictionParameter(2, 16, 80, 80, 60,110);
//    RBBoardSetSwitchingMode(2, 16, SW_MODE_NON_COMPLEMENTARY);
//    RBenableFrictionCompensation(2, 16,ENABLE,ENABLE);
//    RBJointGainOverride(2, 16,0,0,1);
//    usleep(5000);
//    MCsetFrictionParameter(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, 360, 5, 0);
//    MCBoardSetSwitchingMode(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, SW_MODE_NON_COMPLEMENTARY);
//    MCenableFrictionCompensation(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, ENABLE);
//    MCJointGainOverride(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch, 100, 10);
//    usleep(5000);
//    MCsetFrictionParameter(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, 350,6, 0);
//    MCBoardSetSwitchingMode(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, SW_MODE_NON_COMPLEMENTARY);
//    MCenableFrictionCompensation(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, ENABLE);
//    MCJointGainOverride(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch, 100, 10);
//    usleep(5000);

    //---- Rwy2
    //RBsetFrictionParameter(2, 36, 6, 120, 10,0);
//    RBsetFrictionParameter(2, 36, 400, 50, 10,0);
//    RBenableFrictionCompensation(2, 36,ENABLE,DISABLE);
//    RBJointGainOverride(2, 36,0,1000,1);
//    MCsetFrictionParameter(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, 280, 120, 0);
//    MCBoardSetSwitchingMode(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, SW_MODE_NON_COMPLEMENTARY);
//    MCenableFrictionCompensation(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, ENABLE);
//    MCJointGainOverride(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, 65, 10);
//    usleep(5000);

//    //---- RAP
//    RBsetFrictionParameter(0, 4, 60, 70, 10,0);
//    RBBoardSetSwitchingMode(0, 4, SW_MODE_NON_COMPLEMENTARY);
//    RBenableFrictionCompensation(0, 4,ENABLE,DISABLE);
//    RBJointGainOverride(0, 4,0,1000,1);

//    RBJointOLCurrentCommand2ch(2, 13, 0, 0, 0x05);
//    RBJointOLCurrentCommand2ch(2, 14, 0, 0, 0x05);
//    RBJointOLCurrentCommand2ch(2, 15, 0, 0, 0x05);
//    RBJointOLCurrentCommand2ch(2, 16, 0, 0, 0x05);
//    RBJointOLCurrentCommand2ch(2, 36, 0, 0, 0x05);
//    MCJointPWMCommand2chHR(JOINT_INFO[RSP].canch, JOINT_INFO[RSP].bno, 4, 0, 0, 0);
//    MCJointPWMCommand2chHR(JOINT_INFO[RSR].canch, JOINT_INFO[RSR].bno, 4, 0, 0, 0);
//    MCJointPWMCommand2chHR(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, 4, 0, 4, 0);
//    MCJointPWMCommand2chHR(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, 4, 0, 4, 0);
//    MCJointPWMCommand2chHR(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, 4, 0, 0, 0);

    cout<<"Zero Gain Right Arm!"<<endl;

    return 0;
}



// ----------------------------------------------- FeedForward Motino Control
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

//        printf("comp LAR :%f \n",deflection_comp_LAR);
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
//        printf("comp RAR :%f \n",deflection_comp_RAR);
    }
}

//-----------------------------------------------------------------------------------------------//
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
    //    if((GLOBAL_Z_LF>0.01)||(GLOBAL_Z_RF>0.01))
    //    {
    //        JW_InvPattern_A[1][0] =-JW_InvPattern_k/JW_InvPattern_m/2;
    //        JW_InvPattern_A[1][1] =-JW_InvPattern_c/JW_InvPattern_m/2;
    //    }

        JW_InvPattern_l = sqrt((userData->WalkReadyCOM[2])*(userData->WalkReadyCOM[2]) + Pattern1*Pattern1);

        if(pv_Index == 1){

            Y_inv = Pattern1;
            Y_inv_d = Pattern1_d;
            theta = atan2(Pattern1,(userData->WalkReadyCOM[2]));
            theta_d = 0;
            U_I[0] = 0.0f;
            Y_inv_old = Pattern1;
            //printf("Invmodel reset!!!\n");
        }
        U_I[0] +=.1*(Pattern1-Y_inv);
        U[0]   = JW_InvPattern_Klqr[0]*(Pattern1 - Y_inv) + JW_InvPattern_Klqr[1]*(Pattern1_d - Y_inv_d) + U_I[0];

        theta_ref = atan2(U[0],(userData->WalkReadyCOM[2]));
        if(pv_Index == 1){
            theta_ref =-(((9.81/JW_InvPattern_l*sin(theta))/(1/JW_InvPattern_m/(JW_InvPattern_l*JW_InvPattern_l))-JW_InvPattern_c*(theta_d))/JW_InvPattern_k - theta) ;//*(*(theta-theta_ref)+);
            //theta_ref = theta;
        }
        theta_dd = 9.81/JW_InvPattern_l*sin(theta)-1/JW_InvPattern_m/(JW_InvPattern_l*JW_InvPattern_l)*(JW_InvPattern_k*(theta-theta_ref)+JW_InvPattern_c*(theta_d));
        theta_d = theta_d + theta_dd*DEL_T;
        theta   = theta + theta_d*DEL_T;

        //Y_inv_d=Hubo2->WalkReadyCOM[2]/cos(theta)/cos(theta)*theta_d;
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
//==============================//
// Basic Function
//==============================//



void Fifth(double t1,double tf,double p0[3],double pf[3],double ref[3])
{

    double a[6] = {0.0,};
    double theta_0 = p0[0],theta_dot_0 = p0[1],theta_ddot_0 = p0[2],theta_f = pf[0],theta_dot_f = pf[1],theta_ddot_f = pf[2];

    a[0] = theta_0;
    a[1] = theta_dot_0;
    a[2] = theta_ddot_0/2.;
    a[3] = (20.*(theta_f-theta_0) - (8.*theta_dot_f + 12.*theta_dot_0)*tf-(3.*theta_ddot_0 - theta_ddot_f)*tf*tf)/(2.*tf*tf*tf);
    a[4] = ((30.*theta_0 - 30.*theta_f) + (14.*theta_dot_f+16.*theta_dot_0)*tf+(3.*theta_ddot_0-2.*theta_ddot_f)*tf*tf)/(2.*tf*tf*tf*tf);
    a[5] = (12.*theta_f - 12.*theta_0 - (6.*theta_dot_f + 6.*theta_dot_0)*tf - (theta_ddot_0-theta_ddot_f)*tf*tf)/(2.*tf*tf*tf*tf*tf);

    ref[0] = a[0] + a[1]*t1  + a[2]*t1*t1 + a[3]*t1*t1*t1 + a[4]*t1*t1*t1*t1 + a[5]*t1*t1*t1*t1*t1;
    ref[1] = a[1] + 2.0*a[2]*t1 + 3.0*a[3]*t1*t1 + 4.0*a[4]*t1*t1*t1 + 5.0*a[5]*t1*t1*t1*t1;
    ref[2] = 2.0*a[2] + 6.0*a[3]*t1 + 12.0*a[4]*t1*t1 + 20.0*a[5]*t1*t1*t1;
}

void get_WBIK_Q_from_RefAngleCurrent()
{
    jCon->RefreshToCurrentReference();
    for(int i=RHY; i<=LAR; i++) {
       WBIK_Q[i+7] = jCon->Joints[i]->RefAngleCurrent*D2R;
//        printf("%d : %f \n",i,WBIK_Q[i+7]);

    }
//    WBIK_Q[idWST] = joint->GetJointRefAngle(WST)*D2R;
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

    Qub[idRSR] = WBIK_Q[idRSR];// + OFFSET_RSR*D2R;//= 10.*D2R;//joint->Joints[RSR]->RefAngleCurrent*D2R;
    Qub[idRSP] = WBIK_Q[idRSP];//= 40.*D2R;//joint->Joints[RSP]->RefAngleCurrent*D2R;
    Qub[idRSY] = WBIK_Q[idRSY];//= 0;//joint->Joints[RSY]->RefAngleCurrent*D2R;
    Qub[idREB] = WBIK_Q[idREB];// + OFFSET_ELB*D2R;//= -10*D2R;//-130.*D2R;//joint->Joints[REB]->RefAngleCurrent*D2R;
    Qub[idRWY] = WBIK_Q[idRWY];//= 0;//joint->Joints[RWY]->RefAngleCurrent*D2R;
    Qub[idRWP] = WBIK_Q[idRWP];//= 20.*D2R;//joint->Joints[RWP]->RefAngleCurrent*D2R;
    Qub[idRWY] = WBIK_Q[idRWY];
    Qub[idRWY2] = WBIK_Q[idRWY2];

    Qub[idLSR] = WBIK_Q[idLSR];// + OFFSET_LSR*D2R;//= -10.*D2R;//joint->Joints[LSR]->RefAngleCurrent*D2R;
    Qub[idLSP] = WBIK_Q[idLSP];//= 40.*D2R;//joint->Joints[LSP]->RefAngleCurrent*D2R;
    Qub[idLSY] = WBIK_Q[idLSY];//= 0;//joint->Joints[LSY]->RefAngleCurrent*D2R;
    Qub[idLEB] = WBIK_Q[idLEB];// + OFFSET_ELB*D2R;//= -10*D2R;//-130.*D2R;//joint->Joints[LEB]->RefAngleCurrent*D2R;
    Qub[idLWY] = WBIK_Q[idLWY];//= 0;//joint->Joints[LWY]->RefAngleCurrent*D2R;
    Qub[idLWP] = WBIK_Q[idLWP];//= 20.*D2R;//joint->Joints[LWP]->RefAngleCurrent*D2R;
    Qub[idLWY] = WBIK_Q[idLWY];
    Qub[idLWY2] = WBIK_Q[idLWY2];

    Qub[idWST] = WBIK_Q[idWST];//= 0;//-180*D2R;//joint->Joints[WST]->RefAngleCurrent*D2R;


//    for(int i=0; i<=33; i++) {

//        printf("%d : %f \n",i,WBIK_Q[i]);

//    }
}


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

    //ine_drc_hubo.IK_LowerBody_Global(Qub,des_pCOM_3x1, temp_4x1, FK_pRFoot_3x1, FK_qRFoot_4x1, FK_pLFoot_3x1, FK_qLFoot_4x1,WBIK_Q);

}








void WBIK_PARA_CHANGE(){

///////////////////////////////////////////////////////////////////
//// Original without the head
//    kine_drc_hubo4.C_Torso[0] = 0.000941-0.065;
//    kine_drc_hubo4.m_Torso = 24.98723;
//    kine_drc_hubo4.m_RightWrist = 4.5;
//    kine_drc_hubo4.m_LeftWrist = 4.5;
//    kine_drc_hubo4.m_Pelvis = 11.867886 +2.;
//    kine_drc_hubo4.C_Torso[1] =0;
//    kine_drc_hubo4.L_FOOT = 0.113;
//    kine_drc_hubo4.iter_limit = 100;
//    kine_drc_hubo4.converge_criterium = 1e-6;



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







///////////////////////////////////////////////////////////////////

}

//-------------------------------------------------------------------------------------

void Walking_initialize()
{

    printf("##################### Walking Initialize!!!!!!!!!!!!!!!!!!! \n");
    FINAL_TIMER = 0.;
    Controller_initialize();

//    des_pRF_3x1[0] =0;
//    des_pRF_3x1[1] =0;
//    des_pRF_3x1[2] =0;

//    des_pLF_3x1[0] =0;
//    des_pLF_3x1[1] =0;
//    des_pLF_3x1[2] =0;

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
    //printf("com z = %f,pel z = %f\n",FK_pCOM_3x1[2],WBIK_Q[idZ]);


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
    //ine_drc_hubo.IK_LowerBody_Global(Qub,des_pCOM_3x1, temp_4x1, FK_pRFoot_3x1, FK_qRFoot_4x1, FK_pLFoot_3x1, FK_qLFoot_4x1,WBIK_Q);









    //Set Foot print Initial value


//    double temp_yaw,temp_pitch,temp_roll;

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





// --------------------------------------------------------------------------------------------- //

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


    //printf("com z = %f,pel z = %f\n",FK_pCOM_3x1[2],WBIK_Q[idZ]);
    WBIK_Q[idX] = WBIK_Q[idX] - FK_pCOM_3x1[0];//reset to 0;
    WBIK_Q[idY] = WBIK_Q[idY] - FK_pCOM_3x1[1];//reset to 0;
    //printf("WBIK_Q[idZ] = %f,FK_pCOM_3x1[2] = %f,AddComInfos[0][2] = %f\n",WBIK_Q[idZ],FK_pCOM_3x1[2],fsm->AddComInfos[0][2]);
    WBIK_Q[idZ] = WBIK_Q[idZ] - FK_pCOM_3x1[2] + userData->WalkReadyCOM[2];// + fsm->AddComInfos[0][2];//0;
    //printf("WBIK_Q[idZ] = %f,FK_pCOM_3x1[2] = %f,AddComInfos[0][2] = %f\n",WBIK_Q[idZ],FK_pCOM_3x1[2],fsm->AddComInfos[0][2]);

    kine_drc_hubo4.FK_RightFoot_Global(WBIK_Q,FK_pRFoot_3x1,  FK_qRFoot_4x1);
    kine_drc_hubo4.FK_LeftFoot_Global(WBIK_Q,FK_pLFoot_3x1,  FK_qLFoot_4x1);
    kine_drc_hubo4.FK_COM_Global(WBIK_Q,FK_pCOM_3x1);

    printf("FK com = %f,%f,%f,WBIK_Q[idXYZ] = %f,%f,%f\n",FK_pCOM_3x1[0],FK_pCOM_3x1[1],FK_pCOM_3x1[2],WBIK_Q[idX],WBIK_Q[idY],WBIK_Q[idZ]);
    //ine_drc_hubo.IK_LowerBody_Global(Qub,des_pCOM_3x1, temp_4x1, FK_pRFoot_3x1, FK_qRFoot_4x1, FK_pLFoot_3x1, FK_qLFoot_4x1,WBIK_Q);



    printf("FK_pRFoot_3x1[0] = %f, FK_pRFoot_3x1[1] = %f, FK_pRFoot_3x1[2] = %f,FK_pLFoot_3x1[0] = %f, FK_pLFoot_3x1[1] = %f, FK_pLFoot_3x1[2] = %f\n",FK_pRFoot_3x1[0],FK_pRFoot_3x1[1],FK_pRFoot_3x1[2],FK_pLFoot_3x1[0],FK_pLFoot_3x1[1],FK_pLFoot_3x1[2]);

    GLOBAL_Z_RF = 0;//FK_pRFoot_3x1[2];//fsm->RightInfos[0][2];
    GLOBAL_Z_LF = 0;//FK_pLFoot_3x1[2];//fsm->LeftInfos[0][2];
}

//-------------------------------------------------------------------------------------
//  QP Formulation
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



  //    printf("==================================================================\n");
  //    printf("--Ppu : %f    %f  %f    %f    %f  %f    %f    %f  %f    %f    %f  %f \n",Ppu[0][0],Ppu[0][1],Ppu[0][2],Ppu[0][3],Ppu[0][4],Ppu[0][5],Ppu[0][6],Ppu[0][7],Ppu[0][8],Ppu[0][9],Ppu[0][10],Ppu[0][11]);
  //    printf("--Ppu : %f    %f  %f    %f    %f  %f    %f    %f  %f    %f    %f  %f \n",Ppu[1][0],Ppu[1][1],Ppu[1][2],Ppu[1][3],Ppu[1][4],Ppu[1][5],Ppu[1][6],Ppu[1][7],Ppu[1][8],Ppu[1][9],Ppu[1][10],Ppu[1][11]);
  //    printf("--Ppu : %f    %f  %f    %f    %f  %f    %f    %f  %f    %f    %f  %f \n",Ppu[29][0],Ppu[29][1],Ppu[29][2],Ppu[29][3],Ppu[29][4],Ppu[29][5],Ppu[29][6],Ppu[29][7],Ppu[29][8],Ppu[29][9],Ppu[29][10],Ppu[29][11]);
  //    printf("==================================================================\n");

  //    printf("--Pvu : %f    %f  %f    %f    %f  %f    %f    %f  %f    %f    %f  %f \n",Pvu[0][0],Pvu[0][1],Pvu[0][2],Pvu[0][3],Pvu[0][4],Pvu[0][5],Pvu[0][6],Pvu[0][7],Pvu[0][8],Pvu[0][9],Pvu[0][10],Pvu[0][11]);
  //    printf("--Pvu : %f    %f  %f    %f    %f  %f    %f    %f  %f    %f    %f  %f \n",Pvu[1][0],Pvu[1][1],Pvu[1][2],Pvu[1][3],Pvu[1][4],Pvu[1][5],Pvu[1][6],Pvu[1][7],Pvu[1][8],Pvu[1][9],Pvu[1][10],Pvu[1][11]);
  //    printf("--Pvu : %f    %f  %f    %f    %f  %f    %f    %f  %f    %f    %f  %f \n",Pvu[29][0],Pvu[29][1],Pvu[29][2],Pvu[29][3],Pvu[29][4],Pvu[29][5],Pvu[29][6],Pvu[29][7],Pvu[29][8],Pvu[29][9],Pvu[29][10],Pvu[29][11]);

  //    printf("==================================================================\n");
  //    printf("--Pzu : %f    %f  %f    %f    %f  %f    %f    %f  %f    %f    %f  %f \n",Pzu[0][0],Pzu[0][1],Pzu[0][2],Pzu[0][3],Pzu[0][4],Pzu[0][5],Pzu[0][6],Pzu[0][7],Pzu[0][8],Pzu[0][9],Pzu[0][10],Pzu[0][11]);
  //    printf("--Pzu : %f    %f  %f    %f    %f  %f    %f    %f  %f    %f    %f  %f \n",Pzu[1][0],Pzu[1][1],Pzu[1][2],Pzu[1][3],Pzu[1][4],Pzu[1][5],Pzu[1][6],Pzu[1][7],Pzu[1][8],Pzu[1][9],Pzu[1][10],Pzu[1][11]);
  //    printf("--Pzu : %f    %f  %f    %f    %f  %f    %f    %f  %f    %f    %f  %f \n",Pzu[29][0],Pzu[29][1],Pzu[29][2],Pzu[29][3],Pzu[29][4],Pzu[29][5],Pzu[29][6],Pzu[29][7],Pzu[29][8],Pzu[29][9],Pzu[29][10],Pzu[29][11]);

      // MPC_Q


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


            Save_Index++;

            if(Save_Index >= ROW) Save_Index = 0;


    }
}



//-------------------------------------------------------------------------------------

int InvMatrix(int n, double* A, double* b)
{
    double m;
    register int i, j, k;
    double* a = new double[n*n];

    if(a==NULL)
  return 0;
    for(i=0; i<n*n; i++)
  a[i]=A[i];
 for(i=0; i<n; i++)
 {
  for(j=0; j<n; j++)
  {
            b[i*n+j]=(i==j)?1.:0.;
        }
    }
    for(i=0; i<n; i++)
 {
  if(a[i*n+i]==0.)
  {
   if(i==n-1)
   {
    delete[] a;
    return 0;
            }
            for(k=1; i+k<n; k++)
   {
                if(a[i*n+i+k] != 0.)
     break;
            }
            if(i+k>=n)
   {
                delete[] a;
    return 0;
            }
            for(j=0; j<n; j++)
   {
                m = a[i*n+j];
                a[i*n+j] = a[(i+k)*n+j];
                a[(i+k)*n+j] = m;
                m = b[i*n+j];
                b[i*n+j] = b[(i+k)*n+j];
                b[(i+k)*n+j] = m;
            }
        }
        m = a[i*n+i];
        for(j=0; j<n; j++)
  {
            a[i*n+j]/=m;
            b[i*n+j]/=m;
        }
        for(j=0; j<n; j++)
  {
            if(i==j)
    continue;

            m = a[j*n+i];
            for(k=0; k<n; k++)
   {
                a[j*n+k] -= a[i*n+k]*m;
                b[j*n+k] -= b[i*n+k]*m;
            }
        }
    }
    delete[] a;
    return 1;
}



void mat15by3x3by1(double a[15][3],double b[3],double out[15])
{

    double sum = 0.0f;

    for(int i=0;i<15;i++)
    {
            for(int j=0;j<3;j++)
            {
                sum = sum + a[i][j]*b[j];
//                printf("%f \n",a_hat[i][j]*a_hat[j][k]);
            }

            out[i] = sum ;
            sum = 0.0f;


    }

}

void mat6by3x3by1(double a[6][3],double b[3],double out[6])
{

    double sum = 0.0f;

    for(int i=0;i<6;i++)
    {
            for(int j=0;j<3;j++)
            {
                sum = sum + a[i][j]*b[j];
//                printf("%f \n",a_hat[i][j]*a_hat[j][k]);
            }

            out[i] = sum ;
            sum = 0.0f;


    }

}




void mat30by3x3by1(double a[30][3],double b[3],double out[30])
{

    double sum = 0.0f;

    for(int i=0;i<30;i++)
    {
            for(int j=0;j<3;j++)
            {
                sum = sum + a[i][j]*b[j];
//                printf("%f \n",a_hat[i][j]*a_hat[j][k]);
            }

            out[i] = sum ;
            sum = 0.0f;


    }

}



void mat15by15x15by1(double a[15][15],double b[15],double out[15])
{

    double sum = 0.0f;

    for(int i=0;i<15;i++)
    {
            for(int j=0;j<15;j++)
            {
                sum = sum + a[i][j]*b[j];
//                printf("%f \n",a_hat[i][j]*a_hat[j][k]);
            }

            out[i] = sum ;
            sum = 0.0f;


    }

}



void mat30by30x30by1(double a[30][30],double b[30],double out[30])
{

    double sum = 0.0f;

    for(int i=0;i<30;i++)
    {
            for(int j=0;j<30;j++)
            {
                sum = sum + a[i][j]*b[j];
//                printf("%f \n",a_hat[i][j]*a_hat[j][k]);
            }

            out[i] = sum ;
            sum = 0.0f;


    }

}
void mat6by3x3by6(double a[6][3],double b[3][6],double out[6][6])
{

    double sum = 0.0f;

    for(int i=0;i<6;i++)
    {
        for(int k = 0;k<6;k++)
        {
            for(int j=0;j<3;j++)
            {
                sum = sum + a[i][j]*b[j][k];
//                printf("%f \n",a_hat[i][j]*a_hat[j][k]);
            }

            out[i][k] = sum ;
            sum = 0.0f;
        }

    }

}


void mat30by30x30by30(double a[30][30],double b[30][30],double out[30][30])
{

    double sum = 0.0f;

    for(int i=0;i<30;i++)
    {
        for(int k = 0;k<30;k++)
        {
            for(int j=0;j<30;j++)
            {
                sum = sum + a[i][j]*b[j][k];
//                printf("%f \n",a_hat[i][j]*a_hat[j][k]);
            }

            out[i][k] = sum ;
            sum = 0.0f;
        }

    }

}

void mat15by15x15by15(double a[15][15],double b[15][15],double out[15][15])
{

    double sum = 0.0f;

    for(int i=0;i<15;i++)
    {
        for(int k = 0;k<15;k++)
        {
            for(int j=0;j<15;j++)
            {
                sum = sum + a[i][j]*b[j][k];
//                printf("%f \n",a_hat[i][j]*a_hat[j][k]);
            }

            out[i][k] = sum ;
            sum = 0.0f;
        }

    }

}

void mat15by15x15by3(double a[15][15],double b[15][3],double out[15][3])
{

    double sum = 0.0f;

    for(int i=0;i<15;i++)
    {
        for(int k = 0;k<3;k++)
        {
            for(int j=0;j<15;j++)
            {
                sum = sum + a[i][j]*b[j][k];
//                printf("%f \n",a_hat[i][j]*a_hat[j][k]);
            }

            out[i][k] = sum ;
            sum = 0.0f;
        }

    }

}

void mat15by3minus15by3(double a[15][3],double b[15][3],double out[15][3])
{

    for(int i=0;i<15;i++)
    {
        for(int k = 0;k<3;k++)
        {
            out[i][k] = a[i][k] - b[i][k];

        }

    }

}


void mat3by3x3by1(double a[3][3],double b[3],double out[3])
{

    double sum = 0.0f;

    for(int i=0;i<=2;i++)
    {
            for(int j=0;j<=2;j++)
            {
                sum = sum + a[i][j]*b[j];
//                printf("%f \n",a_hat[i][j]*a_hat[j][k]);
            }

            out[i] = sum ;
            sum = 0.0f;


    }

}







//-----------------------------------------------------------------------------------------------//
void Poly_5th(double _time,double _ti,double _tf,double _pi,double _vi,double _ai,double _pf,double *_info_y)
{
    double coe[6], pow5, pow4, pow3, pow2;
    double _y,_y_d,_y_dd;
    _time=_time-_ti;
    _tf=_tf-_ti;

    pow5 = _tf*_tf*_tf*_tf*_tf;//pow(_tf, 5);
    pow4 = _tf*_tf*_tf*_tf;//pow(_tf, 4);
    pow3 = _tf*_tf*_tf;//pow(_tf, 3);
    pow2 = _tf*_tf;//pow(_tf, 2);

    coe[0]= (double)(-6./pow5*_pi -3./pow4*_vi -1./2./pow3*_ai +6./pow5*_pf);
    coe[1] = (double)(15./pow4*_pi +8./pow3*_vi +3./2./pow2*_ai -15./pow4*_pf);
    coe[2] = (double)(-10./pow3*_pi -6./pow2*_vi -3./2./_tf*_ai +10./pow3*_pf);
    coe[3] = (double)(1./2.*_ai);
    coe[4] = (double)(1.*_vi);
    coe[5] = (double)(_pi);

    pow5 = _time*_time*_time*_time*_time;//pow(_time, 5);
    pow4 = _time*_time*_time*_time;//pow(_time, 4);
    pow3 = _time*_time*_time;//pow(_time, 3);
    pow2 = _time*_time;//pow(_time, 2);

    _y      = coe[0]*pow5+coe[1]*pow4+coe[2]*pow3+coe[3]*pow2+coe[4]*(_time)+coe[5];
    _y_d    = 5.*coe[0]*pow4+4.*coe[1]*pow3+3.*coe[2]*pow2+2.*coe[3]*_time+coe[4];
    _y_dd   = 4.*5.*coe[0]*pow3+3.*4.*coe[1]*pow2+2.*3.*coe[2]*_time+2.*coe[3];

    _info_y[0] = _y;
    _info_y[1] = _y_d;
    _info_y[2] = _y_dd;
    //return _y;
}


int convert_euler(double pRF_3x1[], double pLF_3x1[], double euler_global_x, double euler_global_y, double euler_global_z,
                  double &euler_stance_x, double &euler_stance_y, double &euler_stance_z, double qPEL_comp_4x1[])
{
    double prf_3x1[3] = {pRF_3x1[0], pRF_3x1[1], pRF_3x1[2]};
    double plf_3x1[3] = {pLF_3x1[0], pLF_3x1[1], pLF_3x1[2]};
    double stance_x_3x1[3], stance_y_3x1[3];
    double stance_z_3x1[3] = {0, 0, 1};
    double stance_frame_3x3[9], pel_global_3x3[9];
    double temp, temp2_4x1[4], temp3_4x1[4], temp4_4x1[4], temp5_3x3[9];
    double frame_meas_3x3[9];

    prf_3x1[2] = 0.;
    plf_3x1[2] = 0.;

    diff_vv(plf_3x1,3, prf_3x1, stance_y_3x1);
    cross(1., stance_y_3x1, stance_z_3x1, stance_x_3x1);

    temp = norm_v(stance_y_3x1,3);
    if(temp < 1e-6)
        return -1;

    stance_y_3x1[0] /= temp;
    stance_y_3x1[1] /= temp;
    stance_y_3x1[2] /= temp;

    temp = norm_v(stance_x_3x1,3);
    if(temp < 1e-6)
        return -1;

    stance_x_3x1[0] /= temp;
    stance_x_3x1[1] /= temp;
    stance_x_3x1[2] /= temp;

    stance_frame_3x3[0] = stance_x_3x1[0];
    stance_frame_3x3[3] = stance_x_3x1[1];
    stance_frame_3x3[6] = stance_x_3x1[2];

    stance_frame_3x3[1] = stance_y_3x1[0];
    stance_frame_3x3[4] = stance_y_3x1[1];
    stance_frame_3x3[7] = stance_y_3x1[2];

    stance_frame_3x3[2] = stance_z_3x1[0];
    stance_frame_3x3[5] = stance_z_3x1[1];
    stance_frame_3x3[8] = stance_z_3x1[2];

//    qtRZ(euler_global_z, temp2_4x1);
//    qtRY(euler_global_y, temp3_4x1);
//    QTcross(temp2_4x1,temp3_4x1, temp4_4x1);
//    qtRX(euler_global_x, temp2_4x1);
//    QTcross(temp4_4x1,temp2_4x1, temp3_4x1);

    qtRZ(euler_global_z, temp2_4x1);
    qtRX(euler_global_x, temp3_4x1);
    QTcross(temp2_4x1,temp3_4x1, temp4_4x1);
    qtRY(euler_global_y, temp2_4x1);
    QTcross(temp4_4x1,temp2_4x1, temp3_4x1);

    QT2DC(temp3_4x1, pel_global_3x3);

    mult_mm(pel_global_3x3,3,3,stance_frame_3x3,3, frame_meas_3x3);

    transpose(1., stance_frame_3x3, 3,3);
    mult_mm(stance_frame_3x3,3,3, frame_meas_3x3,3, temp5_3x3);

    euler_stance_y = atan2(-temp5_3x3[3*2+0], temp5_3x3[3*2+2]);
    euler_stance_z = atan2(temp5_3x3[3*1+0]*cos(euler_stance_y)+temp5_3x3[3*1+2]*sin(euler_stance_y), temp5_3x3[3*0+0]*cos(euler_stance_y)+temp5_3x3[3*0+2]*sin(euler_stance_y));
    euler_stance_x = atan2(temp5_3x3[3*2+1], temp5_3x3[3*2+2]*cos(euler_stance_y)-temp5_3x3[3*2+0]*sin(euler_stance_y));

    double temp_integral;
    temp_integral=PitchRoll_Ori_Integral(0.0, euler_stance_y, 1);
    //printf("PitchRoll_Ori_Integral = %f\n",temp_integral*R2D);

    qPEL_comp_4x1[0] = cos(temp_integral);
    qPEL_comp_4x1[1] = stance_y_3x1[0]*sin(temp_integral);
    qPEL_comp_4x1[2] = stance_y_3x1[1]*sin(temp_integral);
    qPEL_comp_4x1[3] = stance_y_3x1[2]*sin(temp_integral);

    return 0;
 }
double PitchRoll_Ori_Integral(double _ref, double _angle, int _zero)
{
    static double y;
    static double sume = 0.;
    const double KI = 0.11;
    const double KD =  0.8;
    static double angle_last =0.;

    y = 4.0*(_ref - _angle) + KI*sume + KD*(_angle - angle_last);

    sume += _ref - _angle;

//    if(sume >0.005*D2R/KI) sume = 0.005*D2R/KI;
//    else if(sume < -0.005*D2R/KI) sume = -0.005*D2R/KI;

    if(y > 100.0*D2R) y = 100.0*D2R;
    else if(y < -100.0*D2R) y = -100.0*D2R;

    if(_zero == 0)
    {
        sume = 0.;
        y=0.;
    }

    angle_last = y;

    return y;
}

// ----------------------------------- State Estimator


// p: roll q: pitch r: yaw
// ax: roll ay: pich;

void State_Estimator(double p,double q, double r, double ax, double ay, double orientation[3])
{
    double eye[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}},alpha = 0.9;
    double dA[4][4] = {{0.,-p*0.5*0.005,-q*0.5*0.005,-r*0.5*0.005},{p*0.5*0.005,0.,r*0.5*0.005,-q*0.5*0.005},{q*0.5*0.005,-r*0.5*0.005,0.,p*0.5*0.005},{r*0.5*0.005,q*0.5*0.005,-p*0.5*0.005,0.}};
    double A[4][4] = {{0.,},},z[4] = {0.,};
    double ang[3] = {0.,},state_global_pelvis[6] = {0.,},state_local_pelvis[6] = {0.,0.,0.,0.,0.,0.};
    double GT[4][4] = {{0.0,},}, GP[4][4] = {{0.0,},},RT[4][4] = {{0.0,},},inv_RT[4][4] = {{0.0,},},LT[4][4] = {{0.0,},},inv_LT[4][4] = {{0.0,},};
    double RF_encoder[6]={0.,},LF_encoder[6] = {0.,},RF_pos[6][3]={{0.,},},LF_pos[6][3]={{0.,},};


    //Rotate ACC about World Frame
    double Acc[3] = {ax,ay,0.},Euler[3] = {sharedData->IMU[0].AccX*D2R,sharedData->IMU[0].AccY*D2R,0.,},World_Acc[3] = {0.,};

//    printf("*** raw ori : %f    %f  \n",ax,ay);

//    ACCtoWorldFrame(Euler,Acc,World_Acc);

//    _temp_debug_data[39] = World_Acc[0];
//    _temp_debug_data[40] = World_Acc[1];
//    _temp_debug_data[41] = World_Acc[2];

//    // kalman filter for orientation estimation
    mat4by4plus4by4(eye,dA,A);

//    EulerAccel(ax,ay,ang);

    ang[0] = ax*D2R;
    ang[1] = ay*D2R;
    ang[2] = 0.;

    EulerToQt(ang,z);

    Kalman(A,z,orientation);

//    printf("*** orientation : %f    %f    %f \n",orientation[0]*R2D,orientation[1]*R2D,orientation[2]*R2D);




    //high pass filter
    double a = 0.95;
    HPF_Estimated_Orientation[0] = a*(Old_HPF_Estimated_Orientation[0] + orientation[0] - Old_Estimated_Orientation[0]);
    HPF_Estimated_Orientation[1] = a*(Old_HPF_Estimated_Orientation[1] + orientation[1] - Old_Estimated_Orientation[1]);
    HPF_Estimated_Orientation[2] = a*(Old_HPF_Estimated_Orientation[2] + orientation[2] - Old_Estimated_Orientation[2]);


    Old_Estimated_Orientation[0] = orientation[0];
    Old_Estimated_Orientation[1] = orientation[1];
    Old_Estimated_Orientation[2] = orientation[2];


    Old_HPF_Estimated_Orientation[0] = HPF_Estimated_Orientation[0];
    Old_HPF_Estimated_Orientation[1] = HPF_Estimated_Orientation[1];//orientation[1];
    Old_HPF_Estimated_Orientation[2] = HPF_Estimated_Orientation[2];//orientation[2];


    double comp_a = 0.6;
    Comp_Orientation[0] = ang[0]*(1. - comp_a) + comp_a*p*R2D;//sharedData->FOG.Roll*R2D;
    Comp_Orientation[1] = ang[1]*(1. - comp_a) + comp_a*q*R2D;//sharedData->FOG.Pitch*R2D;
    Comp_Orientation[2] = ang[2]*(1. - comp_a) + comp_a*r*R2D;//sharedData->FOG.Yaw*R2D;

    state_global_pelvis[0] = WBIK_Q[0];
    state_global_pelvis[1] = WBIK_Q[1];
    state_global_pelvis[2] = WBIK_Q[2];


    state_global_pelvis[3] = 0.;//orientation[0];
    state_global_pelvis[4] = 0.;//orientation[1];
    state_global_pelvis[5] = 0.;//orientation[2];

//    RF_encoder[0] = currentPosition(RHY)*D2R;   RF_encoder[1] = currentPosition(RHR)*D2R;   RF_encoder[2] = currentPosition(RHP)*D2R;   RF_encoder[3] = currentPosition(RKN)*D2R;   RF_encoder[4] = currentPosition(RAP)*D2R;   RF_encoder[5] = currentPosition(RAR)*D2R;
//    LF_encoder[0] = currentPosition(LHY)*D2R;   LF_encoder[1] = currentPosition(LHR)*D2R;   LF_encoder[2] = currentPosition(LHP)*D2R;   LF_encoder[3] = currentPosition(LKN)*D2R;   LF_encoder[4] = currentPosition(LAP)*D2R;   LF_encoder[5] = currentPosition(LAR)*D2R;

//    RF_encoder[0] = WBIK_Q[idRHY];
//    RF_encoder[1] = WBIK_Q[idRHR];
//    RF_encoder[2] = WBIK_Q[idRHP];
//    RF_encoder[3] = WBIK_Q[idRKN];
//    RF_encoder[4] = WBIK_Q[idRAP];
//    RF_encoder[5] = WBIK_Q[idRAR];

//    LF_encoder[0] = WBIK_Q[idLHY];
//    LF_encoder[1] = WBIK_Q[idLHR];
//    LF_encoder[2] = WBIK_Q[idLHP];
//    LF_encoder[3] = WBIK_Q[idLKN];
//    LF_encoder[4] = WBIK_Q[idLAP];
//    LF_encoder[5] = WBIK_Q[idLAR];


//    if(RDF >= LDF)
//    {
//        FK_RL(RF_encoder,state_global_pelvis,RF_pos,GT);


////        printf("GT  %f   %f   %f \n",GT[0][3],GT[1][3],GT[2][3]);

//        GT[0][3] = fsm->RightInfos[0][0];
//        GT[1][3] = fsm->RightInfos[0][1];
//        GT[2][3] = fsm->RightInfos[0][2];

//        FK_RL(RF_encoder,state_local_pelvis,RF_pos,RT);

//        InvMatrix(4,(double*)RT,(double*)inv_RT);

//        mat4by4x4by4(GT,inv_RT,GP);

//    }else
//    {
//        FK_LL(LF_encoder,state_global_pelvis,LF_pos,GT);

////                    printf("LT  %f   %f   %f \n",LT[0][3],LT[1][3],LT[2][3]);


//        GT[0][3] = fsm->LeftInfos[0][0];
//        GT[1][3] = fsm->LeftInfos[0][1];
//        GT[2][3] = fsm->LeftInfos[0][2];

//        FK_LL(LF_encoder,state_local_pelvis,LF_pos,LT);

//        InvMatrix(4,(double*)LT,(double*)inv_LT);

//        mat4by4x4by4(GT,inv_LT,GP);

//    }
////                    printf("GP  %f   %f   %f \n",GP[0][3],GP[1][3],GP[2][3]);

////    _temp_debug_data[30] = GP[0][3];
////    _temp_debug_data[31] = GP[1][3];
////    _temp_debug_data[32] = GP[2][3];



//    // LPF

//    static double lpf_pel[3],old_lpf_pel[3];


//    if(pv_Index >=2)
//    {
//        lpf_pel[0] = old_lpf_pel[0]*alpha + GP[0][3]*(1.0-alpha);
//        lpf_pel[1] = old_lpf_pel[1]*alpha + GP[1][3]*(1.0-alpha);
//        lpf_pel[2] = old_lpf_pel[2]*alpha + GP[2][3]*(1.0-alpha);

//    }else
//    {
//        lpf_pel[0] = GP[0][3];
//        lpf_pel[1] = GP[1][3];
//        lpf_pel[2] = GP[2][3];

//    }

//    old_lpf_pel[0] = lpf_pel[0];
//    old_lpf_pel[1] = lpf_pel[1];
//    old_lpf_pel[2] = lpf_pel[2];

//    GP[0][3] = lpf_pel[0];
//    GP[1][3] = lpf_pel[1];
//    GP[2][3] = lpf_pel[2];







//        Calc_CoM(GP,output,output2);


//        _temp_debug_data[33] = output[0];
//        _temp_debug_data[34] = output[1];
//        _temp_debug_data[35] = output[2];

//        _temp_debug_data[36] = output2[0];
//        _temp_debug_data[37] = output2[1];
//        _temp_debug_data[38] = output2[2];

//        printf("Calc CoM : %f   %f  %f \n",output[0],output[1],output[2]);




}

void Kalman(double A[4][4],double z[4],double out[3])
{
    double K[4][4]={{0.0,}},AT[4][4]={{0.0,}},HT[4][4]={{0.0,}},Pp[4][4]={{0.0,}};
    double temp_mat[4][4]={{0.0,}},temp_mat2[4][4]={{0.0,}},err[4]={0.,},Kerr[4]={0.,},H[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
    double Qg = 1,Rg = 0.1,Q[4][4] = {{Qg,0,0,0},{0,Qg,0,0},{0,0,Qg,0},{0,0,0,Qg}},R[4][4] = {{Rg,0,0,0},{0,Rg,0,0},{0,0,Rg,0},{0,0,0,Rg}};
    static double xp[4] = {0.,0.,0.,0.},x[4] = {0.,0.,0.,0.},P[4][4]={{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};

    for(int kal = 0;kal<4;kal++)
    {

        for(int kal2 = 0;kal2<4;kal2++)
        {
            AT[kal2][kal]= A[kal][kal2];
            HT[kal2][kal]= H[kal][kal2];
        }
    }


    // xp = Ax;
    for(int kal = 0;kal<4;kal++)
    {
        xp[kal] = A[kal][0]*x[0] + A[kal][1]*x[1] + A[kal][2]*x[2] + A[kal][3]*x[3];
    }

    // Pp = APA'+Q;

    //APA'
    mat4by4x4by4(A,P,temp_mat);
    mat4by4x4by4(temp_mat,AT,temp_mat2);

    //Pp = APA'+Q
    mat4by4plus4by4(temp_mat2,Q,Pp);

    // K = Pp*H'*inv(H*Pp*H' + R);
    //inv(H*Pp*H' + R)
    mat4by4x4by4(H,Pp,temp_mat);
    mat4by4x4by4(temp_mat,HT,temp_mat2);
    mat4by4plus4by4(temp_mat2,R,temp_mat);

    InvMatrix(4,(double*)temp_mat,(double*)temp_mat2);

    //Pp*H
    mat4by4x4by4(Pp,HT,temp_mat);

    //K = Pp*H'*inv(H*Pp*H' + R);
    mat4by4x4by4(temp_mat,temp_mat2,K);



    // z-Hxp
    for(int kal = 0;kal<4;kal++)
    {
        err[kal] = z[kal] - (H[kal][0]*xp[0] + H[kal][1]*xp[1] + H[kal][2]*xp[2] + H[kal][3]*xp[3]);
    }



    // xp_new = xp + K*(z-Hxp)
    for(int kal = 0;kal<4;kal++)
    {
        Kerr[kal] = K[kal][0]*err[0] + K[kal][1]*err[1] + K[kal][2]*err[2] + K[kal][3]*err[3];
        x[kal] = xp[kal] + Kerr[kal];

    }

    // P = Pp-K*H*Pp
    mat4by4x4by4(K,H,temp_mat);
    mat4by4x4by4(temp_mat,Pp,temp_mat2);

    mat4by4minus4by4(Pp,temp_mat2,P);


    // Phi theta psi

    out[0] = atan2(2.0*(x[2]*x[3] + x[0]*x[1]),1.0-2.0*(x[1]*x[1] + x[2]*x[2]));
    out[1] = -asin(2.0*(x[1]*x[3] - x[0]*x[2]));
    out[2] = atan2(2.0*(x[1]*x[2] + x[0]*x[3]),1.0-2.*(x[2]*x[2] + x[3]*x[3]));



}



void EulerAccel(double ax,double ay, double out[3])
{
    double g = 9.81,temp=0.;

    out[1] = temp = asin(ax/g);

    out[0] = asin(-ay/(g*cos(temp)));

    out[2] = 0.;


}

void EulerToQt(double ang[3],double z[4])
{
    double sinphi = sin(ang[0]/2.),   cosphi = cos(ang[0]/2.);
    double sintheta = sin(ang[1]/2.), costheta = cos(ang[1]/2.);
    double sinpsi = sin(ang[2]/2.),   cospsi = cos(ang[2]/2.);

    z[0] = cosphi*costheta*cospsi + sinphi*sintheta*sinpsi;
    z[1] = sinphi*costheta*cospsi - cosphi*sintheta*sinpsi;
    z[2] = cosphi*sintheta*cospsi + sinphi*costheta*sinpsi;
    z[3] = cosphi*costheta*sinpsi - sinphi*sintheta*cospsi;
}



//void QtToRotationMat(double q[4],double R[3][3])
//{
//    double a = {{(2.0*q[0]*q[0]-1.0), 0., 0.},{0.,(2.0*q[0]*q[0]-1.0), 0.},{0.,0.,(2.0*q[0]*q[0]-1.0)}};
//    double b = {{0.,2.0*q[0]*q[3],-2.0*q[0]*q[2]},{-2.0*q[0]*q[3],0.,2.0*q[0]*q[1]},{2.0*q[0]*q[2],-2.0*q[0]*q[1],0.}};



//}

void ACCtoWorldFrame(double euler[3],double a[3], double new_a[3])
{
    // Rx*Ry ... Roll*Pitch Euler
//    double R[3][3] = {{cos(euler[1]),0.,sin(euler[1])},
//                      {sin(euler[0])*sin(euler[1]),cos(euler[0]),-sin(euler[0])*cos(euler[1])},
//                      {-cos(euler[0])*sin(euler[1]),sin(euler[0]),cos(euler[0])*cos(euler[1])}};

    double R[3][3] = {{cos(euler[1]),sin(euler[1])*sin(euler[0]),sin(euler[1])*cos(euler[0])},
                      {0.,cos(euler[0]),-sin(euler[0])},
                      {-sin(euler[1]),cos(euler[1])*sin(euler[0]),cos(euler[0])*cos(euler[1])}};


    mat3by3x3by1(R,a,new_a);

    new_a[2] = new_a[2] + 9.81;

}



void mat4by4minus4by4(double a[4][4],double b[4][4],double out[4][4])
{
    for(int i=0;i<4;i++)
    {
        for(int j = 0;j<4;j++)
        {
            out[i][j] = a[i][j] - b[i][j] ;

        }

    }

}

void mat4by4plus4by4(double a[4][4],double b[4][4],double out[4][4])
{
    for(int i=0;i<4;i++)
    {
        for(int j = 0;j<4;j++)
        {
            out[i][j] = a[i][j] + b[i][j] ;

        }

    }

}
void mat4by4x4by4(double a[4][4],double b[4][4],double out[4][4])
{

    double sum = 0.0f;

    for(int i=0;i<4;i++)
    {
        for(int k = 0;k<4;k++)
        {
            for(int j=0;j<4;j++)
            {
                sum = sum + a[i][j]*b[j][k];
//                printf("%f \n",a_hat[i][j]*a_hat[j][k]);
            }

            out[i][k] = sum ;
            sum = 0.0f;
        }

    }

}
void cp_ref_generator(double h,int N)
{
    double zmp[2][600];
    double cp_ref[2][300],next_xk[2][3]={{0.,},},next_sum_error[2]={0.,};
    static double old_xk[2][3]={{0.,},},old_sum_error[2] ={0.,};

    for(int k=0;k<N;k++){
            zmp[0][k] = window[k].zmp.x;
    }

    for(int k=0;k<N;k++){
            zmp[1][k] = window[k].zmp.y;
    }
//void PreviewControl(double h,int N,double old_xk[2][3],double old_sum_error[2],double ZMP_REF[2][600],double **cp_ref,double *next_sum_error,double **next_xk)
    PreviewControl(h,N,old_xk,old_sum_error,zmp,cp_ref,next_sum_error,next_xk);

    for(int k = 0;k<2;k++){
        for(int l=0;l<3;l++){
            old_xk[k][l] = next_xk[k][l];
        }
        old_sum_error[k] = next_sum_error[k];
    }


}

void PreviewControl(double h,int N,double old_xk[2][3],double old_sum_error[2],double ZMP_REF[2][600],double cp_ref[2][300],double next_sum_error[2],double next_xk[2][3])
{
    double pv_state[2][3] = {{0.0f}}, pv_state_old[2][3] = {{0.0f}};
    double pv_ZMP[2] = {0.f,};
    double pv_Err[2] = {0.f,};
    double pv_U[2] = {0.f,};
    double temp_sum[2]={0.0,0.0,};
    double w = sqrt(9.81/h);

    pv_state_old[0][0] = old_xk[0][0];
    pv_state_old[0][1] = old_xk[0][1];
    pv_state_old[0][2] = old_xk[0][2];

    pv_state_old[1][0] = old_xk[1][0];
    pv_state_old[1][1] = old_xk[1][1];
    pv_state_old[1][2] = old_xk[1][2];

    pv_Err[0] = old_sum_error[0];
    pv_Err[1] = old_sum_error[1];

    for(int k = 0;k<N;k++)
    {
        for(int i=0;i<=1;i++)
        {

            cp_ref[i][k] = pv_state_old[i][0] + pv_state_old[i][1]/w;

            pv_ZMP[i] = pv_C[0][0]*pv_state_old[i][0] + pv_C[0][1]*pv_state_old[i][1]+ pv_C[0][2]*pv_state_old[i][2];

            pv_Err[i] = pv_Err[i] + pv_ZMP[i] - ZMP_REF[i][k];


            temp_sum[i] = 0.0f;

            for(int pv_time_Index=0;pv_time_Index<= N ;pv_time_Index++)
            {
                temp_sum[i] = temp_sum[i] + pv_Gd[pv_time_Index+1]*(ZMP_REF[i][pv_time_Index + k]);
            }

            pv_U[i] = - pv_Gi[1]*pv_Err[i] - (pv_Gx[1]*pv_state_old[i][0] + pv_Gx[2]*pv_state_old[i][1] + pv_Gx[3]*pv_state_old[i][2]) - temp_sum[i];

            pv_state[i][0] = (pv_A[0][0]*pv_state_old[i][0] + pv_A[0][1]*pv_state_old[i][1] + pv_A[0][2]*pv_state_old[i][2]) + (pv_B[0][0])*pv_U[i];
            pv_state[i][1] = (pv_A[1][0]*pv_state_old[i][0] + pv_A[1][1]*pv_state_old[i][1] + pv_A[1][2]*pv_state_old[i][2]) + (pv_B[1][0])*pv_U[i];
            pv_state[i][2] = (pv_A[2][0]*pv_state_old[i][0] + pv_A[2][1]*pv_state_old[i][1] + pv_A[2][2]*pv_state_old[i][2]) + (pv_B[2][0])*pv_U[i];

            pv_state_old[i][0] = pv_state[i][0];
            pv_state_old[i][1] = pv_state[i][1];
            pv_state_old[i][2] = pv_state[i][2];

            if(k == 0)
            {
                next_xk[i][0] = pv_state[i][0];
                next_xk[i][1] = pv_state[i][1];
                next_xk[i][2] = pv_state[i][2];

                next_sum_error[i] = pv_Err[i];
            }

        }
    }

}

void Prediction(double h,int n,int t1,int t2,int t3)
{
    int cnt = 0;
    double An = 1,A=0,B=0,New_AB=0.,New_CAB=0.,Ps[15]={0.,},Pu[15][15]={0.,};
    double w = sqrt(9.81/h);
    double C=1;
    double dT[15] = {0.,};

    for(int i =0;i<n;i++)
    {
        if(i == 0)
        {
            dT[i] = t1;
        }else if(i ==1)
        {
            dT[i] = t2;
        }else
        {
            dT[i] = t3;
        }
    }

    for(int i = 0 ;i<n;i++)
    {
        A = exp(w*dT[i]);
        An = A*An;

        Ps[i] = An;
    }

    for(int i = 0 ;i<n;i++)
    {
        for(int j = i ;j<n;j++)
        {
            if(j == i)
            {
                A = 1.;
                cnt = 0;
                An = A;
            }else
            {
                A = exp(w*dT[j]);
            }

            B = 1.0 -exp(w*dT[i]);
            cnt = cnt+1;

            if(cnt == 1)
            {
                New_AB = B;
                New_CAB = C*New_AB;
            }else
            {
                for(int k=0;k<cnt-1;k++)
                {
                    if(k == 0)
                    {
                        An = 1;
                    }
                    An = A*An;
                }
                New_AB = An*B;
                New_CAB = C*New_AB;
            }

            Pu[j][i] = New_CAB;


        }
    }

}



void GetGain(double H)
{
    int nCount=0;
    double temp_Gd_gain,temp_Gx_gain,temp_Gi_gain;

    pv_A[0][0] = 1.0f;
    pv_A[0][1] = 0.005f;
    pv_A[0][2] = 0.005f*0.005f;

    pv_A[1][0] = 0.0f;
    pv_A[1][1] = 1.0f;
    pv_A[1][2] = 0.005f;

    pv_A[2][0] = 0.0f;
    pv_A[2][1] = 0.0f;
    pv_A[2][2] = 1.0f;

    pv_B[0][0] = 0.005f*0.005f*0.005f/6.0f;
    pv_B[1][0] = 0.005f*0.005f/2.0f;
    pv_B[2][0] = 0.005f;


    pv_C[0][0] = 1.0f;
    pv_C[0][1] = 0.0f;
    pv_C[0][2] = -H/9.81f;

    pv_Index= 0;

    if(H >= 0.49 && H < 0.51)
        {   //printf("H =0.50 \n");
            fp2 = fopen("../share/Gain/Gd50.txt","r");
            if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
            while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
            fclose(fp2);
            fp3 = fopen("../share/Gain/Gx50.txt","r");nCount = 0;
            if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
            while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
            fclose(fp3);
            fp4 = fopen("../share/Gain/Gi50.txt","r");nCount = 0;
            if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
            while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
            fclose(fp4);
        }
    else if(H >= 0.51 && H < 0.53)
    {   //printf("H =0.52 \n");
        fp2 = fopen("../share/Gain/Gd52.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx52.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi52.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.53 && H < 0.55)
    {   //printf("H =0.54 \n");
        fp2 = fopen("v/Gd54.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx54.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi54.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.55 && H < 0.56)
    {   //printf("H =0.55 \n");
        fp2 = fopen("../share/Gain/Gd55.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx55.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi55.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.56 && H < 0.57)
    {   //printf("H =0.56 \n");
        fp2 = fopen("../share/Gain/Gd56.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx56.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi56.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.57 && H < 0.58)
    {   //printf("H =0.57 \n");
        fp2 = fopen("../share/Gain/Gd57.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx57.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi57.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.58 && H < 0.59)
    {   //printf("H =0.58 \n");
        fp2 = fopen("../share/Gain/Gd58.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx58.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi58.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.59 && H < 0.60)
    {   //printf("H =0.59 \n");
        fp2 = fopen("../share/Gain/Gd59.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx59.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi59.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.60 && H < 0.61)
    {   //printf("H =0.60 \n");
        fp2 = fopen("../share/Gain/Gd60.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx60.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi60.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.61 && H < 0.62)
    {   //printf("H =0.61 \n");
        fp2 = fopen("../share/Gain/Gd61.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx61.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi61.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.62 && H < 0.63)
    {   //printf("H =0.62 \n");
        fp2 = fopen("../share/Gain/Gd62.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx62.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi62.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.63 && H < 0.65)
        {   //printf("H =0.63 \n");
            fp2 = fopen("../share/Gain/Gd63.txt","r");
            if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
            while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
            fclose(fp2);
            fp3 = fopen("../share/Gain/Gx63.txt","r");nCount = 0;
            if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
            while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
            fclose(fp3);
            fp4 = fopen("../share/Gain/Gi63.txt","r");nCount = 0;
            if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
            while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
            fclose(fp4);
        }
    else if(H >= 0.65 && H < 0.66)
    {   //printf("H =0.65 \n");
        fp2 = fopen("../share/Gain/Gd65.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx65.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi65.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.66 && H < 0.67)
    {   //printf("H =0.66 \n");
        fp2 = fopen("../share/Gain/Gd66.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx66.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi66.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.67 && H < 0.68)
    {   //printf("H =0.67 \n");
        fp2 = fopen("../share/Gain/Gd67.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx67.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi67.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.68 && H < 0.69)
    {//printf("H =0.68 \n");
        fp2 = fopen("../share/Gain/Gd68.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx68.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi68.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.69 && H < 0.70)
    {//printf("H =0.69 \n");
        fp2 = fopen("../share/Gain/Gd69.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx69.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi69.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.70 && H < 0.71)
    {//printf("H =0.70 \n");
        fp2 = fopen("../share/Gain/Gd70.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx70.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi70.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.71 && H < 0.72)
    {//printf("H =0.71 \n");
        fp2 = fopen("../share/Gain/Gd71.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx71.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi71.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.72 && H < 0.73)
    {//printf("H =0.72 \n");
        fp2 = fopen("../share/Gain/Gd72.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx72.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi72.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.73 && H < 0.74)
    {//printf("H =0.73 \n");
        fp2 = fopen("../share/Gain/Gd73.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx73.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi73.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.74 && H < 0.75)
    {//printf("H =0.74 \n");
        fp2 = fopen("../share/Gain/Gd74.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx74.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi74.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.75 && H < 0.76)
    {//printf("H =0.75 \n");
        fp2 = fopen("../share/Gain/Gd75.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx75.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi75.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.76 && H < 0.77)
    {//printf("H =0.76 \n");
        fp2 = fopen("../share/Gain/Gd76.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx76.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi76.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.77 && H < 0.78)
    {//printf("H =0.77 \n");
        fp2 = fopen("../share/Gain/Gd77.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx77.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi77.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.78 && H < 0.79)
    {//printf("H =0.78 \n");
        fp2 = fopen("../share/Gain/Gd78.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx78.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi78.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.79 && H < 0.80)
    {//printf("H =0.79 \n");
        fp2 = fopen("../share/Gain/Gd79.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx79.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi79.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.80 && H < 0.81)
    {//printf("H =0.80 \n");
        fp2 = fopen("../share/Gain/Gd80.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx80.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi80.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.81 && H < 0.82)
    {//printf("H =0.81 \n");
        fp2 = fopen("../share/Gain/Gd81.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx81.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi81.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.82 && H < 0.83)
    {//printf("H =0.82 \n");
        fp2 = fopen("../share/Gain/Gd82.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx82.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi82.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.83 && H < 0.84)
    {//printf("H =0.83 \n");
        fp2 = fopen("../share/Gain/Gd83.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx83.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi83.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    else if(H >= 0.84 && H < 0.85)
    {//printf("H =0.84 \n");
        fp2 = fopen("../share/Gain/Gd84.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx84.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi84.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }
    //else if(H >= 0.85 && H < 0.86)
    else if(H >= 0.85)
    {//printf("H =0.85 \n");
        fp2 = fopen("../share/Gain/Gd85.txt","r");
        if(fp2 == NULL)printf("CAN NOT OPEN Gd TEXT FILE \n");
        while(fscanf(fp2,"%lf",&temp_Gd_gain)==1){nCount++;pv_Gd[nCount] = temp_Gd_gain;}
        fclose(fp2);
        fp3 = fopen("../share/Gain/Gx85.txt","r");nCount = 0;
        if(fp3 == NULL)printf("CAN NOT OPEN Gx TEXT FILE \n");
        while(fscanf(fp3,"%lf",&temp_Gx_gain)==1){nCount++;pv_Gx[nCount] = temp_Gx_gain;}
        fclose(fp3);
        fp4 = fopen("../share/Gain/Gi85.txt","r");nCount = 0;
        if(fp4 == NULL)printf("CAN NOT OPEN Gi TEXT FILE \n");
        while(fscanf(fp4,"%lf",&temp_Gi_gain)==1){nCount++;pv_Gi[nCount] = temp_Gi_gain;}
        fclose(fp4);
    }

}
