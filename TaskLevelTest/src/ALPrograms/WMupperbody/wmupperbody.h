#ifndef WMUPPERBODY_H
#define WMUPPERBODY_H
//-----------------------------------------------------
// Basic Constant
//-----------------------------------------------------
#ifndef PI
#define PI			3.141592653589793
#endif
#ifndef D2R
#define D2R			1.745329251994330e-2
#endif
#ifndef R2D
#define R2D			5.729577951308232e1
#endif

#define RIGHT_HAND                  0
#define LEFT_HAND                   1

#define RIGHT_DOOR_MODE             0
#define LEFT_DOOR_MODE              1

#define RIGHT_FIRST_PLUG            0
#define LEFT_FIRST_PLUG             1

#define PLUG_STAND_MODE             0
#define PLUG_WHEEL_MODE             1
//-----------------------------------------------------
// Basic Constant
//-----------------------------------------------------
enum WMupperbody_ALCOMMAND
{
    WMupperbody_AL_NO_ACT = 100,
    WMupperbody_AL_GENERAL,
    WMupperbody_AL_POS,
    WMupperbody_AL_GOTOPOINT,
    WMupperbody_AL_HANDLE_ROTATE,
    WMupperbody_AL_HAND,
    WMupperbody_AL_CONTROL,
    WMupperbody_AL_WST,
    WMupperbody_AL_GRAB,
    WMupperbody_AL_DOORIN_GRAB,
    WMupperbody_AL_DOORIN_HANDLE_ROTATE,
    WMupperbody_AL_DOORIN_WHEEL_AND_TASK,
    WMupperbody_AL_DOORIN_MOTION,
    WMupperbody_AL_HOSE_GET,
    WMupperbody_AL_HOSE_PLUG,
    WMupperbody_AL_HOSE_APPROACH,
    WMupperbody_AL_HOSE_APPROACH2,
    WMupperbody_AL_MANUAL,
    WMupperbody_AL_MANUAL_ORI,
    WMupperbody_AL_MANUAL_ELB,
    WMupperbody_AL_ESTOP,
    WMupperbody_AL_PREDEFINED,
    WMupperbody_AL_WALKPUSH_STEP,
    WMupperbody_AL_WALKPUSH_EM,
    WMupperbody_AL_WALKPUSH_ETC
};
enum WALKREADYCOMMAND
{
    WALKREADY_NO_ACT = 100,
    WALKREADY_GO_WALKREADYPOS,
    WALKREADY_GO_HOMEPOS,
    WALKREADY_WHEELCMD,
    WALKREADY_INFINITY
};
enum OMNIWHEEL_ALCOMMAND
{
    OMNIWHEEL_AL_NO_ACT = 100,
    OMNIWHEEL_AL_GOTODES,
    OMNIWHEEL_AL_VELMODE,
    OMNIWHEEL_AL_CHANGEPOS,
    OMNIWHEEL_AL_CONTROL,
    OMNIWHEEL_AL_MANUAL,
    OMNIWHEEL_AL_RADIUS
};
enum FREEWALKCOMMAND

{
    FREEWALK_NO_ACT = 100,
    FREEWALK_CUR_DSP,
    FREEWALK_WALK,
    FREEWALK_SAVE,
    FREEWALK_CONTROL_TEST,
    FREEWALK_PRE_WALK,
    FREEWALK_MOTION_CHECK,
    FREEWALK_INITIALIZE,
    FREEWALK_TERRAIN,
    FREEWALK_ADDMASS,
    FREEWALK_WIDE,
    FREEWALK_DSP_HOLD_WALK
};
enum{
    NORMAL_WALKING =0,
    TERRAIN_WALKING,
    TERRAIN_WALKING_ONE_STEP,
    LADDER_WALKING,
    GOAL_WALKING
};
enum Inside_OutsideMode
{
    INSIDE_WALKING = 0,
    OUTSIDE_WALKING
};
enum WalkingModeCommand
{
    FORWARD_WALKING =0,
    BACKWARD_WALKING,
    RIGHTSIDE_WALKING,
    LEFTSIDE_WALKING,
    CWROT_WALKING,
    CCWROT_WALKING,
    GOTOWR_WALKING
};
enum WalkingStopModeCommand
{
    COMPLETE_STOP_WALKING = 0,
    SPREAD_STOP_WALKING
};
enum WalkingGoalGoModeCommand
{
    GOALGO_NORMAL = 0,
    GOALGO_FOR_SIDE,
    GOALGO_SIDE_FOR,
    GOALGO_DIAGONAL,
};
enum TrapMotion_Command
{
    TR_IDLE =0,
    TR_HANDLE_ROTATE,
    TR_DOOR_ROTATE
};

//unsigned char DOOR_MODE = LEFT_DOOR_MODE;
unsigned char DOOR_MODE = RIGHT_DOOR_MODE;
unsigned char PLUG_FIRST = LEFT_FIRST_PLUG;
unsigned char PLUG_SECOND = RIGHT_FIRST_PLUG;
unsigned char PLUG_POSTURE = PLUG_STAND_MODE;

//-----------------------------------------------------
// Joint space arm pos functions
//-----------------------------------------------------
void Change_Arm_pos_Task(const double postime, const double dWST);
void Change_Arm_pos_Move(const double postime);
void Change_Arm_pos_Pass(const double postime);
void Change_Arm_pos_Task_with_WST(const double temp_WST, const double tasktime);
void Change_Arm_pos_Task_with_WST_READY(const double temp_WST, const double tasktime);
//-----------------------------------------------------
// Trapizoidal variable variation
//-----------------------------------------------------
int TrapFunction(void);

int _isTrap = true;

double _Trap_togo = 0;
double _Trap_maxVel = 0;
double _Trap_maxAcc = 0;

int _Trap_indicator;
unsigned long _TrapCount=0;
int _is_Trap_First;

double _Trap_T1, _Trap_T2, _Trap_T3;
unsigned long _Trap_T1_count, _Trap_T2_count, _Trap_T3_count;
unsigned long _Trap_gap1, _Trap_gap2, _Trap_gap3;

double _Trap_GOAL_DELTA =0.;
unsigned int _Trap_FINISH_FLAG = false;
char _Trap_Motion_Command;
//-----------------------------------------------------
// WBIK variables/functionsc
//-----------------------------------------------------
void StartWBIKmotion(int _mode);
void StartWBIK_Globalmotion(int _mode);
void PrintWBIKinfo(void);
//-----------------------------------------------------
// ENC based functions
//-----------------------------------------------------
void GetPosFromENC(void);
int FingerPositionInput_RIGHT(const long des_right);
int FingerPositionInput_LEFT(const long des_left);
//-----------------------------------------------------
// Global variables / Flag
//-----------------------------------------------------
int Handle_rotate_done_Flag = false;
double _door_getin_radius = 0.5;
//-----------------------------------------------------
// DoorIn global parameter
//-----------------------------------------------------
double door_open_angle = 10.;
double saved_normal[3];
//-----------------------------------------------------
// Walking Push door parameter
//-----------------------------------------------------
double walk_door_open_angle = 12.;
double PEL2FOOT = 0.03;
//-----------------------------------------------------
// Motion checker 1
//-----------------------------------------------------
int FTavgFlag = false;
unsigned int FTavgCount =0;
double FTavgSUM =0.;
double FTavg = 0.;
double FTavgSUM_2 =0.;
double FTavg_2 = 0.;
double befor_FTavg =0.;
double after_FTavg =0.;
//-----------------------------------------------------
//-----------------------------------------------------
//-----------------------------------------------------
// Emergency backup
//-----------------------------------------------------
// PULL DOOR
double _Task_WST_ANGLE = -20.;
double _EM_AP_POS[3];
double _EM_AP_ORI[4];
double _EM_LH_POS[3];
double _EM_LH_ORI[4];

// HOSE
double _EM_HOSE_RETURN_GRAB_POS[3];
double _EM_WYE_RETURN_POS[3];
double _EM_WYE_RETURN_ORI[4];
int EM_GRAB_INDEX=0;

// PUSH DOOR
double _EM_PUSHDOOR_AP_POS[3];
double _EM_PUSHDOOR_AP_ORI[4];

// WALKING_PUSH_DOOR
double _EM_WALKPUSHDOOR_AP_POS[3];
double _EM_WALKPUSHDOOR_AP_ORI[4];


int sck=0;
//-----------------------------------------------------
// While count checker
//-----------------------------------------------------
int whilecnt = 0;
//-----------------------------------------------------
// Time set variable
//-----------------------------------------------------
double pos_sec = 3.;
double postime = 3000.;
//-----------------------------------------------------
// Save Parameter
//-----------------------------------------------------
unsigned int saveFlag=0;
unsigned int saveIndex=0;
float DataBuf[3][100000];
void SaveFile(void);
unsigned int debugFlag=0;
//-----------------------------------------------------
// WHEEL kinematics
//-----------------------------------------------------
const double WH2PEL_x = -0.1875;//-0.2025;//-0.1908;//new robot
const double WH2PEL_y = 0.;
const double WH2PEL_z = 0.332;//0.322;//0.3305;//new robot

const double PEL2NECK_x = 0;
const double PEL2NECK_y = 0.;
const double PEL2NECK_z = 0.521;//new robot

const double NECK2VIS_x = 0.149;//new vision
const double NECK2VIS_y = 0.0;//new vision
const double NECK2VIS_z = 0.165;//new vision
//-----------------------------------------------------
// Manual CAN
//-----------------------------------------------------
int eq_gain_0 = 65;//100
int eq_gain_1 = 40;
int eq_gain_1000 =0;

//-----------------------------------------------------
// Motion modification
//-----------------------------------------------------
double LEFT_NOBE_ROT_ANGLE = 60.;
double RIGHT_NOBE_ROT_ANGLE = 60.;

double DOOR_NOBE_LENGTH = 0.125;

double LEFT_NOBE_PUSH_X = 0.075;
double LEFT_NOBE_PUSH_Y = 0.035;
double RIGHT_NOBE_PUSH_X = 0.075;
double RIGHT_NOBE_PUSH_Y = 0.049;

double finger_scale = -488.88889;
#endif // WMUPPERBODY_H
