#ifndef COMMANDLIST_H
#define COMMANDLIST_H

enum APPROACHBOXCOMMAND
{
    APPROACHBOX_NO_ACT = 100,
    APPROACHBOX_AL_TEST,
    APPROACHBOX_WALK_START,
    APPROACHBOX_WALK_STOP,
    APPROACHBOX_WALK_READY,
    APPROACHBOX_FORWARD_WALKING,
    APPROACHBOX_RIGHT_WALKING,
    APPROACHBOX_LEFT_WALKING,
    APPROACHBOX_CW_WALKING,
    APPROACHBOX_CCW_WALKING,
    APPROACHBOX_DATA_SAVE,
    APPROACHBOX_COMPLIANCE_START,
    APPROACHBOX_COMPLIANCE_STOP,
    APPROACHBOX_PUSH_DOOR,
    APPROACHBOX_REALWALK,
    SINGLELOG_WALK
};

enum RealWalk
{
    REALWALK_NO_ACT,
    REALWALK_WALK_START,
    REALWALK_SINGLELOG_START,
    REALWALK_SINGLELOG,
    REALWALK_STOP
};

enum WBWALKCOMMAND
{
    WBWALK_NO_ACT = 100,
    WBWALK_GO_CART,
    WBWALK_GRASP_CART,
    WBWALK_WALKING,
    WBWALK_RELEASE_CART,
    WBWALK_SAVE
};

enum LIFTBOXCOMMAND
{
    LIFTBOX_NO_ACT = 100,
    LIFTBOX_SIT_DOWN,
    LIFTBOX_HOLD_BOX,
    LIFTBOX_LIFT_BOX,
    LIFTBOX_STAND_UP
};

enum WALKREADYCOMMAND
{
    WALKREADY_NO_ACT = 100,
    WALKREADY_GO_WALKREADYPOS,
    WALKREADY_GO_HOMEPOS,
    WALKREADY_WHEELCMD,
    WALKREADY_INFINITY
};

enum MISSIONDOORCOMMAND
{
    MISSIONDOOR_NO_ACT = 100,
    MISSIONDOOR_COMPLIANCE_START,
    MISSIONDOOR_COMPLIANCE_STOP,
    MISSIONDOOR_PUSH_DOOR,
    MISSIONDOOR_LEANED_FORWARD,
    MISSIONDOOR_SAVE
};

enum INVERSECHECKCOMMAND
{
    INVERSECHECK_NO_ACT = 130,
    INVERSECHECK_GO_AND_IK,
    INVERSECHECK_FK
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
#endif // COMMANDLIST_H
