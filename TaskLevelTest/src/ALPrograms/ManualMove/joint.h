#ifndef JOINT_H
#define JOINT_H

#include <iostream>
#include <math.h>

#include "../../../share/Headers/JointInformation.h"
#include "../../../share/Headers/RBSharedMemory.h"
#include "../../../share/Headers/UserSharedMemory.h"
#include "RBLog.h"
#include "taskGeneral.h"

class JointClass;
typedef QVector<JointClass*> JointVector;


enum MoveCommandMode{
	MOVE_RELATIVE = 0,
	MOVE_ABSOLUTE
};



class JointClass
{
private:    
    int             MCId;
    int             MCCh;

	double			RefAngleCurrent;
	double			RefAngleDelta;
	double			RefAngleToGo;
	double			RefAngleInitial;
	unsigned long	GoalTimeCount;
	unsigned long	DelayTimeCount[2];
	unsigned long	CurrentTimeCount;
	unsigned char	MoveFlag;

public:
	JointClass()							{RefAngleCurrent = 0.f; MoveFlag = DISABLE;}
	JointClass(const int id, const int ch)	{MCId = id; MCCh = ch; RefAngleCurrent = 0.f; MoveFlag = DISABLE;}

	int				GetId()									{return MCId;}
	int				GetCh()									{return MCCh;}
	void			SetRefAngleCurrent(const double ref)	{RefAngleCurrent = ref;}
	double			GetRefAngleCurrent()					{return RefAngleCurrent;}

	unsigned char	SetMoveJoint(const double _angle, const double _msTime, const unsigned int _mode);
	unsigned char	MoveJoint();
};

// JointControlClass with JointClass Vector
class JointControlClass
{
private:


    int             PODONum;
    pRBCORE_SHM     Shm;
public:

    JointVector		Joints;
    explicit JointControlClass(pRBCORE_SHM _shm, int _podoNum);

    void			RefreshToCurrentReference();

    int				GetJointNumber(const int _mcNum, const int _mcCh);
    JointClass*		GetJointClass(const int n)							{return Joints[n];}
    double			GetJointRefAngle(const int n)						{return Joints[n]->GetRefAngleCurrent();}
    void			SetJointRefAngle(const int n, const double _ref)	{Joints[n]->SetRefAngleCurrent(_ref);}

    void			SetMotionOwner(const int _jointNum);
    void			SetAllMotionOwner();

    unsigned char	SetMoveJoint(const int _jointNum, const double _angle, const double _msTime, const unsigned int _mode);
    unsigned char	MoveJoint(const int _jointNum);
    void            MoveAllJoint();
    void			JointUpdate();
    int             RefreshToCurrentEncoder();
    int             RefreshToCurrentEncoder_RAP_RF1();
};

#endif // JOINT_H

