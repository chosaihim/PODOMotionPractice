/*
 * This header file declares the functions FK and IK and the variables used.
 * It can solve the Whole body IK
 * by adding the upper body ik to the lower body ik used by MPCWalking.
 *
 * 18-07-24 YU
 */


#ifndef WBWALK_H
#define WBWALK_H

#include "BasicFiles/BasicMatrix.h"
#include "FKIK.h"
/******************************** 1. Structs **************************************/
class quat;


struct TrajectoryPos
{
    int info;
    double current;
    double target;
    double start;
    double reference;
    double timesec;
    double currenttime;
    void UpdateRef(double _nTime, double _nTimebef);
    inline void Clear()
    {
        timesec = 0.;
        reference = 0.;
        currenttime = 0.;
        target = current;
    }

};

struct TrajectoryQuat
{
    double current[4];
    double target[4];
    double reference[4];
    double timesec;
    double currenttime;
    void UpdateRef(double _nTime, double _nTimebef);
    inline void Clear()
    {
        timesec = 0.;
        currenttime = 0.;
    }
};

class WBWalk
{
public:

    explicit WBWalk();
    ~WBWalk();

    FKIK fkik;
    void UpdateAll();
    int UpdateTrajectory(TrajectoryPos *_pos);
    int UpdateTrajectory(TrajectoryQuat *_quat);
    void RefreshCurrent();

    void AddRHpos(double _x, double _y, double _z, double _sec);
    void AddLHpos(double _x, double _y, double _z, double _sec);
    void AddRHOri(quat _q, double _sec);
    void AddLHOri(quat _q, double _sec);
    void AddWSTang(double _theta, double _sec);
    void AddREBang(double _theta, double _sec);
    void AddLEBang(double _theta, double _sec);

    void UBIK();
public:
    TrajectoryPos pRH[3];
    TrajectoryPos pLH[3];
    TrajectoryQuat qRH;
    TrajectoryQuat qLH;
    TrajectoryPos thWST;
    TrajectoryPos thREB;
    TrajectoryPos thLEB;

};




#endif // WBWALK_H
