#include "WBWalk.h"

WBWalk::WBWalk()
{

}

WBWalk::~WBWalk()
{}

void WBWalk::UpdateAll()
{

    for(int i=0;i<3;i++)
    {
        if(UpdateTrajectory(&pRH[i]))    pRH[i].Clear();
        if(UpdateTrajectory(&pLH[i]))    pLH[i].Clear();
    }

    if(UpdateTrajectory(&qRH))    qRH.Clear();
    if(UpdateTrajectory(&qLH))    qLH.Clear();

    if(UpdateTrajectory(&thWST))    thWST.Clear();
    if(UpdateTrajectory(&thREB))    thREB.Clear();
    if(UpdateTrajectory(&thLEB))    thLEB.Clear();

}

int WBWalk::UpdateTrajectory(TrajectoryPos *_pos)
{
    if(_pos->current == _pos->target)
    {
        return 0;
    }

    double nTimebef = _pos->currenttime/_pos->timesec;
    _pos->currenttime += 0.005;
    double nTime = _pos->currenttime/_pos->timesec;

    if(nTime > 1.0)
    {
        nTime = 1.0;
        _pos->UpdateRef(nTime,nTimebef);
        return 1;
    }
    _pos->UpdateRef(nTime,nTimebef);
    return 0;
}

int WBWalk::UpdateTrajectory(TrajectoryQuat *_quat)
{
    if(_quat->current == _quat->target)
        return 0;

    double nTimebef = _quat->currenttime/_quat->timesec;
    _quat->currenttime += 0.005;
    double nTime = _quat->currenttime/_quat->timesec;

    if(nTime > 1.0)
    {
        nTime = 1.0;
        _quat->UpdateRef(nTime,nTimebef);
        return 1;
    }
    _quat->UpdateRef(nTime,nTimebef);
    return 0;
}


void WBWalk::RefreshCurrent()
{
    for(int i=0;i<3;i++)
    {
        pRH[i].current = fkik.FK.pRH_3x1[i];
        pLH[i].current = fkik.FK.pLH_3x1[i];
    }
    memcpy(qRH.current, fkik.FK.qRH_4x1, sizeof(double)*4);
    memcpy(qLH.current, fkik.FK.qLH_4x1, sizeof(double)*4);
//    qRH.current = quat(fkik.FK.qRH_4x1[0],fkik.FK.qRH_4x1[1],fkik.FK.qRH_4x1[2],fkik.FK.qRH_4x1[3]);
//    qLH.current = quat(fkik.FK.qLH_4x1[0],fkik.FK.qLH_4x1[1],fkik.FK.qLH_4x1[2],fkik.FK.qLH_4x1[3]);
    thWST.current = fkik.FK.WST_ang;
    thREB.current = fkik.FK.REB_ang;
    thLEB.current = fkik.FK.LEB_ang;

    UpdateAll();

}

void WBWalk::AddRHpos(double _x, double _y, double _z, double _sec)
{
    for(int i=0;i<3;i++)
    {
        pRH[i].timesec = _sec;
    }
    pRH[0].target = _x;
    pRH[1].target = _y;
    pRH[2].target = _z;

    RefreshCurrent();

    pRH[0].start = pRH[0].current;
    pRH[1].start = pRH[1].current;
    pRH[2].start = pRH[2].current;

    printf("New target pos RH : %f, %f, %f\n",pRH[0].target, pRH[1].target, pRH[2].target);
}

void WBWalk::AddLHpos(double _x, double _y, double _z, double _sec)
{
    for(int i=0;i<3;i++)
    {
        pLH[i].timesec = _sec;
    }
    pLH[0].target = _x;
    pLH[1].target = _y;
    pLH[2].target = _z;

    RefreshCurrent();

    pLH[0].start = pLH[0].current;
    pLH[1].start = pLH[1].current;
    pLH[2].start = pLH[2].current;

    printf("New target pos LH : %f, %f, %f\n",pLH[0].target, pLH[1].target, pLH[2].target);
}

void WBWalk::AddRHOri(quat _q, double _sec)
{
    qRH.timesec = _sec;
    for(int k=0;k<4;k++)
        qRH.target[k] = _q[k];
    printf("New target ori RH : %f, %f, %f, %f\n",qRH.target[0], qRH.target[1], qRH.target[2], qRH.target[3]);
}

void WBWalk::AddLHOri(quat _q, double _sec)
{
    qLH.timesec = _sec;
    for(int k=0;k<4;k++)
        qLH.target[k] = _q[k];
    printf("New target ori LH : %f, %f, %f, %f\n",qLH.target[0], qLH.target[1], qLH.target[2], qLH.target[3]);
}

void WBWalk::AddWSTang(double _theta, double _sec)
{
    thWST.timesec = _sec;
    thWST.target = _theta;
    printf("New target ang WST : %f\n", thWST.target);
}

void WBWalk::AddREBang(double _theta, double _sec)
{

    thREB.timesec = _sec;
    thREB.target = _theta;
    printf("New target ang REB : %f\n", thREB.target);
}

void WBWalk::AddLEBang(double _theta, double _sec)
{

    thLEB.timesec = _sec;
    thLEB.target = _theta;
    printf("New target ang LEB : %f\n", thLEB.target);
}

void TrajectoryPos::UpdateRef(double _nTime, double _nTimebef)
{
    double ref_cur = (target-start)*0.5*(1.0-cos(PIf*_nTime));
    double ref_bef = (target-start)*0.5*(1.0-cos(PIf*_nTimebef));
    reference = ref_cur - ref_bef;
}

int iQTinv(const double *qt_4x1, double *result_4x1)
{
    result_4x1[0] = qt_4x1[0]/((qt_4x1[0])*(qt_4x1[0])+(qt_4x1[1])*(qt_4x1[1])+(qt_4x1[2])*(qt_4x1[2])+(qt_4x1[3])*(qt_4x1[3]));
    result_4x1[1] = -qt_4x1[1]/((qt_4x1[0])*(qt_4x1[0])+(qt_4x1[1])*(qt_4x1[1])+(qt_4x1[2])*(qt_4x1[2])+(qt_4x1[3])*(qt_4x1[3]));
    result_4x1[2] = -qt_4x1[2]/((qt_4x1[0])*(qt_4x1[0])+(qt_4x1[1])*(qt_4x1[1])+(qt_4x1[2])*(qt_4x1[2])+(qt_4x1[3])*(qt_4x1[3]));
    result_4x1[3] = -qt_4x1[3]/((qt_4x1[0])*(qt_4x1[0])+(qt_4x1[1])*(qt_4x1[1])+(qt_4x1[2])*(qt_4x1[2])+(qt_4x1[3])*(qt_4x1[3]));

    return 0;
}
int iQT2RV(const double *qt_4x1, double *rv)
{
    double EPS (2.e-6);

    double temp;
    rv[0] = acosf(qt_4x1[0])*2.f;

    if(fabs(sinf(rv[0]/2.f)) < EPS)
    {
        rv[1] = qt_4x1[1];
        rv[2] = qt_4x1[2];
        rv[3] = qt_4x1[3];
    }
    else
    {
        rv[1] = qt_4x1[1]/sinf(rv[0]/2.f);
        rv[2] = qt_4x1[2]/sinf(rv[0]/2.f);
        rv[3] = qt_4x1[3]/sinf(rv[0]/2.f);


        temp = sqrt(rv[1]*rv[1]+rv[2]*rv[2]+rv[3]*rv[3]);
        rv[1] /= temp;
        rv[2] /= temp;
        rv[3] /= temp;
    }

    return 0;
}
int iRV2QT(const double *rv, double *qt_4x1)
{
    double temp = sqrtf(rv[1]*rv[1]+rv[2]*rv[2]+rv[3]*rv[3]);

    if(temp > 0.5f)
    {
        qt_4x1[0] = cosf(rv[0]/2.f);
        qt_4x1[1] = rv[1]/temp*sinf(rv[0]/2.f);
        qt_4x1[2] = rv[2]/temp*sinf(rv[0]/2.f);
        qt_4x1[3] = rv[3]/temp*sinf(rv[0]/2.f);
    }
    else
    {
        qt_4x1[0] = 1.f;
        qt_4x1[1] = 0.f;
        qt_4x1[2] = 0.f;
        qt_4x1[3] = 0.f;
    }

    return 0;
}
void TrajectoryQuat::UpdateRef(double _nTime, double _nTimebef)
{
    quat start_quat, goal_quat;
    for(int k=0;k<4;k++)
    {
        start_quat[k]=current[k];
        goal_quat[k]=target[k];
    }
    double start_quat_double[4], start_quat_inv_double[4];
    for(int k=0;k<4;k++)
    {
        start_quat_double[k]=start_quat[k];
    }
    iQTinv(start_quat_double, start_quat_inv_double);
    quat start_quat_inv;
    for(int k=0;k<4;k++)
    {
        start_quat_inv[k]=start_quat_inv_double[k];
    }
    quat delta_quat=start_quat_inv*goal_quat;

    double rv[4];
    double delta_quat_d[4];
    for(int k=0;k<4;k++)
        delta_quat_d[k]=delta_quat[k];
    iQT2RV(delta_quat_d, rv);

    rv[0] = rv[0]*0.5f*(1.0f-cosf(PIf*_nTime));

    iRV2QT(rv, delta_quat_d);

    for(int k=0;k<4;k++)
        delta_quat[k]=delta_quat_d[k];

    quat ref = start_quat*delta_quat;

    for(int k=0;k<4;k++)
        reference[k]=ref[k];

}
