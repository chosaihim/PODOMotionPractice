#ifndef FKIK_H
#define FKIK_H

#include "JointInformation.h"

struct taskmotion
{
    double pCOM_3x1[3];
    double pPCz;
    double qPEL_4x1[4];
    double pRF_3x1[3];
    double pLF_3x1[3];
    double qRF_4x1[4];
    double qLF_4x1[4];
    double pRH_3x1[3];
    double pLH_3x1[3];
    double qRH_4x1[4];
    double qLH_4x1[4];
    double WST_ang;
    double REB_ang;
    double LEB_ang;

    void initialize()
    {
        for(int i=0;i<3;i++)
        {
            pCOM_3x1[i] = 0;
            pRF_3x1[i] = 0;
            pLF_3x1[i] = 0;
            pRH_3x1[i] = 0;
            pLH_3x1[i] = 0;
            qPEL_4x1[i] = 0;
            qRF_4x1[i] = 0;
            qLF_4x1[i] = 0;
            qRH_4x1[i] = 0;
            qLH_4x1[i] = 0;
        }
        qPEL_4x1[0] = 1;
        qRF_4x1[0] = 1;
        qLF_4x1[0] = 1;
        qRH_4x1[0] = 1;
        qLH_4x1[0] = 1;
        qPEL_4x1[3] = 0;
        qRF_4x1[3] = 0;
        qLF_4x1[3] = 0;
        qRH_4x1[3] = 0;
        qLH_4x1[3] = 0;
        WST_ang = 0;
        REB_ang = 0;
        LEB_ang = 0;
    }
};

class FKIK
{
public:
    FKIK();

    taskmotion desired;
    taskmotion desired_hat;
    taskmotion FK;

    double RH_ref_frame;
    double LH_ref_frame;

    double WBIK_Q[34];
    double Qub[34];
    double WBIK_Q0[34];
    double FWRefAngleCurrent[NO_OF_JOINTS];

};

#endif // FKIK_H
