#ifndef GLOBALVARIABLES_H
#define GLOBALVARIABLES_H

#include "BasicFiles/BasicSetting.h"
#include "BasicFiles/BasicMatrix.h"
#include "../../share/Headers/commandlist.h"
#include "solver/solver.h"
#include "StateMachine.h"
#include "ManualCAN.h"
#include "FKIK.h"
#include "../../../share/Headers/ik_math4.h"
#include "../../../share/Headers/kine_drc_hubo4.h"
#include "../../../share/Headers/kine_drc_hubo2.h"

#define D2R 3.141592/180.0
#define R2D 180.0/3.141592

/********************************** enum ************************************/
enum{
    OFF , ON
};


/**************************** Structs Definitions *********************************/
/* Shared Memory */
    extern pRBCORE_SHM             sharedData;
    extern pUSER_SHM               userData;

/* Basic Control */
    extern JointControlClass       *jCon;


    extern _footprint_info    last_short_foot;

    extern FKIK fkik;
/* BasicTrajectory mode */
    enum{MODE_ZEROtoONE , MODE_ONEtoZERO};


/**************************** Variables Definitions (Basic) ***********************/
/* Basic Trajectory */
    extern KINE_DRC_HUBO4          kine_drc_hubo4;



/**************************** 3. Variables Definitions (Trajectory) ******************/
/* Global Variable For Task Trajecotry (CoM, Foot, Pelvis ori...etc) */
//    double des_pCOM_3x1[3],des_pCOM_3x1_hat[3],des_pCOM_3x1_LPF[3], des_pPCz, des_qPEL_4x1[4], des_pRF_3x1[3], des_pRF_3x1_hat[3], des_qRF_4x1[4], des_qRF_4x1_hat[4], des_pLF_3x1[3],des_pLF_3x1_hat[3], des_qLF_4x1[4], des_qLF_4x1_hat[4];
//    double des_pRH_3x1[3],des_pLH_3x1[3],des_qRH_4x1[4],des_qLH_4x1[4],des_RElb_ang,des_LElb_ang,RH_ref_frame,LH_ref_frame,des_wst_ang;
//    double FK_pCOM_3x1[3],FK_pRFoot_3x1[3],FK_qRFoot_4x1[4],FK_pLFoot_3x1[3],FK_qLFoot_4x1[4];
//    double FK_pRHand_3x1[3],FK_pLHand_3x1[3],FK_qRHand_4x1[4],FK_qLHand_4x1[4];
//    double FK_REB_ang, FK_LEB_ang, FK_WST_ang;


/* Global Variable For Joextern int Trajecotry (Joextern int angle) */
//    extern double WBIK_Q[34] ,Qub[34],WBIK_Q0[34] ;
//    extern double FWRefAngleCurrent[NO_OF_JOINTS] ;








#endif // GLOBALVARIABLES_H
