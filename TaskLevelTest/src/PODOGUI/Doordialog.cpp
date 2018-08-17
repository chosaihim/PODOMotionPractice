#include "Doordialog.h"
#include "ui_Doordialog.h"
#include "BasicFiles/PODOALDialog.h"
#include "../../../share/Headers/ik_math4.h"
#include "../../../share/Headers/kine_drc_hubo4.h"

DoorDialog::DoorDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DoorDialog)
{
    ui->setupUi(this);
    ALNum_MissionDoor = PODOALDialog::GetALNumFromFileName("MissionDoor");
    ALNum_ApproachBox = PODOALDialog::GetALNumFromFileName("ApproachBox");
    ALNum_WalkReady = PODOALDialog::GetALNumFromFileName("WalkReady");

    ui->GRP_ZMPFOOT->addGraph();
    ui->GRP_ZMPFOOT->graph(0)->setPen(QPen(Qt::red));
    ui->GRP_ZMPFOOT->addGraph();
    ui->GRP_ZMPFOOT->graph(1)->setPen(QPen(Qt::blue));
    ui->GRP_ZMPFOOT->graph(1)->setLineStyle(QCPGraph::lsNone);
    ui->GRP_ZMPFOOT->graph(1)->setScatterStyle(QCPScatterStyle::ssDisc);
    ui->GRP_ZMPFOOT->xAxis->setRange(-0.5,0.5);
    ui->GRP_ZMPFOOT->yAxis->setRange(-0.5,0.5);

    connect(pLAN,SIGNAL(NewPODOData()),this,SLOT(UpdateLineEdit()));
    connect(pLAN, SIGNAL(NewPODOData()),this,SLOT(AddData()));
    connect(&graphTimer, SIGNAL(timeout()), this, SLOT(Plot_GRP_ZMPFOOT()));

    LeanedBACKWARDcnt = LeanedFORWARDcnt = 0;
    ui->MODE_NORMAL->setChecked(true);

    graphTimer.start(50);
}

DoorDialog::~DoorDialog()
{
    delete ui;
}

void DoorDialog::AddData()
{
    get_zmp();
    qv_zmpx.append(zmp[0]);
    qv_zmpy.append(zmp[1]);
}

void DoorDialog::ClearData()
{
    qv_zmpx.clear();
    qv_zmpy.clear();
}

void DoorDialog::Plot_GRP_ZMPFOOT()
{
    if(ui->CB_GRAPHONOFF->isChecked() == true)
    {
        get_zmp();
        qv_zmpx.append(zmp[0]);
        qv_zmpy.append(zmp[1]);
        float RFpos[2] = {0.0,-0.105};
        float LFpos[2]  = {0.0,0.105};
        float FootSize[2]  = {0.16,0.24};

        QCPItemRect *RightFoot = new QCPItemRect(ui->GRP_ZMPFOOT);
        RightFoot->topLeft->setCoords(RFpos[0]-FootSize[1]/2.,RFpos[1]+FootSize[0]/2.);
        RightFoot->bottomRight->setCoords(RFpos[0]+FootSize[1]/2.,RFpos[1]-FootSize[0]/2.);

        QCPItemRect *LeftFoot = new QCPItemRect(ui->GRP_ZMPFOOT);
        LeftFoot->topLeft->setCoords(LFpos[0]-FootSize[1]/2.,LFpos[1]+FootSize[0]/2.);
        LeftFoot->bottomRight->setCoords(LFpos[0]+FootSize[1]/2.,LFpos[1]-FootSize[0]/2.);

        ui->GRP_ZMPFOOT->graph(0)->setData(qv_zmpx,qv_zmpy);
        ui->GRP_ZMPFOOT->graph(1)->data()->clear();
        ui->GRP_ZMPFOOT->graph(1)->addData(zmp[0],zmp[1]);
        ui->GRP_ZMPFOOT->replot();
        ui->GRP_ZMPFOOT->update();
    }
}
void DoorDialog::get_zmp()
{
        // ZMP 읽기 //
        double M_LF[3],M_RF[3],M_LF_Global[3],M_RF_Global[3],F_LF[3],F_RF[3],F_LF_Global[3],F_RF_Global[3];
        double pCenter[3],qCenter[4],qCenter_bar[4];

        PODO_DATA.UserM2G.pRF[0] = 0.0;
        PODO_DATA.UserM2G.pRF[1] = -0.105;
        PODO_DATA.UserM2G.pRF[2] = 0.0;
        PODO_DATA.UserM2G.pLF[1] = 0.105;
        PODO_DATA.UserM2G.qLF[0] = 1;
        PODO_DATA.UserM2G.qRF[0] = 1;

        // Foot Center in Global Coord.
        pCenter[0] = (PODO_DATA.UserM2G.pRF[0] + PODO_DATA.UserM2G.pLF[0])/2.;
        pCenter[1] = (PODO_DATA.UserM2G.pRF[1] + PODO_DATA.UserM2G.pLF[1])/2.;
        pCenter[2] = (PODO_DATA.UserM2G.pRF[2] + PODO_DATA.UserM2G.pLF[2])/2.;

        if(PODO_DATA.CoreData.FT[0].Fz + PODO_DATA.CoreData.FT[1].Fz > 50.)
        {
            M_LF[0] =  PODO_DATA.CoreData.FT[1].Mx;
            M_LF[1] =  PODO_DATA.CoreData.FT[1].My;
            M_LF[2] =  PODO_DATA.CoreData.FT[1].Mz;

            QTtransform(PODO_DATA.UserM2G.qLF,M_LF,M_LF_Global);

            M_RF[0] =  PODO_DATA.CoreData.FT[0].Mx;
            M_RF[1] =  PODO_DATA.CoreData.FT[0].My;
            M_RF[2] =  PODO_DATA.CoreData.FT[0].Mz;

            QTtransform(PODO_DATA.UserM2G.qRF,M_RF,M_RF_Global);

            F_LF[0] = PODO_DATA.CoreData.FT[1].Fx;
            F_LF[1] = PODO_DATA.CoreData.FT[1].Fy;
            F_LF[2] = PODO_DATA.CoreData.FT[1].Fz;

            QTtransform(PODO_DATA.UserM2G.qLF,F_LF,F_LF_Global);

            F_RF[0] = PODO_DATA.CoreData.FT[0].Fx;
            F_RF[1] = PODO_DATA.CoreData.FT[0].Fy;
            F_RF[2] = PODO_DATA.CoreData.FT[0].Fz;

            QTtransform(PODO_DATA.UserM2G.qRF,F_RF,F_RF_Global);

            double temp1[3],temp2[3],temp3[3],temp4[3];

            diff_vv(PODO_DATA.UserM2G.pRF,3,pCenter,temp1);// (despRF - pCenter)
            diff_vv(PODO_DATA.UserM2G.pLF,3,pCenter,temp2);// (despLF - pCenter)

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

//            X_ZMP_Local = 1000.*zmp_local[0];
//            Y_ZMP_Local = 1000.*zmp_local[1];

//            X_ZMP_Global = 1000.*zmp[0];
//            Y_ZMP_Global = 1000.*zmp[1];
        }
}


void DoorDialog::on_BTN_Walk_Ready_clicked()
{
    USER_COMMAND cmd;

    cmd.COMMAND_TARGET = ALNum_WalkReady;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_GO_WALKREADYPOS;

    pLAN->SendCommand(cmd);
}

void DoorDialog::on_BTN_FORWARD_clicked()
{
    USER_COMMAND cmd;

    pLAN->G2MData->StepNum = ui->LE_STEP_NUM->text().toInt();
    pLAN->G2MData->StepLength = ui->LE_STEP_LENGTH->text().toDouble();
    pLAN->G2MData->StepAngle = ui->LE_STEP_ANGLE->text().toDouble();
    pLAN->G2MData->StepOffset = ui->LE_STEP_OFFSET->text().toDouble();


    cmd.COMMAND_TARGET = ALNum_ApproachBox;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_FORWARD_WALKING;

    pLAN->SendCommand(cmd);
}

void DoorDialog::on_BTN_LEFT_clicked()
{
    USER_COMMAND cmd;

        pLAN->G2MData->StepNum = ui->LE_STEP_NUM->text().toInt();
        pLAN->G2MData->StepLength = ui->LE_STEP_LENGTH->text().toDouble();
        pLAN->G2MData->StepAngle = ui->LE_STEP_ANGLE->text().toDouble();
        pLAN->G2MData->StepOffset = ui->LE_STEP_OFFSET->text().toDouble();


        cmd.COMMAND_TARGET = ALNum_ApproachBox;
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;
        cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_LEFT_WALKING;

        pLAN->SendCommand(cmd);
}

void DoorDialog::on_BTN_RIGHT_clicked()
{
    USER_COMMAND cmd;

    pLAN->G2MData->StepNum = ui->LE_STEP_NUM->text().toInt();
    pLAN->G2MData->StepLength = ui->LE_STEP_LENGTH->text().toDouble();
    pLAN->G2MData->StepAngle = ui->LE_STEP_ANGLE->text().toDouble();
    pLAN->G2MData->StepOffset = ui->LE_STEP_OFFSET->text().toDouble();


    cmd.COMMAND_TARGET = ALNum_ApproachBox;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_RIGHT_WALKING;

    pLAN->SendCommand(cmd);
}

void DoorDialog::on_PB_READY_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_WALK_READY;
    cmd.COMMAND_TARGET = ALNum_ApproachBox;
    pLAN->SendCommand(cmd);
}

void DoorDialog::on_PB_START_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_WALK_START;
    cmd.COMMAND_TARGET = ALNum_ApproachBox;
    pLAN->SendCommand(cmd);
}

void DoorDialog::on_PB_STOP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_WALK_STOP;

    if(ui->MODE_NORMAL->isChecked() == true)
    {
        cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
    } else if(ui->MODE_LIFTBOX->isChecked() == true)
    {
        cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
    } else if(ui->MODE_DOOR->isChecked() == true)
    {
        cmd.COMMAND_DATA.USER_PARA_INT[0] = 2;
    }

    cmd.COMMAND_TARGET = ALNum_ApproachBox;
    pLAN->SendCommand(cmd);
}

void DoorDialog::on_BTN_SAVE_START_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNum_ApproachBox;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
    cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_DATA_SAVE;
    pLAN->SendCommand(cmd);

    USER_COMMAND cmd1;
    cmd1.COMMAND_TARGET = ALNum_MissionDoor;
    cmd1.COMMAND_DATA.USER_PARA_INT[0] = 1;
    cmd1.COMMAND_DATA.USER_COMMAND = MISSIONDOOR_SAVE;
    pLAN->SendCommand(cmd1);
}

void DoorDialog::on_BTN_DATA_SAVE_clicked()
{
    USER_COMMAND cmd;

    cmd.COMMAND_TARGET = ALNum_ApproachBox;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_DATA_SAVE;
    pLAN->SendCommand(cmd);

    USER_COMMAND cmd1;
    cmd1.COMMAND_TARGET = ALNum_MissionDoor;
    cmd1.COMMAND_DATA.USER_PARA_INT[0] = 0;
    cmd1.COMMAND_DATA.USER_COMMAND = MISSIONDOOR_SAVE;
    pLAN->SendCommand(cmd1);
}

void DoorDialog::on_PB_COMPLIANCE_START_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNum_MissionDoor;
    cmd.COMMAND_DATA.USER_COMMAND = MISSIONDOOR_COMPLIANCE_START;
    pLAN->SendCommand(cmd);
}

void DoorDialog::on_PB_PUSHDOOR_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNum_MissionDoor;
    cmd.COMMAND_DATA.USER_COMMAND = MISSIONDOOR_PUSH_DOOR;
    pLAN->SendCommand(cmd);
}

void DoorDialog::on_PB_LEANEDFORWARD_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNum_MissionDoor;
    cmd.COMMAND_DATA.USER_PARA_INT[1] = 1;
    cmd.COMMAND_DATA.USER_COMMAND = MISSIONDOOR_LEANED_FORWARD;
    pLAN->SendCommand(cmd);

}

void DoorDialog::on_PB_LEANED_F_pressed()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNum_MissionDoor;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
    cmd.COMMAND_DATA.USER_COMMAND = MISSIONDOOR_LEANED_FORWARD;
    pLAN->SendCommand(cmd);
}

void DoorDialog::on_PB_LEANED_B_pressed()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNum_MissionDoor;
    cmd.COMMAND_DATA.USER_PARA_INT[2] = 1;
    cmd.COMMAND_DATA.USER_COMMAND = MISSIONDOOR_LEANED_FORWARD;
    pLAN->SendCommand(cmd);
}

void DoorDialog::on_PB_LEANED_F_released()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNum_MissionDoor;
    cmd.COMMAND_DATA.USER_PARA_INT[2] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = MISSIONDOOR_LEANED_FORWARD;
    pLAN->SendCommand(cmd);

}

void DoorDialog::on_PB_LEANED_B_released()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNum_MissionDoor;
    cmd.COMMAND_DATA.USER_PARA_INT[2] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = MISSIONDOOR_LEANED_FORWARD;
    pLAN->SendCommand(cmd);
}


void DoorDialog::UpdateLineEdit()
{
    ui->LE_LEANEDF_COMX->setText(QString().sprintf("%f",PODO_DATA.UserM2G.pCOM[0]));
}

void DoorDialog::on_PB_CLEAR_ZMPFOOT_clicked()
{
    qv_zmpx.clear();
    qv_zmpy.clear();
}
