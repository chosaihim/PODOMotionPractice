#include "inversecheckdialog.h"
#include "ui_inversecheckdialog.h"
#include "BasicFiles/PODOALDialog.h"

InverseCheckDialog::InverseCheckDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::InverseCheckDialog)
{
    ui->setupUi(this);
    alnumWalkReady = PODOALDialog::GetALNumFromFileName("WalkReady");
    alnumInverseCheck = PODOALDialog::GetALNumFromFileName("InverseCheck");

    connect(pLAN,SIGNAL(NewPODOData()),this,SLOT(UpdateValue()));
}

InverseCheckDialog::~InverseCheckDialog()
{
    delete ui;
}

void InverseCheckDialog::on_PB_IKandGO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = alnumInverseCheck;
    cmd.COMMAND_DATA.USER_COMMAND = INVERSECHECK_GO_AND_IK;

    pLAN->G2MData->InputX = ui->LE_InputX->text().toDouble();
    pLAN->G2MData->InputY = ui->LE_InputY->text().toDouble();
    pLAN->G2MData->InputZ = ui->LE_InputZ->text().toDouble();
    pLAN->G2MData->InputRoll = ui->LE_InputRoll->text().toDouble();
    pLAN->G2MData->InputPitch = ui->LE_InputPitch->text().toDouble();
    pLAN->G2MData->InputYaw = ui->LE_InputYaw->text().toDouble();

    if(ui->CB_X->isChecked()==true)
        pLAN->G2MData->OnOff[0] = 1;
    else
        pLAN->G2MData->OnOff[0] = 0;

    if(ui->CB_Y->isChecked()==true)
        pLAN->G2MData->OnOff[1] = 1;
    else
        pLAN->G2MData->OnOff[1] = 0;

    if(ui->CB_Z->isChecked()==true)
        pLAN->G2MData->OnOff[2] = 1;
    else
        pLAN->G2MData->OnOff[2] = 0;

    if(ui->CB_R->isChecked()==true)
        pLAN->G2MData->OnOff[3] = 1;
    else
        pLAN->G2MData->OnOff[3] = 0;

    if(ui->CB_P->isChecked()==true)
        pLAN->G2MData->OnOff[4] = 1;
    else
        pLAN->G2MData->OnOff[4] = 0;

    if(ui->CB_Yaw->isChecked()==true)
        pLAN->G2MData->OnOff[5] = 1;
    else
        pLAN->G2MData->OnOff[5] = 0;


    printf("OnOff = %d %d %d %d %d %d\n",pLAN->G2MData->OnOff[0],pLAN->G2MData->OnOff[1],pLAN->G2MData->OnOff[2],pLAN->G2MData->OnOff[3],pLAN->G2MData->OnOff[4],pLAN->G2MData->OnOff[5]);
    pLAN->SendCommand(cmd);
}

void InverseCheckDialog::on_PB_FK_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = alnumInverseCheck;
    cmd.COMMAND_DATA.USER_COMMAND = INVERSECHECK_FK;


    pLAN->SendCommand(cmd);


}

void InverseCheckDialog::UpdateValue()
{
    ui->LE_SY->setText(QString().sprintf("%.3f", PODO_DATA.CoreData.WBIK_Q[idRSY]));
    ui->LE_SR->setText(QString().sprintf("%.3f", PODO_DATA.CoreData.WBIK_Q[idRSR]));
    ui->LE_SP->setText(QString().sprintf("%.3f", PODO_DATA.CoreData.WBIK_Q[idRSP]));
    ui->LE_EB->setText(QString().sprintf("%.3f", PODO_DATA.CoreData.WBIK_Q[idREB]));
    ui->LE_WY->setText(QString().sprintf("%.3f", PODO_DATA.CoreData.WBIK_Q[idRWY]));
    ui->LE_WY2->setText(QString().sprintf("%.3f", PODO_DATA.CoreData.WBIK_Q[idRWY2]));
    ui->LE_WP->setText(QString().sprintf("%.3f", PODO_DATA.CoreData.WBIK_Q[idRWP]));

    ui->LE_OutputX->setText(QString().sprintf("%.3f", PODO_DATA.UserM2G.OutputX));
    ui->LE_OutputY->setText(QString().sprintf("%.3f", PODO_DATA.UserM2G.OutputY));
    ui->LE_OutputZ->setText(QString().sprintf("%.3f", PODO_DATA.UserM2G.OutputZ));
    ui->LE_OutputRoll->setText(QString().sprintf("%.3f", PODO_DATA.UserM2G.OutputRoll));
    ui->LE_OutputPitch->setText(QString().sprintf("%.3f", PODO_DATA.UserM2G.OutputPitch));
    ui->LE_OutputYaw->setText(QString().sprintf("%.3f", PODO_DATA.UserM2G.OutputYaw));
}

void InverseCheckDialog::on_PB_WALKREADY_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = alnumWalkReady;
    cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_GO_WALKREADYPOS;
    pLAN->SendCommand(cmd);
}
