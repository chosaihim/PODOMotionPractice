#include "RealWalkDialog.h"
#include "ui_RealWalkDialog.h"
#include "BasicFiles/PODOALDialog.h"
#include "../../../share/Headers/commandlist.h"
#include "CommonHeader.h"

RealWalkDialog::RealWalkDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::RealWalkDialog)
{
    ui->setupUi(this);
    alNumApproachBox = PODOALDialog::GetALNumFromFileName("ApproachBox");
    alNumWalkReady = PODOALDialog::GetALNumFromFileName("WalkReady");
}

RealWalkDialog::~RealWalkDialog()
{
    delete ui;
}

void RealWalkDialog::on_PB_WalkReady_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = alNumWalkReady;
    cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_GO_WALKREADYPOS;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;
    pLAN->SendCommand(cmd);
}

void RealWalkDialog::on_PB_Walking_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = alNumApproachBox;
    cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_REALWALK;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = REALWALK_WALK_START;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_StepLength->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;
    pLAN->SendCommand(cmd);
}

void RealWalkDialog::on_PB_SingleLog_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = alNumApproachBox;
    cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_REALWALK;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = REALWALK_SINGLELOG_START;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_StepNum->text().toDouble();
    pLAN->SendCommand(cmd);
}

void RealWalkDialog::on_PB_WalkStop_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = alNumApproachBox;
    cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_REALWALK;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = REALWALK_STOP;
    pLAN->SendCommand(cmd);
}

void RealWalkDialog::on_PB_WalkingReady_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = alNumApproachBox;
    cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_REALWALK;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = REALWALK_WALK_START;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;
    pLAN->SendCommand(cmd);
}

void RealWalkDialog::on_PB_SAVE_START_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = alNumApproachBox;
    cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_DATA_SAVE;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
    pLAN->SendCommand(cmd);
}

void RealWalkDialog::on_PB_SAVE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = alNumApproachBox;
    cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_DATA_SAVE;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
    pLAN->SendCommand(cmd);
}
