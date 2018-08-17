#include "WBWalkDialog.h"
#include "ui_WBWalkDialog.h"
#include "BasicFiles/PODOALDialog.h"
#include "CommonHeader.h"
#include "../../share/Headers/commandlist.h"

WBWalkDialog::WBWalkDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::WBWalkDialog)
{
    ui->setupUi(this);

    ALNum_WalkReady = PODOALDialog::GetALNumFromFileName("WalkReady");
    ALNum_WBWalk = PODOALDialog::GetALNumFromFileName("WBWalk");
}

WBWalkDialog::~WBWalkDialog()
{
    delete ui;
}

void WBWalkDialog::on_BTN_Walk_Ready_clicked()
{
    USER_COMMAND cmd;

    cmd.COMMAND_TARGET = ALNum_WalkReady;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_GO_WALKREADYPOS;

    pLAN->SendCommand(cmd);
}

void WBWalkDialog::on_BTN_FORWARD_clicked()
{
    USER_COMMAND cmd;

    pLAN->G2MData->StepNum = ui->LE_STEP_NUM->text().toInt();
    pLAN->G2MData->StepLength = ui->LE_STEP_LENGTH->text().toDouble();
    pLAN->G2MData->StepAngle = ui->LE_STEP_ANGLE->text().toDouble();
    pLAN->G2MData->StepOffset = ui->LE_STEP_OFFSET->text().toDouble();


    cmd.COMMAND_TARGET = ALNum_WBWalk;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = WBWALK_WALKING;

    pLAN->SendCommand(cmd);
}

void WBWalkDialog::on_BTN_SAVE_START_clicked()
{
    USER_COMMAND cmd;

    cmd.COMMAND_TARGET = ALNum_WBWalk;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
    cmd.COMMAND_DATA.USER_COMMAND = WBWALK_SAVE;
    pLAN->SendCommand(cmd);
}

void WBWalkDialog::on_BTN_DATA_SAVE_clicked()
{
    USER_COMMAND cmd;

    cmd.COMMAND_TARGET = ALNum_WBWalk;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = WBWALK_SAVE;
    pLAN->SendCommand(cmd);
}

void WBWalkDialog::on_PB_APPROACH_H_clicked()
{
    USER_COMMAND cmd;

    cmd.COMMAND_TARGET = ALNum_WBWalk;
    cmd.COMMAND_DATA.USER_COMMAND = WBWALK_GO_CART;
    pLAN->SendCommand(cmd);
}

void WBWalkDialog::on_PB_GRASP_H_clicked()
{
    USER_COMMAND cmd;

    cmd.COMMAND_TARGET = ALNum_WBWalk;
    cmd.COMMAND_DATA.USER_COMMAND = WBWALK_GRASP_CART;
    pLAN->SendCommand(cmd);
}

void WBWalkDialog::on_PB_QUIT_H_clicked()
{
    USER_COMMAND cmd;

    cmd.COMMAND_TARGET = ALNum_WBWalk;
    cmd.COMMAND_DATA.USER_COMMAND = WBWALK_RELEASE_CART;
    pLAN->SendCommand(cmd);
}
