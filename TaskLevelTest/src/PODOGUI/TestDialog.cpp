#include "TestDialog.h"
#include "ui_TestDialog.h"

#include "CommonHeader.h"
#include "BasicFiles/PODOALDialog.h"

// Command Set =========================================
enum WALKREADYCOMMAND
{
    WALKREADY_NO_ACT = 100,
    WALKREADY_GO_WALKREADYPOS,
    WALKREADY_GO_HOMEPOS,
    WALKREADY_WHEELCMD,
    WALKREADY_INFINITY,
    WALKREADY_HIT_READY,
    WALKREADY_HIT_HIT,
    WALKREADY_HIT_RETURN,
    WALKREADY_HIT_INIT_POS,
};
enum HANDSUPCOMMAND
{
    HANDSUP_NO_ACT = 200,
    HANDSUP_LEFT,
    HANDSUP_WHOLEBODY,
    HANDSUP_HOW,
    HANDSUP_WALKREADY,
};
// =====================================================

TestDialog::TestDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::TestDialog)
{
    ui->setupUi(this);
    AlnumOmniWheel = PODOALDialog::GetALNumFromFileName("OmniWheel");
    AlnumWalkReady = PODOALDialog::GetALNumFromFileName("WalkReady");
    AlnumHandsUp = PODOALDialog::GetALNumFromFileName("HandsUp");

    displayTimer=new QTimer(this);
    connect(displayTimer,SIGNAL(timeout()),this,SLOT(DisplayUpdate));
    displayTimer->start(50);    //set the time interval(50ms)

}

TestDialog::~TestDialog()
{
    delete ui;
}

void TestDialog::on_PB_WalkReady_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = AlnumWalkReady;    //TARGET(for sending data) AL is WalkReady
    cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_GO_WALKREADYPOS;  //COMMAND for what ation you want
    cmd.COMMAND_DATA.USER_PARA_INT[0] =0;   //Choose an option for that COMMAND
    pLAN->SendCommand(cmd);     //send command through pLAN comm.
}

void TestDialog::on_PB_HandsUp_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = AlnumHandsUp;
    cmd.COMMAND_DATA.USER_COMMAND = HANDSUP_LEFT;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_WholeBody_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = AlnumHandsUp;
    cmd.COMMAND_DATA.USER_COMMAND = HANDSUP_WHOLEBODY;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_motion1_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = AlnumHandsUp;
    cmd.COMMAND_DATA.USER_COMMAND = HANDSUP_HOW;
    pLAN->SendCommand(cmd);
}

void TestDialog::on_PB_WalkReady2_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = AlnumHandsUp;
    cmd.COMMAND_DATA.USER_COMMAND = HANDSUP_WALKREADY;
    pLAN->SendCommand(cmd);
}
