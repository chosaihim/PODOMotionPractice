#include "liftboxdialog.h"
#include "ui_liftboxdialog.h"
#include "BasicFiles/PODOALDialog.h"

LiftBoxDialog::LiftBoxDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::LiftBoxDialog)
{
    ui->setupUi(this);
    ALNum_LiftBox = PODOALDialog::GetALNumFromFileName("LiftBox");
    ALNum_ApproachBox = PODOALDialog::GetALNumFromFileName("ApproachBox");
    ALNum_WalkReady = PODOALDialog::GetALNumFromFileName("WalkReady");
    displayTimer = new QTimer(this);
    connect(displayTimer, SIGNAL(timeout()), this, SLOT(DisplayUpdate()));
    displayTimer->start(50);
}

LiftBoxDialog::~LiftBoxDialog()
{
    delete ui;
}

void LiftBoxDialog::on_pushButton_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_AL_TEST;
    cmd.COMMAND_TARGET = ALNum_ApproachBox;
    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::DisplayUpdate()
{
    QString str;

    ui->lineEdit->setText(str.sprintf("%.2f", PODO_DATA.UserM2G.V2G.pos[0]));
    ui->lineEdit_2->setText(str.sprintf("%.2f", PODO_DATA.UserM2G.V2G.pos[1]));
    ui->lineEdit_3->setText(str.sprintf("%.2f", PODO_DATA.UserM2G.V2G.pos[2]));
    ui->lineEdit_4->setText(str.sprintf("%.2f", PODO_DATA.UserM2G.V2G.ori[0]));
    ui->lineEdit_5->setText(str.sprintf("%.2f", PODO_DATA.UserM2G.V2G.ori[1]));
    ui->lineEdit_6->setText(str.sprintf("%.2f", PODO_DATA.UserM2G.V2G.ori[2]));
    ui->lineEdit_7->setText(str.sprintf("%.2f", PODO_DATA.UserM2G.V2G.ori[3]));
}

void LiftBoxDialog::on_PB_READY_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_WALK_READY;
    cmd.COMMAND_TARGET = ALNum_ApproachBox;
    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_PB_START_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_WALK_START;
    cmd.COMMAND_TARGET = ALNum_ApproachBox;
    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_PB_STOP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_WALK_STOP;
    cmd.COMMAND_TARGET = ALNum_ApproachBox;
    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_pushButton_2_clicked()
{
}

void LiftBoxDialog::on_BTN_FORWARD_clicked()
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

void LiftBoxDialog::on_BTN_LEFT_clicked()
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

void LiftBoxDialog::on_BTN_RIGHT_clicked()
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

void LiftBoxDialog::on_BTN_CCW_clicked()
{
    USER_COMMAND cmd;

    pLAN->G2MData->StepNum = ui->LE_STEP_NUM->text().toInt();
    pLAN->G2MData->StepLength = ui->LE_STEP_LENGTH->text().toDouble();
    pLAN->G2MData->StepAngle = ui->LE_STEP_ANGLE->text().toDouble();
    pLAN->G2MData->StepOffset = ui->LE_STEP_OFFSET->text().toDouble();


    cmd.COMMAND_TARGET = ALNum_ApproachBox;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_CCW_WALKING;

    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_BTN_CW_clicked()
{
    USER_COMMAND cmd;

    pLAN->G2MData->StepNum = ui->LE_STEP_NUM->text().toInt();
    pLAN->G2MData->StepLength = ui->LE_STEP_LENGTH->text().toDouble();
    pLAN->G2MData->StepAngle = ui->LE_STEP_ANGLE->text().toDouble();
    pLAN->G2MData->StepOffset = ui->LE_STEP_OFFSET->text().toDouble();


    cmd.COMMAND_TARGET = ALNum_ApproachBox;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_CW_WALKING;

    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_BTN_Walk_Ready_clicked()
{
    USER_COMMAND cmd;

    cmd.COMMAND_TARGET = ALNum_WalkReady;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_GO_WALKREADYPOS;

    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_BTN_SAVE_START_clicked()
{
    USER_COMMAND cmd;

    cmd.COMMAND_TARGET = ALNum_ApproachBox;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
    cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_DATA_SAVE;
    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_BTN_DATA_SAVE_clicked()
{
    USER_COMMAND cmd;

    cmd.COMMAND_TARGET = ALNum_ApproachBox;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_DATA_SAVE;
    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_PB_SIT_DOWN_clicked()
{
    printf("LiftBox_Sit_Down\n");
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNum_LiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_SIT_DOWN;
    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_PB_STAND_UP_clicked()
{
    printf("LiftBox_Stand_Up\n");
    USER_COMMAND cmd;

    cmd.COMMAND_TARGET = ALNum_LiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_STAND_UP;
    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_PB_STAND_UP_2_clicked()
{
    printf("LiftBox_Hold_BOX\n");
    USER_COMMAND cmd;

    cmd.COMMAND_TARGET = ALNum_LiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_HOLD_BOX;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_PB_STAND_UP_3_clicked()
{
    printf("LiftBox_Lift_Box\n");
    USER_COMMAND cmd;

    cmd.COMMAND_TARGET = ALNum_LiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_LIFT_BOX;
    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_PB_BOX__clicked()
{
    printf("LiftBox_Hold_BOX\n");
    USER_COMMAND cmd;

    cmd.COMMAND_TARGET = ALNum_LiftBox;
    cmd.COMMAND_DATA.USER_COMMAND = LIFTBOX_HOLD_BOX;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
    pLAN->SendCommand(cmd);
}

void LiftBoxDialog::on_PB_SINGLELOG_WALK_clicked()
{
    USER_COMMAND cmd;

    cmd.COMMAND_TARGET = ALNum_ApproachBox;
    cmd.COMMAND_DATA.USER_COMMAND = SINGLELOG_WALK;

    pLAN->G2MData->StepNum = ui->LE_STEP_NUM_2->text().toInt();
    pLAN->G2MData->StepLength = ui->LE_STEP_LENGTH_2->text().toDouble();

    printf("StepNum = %d, StepLength = %f\n",pLAN->G2MData->StepNum, pLAN->G2MData->StepLength);
    pLAN->SendCommand(cmd);
}
