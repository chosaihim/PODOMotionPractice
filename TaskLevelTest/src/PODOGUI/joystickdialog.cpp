#include "joystickdialog.h"
#include "ui_joystickdialog.h"

#include <iostream>

#include "JoyStick/joystickclass.h"
#include "JoyStick/joystickvariable.h"
#include "BasicFiles/PODOALDialog.h"


enum OMNIWHEEL_ALCOMMAND
{
    OMNIWHEEL_AL_NO_ACT = 100,
    OMNIWHEEL_AL_GOTODES,
    OMNIWHEEL_AL_VELMODE,
    OMNIWHEEL_AL_CHANGEPOS,
    OMNIWHEEL_AL_CONTROL,
    OMNIWHEEL_AL_MANUAL,
    OMNIWHEEL_AL_RADIUS
};
enum ManualMove_ALCOMMAND
{
    ManualMove_AL_NO_ACT = 100,
    ManualMove_AL_UPPER_TASK_POSE,
    ManualMove_AL_MANUAL_MODE_START,
    ManualMove_AL_MANUAL_ONE_HAND_STADING_START,
    ManualMove_AL_MANUAL_BOTH_HAND_STADING_START,
    ManualMove_AL_MANUAL_FOOT_MODE_START,
    ManualMove_AL_MANUAL_BOTH_FOOT_MODE_START,
    ManualMove_AL_MANUAL_BOTH_HAND_MODE_START,
    ManualMove_AL_MANUAL_MODE_STOP,
    ManualMove_AL_HAND,
    ManualMove_AL_GAIN,
    ManualMove_AL_E_STOP,
    ManualMove_AL_DRIVE_MODE,
    ManualMove_AL_JOYSTICK_MODE
};


char JoyModeFlag=false;
char WheelModeFlag=false;
bool ManualModeFlag = false;
unsigned long WheelModeCount=0;
unsigned long ManualModeCount=0;

JoyStickDialog::JoyStickDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::JoyStickDialog)
{
    ui->setupUi(this);

    AlnumOmniWheel=PODOALDialog::GetALNumFromALName("OmniWheel");
//    AlunumWMupperbody=PODOALDialog::GetALNumFromALName("WMupperbody_AL");
    AlnumManualMove = PODOALDialog::GetALNumFromALName("ManualMove");

    ui->JOY_TABLE_INFO_LEFT->setColumnWidth(0, 60);
    ui->JOY_TABLE_INFO_RIGHT->setColumnWidth(0, 60);

    // Joy Stick variables

    joystick = new RBJoystick();
    joystick->ConnectJoy();

    displayTimer = new QTimer(this);
    connect(displayTimer, SIGNAL(timeout()), this, SLOT(DisplayUpdate()));
    displayTimer->start(50);//50
}

JoyStickDialog::~JoyStickDialog()
{
    delete ui;
}
void JoyStickDialog::DisplayUpdate()
{
    // JoyStick Input
    if(JoyModeFlag==true)
        GetJoystickData();
    else
        InitJoystickData();

    // If Manual Wheel Mode
    if(WheelModeFlag==true){
        if(WheelModeCount<10){
            WheelModeCount++;
        }else{
            USER_COMMAND cmd;
            cmd.COMMAND_DATA.USER_PARA_INT[0]=(int)JOY_LJOG_RL;
            cmd.COMMAND_DATA.USER_PARA_INT[1]=(int)JOY_AROW_RL;
            cmd.COMMAND_DATA.USER_PARA_INT[2]=(int)JOY_RJOG_UD;
            cmd.COMMAND_DATA.USER_COMMAND=OMNIWHEEL_AL_NO_ACT;
            cmd.COMMAND_TARGET = AlnumOmniWheel;
            pLAN->SendCommand(cmd);
        }
        if(WheelModeCount>1000000)
            WheelModeCount=0;
    }else{
        ;
    }

    // If Manual Hand Mode
    if(ManualModeFlag == true){
        if(ManualModeCount<10){
            ManualModeCount++;
        }else{
//        cmd.COMMAND_DATA.USER_PARA_CHAR[0]=(char)JOY_X;
//        cmd.COMMAND_DATA.USER_PARA_CHAR[1]=(char)JOY_Y;
//        cmd.COMMAND_DATA.USER_PARA_CHAR[2]=(char)JOY_A;
//        cmd.COMMAND_DATA.USER_PARA_CHAR[3]=(char)JOY_B;
//        cmd.COMMAND_DATA.USER_PARA_INT[4]=(int)JOY_RT;
//        cmd.COMMAND_DATA.USER_PARA_INT[5]=(int)JOY_RB;
//        cmd.COMMAND_DATA.USER_PARA_INT[6]=(int)JOY_LT;
//        cmd.COMMAND_DATA.USER_PARA_INT[7]=(int)JOY_LB;
//        cmd.COMMAND_DATA.USER_PARA_INT[8]=(int)JOY_AROW_RL;
//        cmd.COMMAND_DATA.USER_PARA_INT[9]=(int)JOY_AROW_UD;
//        cmd.COMMAND_DATA.USER_PARA_INT[0]=(int)JOY_RJOG_RL;
//        cmd.COMMAND_DATA.USER_PARA_INT[1]=(int)JOY_LJOG_RL;
//        cmd.COMMAND_DATA.USER_PARA_INT[2]=(int)JOY_RJOG_UD;
//        cmd.COMMAND_DATA.USER_PARA_INT[3]=(int)JOY_LJOG_UD;
//        cmd.COMMAND_DATA.USER_PARA_INT[10]=(int)JOY_BACK;
//        cmd.COMMAND_DATA.USER_PARA_INT[11]=(int)JOY_START;
//        cmd.COMMAND_DATA.USER_PARA_INT[12]=(int)JOY_RJOG_PUSH;
//        cmd.COMMAND_DATA.USER_PARA_INT[13]=(int)JOY_LJOG_PUSH;
//        cmd.COMMAND_DATA.USER_COMMAND=ManualMove_AL_NO_ACT;
//        cmd.COMMAND_TARGET = AlnumManualMove;
//        pLAN->SendCommand(cmd);
        }
        if(ManualModeCount>1000000)
            ManualModeCount=0;
    }
    else{
        ;
    }
    // Data Show
    QString str;
    QTableWidgetItem *tempItem;
    tempItem = new QTableWidgetItem(); tempItem->setText(str.sprintf("%d", JOY_LT));
    ui->JOY_TABLE_INFO_LEFT->setItem(0,0,tempItem);
    tempItem = new QTableWidgetItem(); tempItem->setText(str.sprintf("%d", JOY_LB));
    ui->JOY_TABLE_INFO_LEFT->setItem(1,0,tempItem);
    tempItem = new QTableWidgetItem(); tempItem->setText(str.sprintf("%d", JOY_LJOG_RL));
    ui->JOY_TABLE_INFO_LEFT->setItem(2,0,tempItem);
    tempItem = new QTableWidgetItem(); tempItem->setText(str.sprintf("%d", JOY_LJOG_UD));
    ui->JOY_TABLE_INFO_LEFT->setItem(3,0,tempItem);
    tempItem = new QTableWidgetItem(); tempItem->setText(str.sprintf("%d", JOY_AROW_RL));
    ui->JOY_TABLE_INFO_LEFT->setItem(4,0,tempItem);
    tempItem = new QTableWidgetItem(); tempItem->setText(str.sprintf("%d", JOY_AROW_UD));
    ui->JOY_TABLE_INFO_LEFT->setItem(5,0,tempItem);
    tempItem = new QTableWidgetItem(); tempItem->setText(str.sprintf("%d", JOY_RJOG_PUSH));
    ui->JOY_TABLE_INFO_LEFT->setItem(6,0,tempItem);
    tempItem = new QTableWidgetItem(); tempItem->setText(str.sprintf("%d", JOY_LJOG_PUSH));
    ui->JOY_TABLE_INFO_LEFT->setItem(7,0,tempItem);
    tempItem = new QTableWidgetItem(); tempItem->setText(str.sprintf("%d", JOY_BACK));
    ui->JOY_TABLE_INFO_LEFT->setItem(8,0,tempItem);

    tempItem = new QTableWidgetItem(); tempItem->setText(str.sprintf("%d", JOY_RT));
    ui->JOY_TABLE_INFO_RIGHT->setItem(0,0,tempItem);
    tempItem = new QTableWidgetItem(); tempItem->setText(str.sprintf("%d", JOY_RB));
    ui->JOY_TABLE_INFO_RIGHT->setItem(1,0,tempItem);
    tempItem = new QTableWidgetItem(); tempItem->setText(str.sprintf("%d", JOY_RJOG_RL));
    ui->JOY_TABLE_INFO_RIGHT->setItem(2,0,tempItem);
    tempItem = new QTableWidgetItem(); tempItem->setText(str.sprintf("%d", JOY_RJOG_UD));
    ui->JOY_TABLE_INFO_RIGHT->setItem(3,0,tempItem);
    tempItem = new QTableWidgetItem(); tempItem->setText(str.sprintf("%d", JOY_Y));
    ui->JOY_TABLE_INFO_RIGHT->setItem(4,0,tempItem);
    tempItem = new QTableWidgetItem(); tempItem->setText(str.sprintf("%d", JOY_X));
    ui->JOY_TABLE_INFO_RIGHT->setItem(5,0,tempItem);
    tempItem = new QTableWidgetItem(); tempItem->setText(str.sprintf("%d", JOY_B));
    ui->JOY_TABLE_INFO_RIGHT->setItem(6,0,tempItem);
    tempItem = new QTableWidgetItem(); tempItem->setText(str.sprintf("%d", JOY_A));
    ui->JOY_TABLE_INFO_RIGHT->setItem(7,0,tempItem);
    tempItem = new QTableWidgetItem(); tempItem->setText(str.sprintf("%d", JOY_START));
    ui->JOY_TABLE_INFO_RIGHT->setItem(8,0,tempItem);
}
void JoyStickDialog::GetJoystickData(void)
{
    if(joystick->isConnected() == false){
        printf("JoyStick connection failure...!!!\n");
        return;
    }

    // Button Data
    JOY_X = joystick->JoyButton[2];
    JOY_A = joystick->JoyButton[0];
    JOY_B = joystick->JoyButton[1];
    JOY_Y = joystick->JoyButton[3];
    JOY_LB = joystick->JoyButton[4];
    JOY_RB = joystick->JoyButton[5];
    JOY_LT = joystick->JoyAxis[2];
    JOY_RT = joystick->JoyAxis[5];


    if((int)(JOY_LT) == -1){
        JOY_LT = 1;
    }
    else{
        JOY_LT = 0;
    }
    if((int)(JOY_RT) == -1){
        JOY_RT = 1;
    }
    else{
        JOY_RT = 0;
    }

    JOY_BACK = joystick->JoyButton[6];
    JOY_START = joystick->JoyButton[7];
    JOY_LJOG_PUSH = joystick->JoyButton[9];
    JOY_RJOG_PUSH = joystick->JoyButton[10];


    // AXIS Data
    JOY_LJOG_RL = joystick->JoyAxis[0];
    JOY_LJOG_UD = -joystick->JoyAxis[1];
    JOY_RJOG_RL = joystick->JoyAxis[3];
    JOY_RJOG_UD = -joystick->JoyAxis[4];

    // Hyo bin
//    if(JOY_LJOG_RL > 30000) JOY_LJOG_RL = 32767;
//    else if(JOY_LJOG_RL < -30000) JOY_LJOG_RL = -32767;
//    else JOY_LJOG_RL = 0;
//    if(JOY_LJOG_UD > 30000) JOY_LJOG_UD = 32767;
//    else if(JOY_LJOG_UD < -30000) JOY_LJOG_UD = -32767;
//    else JOY_LJOG_UD = 0;
//    if(JOY_RJOG_RL > 30000) JOY_RJOG_RL = 32767;
//    else if(JOY_RJOG_RL < -30000) JOY_RJOG_RL = -32767;
//    else JOY_RJOG_RL = 0;
//    if(JOY_RJOG_UD > 30000) JOY_RJOG_UD = 32767;
//    else if(JOY_RJOG_UD < -30000) JOY_RJOG_UD = -32767;
//    else JOY_RJOG_UD = 0;

    // Hyo in
    double th_hyo = 3000;
    if((JOY_LJOG_RL < th_hyo) && (JOY_LJOG_RL > -th_hyo)) JOY_LJOG_RL = 0;
    if((JOY_LJOG_UD < th_hyo) && (JOY_LJOG_UD > -th_hyo)) JOY_LJOG_UD = 0;
    if((JOY_RJOG_RL < th_hyo) && (JOY_RJOG_RL > -th_hyo)) JOY_RJOG_RL = 0;
    if((JOY_RJOG_UD < th_hyo) && (JOY_RJOG_UD > -th_hyo)) JOY_RJOG_UD = 0;

    JOY_AROW_RL = joystick->JoyAxis[6];
    JOY_AROW_UD = -joystick->JoyAxis[7];

    //for XBox Controller--------------------------------------------------

//    // Button Data
//    JOY_X = joystick->JoyButton[2];
//    JOY_A = joystick->JoyButton[0];
//    JOY_B = joystick->JoyButton[1];
//    JOY_Y = joystick->JoyButton[3];
//    JOY_LB = joystick->JoyButton[4];
//    JOY_RB = joystick->JoyButton[5];
//    JOY_LT = joystick->JoyAxis[5];
//    JOY_RT = joystick->JoyAxis[4];


//    if((int)(JOY_LT) == -1){
//        JOY_LT = 1;
//    }
//    else{
//        JOY_LT = 0;
//    }
//    if((int)(JOY_RT) == -1){
//        JOY_RT = 1;
//    }
//    else{
//        JOY_RT = 0;
//    }

//    JOY_BACK = joystick->JoyButton[6];
//    JOY_START = joystick->JoyButton[7];
//    JOY_LJOG_PUSH = joystick->JoyButton[9];
//    JOY_RJOG_PUSH = joystick->JoyButton[10];


//    // AXIS Data
//    JOY_LJOG_RL = joystick->JoyAxis[0];
//    JOY_LJOG_UD = -joystick->JoyAxis[1];
//    JOY_RJOG_RL = joystick->JoyAxis[2];
//    JOY_RJOG_UD = -joystick->JoyAxis[3];

//    // Hyo bin
////    if(JOY_LJOG_RL > 30000) JOY_LJOG_RL = 32767;
////    else if(JOY_LJOG_RL < -30000) JOY_LJOG_RL = -32767;
////    else JOY_LJOG_RL = 0;
////    if(JOY_LJOG_UD > 30000) JOY_LJOG_UD = 32767;
////    else if(JOY_LJOG_UD < -30000) JOY_LJOG_UD = -32767;
////    else JOY_LJOG_UD = 0;
////    if(JOY_RJOG_RL > 30000) JOY_RJOG_RL = 32767;
////    else if(JOY_RJOG_RL < -30000) JOY_RJOG_RL = -32767;
////    else JOY_RJOG_RL = 0;
////    if(JOY_RJOG_UD > 30000) JOY_RJOG_UD = 32767;
////    else if(JOY_RJOG_UD < -30000) JOY_RJOG_UD = -32767;
////    else JOY_RJOG_UD = 0;

//    // Hyo in
//    double th_hyo = 10000;
//    if((JOY_LJOG_RL < th_hyo) && (JOY_LJOG_RL > -th_hyo)) JOY_LJOG_RL = 0;
//    if((JOY_LJOG_UD < th_hyo) && (JOY_LJOG_UD > -th_hyo)) JOY_LJOG_UD = 0;
//    if((JOY_RJOG_RL < th_hyo) && (JOY_RJOG_RL > -th_hyo)) JOY_RJOG_RL = 0;
//    if((JOY_RJOG_UD < th_hyo) && (JOY_RJOG_UD > -th_hyo)) JOY_RJOG_UD = 0;

//    JOY_AROW_RL = joystick->JoyAxis[6];
//    JOY_AROW_UD = -joystick->JoyAxis[7];
    //for XBox Controller--------------------------------------------------


}
void JoyStickDialog::InitJoystickData(void)
{
    JOY_LT=JOY_LB=0;
    JOY_LJOG_RL=JOY_LJOG_UD=0;
    JOY_AROW_RL=JOY_AROW_UD=0;
    JOY_LJOG_PUSH=0;

    JOY_RT=JOY_RB=0;
    JOY_RJOG_RL=JOY_RJOG_UD=0;
    JOY_A=JOY_B=JOY_X=JOY_Y=0;
    JOY_RJOG_PUSH=0;

    JOY_BACK=JOY_START=0;
}

void JoyStickDialog::on_JOY_BTN_START_clicked()
{
    // Joy Stick Start
    JoyModeFlag=true;
    ui->JOY_BTN_START->setDisabled(true);
    ui->JOY_BTN_STOP->setDisabled(false);
    ui->JOY_TAB_WHEELSTART->setDisabled(false);
    ui->JOY_TAB_WHEELSTOP->setDisabled(true);

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = ManualMove_AL_JOYSTICK_MODE;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0; // On
    cmd.COMMAND_TARGET = AlnumManualMove;
    pLAN->SendCommand(cmd);

//    ui->MANUAL_BTN_START->setEnabled(true);
//    ui->MANUAL_BTN_STOP->setDisabled(true);
}

void JoyStickDialog::on_JOY_BTN_STOP_clicked()
{
    // Joy Stick Stop
    JoyModeFlag=false;
    ui->JOY_BTN_STOP->setDisabled(true);
    ui->JOY_BTN_START->setDisabled(false);
    ui->JOY_TAB_WHEELSTART->setDisabled(true);
    ui->JOY_TAB_WHEELSTOP->setDisabled(true);

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = ManualMove_AL_JOYSTICK_MODE;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1; // Off
    cmd.COMMAND_TARGET = AlnumManualMove;
    pLAN->SendCommand(cmd);

//    ui->MANUAL_BTN_START->setDisabled(true);
//    ui->MANUAL_BTN_STOP->setDisabled(true);
}

void JoyStickDialog::on_JOY_TAB_WHEELSTART_clicked()
{
    JoyModeFlag=true;
    ui->JOY_BTN_STOP->setDisabled(true);
    ui->JOY_BTN_START->setDisabled(true);
    ui->JOY_TAB_WHEELSTART->setDisabled(true);
    ui->JOY_TAB_WHEELSTOP->setDisabled(false);

    // WHEEL MANUAL START
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]=1;
    cmd.COMMAND_DATA.USER_PARA_INT[0]=(int)0;
    cmd.COMMAND_DATA.USER_PARA_INT[1]=(int)0;
    cmd.COMMAND_DATA.USER_PARA_INT[2]=(int)0;
    cmd.COMMAND_DATA.USER_COMMAND = OMNIWHEEL_AL_MANUAL;
    cmd.COMMAND_TARGET = AlnumOmniWheel;
    pLAN->SendCommand(cmd);
    WheelModeFlag=true;
}

void JoyStickDialog::on_JOY_TAB_WHEELSTOP_clicked()
{
    JoyModeFlag=false;
    ui->JOY_BTN_STOP->setDisabled(false);
    ui->JOY_BTN_START->setDisabled(true);
    ui->JOY_TAB_WHEELSTART->setDisabled(false);
    ui->JOY_TAB_WHEELSTOP->setDisabled(true);

    // WHEEL MANUAL STOP
    WheelModeCount=0;
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]=0;
    cmd.COMMAND_DATA.USER_PARA_INT[0]=(int)0;
    cmd.COMMAND_DATA.USER_PARA_INT[1]=(int)0;
    cmd.COMMAND_DATA.USER_PARA_INT[2]=(int)0;
    cmd.COMMAND_DATA.USER_COMMAND = OMNIWHEEL_AL_MANUAL;
    cmd.COMMAND_TARGET = AlnumOmniWheel;
    pLAN->SendCommand(cmd);

    WheelModeFlag=false;
}

void JoyStickDialog::on_MANUAL_BTN_START_clicked()
{
    ManualModeCount =0;
    ui->JOY_BTN_STOP->setEnabled(true);
    ui->JOY_BTN_START->setEnabled(false);

//    ui->MANUAL_BTN_START->setDisabled(true);
//    ui->MANUAL_BTN_STOP->setDisabled(false);

    // Manual Hand Start
    ManualModeFlag = true;
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = ManualMove_AL_MANUAL_MODE_START;
    cmd.COMMAND_TARGET = AlnumManualMove;
    pLAN->SendCommand(cmd);
}

void JoyStickDialog::on_MANUAL_BTN_STOP_clicked()
{
    ui->JOY_BTN_STOP->setDisabled(false);
    ui->JOY_BTN_START->setDisabled(true);

    // Manual Hand Stop
    ManualModeCount =0;
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = ManualMove_AL_MANUAL_MODE_STOP;
    cmd.COMMAND_TARGET = AlnumManualMove;
    pLAN->SendCommand(cmd);
    ManualModeFlag = false;
}

void JoyStickDialog::on_MANUAL_BTN_GOTOTASKPOSE_0_clicked()
{
    // Goto Task pos 0
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;
    cmd.COMMAND_DATA.USER_COMMAND = ManualMove_AL_UPPER_TASK_POSE;
    cmd.COMMAND_TARGET = AlnumManualMove;
    pLAN->SendCommand(cmd);
}

void JoyStickDialog::on_MANUAL_BTN_GOTOTASKPOSE_180_clicked()
{
    // Goto Task pos 180
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;
    cmd.COMMAND_DATA.USER_COMMAND = ManualMove_AL_UPPER_TASK_POSE;
    cmd.COMMAND_TARGET = AlnumManualMove;
    pLAN->SendCommand(cmd);
}

void JoyStickDialog::on_MANUAL_BTN_GOTOMOVEPOSE_0_clicked()
{
    // Goto Move pos 0
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 2;
    cmd.COMMAND_DATA.USER_COMMAND = ManualMove_AL_UPPER_TASK_POSE;
    cmd.COMMAND_TARGET = AlnumManualMove;
    pLAN->SendCommand(cmd);
}

void JoyStickDialog::on_MANUAL_BTN_GOTOMOVEPOSE_180_clicked()
{
    // Goto Move pos 180
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 3;
    cmd.COMMAND_DATA.USER_COMMAND = ManualMove_AL_UPPER_TASK_POSE;
    cmd.COMMAND_TARGET = AlnumManualMove;
    pLAN->SendCommand(cmd);
}

void JoyStickDialog::on_MANUAL_HP_PLUS_clicked()
{
    // HP +
    QString HPang = ui->MANUAL_HP_ANGLE->text();
    bool bSuccess=false;
    double HP_ang = fabs(HPang.toDouble(&bSuccess));

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 4;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = HP_ang;
    cmd.COMMAND_DATA.USER_COMMAND = ManualMove_AL_UPPER_TASK_POSE;
    cmd.COMMAND_TARGET = AlnumManualMove;
    pLAN->SendCommand(cmd);
}

void JoyStickDialog::on_MANUAL_HP_MINUS_clicked()
{
    // HP -
    QString HPang = ui->MANUAL_HP_ANGLE->text();
    bool bSuccess=false;
    double HP_ang = fabs(HPang.toDouble(&bSuccess));

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 4;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = -HP_ang;
    cmd.COMMAND_DATA.USER_COMMAND = ManualMove_AL_UPPER_TASK_POSE;
    cmd.COMMAND_TARGET = AlnumManualMove;
    pLAN->SendCommand(cmd);
}

void JoyStickDialog::on_MANUAL_RH_GRIB_clicked()
{
    // RH grib
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;//right
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 1;// 1 grib 0 stop -1 open
    cmd.COMMAND_DATA.USER_COMMAND = ManualMove_AL_HAND;
    cmd.COMMAND_TARGET = AlnumManualMove;
    pLAN->SendCommand(cmd);
}

void JoyStickDialog::on_MANUAL_RH_STOP_clicked()
{
    // RH stop
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;//right
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 0;// 1 grib 0 stop -1 open
    cmd.COMMAND_DATA.USER_COMMAND = ManualMove_AL_HAND;
    cmd.COMMAND_TARGET = AlnumManualMove;
    pLAN->SendCommand(cmd);
}

void JoyStickDialog::on_MANUAL_RH_OPEN_clicked()
{
    // RH open
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;//right
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = -1;// 1 grib 0 stop -1 open
    cmd.COMMAND_DATA.USER_COMMAND = ManualMove_AL_HAND;
    cmd.COMMAND_TARGET = AlnumManualMove;
    pLAN->SendCommand(cmd);
}

void JoyStickDialog::on_MANUAL_LH_GRIB_clicked()
{
    // LH grib
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;//left
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 1;// 1 grib 0 stop -1 open
    cmd.COMMAND_DATA.USER_COMMAND = ManualMove_AL_HAND;
    cmd.COMMAND_TARGET = AlnumManualMove;
    pLAN->SendCommand(cmd);
}

void JoyStickDialog::on_MANUAL_LH_STOP_clicked()
{
    // LH stop
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;//left
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 0;// 1 grib 0 stop -1 open
    cmd.COMMAND_DATA.USER_COMMAND = ManualMove_AL_HAND;
    cmd.COMMAND_TARGET = AlnumManualMove;
    pLAN->SendCommand(cmd);
}

void JoyStickDialog::on_MANUAL_LH_OPEN_clicked()
{
    // LH open
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;//left
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = -1;// 1 grib 0 stop -1 open
    cmd.COMMAND_DATA.USER_COMMAND = ManualMove_AL_HAND;
    cmd.COMMAND_TARGET = AlnumManualMove;
    pLAN->SendCommand(cmd);
}

void JoyStickDialog::on_MANUAL_RARM_GAIN_OVER_START_clicked()
{
    // R arm low gain
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;//right
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 0;// 0 Low gain 1 high gain
    cmd.COMMAND_DATA.USER_COMMAND = ManualMove_AL_GAIN;
    cmd.COMMAND_TARGET = AlnumManualMove;
    pLAN->SendCommand(cmd);
}

void JoyStickDialog::on_MANUAL_RARM_GAIN_OVER_RETURN_clicked()
{
    // R arm high gain
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;//right
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 1;// 0 Low gain 1 high gain
    cmd.COMMAND_DATA.USER_COMMAND = ManualMove_AL_GAIN;
    cmd.COMMAND_TARGET = AlnumManualMove;
    pLAN->SendCommand(cmd);
}

void JoyStickDialog::on_MANUAL_LARM_GAIN_OVER_START_clicked()
{
    // L arm low gain
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;//left
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 0;// 0 Low gain 1 high gain
    cmd.COMMAND_DATA.USER_COMMAND = ManualMove_AL_GAIN;
    cmd.COMMAND_TARGET = AlnumManualMove;
    pLAN->SendCommand(cmd);
}

void JoyStickDialog::on_MANUAL_LARM_GAIN_OVER_RETURN_clicked()
{
    // L arm high gain
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;//left
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 1;// 0 Low gain 1 high gain
    cmd.COMMAND_DATA.USER_COMMAND = ManualMove_AL_GAIN;
    cmd.COMMAND_TARGET = AlnumManualMove;
    pLAN->SendCommand(cmd);
}

void JoyStickDialog::on_MANUAL_BTN_TWO_HAND_START_clicked()
{
    ManualModeCount =0;
    ui->JOY_BTN_STOP->setEnabled(true);
    ui->JOY_BTN_START->setEnabled(false);
    // Manual Foot Start
    ManualModeFlag = true;
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = ManualMove_AL_MANUAL_BOTH_HAND_MODE_START;
    cmd.COMMAND_TARGET = AlnumManualMove;
    pLAN->SendCommand(cmd);
}

void JoyStickDialog::on_MANUAL_BTN_FOOT_START_clicked()
{
    ManualModeCount =0;
    ui->JOY_BTN_STOP->setEnabled(true);
    ui->JOY_BTN_START->setEnabled(false);
    // Manual Foot Start
    ManualModeFlag = true;
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = ManualMove_AL_MANUAL_FOOT_MODE_START;
    cmd.COMMAND_TARGET = AlnumManualMove;
    pLAN->SendCommand(cmd);
}

void JoyStickDialog::on_MANUAL_BTN_TWO_FOOT_START_clicked()
{
    ManualModeCount =0;
    ui->JOY_BTN_STOP->setEnabled(true);
    ui->JOY_BTN_START->setEnabled(false);
    // Manual Foot Start
    ManualModeFlag = true;
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = ManualMove_AL_MANUAL_BOTH_FOOT_MODE_START;
    cmd.COMMAND_TARGET = AlnumManualMove;
    pLAN->SendCommand(cmd);
}

void JoyStickDialog::on_MANUAL_BTN_STANDING_ONE_HAND_clicked()
{
    ManualModeCount =0;
    ui->JOY_BTN_STOP->setEnabled(true);
    ui->JOY_BTN_START->setEnabled(false);

    // Manual Hand Start
    ManualModeFlag = true;
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = ManualMove_AL_MANUAL_ONE_HAND_STADING_START;
    cmd.COMMAND_TARGET = AlnumManualMove;
    printf("COMMAND_TARGET = %d\n",AlnumManualMove);
    pLAN->SendCommand(cmd);
}

void JoyStickDialog::on_MANUAL_BTN_STANDING_TWO_HAND_clicked()
{
    ManualModeCount =0;
    ui->JOY_BTN_STOP->setEnabled(true);
    ui->JOY_BTN_START->setEnabled(false);

    // Manual Hand Start
    ManualModeFlag = true;
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = ManualMove_AL_MANUAL_BOTH_HAND_STADING_START;
    cmd.COMMAND_TARGET = AlnumManualMove;
    pLAN->SendCommand(cmd);
}

void JoyStickDialog::on_MANUAL_BTN_DRIVE_ON_clicked()
{
    ManualModeCount =0;
    ui->JOY_BTN_STOP->setEnabled(true);
    ui->JOY_BTN_START->setEnabled(false);

    // Manual Hand Start
    ManualModeFlag = true;
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0; //On
    cmd.COMMAND_DATA.USER_COMMAND = ManualMove_AL_DRIVE_MODE;
    cmd.COMMAND_TARGET = AlnumManualMove;
    pLAN->SendCommand(cmd);
}

void JoyStickDialog::on_MANUAL_BTN_DRIVE_OFF_clicked()
{
    ManualModeCount =0;
    ui->JOY_BTN_STOP->setEnabled(true);
    ui->JOY_BTN_START->setEnabled(false);

    // Manual Hand Start
    ManualModeFlag = true;
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1; //Off
    cmd.COMMAND_DATA.USER_COMMAND = ManualMove_AL_DRIVE_MODE;
    cmd.COMMAND_TARGET = AlnumManualMove;
    pLAN->SendCommand(cmd);
}
