#ifndef JOYSTICKDIALOG_H
#define JOYSTICKDIALOG_H

#include <QDialog>
#include "CommonHeader.h"
#include "JoyStick/joystickclass.h"
#include "JoyStick/joystickvariable.h"

namespace Ui {
class JoyStickDialog;
}

class JoyStickDialog : public QDialog
{
    Q_OBJECT

public:
    explicit JoyStickDialog(QWidget *parent = 0);
    ~JoyStickDialog();

private slots:
    void DisplayUpdate();

    void GetJoystickData(void);

    void InitJoystickData(void);

    void on_JOY_BTN_START_clicked();

    void on_JOY_BTN_STOP_clicked();

    void on_JOY_TAB_WHEELSTART_clicked();

    void on_JOY_TAB_WHEELSTOP_clicked();

    void on_MANUAL_BTN_START_clicked();

    void on_MANUAL_BTN_STOP_clicked();

    void on_MANUAL_BTN_GOTOTASKPOSE_0_clicked();

    void on_MANUAL_BTN_GOTOTASKPOSE_180_clicked();

    void on_MANUAL_BTN_GOTOMOVEPOSE_0_clicked();

    void on_MANUAL_BTN_GOTOMOVEPOSE_180_clicked();

    void on_MANUAL_HP_PLUS_clicked();

    void on_MANUAL_HP_MINUS_clicked();

    void on_MANUAL_RH_GRIB_clicked();

    void on_MANUAL_RH_STOP_clicked();

    void on_MANUAL_RH_OPEN_clicked();

    void on_MANUAL_LH_GRIB_clicked();

    void on_MANUAL_LH_STOP_clicked();

    void on_MANUAL_LH_OPEN_clicked();

    void on_MANUAL_RARM_GAIN_OVER_START_clicked();

    void on_MANUAL_RARM_GAIN_OVER_RETURN_clicked();

    void on_MANUAL_LARM_GAIN_OVER_START_clicked();

    void on_MANUAL_LARM_GAIN_OVER_RETURN_clicked();

    void on_MANUAL_BTN_TWO_HAND_START_clicked();

    void on_MANUAL_BTN_FOOT_START_clicked();

    void on_MANUAL_BTN_TWO_FOOT_START_clicked();

    void on_MANUAL_BTN_STANDING_ONE_HAND_clicked();

    void on_MANUAL_BTN_STANDING_TWO_HAND_clicked();

    void on_MANUAL_BTN_DRIVE_ON_clicked();

    void on_MANUAL_BTN_DRIVE_OFF_clicked();


private:
    LANDialog			*lanData;
    Ui::JoyStickDialog *ui;
    QTimer				*displayTimer;

    RBJoystick          *joystick;
    char JOY_LT, JOY_LB;
    int JOY_LJOG_RL, JOY_LJOG_UD;
    int JOY_AROW_RL, JOY_AROW_UD;
    char JOY_LJOG_PUSH;

    char JOY_RT, JOY_RB;
    int JOY_RJOG_RL, JOY_RJOG_UD;
    char JOY_A, JOY_B, JOY_X, JOY_Y;
    char JOY_RJOG_PUSH;

    char JOY_BACK, JOY_START;


    int AlunumWMupperbody;
    int AlnumOmniWheel;
    int AlnumManualMove;

};

#endif // JOYSTICKDIALOG_H
