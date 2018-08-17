#ifndef LIFTBOXDIALOG_H
#define LIFTBOXDIALOG_H

#include <QDialog>
#include "CommonHeader.h"
#include "../../share/Headers/commandlist.h"
#include "../../share/Headers/LANData/VisionLANData.h"

namespace Ui {
class LiftBoxDialog;
}

class LiftBoxDialog : public QDialog
{
    Q_OBJECT

public:
    explicit LiftBoxDialog(QWidget *parent = 0);
    ~LiftBoxDialog();

private slots:
    void on_pushButton_clicked();
    void DisplayUpdate();

    void on_PB_READY_clicked();

    void on_PB_START_clicked();

    void on_PB_STOP_clicked();

    void on_pushButton_2_clicked();

    void on_BTN_FORWARD_clicked();

    void on_BTN_LEFT_clicked();

    void on_BTN_RIGHT_clicked();

    void on_BTN_CCW_clicked();

    void on_BTN_CW_clicked();

    void on_BTN_Walk_Ready_clicked();

    void on_BTN_SAVE_START_clicked();

    void on_BTN_DATA_SAVE_clicked();

    void on_PB_SIT_DOWN_clicked();

    void on_PB_STAND_UP_clicked();

    void on_PB_STAND_UP_2_clicked();

    void on_PB_STAND_UP_3_clicked();

    void on_PB_BOX__clicked();

    void on_PB_SINGLELOG_WALK_clicked();

private:
    Ui::LiftBoxDialog   *ui;
    QTimer              *displayTimer;
    int ALNum_LiftBox;
    int ALNum_ApproachBox;
    int ALNum_WalkReady;
};

#endif // LIFTBOXDIALOG_H
