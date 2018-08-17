#ifndef DOORDIALOG_H
#define DOORDIALOG_H

#include <QDialog>
#include "CommonHeader.h"
#include "qcustomplot.h"
#include "../../share/Headers/commandlist.h"

namespace Ui {
class DoorDialog;
}

class DoorDialog : public QDialog
{
    Q_OBJECT

public:
    explicit DoorDialog(QWidget *parent = 0);
    ~DoorDialog();
    void get_zmp();
    void ClearData();
signals:
    void FORWARDpressed();
    void BACKWARDpressed();
    void FORWARDreleased();
    void BACKWARDreleased();
private slots:
    void on_BTN_Walk_Ready_clicked();

    void on_BTN_FORWARD_clicked();
    void AddData();

    void on_BTN_LEFT_clicked();

    void on_BTN_RIGHT_clicked();

    void on_PB_READY_clicked();

    void on_PB_START_clicked();

    void on_PB_STOP_clicked();

    void on_BTN_SAVE_START_clicked();

    void on_BTN_DATA_SAVE_clicked();

    void on_PB_COMPLIANCE_START_clicked();

    void on_PB_PUSHDOOR_clicked();

    void on_PB_LEANEDFORWARD_clicked();

    void on_PB_LEANED_F_pressed();
    void Plot_GRP_ZMPFOOT();
    void on_PB_LEANED_B_pressed();

    void on_PB_LEANED_F_released();

    void on_PB_LEANED_B_released();
    void UpdateLineEdit();

    void on_PB_CLEAR_ZMPFOOT_clicked();

private:
    Ui::DoorDialog *ui;
    QTimer              graphTimer;
    int ALNum_MissionDoor;
    int ALNum_ApproachBox;
    int ALNum_WalkReady;

    int LeanedFORWARDcnt;
    int LeanedBACKWARDcnt;

    double      zmp[3];
    double      zmp_local[3];
    QVector<double> qv_zmpx,qv_zmpy;
};

#endif // DOORDIALOG_H
