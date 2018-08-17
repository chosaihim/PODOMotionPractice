#ifndef ZMPGRAPHDIALOG_H
#define ZMPGRAPHDIALOG_H

#include <QDialog>
#include <QTimer>
#include "qcustomplot.h"
#include "CommonHeader.h"
#include "../../share/Headers/commandlist.h"

namespace Ui {
class ZMPGraphDialog;
}

class ZMPGraphDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ZMPGraphDialog(QWidget *parent = 0);
    ~ZMPGraphDialog();
    void get_zmp();
    void ClearData();


private slots:
    void RealTimeDataSlot();
    //void RealTimeDrawZMPonFootPrint();
    void AddData();
    void on_PB_COMPLIANCE_START_clicked();
    void Plot_GRP_ZMPFOOT();
    void on_PB_CLEAR_ZMPFOOT_clicked();

    void on_PB_COMPLIANCE_STOP_clicked();

    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

private:
    Ui::ZMPGraphDialog *ui;
    QTimer              graphTimer;
    int                 AlnumMissionDoor;
    int                 AlnumWalk;

    double      zmp[3];
    double      zmp_local[3];
    QVector<double> qv_zmpx,qv_zmpy;
    //float       RFpos[2],LFpos[2],FootSize[2];
};

#endif // ZMPGRAPHDIALOG_H
