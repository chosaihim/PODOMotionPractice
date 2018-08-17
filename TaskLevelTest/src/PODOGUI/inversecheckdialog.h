#ifndef INVERSECHECKDIALOG_H
#define INVERSECHECKDIALOG_H

#include <QDialog>
#include "CommonHeader.h"
#include "../../share/Headers/commandlist.h"

namespace Ui {
class InverseCheckDialog;
}

class InverseCheckDialog : public QDialog
{
    Q_OBJECT

public:
    explicit InverseCheckDialog(QWidget *parent = 0);
    ~InverseCheckDialog();

private slots:
    void on_PB_IKandGO_clicked();
    void UpdateValue();

    void on_PB_FK_clicked();

    void on_PB_WALKREADY_clicked();

private:
    Ui::InverseCheckDialog *ui;
    int alnumWalkReady;
    int alnumInverseCheck;
};

#endif // INVERSECHECKDIALOG_H
