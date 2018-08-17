#ifndef TESTDIALOG_H
#define TESTDIALOG_H

#include <QDialog>
#include "CommonHeader.h"


namespace Ui {
class TestDialog;
}

class TestDialog : public QDialog
{
    Q_OBJECT

public:
    explicit TestDialog(QWidget *parent = 0);
    ~TestDialog();

private slots:
    void on_PB_WalkReady_clicked();

    void on_PB_HandsUp_clicked();

    void on_PB_WholeBody_clicked();

    void on_PB_motion1_clicked();

    void on_PB_WalkReady2_clicked();

private:
    Ui::TestDialog *ui;
    LANDialog *lanData;
    int AlnumWalkReady;
    int AlnumOmniWheel;
    int AlnumHandsUp;
    QTimer *displayTimer;
};

#endif // TESTDIALOG_H
