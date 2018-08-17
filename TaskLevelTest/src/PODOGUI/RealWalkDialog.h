#ifndef REALWALKDIALOG_H
#define REALWALKDIALOG_H

#include <QDialog>

namespace Ui {
class RealWalkDialog;
}

class RealWalkDialog : public QDialog
{
    Q_OBJECT

public:
    explicit RealWalkDialog(QWidget *parent = 0);
    ~RealWalkDialog();

private slots:
    void on_PB_WalkReady_clicked();

    void on_PB_Walking_clicked();

    void on_PB_SingleLog_clicked();

    void on_PB_WalkStop_clicked();

    void on_PB_WalkingReady_clicked();

    void on_PB_SAVE_START_clicked();

    void on_PB_SAVE_clicked();

private:
    Ui::RealWalkDialog *ui;
    int alNumApproachBox;
    int alNumWalkReady;
};

#endif // REALWALKDIALOG_H
