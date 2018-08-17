#ifndef WBWALKDIALOG_H
#define WBWALKDIALOG_H

#include <QDialog>

namespace Ui {
class WBWalkDialog;
}

class WBWalkDialog : public QDialog
{
    Q_OBJECT

public:
    explicit WBWalkDialog(QWidget *parent = 0);
    ~WBWalkDialog();

private slots:
    void on_BTN_Walk_Ready_clicked();

    void on_BTN_FORWARD_clicked();

    void on_BTN_SAVE_START_clicked();

    void on_BTN_DATA_SAVE_clicked();

    void on_PB_APPROACH_H_clicked();

    void on_PB_GRASP_H_clicked();

    void on_PB_QUIT_H_clicked();

private:
    Ui::WBWalkDialog *ui;
    int ALNum_WalkReady;
    int ALNum_WBWalk;
};

#endif // WBWALKDIALOG_H
