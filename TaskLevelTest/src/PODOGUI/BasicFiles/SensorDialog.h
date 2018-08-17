#ifndef SENSORDIALOG_H
#define SENSORDIALOG_H

#include <QDialog>
#include "CommonHeader.h"


namespace Ui {
class SensorDialog;
}

class SensorDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SensorDialog(QWidget *parent = 0);
    ~SensorDialog();



private slots:
    void on_BTN_SENSOR_ENABLE_clicked();
    void on_BTN_SENSOR_DISABLE_clicked();
    void on_BTN_SENSOR_FT_NULL_clicked();
    void on_BTN_SENSOR_IMU_NULL_clicked();
    void on_BTN_CIMU_GET_OFFSET_clicked();
    void on_BTN_CIMU_SET_OFFSET_clicked();
    void on_BTN_SENSOR_FOG_ZERO_clicked();
    void on_BTN_SENSOR_FOG_NULL_clicked();
    void on_BTN_OPTZERO_clicked();

    void UpdateSensors();


    void on_BTN_OPT_LAMP_ON_clicked();

    void on_BTN_OPT_LAMP_OFF_clicked();

    void on_BTN_SENSOR_FOG_USB_RESET_clicked();

    void on_BTN_NEW_IMU_ENABLE_clicked();

    void on_BTN_NEW_IMU_NULL_clicked();

    void on_BTN_NEW_IMU_RESET_clicked();

private:
    Ui::SensorDialog *ui;
};

#endif // SENSORDIALOG_H
