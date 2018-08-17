#include "ZMPGraphDialog.h"
#include "ui_ZMPGraphDialog.h"
#include "../../../share/Headers/ik_math4.h"
#include "../../../share/Headers/kine_drc_hubo4.h"
#include "BasicFiles/PODOALDialog.h"

ZMPGraphDialog::ZMPGraphDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ZMPGraphDialog)
{
    ui->setupUi(this);

    AlnumMissionDoor = PODOALDialog::GetALNumFromFileName("MissionDoor");
    AlnumWalk = PODOALDialog::GetALNumFromFileName("ApproachBox");

    ui->GRP_RT->addGraph();
    ui->GRP_RT->graph(0)->setPen(QPen(Qt::red));
    ui->GRP_RT->addGraph();
    ui->GRP_RT->graph(1)->setPen(QPen(Qt::blue));
    ui->GRP_RT->addGraph();
    ui->GRP_RT->graph(2)->setPen(QPen(Qt::green));

    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    timeTicker->setTimeFormat("%h:%m:%s");
    ui->GRP_RT->xAxis->setTicker(timeTicker);
    ui->GRP_RT->axisRect()->setupFullAxesBox();
    ui->GRP_RT->yAxis->setRange(-100,100);

    ui->GRP_ZMPFOOT->addGraph();
    ui->GRP_ZMPFOOT->graph(0)->setPen(QPen(Qt::red));
    ui->GRP_ZMPFOOT->addGraph();
    ui->GRP_ZMPFOOT->graph(1)->setPen(QPen(Qt::blue));
    ui->GRP_ZMPFOOT->graph(1)->setLineStyle(QCPGraph::lsNone);
    ui->GRP_ZMPFOOT->graph(1)->setScatterStyle(QCPScatterStyle::ssDisc);
    ui->GRP_ZMPFOOT->xAxis->setRange(-0.5,0.5);
    ui->GRP_ZMPFOOT->yAxis->setRange(-0.5,0.5);

    connect(pLAN, SIGNAL(NewPODOData()),this,SLOT(AddData()));

    connect(&graphTimer, SIGNAL(timeout()), this, SLOT(RealTimeDataSlot()));
    connect(&graphTimer, SIGNAL(timeout()), this, SLOT(Plot_GRP_ZMPFOOT()));
    graphTimer.start(50); // Interval 0 means to refresh as fast as possible

}

ZMPGraphDialog::~ZMPGraphDialog()
{
    delete ui;
}

void ZMPGraphDialog::RealTimeDataSlot()
{
    if(ui->CB_GRAPHONOFF->isChecked() == true)
    {
          static QTime time(QTime::currentTime());
          // calculate two new data points:
          double key = time.elapsed()/1000.0; // time elapsed since start of demo, in seconds
          static double lastPointKey = 0;
          if (key-lastPointKey > 0.005) // at most add point every 2 ms
          {
            // add data to lines:
            //ui->customPlot->graph(0)->addData(key, qSin(key)+qrand()/(double)RAND_MAX*1*qSin(key/0.3843));

            //ui->GRP_RT->graph(0)->addData(key, zmp[0]);
            //ui->GRP_RT->graph(1)->addData(key, zmp[1]);

            //ui->GRP_RT->graph(0)->addData(key, PODO_DATA.CoreData.FT[0].Fz);
            //ui->GRP_RT->graph(1)->addData(key, PODO_DATA.UserM2G.LPF_Fz);

        //    ui->GRP_RT->graph(0)->addData(key, PODO_DATA.UserM2G.LPF_Fx);
        //    ui->GRP_RT->graph(1)->addData(key, PODO_DATA.UserM2G.LPF_Fy);
        //    ui->GRP_RT->graph(2)->addData(key, PODO_DATA.UserM2G.LPF_Fz);

              ui->GRP_RT->graph(0)->addData(key, PODO_DATA.UserM2G.Des_posX);
              ui->GRP_RT->graph(1)->addData(key, PODO_DATA.UserM2G.Des_velX);
              ui->GRP_RT->graph(2)->addData(key, PODO_DATA.UserM2G.LPF_Fz/10.);

        //      ui->GRP_RT->graph(0)->addData(key, PODO_DATA.UserM2G.curZMP[0]);
              //ui->GRP_RT->graph(0)->addData(key, PODO_DATA.UserM2G.curZMP[1]);
             // ui->GRP_RT->graph(1)->addData(key, PODO_DATA.UserM2G.curZMP[0]);


            // rescale value (vertical) axis to fit the current data:
            //ui->customPlot->graph(0)->rescaleValueAxis();
            //ui->customPlot->graph(1)->rescaleValueAxis(true);
            lastPointKey = key;
          }
          // make key axis range scroll with the data (at a constant range size of 8):
          ui->GRP_RT->xAxis->setRange(key, 8, Qt::AlignRight);
          ui->GRP_RT->replot();

          // calculate frames per second:
          static double lastFpsKey;
          static int frameCount;
          ++frameCount;
          if (key-lastFpsKey > 2) // average fps over 2 seconds
          {
            /*ui->statusBar->showMessage(
                  QString("%1 FPS, Total Data points: %2")
                  .arg(frameCount/(key-lastFpsKey), 0, 'f', 0)
                  .arg(ui->customPlot->graph(0)->data()->size()+ui->customPlot->graph(1)->data()->size())
                  , 0);*/
            lastFpsKey = key;
            frameCount = 0;
          }
    }
}

void ZMPGraphDialog::AddData()
{
    get_zmp();
    qv_zmpx.append(zmp[0]);
    qv_zmpy.append(zmp[1]);
}

void ZMPGraphDialog::ClearData()
{
    qv_zmpx.clear();
    qv_zmpy.clear();
}

void ZMPGraphDialog::Plot_GRP_ZMPFOOT()
{
    if(ui->CB_GRAPHONOFF->isChecked() == true)
    {
        get_zmp();
        qv_zmpx.append(zmp[0]);
        qv_zmpy.append(zmp[1]);
        float RFpos[2] = {0.0,-0.105};
        float LFpos[2]  = {0.0,0.105};
        float FootSize[2]  = {0.16,0.24};

        QCPItemRect *RightFoot = new QCPItemRect(ui->GRP_ZMPFOOT);
        RightFoot->topLeft->setCoords(RFpos[0]-FootSize[1]/2.,RFpos[1]+FootSize[0]/2.);
        RightFoot->bottomRight->setCoords(RFpos[0]+FootSize[1]/2.,RFpos[1]-FootSize[0]/2.);

        QCPItemRect *LeftFoot = new QCPItemRect(ui->GRP_ZMPFOOT);
        LeftFoot->topLeft->setCoords(LFpos[0]-FootSize[1]/2.,LFpos[1]+FootSize[0]/2.);
        LeftFoot->bottomRight->setCoords(LFpos[0]+FootSize[1]/2.,LFpos[1]-FootSize[0]/2.);

        ui->GRP_ZMPFOOT->graph(0)->setData(qv_zmpx,qv_zmpy);
        ui->GRP_ZMPFOOT->graph(1)->data()->clear();
        ui->GRP_ZMPFOOT->graph(1)->addData(zmp[0],zmp[1]);
        ui->GRP_ZMPFOOT->replot();
        ui->GRP_ZMPFOOT->update();
    }
}

void ZMPGraphDialog::get_zmp()
{
        // ZMP 읽기 //
        double M_LF[3],M_RF[3],M_LF_Global[3],M_RF_Global[3],F_LF[3],F_RF[3],F_LF_Global[3],F_RF_Global[3];
        double pCenter[3],qCenter[4],qCenter_bar[4];

        PODO_DATA.UserM2G.pRF[0] = 0.0;
        PODO_DATA.UserM2G.pRF[1] = -0.105;
        PODO_DATA.UserM2G.pRF[2] = 0.0;
        PODO_DATA.UserM2G.pLF[1] = 0.105;
        PODO_DATA.UserM2G.qLF[0] = 1;
        PODO_DATA.UserM2G.qRF[0] = 1;

        // Foot Center in Global Coord.
        pCenter[0] = (PODO_DATA.UserM2G.pRF[0] + PODO_DATA.UserM2G.pLF[0])/2.;
        pCenter[1] = (PODO_DATA.UserM2G.pRF[1] + PODO_DATA.UserM2G.pLF[1])/2.;
        pCenter[2] = (PODO_DATA.UserM2G.pRF[2] + PODO_DATA.UserM2G.pLF[2])/2.;

        if(PODO_DATA.CoreData.FT[0].Fz + PODO_DATA.CoreData.FT[1].Fz > 50.)
        {
            M_LF[0] =  PODO_DATA.CoreData.FT[1].Mx;
            M_LF[1] =  PODO_DATA.CoreData.FT[1].My;
            M_LF[2] =  PODO_DATA.CoreData.FT[1].Mz;

            QTtransform(PODO_DATA.UserM2G.qLF,M_LF,M_LF_Global);

            M_RF[0] =  PODO_DATA.CoreData.FT[0].Mx;
            M_RF[1] =  PODO_DATA.CoreData.FT[0].My;
            M_RF[2] =  PODO_DATA.CoreData.FT[0].Mz;

            QTtransform(PODO_DATA.UserM2G.qRF,M_RF,M_RF_Global);

            F_LF[0] = PODO_DATA.CoreData.FT[1].Fx;
            F_LF[1] = PODO_DATA.CoreData.FT[1].Fy;
            F_LF[2] = PODO_DATA.CoreData.FT[1].Fz;

            QTtransform(PODO_DATA.UserM2G.qLF,F_LF,F_LF_Global);

            F_RF[0] = PODO_DATA.CoreData.FT[0].Fx;
            F_RF[1] = PODO_DATA.CoreData.FT[0].Fy;
            F_RF[2] = PODO_DATA.CoreData.FT[0].Fz;

            QTtransform(PODO_DATA.UserM2G.qRF,F_RF,F_RF_Global);

            double temp1[3],temp2[3],temp3[3],temp4[3];

            diff_vv(PODO_DATA.UserM2G.pRF,3,pCenter,temp1);// (despRF - pCenter)
            diff_vv(PODO_DATA.UserM2G.pLF,3,pCenter,temp2);// (despLF - pCenter)

            cross(1,temp1,F_RF_Global,temp3);// (despRF - pCenter)x(F_RF_Global)
            cross(1,temp2,F_LF_Global,temp4);// (despLF - pCenter)x(F_LF_Global)

            sum_vv(temp3,3,temp4,temp3); // (despRF - pCenter)x(F_RF_Global) + (despLF - pCenter)x(F_LF_Global)
            sum_vv(temp3,3,M_RF_Global,temp3); // (despRF - pCenter)x(F_RF_Global) + (despLF - pCenter)x(F_LF_Global) + M_RF_Global
            sum_vv(temp3,3,M_LF_Global,temp3); // (despRF - pCenter)x(F_RF_Global) + (despLF - pCenter)x(F_LF_Global) + M_RF_Global + M_LF_Global

            zmp[0] = (-temp3[1])/(F_RF_Global[2] + F_LF_Global[2]) + pCenter[0];
            zmp[1] = (temp3[0])/(F_RF_Global[2] + F_LF_Global[2]) + pCenter[1];
            zmp[2] = 0.;

            diff_vv(zmp,3,pCenter,temp1); // zmp - pCenter

            qCenter_bar[0] = qCenter[0];
            qCenter_bar[1] = -qCenter[1];
            qCenter_bar[2] = -qCenter[2];
            qCenter_bar[3] = -qCenter[3];

            QTtransform(qCenter_bar, temp1, zmp_local); // qCenter_bar*(zmp-pCenter)

//            X_ZMP_Local = 1000.*zmp_local[0];
//            Y_ZMP_Local = 1000.*zmp_local[1];

//            X_ZMP_Global = 1000.*zmp[0];
//            Y_ZMP_Global = 1000.*zmp[1];
        }
}

void ZMPGraphDialog::on_PB_COMPLIANCE_START_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = AlnumWalk;
    cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_COMPLIANCE_START;
    pLAN->SendCommand(cmd);
}


void ZMPGraphDialog::on_PB_CLEAR_ZMPFOOT_clicked()
{
    qv_zmpx.clear();
    qv_zmpy.clear();
}

void ZMPGraphDialog::on_PB_COMPLIANCE_STOP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = AlnumWalk;
    cmd.COMMAND_DATA.USER_COMMAND = APPROACHBOX_COMPLIANCE_STOP;
    pLAN->SendCommand(cmd);
}

void ZMPGraphDialog::on_pushButton_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = AlnumMissionDoor;
    cmd.COMMAND_DATA.USER_COMMAND = MISSIONDOOR_SAVE;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
    pLAN->SendCommand(cmd);
}

void ZMPGraphDialog::on_pushButton_2_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = AlnumMissionDoor;
    cmd.COMMAND_DATA.USER_COMMAND = MISSIONDOOR_SAVE;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
    pLAN->SendCommand(cmd);
}
