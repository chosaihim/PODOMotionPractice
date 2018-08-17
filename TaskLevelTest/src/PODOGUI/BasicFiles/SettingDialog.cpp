#include "SettingDialog.h"
#include "ui_SettingDialog.h"

enum _SET_JointSequentialNumber_0{
    S_RHY = 0, S_RHR, S_RHP, S_RKN, S_RAP, S_RAR, S_RWH, S_NUM_0
};
enum _SET_JointSequentialNumber_1{
    S_LHY = 0, S_LHR, S_LHP, S_LKN, S_LAP, S_LAR, S_LWH, S_WST, S_NUM_1
};
enum _SET_JointSequentialNumber_2{
    S_RSP = 0, S_RSR, S_RSY, S_REB, S_RWY, S_RWP, S_RWY2, S_RHAND, S_NUM_2
};
enum _SET_JointSequentialNumber_3{
    S_LSP = 0, S_LSR, S_LSY, S_LEB, S_LWY, S_LWP, S_LWY2, S_LHAND, S_NUM_3
};
enum _SET_JointSequentialNumber_4{
    S_NKP1, S_NKY, S_NKP2, SNKR, S_NUM_4
};
enum _SET_JointSequentialNumber_5{
    S_RF1, S_RF2, S_RF3, S_RF4, S_NUM_5
};
enum _SET_JointSequentialNumber_6{
    S_LF1, S_LF2, S_LF3, S_LF4, S_NUM_6
};



QString table_0_joint_name[S_NUM_0] = {
    "RHY", "RHR", "RHP", "RKN", "RAP", "RAR", "RWH"
};
QString table_1_joint_name[S_NUM_1] = {
    "LHY", "LHR", "LHP", "LKN", "LAP", "LAR", "LWH", "WST"
};
QString table_2_joint_name[S_NUM_2] = {
    "RSP", "RSR", "RSY", "REB", "RWY", "RWP", "RWY2", "RHand"
};
QString table_3_joint_name[S_NUM_3] = {
    "LSP", "LSR", "LSY", "LEB", "LWY", "LWP", "LWY2", "LHand"
};
QString table_4_joint_name[S_NUM_4] = {
    "NKP1", "NKY", "NKP2", "NKR"
};
QString table_5_joint_name[S_NUM_5] = {
    "RF1", "RF2", "RF3", "RF4"
};
QString table_6_joint_name[S_NUM_6] = {
    "LF1", "LF2", "LF3", "LF4"
};


const struct {
    int tw;
    int row;
} TW_ROW_Pairs[NO_OF_JOINTS] = {
    {0, 0}, {0, 1}, {0, 2}, {0, 3}, {0, 4}, {0, 5},
    {1, 0}, {1, 1}, {1, 2}, {1, 3}, {1, 4}, {1, 5},
    {2, 0}, {2, 1}, {2, 2}, {2, 3}, {2, 4}, {2, 5},
    {3, 0}, {3, 1}, {3, 2}, {3, 3}, {3, 4}, {3, 5},
    {1, 7},
    {2, 6}, {2, 7}, {3, 6}, {3, 7},
    {0, 6}, {1, 6},
    {4, 0}, {4, 1}, {4, 2}, {4, 3},
    {5, 0}, {5, 1}, {5, 2}, {5, 3},
    {6, 0}, {6, 1}, {6, 2}, {6, 3}
};




SettingDialog::SettingDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SettingDialog)
{
    ui->setupUi(this);


    InitTable(ui->TW_0, table_0_joint_name, S_NUM_0);
    InitTable(ui->TW_1, table_1_joint_name, S_NUM_1);
    InitTable(ui->TW_2, table_2_joint_name, S_NUM_2);
    InitTable(ui->TW_3, table_3_joint_name, S_NUM_3);
    InitTable(ui->TW_4, table_4_joint_name, S_NUM_4);
    InitTable(ui->TW_5, table_5_joint_name, S_NUM_5);
    InitTable(ui->TW_6, table_6_joint_name, S_NUM_6);

    lastSelected = RHY;
    select_working = false;
    ChangeSelectedJoint();

    connect(pLAN, SIGNAL(NewPODOData()), this, SLOT(UpdateSettings()));
}

SettingDialog::~SettingDialog()
{
    delete ui;
}

void SettingDialog::UpdateSettings(){
    int mcId, mcCh, row;
    QTableWidget *tw;
    QString str;

    for(int i=0; i<NO_OF_JOINTS; i++){
        mcId = MC_GetID(i);
        mcCh = MC_GetCH(i);
        row = TW_ROW_Pairs[i].row;

        switch(TW_ROW_Pairs[i].tw){
        case 0:
            tw = ui->TW_0;
            break;
        case 1:
            tw = ui->TW_1;
            break;
        case 2:
            tw = ui->TW_2;
            break;
        case 3:
            tw = ui->TW_3;
            break;
        case 4:
            tw = ui->TW_4;
            break;
        case 5:
            tw = ui->TW_5;
            break;
        case 6:
            tw = ui->TW_6;
            break;
        }

        mSTAT stat = PODO_DATA.CoreData.MCStatus[mcId][mcCh];
        str = "";
        if(PODO_DATA.CoreData.ENCODER[mcId][mcCh].BoardConnection) str += "C  ";
        else                                                      str += "N  ";
        if(stat.b.HIP == 1) str += "H/";
        else                str += "-/";
        if(stat.b.RUN == 1) str += "R/";
        else                str += "-/";
        str += QString().sprintf("%d", stat.b.HME);
        tw->item(row, 0)->setText(str);
        if(stat.b.RUN == 1 && stat.b.HME == 6){
            tw->item(row, 0)->setBackgroundColor(QColor(100, 255, 100));    // green    Home(6) Run(on)
        }else if(stat.b.HME != 0){
            tw->item(row, 0)->setBackgroundColor(QColor(255, 100, 100));    // red      Home(x) Run(x)
        }else{
            tw->item(row, 0)->setBackgroundColor(QColor(255, 255, 100));    // yellow   Home(0)
        }


        str = "";
        if(stat.b.JAM == 1) str += "JAM ";
        if(stat.b.PWM == 1) str += "PWM ";
        if(stat.b.BIG == 1) str += "BIG ";
        if(stat.b.INP == 1) str += "INP ";
        if(stat.b.FLT == 1) str += "FLT ";
        if(stat.b.ENC == 1) str += "ENC ";
        if(stat.b.CUR == 1) str += "CUR ";
        if(stat.b.TMP == 1) str += "TMP ";
        if(stat.b.PS1 == 1) str += "PS1 ";
        if(stat.b.PS2 == 1) str += "PS2 ";
        if(str == ""){
            str = "-";
            tw->item(row, 1)->setBackgroundColor(QColor(255, 255, 255));
        }else{
            tw->item(row, 1)->setBackgroundColor(QColor(255, 100, 100));
        }
        tw->item(row, 1)->setText(str);

        //tw->item(row, 2)->setText(QString().sprintf("%2d", (int)PODO_DATA.CoreData.MCTemperature[mcId]));
        tw->item(row, 2)->setText(QString().sprintf("%2d", (int)PODO_DATA.CoreData.MotorTemperature[mcId][mcCh]));
        //if(PODO_DATA.CoreData.MCTemperature[mcId] > 60)
        if(PODO_DATA.CoreData.MotorTemperature[mcId][mcCh] > 60)
            tw->item(row, 2)->setBackgroundColor(QColor(255, 100, 100));
        else
            tw->item(row, 2)->setBackgroundColor(QColor(255, 255, 255));
    }

}

void SettingDialog::InitTable(QTableWidget *table, QString j_names[], int num){
    QFont tableFont;
    tableFont.setPointSize(8);

    const int item_height = 30;
    const int item_width = 50;
    const int col_0_width = 60;
    const int col_1_width = 100;
    const int col_2_width = 30;


    // Horizontal - Column
    for(int i=0; i<3; i++){
        table->insertColumn(i);
        table->setHorizontalHeaderItem(i, new QTableWidgetItem());
        table->horizontalHeaderItem(i)->setTextAlignment(Qt::AlignHCenter|Qt::AlignVCenter|Qt::AlignCenter);
        table->horizontalHeaderItem(i)->setFont(tableFont);
    }
    table->horizontalHeaderItem(0)->setSizeHint(QSize(col_0_width, item_height));
    table->horizontalHeaderItem(1)->setSizeHint(QSize(col_1_width, item_height));
    table->horizontalHeaderItem(2)->setSizeHint(QSize(col_2_width, item_height));
    table->setColumnWidth(0, col_0_width);
    table->setColumnWidth(1, col_1_width);
    table->setColumnWidth(2, col_2_width);
    table->horizontalHeaderItem(0)->setText("Status");
    table->horizontalHeaderItem(1)->setText("Error");
    table->horizontalHeaderItem(2)->setText("T");

    // Vertical - Row
    for(int i=0; i<num; i++){
        table->insertRow(i);
        table->setRowHeight(i,30);
        table->setVerticalHeaderItem(i, new QTableWidgetItem());
        table->verticalHeaderItem(i)->setText(j_names[i]);
        table->verticalHeaderItem(i)->setSizeHint(QSize(item_width, item_height));
        table->verticalHeaderItem(i)->setTextAlignment(Qt::AlignHCenter|Qt::AlignVCenter|Qt::AlignCenter);
        table->verticalHeaderItem(i)->setFont(tableFont);
    }

    for(int i=0; i<num; i++){
        for(int j=0; j<3; j++){
            table->setItem(i, j, new QTableWidgetItem());
            table->item(i,j)->setTextAlignment(Qt::AlignHCenter|Qt::AlignVCenter|Qt::AlignCenter);
            table->item(i,j)->setFlags(table->item(i,j)->flags() & ~Qt::ItemIsEditable);
            table->item(i,j)->setFont(tableFont);
        }
    }

    table->setMinimumWidth(item_width + col_0_width + col_1_width + col_2_width + 2);
    table->setMaximumWidth(item_width + col_0_width + col_1_width + col_2_width + 2);
    table->setMinimumHeight(30*(num+1) + 2);
    table->setMaximumHeight(30*(num+1) + 2);
    //table->resizeColumnsToContents();

    table->setSelectionBehavior(QAbstractItemView::SelectRows);
    table->setSelectionMode(QAbstractItemView::SingleSelection);
    table->horizontalHeader()->setSectionResizeMode(QHeaderView::Fixed);
    table->verticalHeader()->setSectionResizeMode(QHeaderView::Fixed);
}

void SettingDialog::UnselectOtherTable(int table){
    QTableWidget *tb;

    for(int i=0; i<7; i++){
        if(i == table)
            continue;

        switch(i){
        case 0:
            tb = ui->TW_0;
            break;
        case 1:
            tb = ui->TW_1;
            break;
        case 2:
            tb = ui->TW_2;
            break;
        case 3:
            tb = ui->TW_3;
            break;
        case 4:
            tb = ui->TW_4;
            break;
        case 5:
            tb = ui->TW_5;
            break;
        case 6:
            tb = ui->TW_6;
            break;
        }

        QList<QTableWidgetItem*> itemlist = tb->selectedItems();
        for(int j=0; j<itemlist.size(); j++){
            itemlist[j]->setSelected(false);
        }
    }
}

void SettingDialog::on_TW_0_itemSelectionChanged(){
    if(select_working == true)
        return;

    select_working = true;
    lastSelected = FindLastSelected(0, ui->TW_0->currentRow());
    UnselectOtherTable(0);
    ChangeSelectedJoint();
    select_working = false;
}
void SettingDialog::on_TW_1_itemSelectionChanged(){
    if(select_working == true)
        return;

    select_working = true;
    lastSelected = FindLastSelected(1, ui->TW_1->currentRow());
    UnselectOtherTable(1);
    ChangeSelectedJoint();
    select_working = false;
}
void SettingDialog::on_TW_2_itemSelectionChanged(){
    if(select_working == true)
        return;

    select_working = true;
    lastSelected = FindLastSelected(2, ui->TW_2->currentRow());
    UnselectOtherTable(2);
    ChangeSelectedJoint();
    select_working = false;
}
void SettingDialog::on_TW_3_itemSelectionChanged(){
    if(select_working == true)
        return;

    select_working = true;
    lastSelected = FindLastSelected(3, ui->TW_3->currentRow());
    UnselectOtherTable(3);
    ChangeSelectedJoint();
    select_working = false;
}
void SettingDialog::on_TW_4_itemSelectionChanged(){
    if(select_working == true)
        return;

    select_working = true;
    lastSelected = FindLastSelected(4, ui->TW_4->currentRow());
    UnselectOtherTable(4);
    ChangeSelectedJoint();
    select_working = false;
}
void SettingDialog::on_TW_5_itemSelectionChanged(){
    if(select_working == true)
        return;

    select_working = true;
    lastSelected = FindLastSelected(5, ui->TW_5->currentRow());
    UnselectOtherTable(5);
    ChangeSelectedJoint();
    select_working = false;
}
void SettingDialog::on_TW_6_itemSelectionChanged(){
    if(select_working == true)
        return;

    select_working = true;
    lastSelected = FindLastSelected(6, ui->TW_6->currentRow());
    UnselectOtherTable(6);
    ChangeSelectedJoint();
    select_working = false;
}

int SettingDialog::FindLastSelected(int tw, int row){
    for(int i=0; i<NO_OF_JOINTS; i++){
        if(TW_ROW_Pairs[i].tw == tw && TW_ROW_Pairs[i].row == row){
            FILE_LOG(logINFO) << i;
            return i;
        }
    }
    return -1;
}
void SettingDialog::ChangeSelectedJoint(){
    ui->LB_SELECTED->setText("Selected: " + JointNameList[lastSelected]);
}


void SettingDialog::on_BTN_CAN_CHECK_clicked(){
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_INIT_CHECK_DEVICE;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SettingDialog::on_BTN_FIND_HOME_clicked(){
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_INIT_FIND_HOME;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = -1; // all
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SettingDialog::on_BTN_MOVE_JOINT_clicked(){
    if(lastSelected < 0)
        return;

    int id = MC_GetID(lastSelected);
    int ch = MC_GetCH(lastSelected);
    float time = ui->LE_MOVE_TIME->text().toFloat();
    float angle = ui->LE_MOVE_DEGREE->text().toFloat();
    if(lastSelected == RWH || lastSelected == LWH)
        angle /= 200.0;

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
    if(lastSelected == RHAND || lastSelected == LHAND ||
            lastSelected == RF1 || lastSelected == RF2 ||
            lastSelected == RF3 || lastSelected == RF4 ||
            lastSelected == LF1 || lastSelected == LF2 ||
            lastSelected == LF3 || lastSelected == LF4)
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 0;     // absolute
    else
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1;     // relative
    cmd.COMMAND_DATA.USER_PARA_FLOAT[0] = time;     // time(ms)
    cmd.COMMAND_DATA.USER_PARA_FLOAT[1] = angle;	// angle
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_MOTION_MOVE;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SettingDialog::on_BTN_GAIN_OVERRIDE_clicked(){
    if(lastSelected < 0)
        return;

    int id = MC_GetID(lastSelected);
    int ch = MC_GetCH(lastSelected);
    float time = ui->LE_GO_TIME->text().toFloat();
    float gain = ui->LE_GO_GAIN->text().toFloat();

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
    cmd.COMMAND_DATA.USER_PARA_FLOAT[0] = time;
    cmd.COMMAND_DATA.USER_PARA_FLOAT[1] = gain;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_MOTION_GAIN_OVERRIDE;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}


void SettingDialog::on_BTN_EXECUTE_COMMAND_clicked(){
    if(lastSelected < 0)
        return;

    int id = MC_GetID(lastSelected);
    int ch = MC_GetCH(lastSelected);

    USER_COMMAND cmd;
    if(ui->RB_INIT_POS->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON_INIT_FIND_HOME;
    }else if(ui->RB_ENC_ZERO->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_ENCODER_RESET;
    }else if(ui->RB_FET_ON->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1;     //on
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON_INIT_FET_ONOFF;
    }else if(ui->RB_FET_OFF->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 0;     //off
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON_INIT_FET_ONOFF;
    }else if(ui->RB_CTRL_ON->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1;     //on
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF;
    }else if(ui->RB_CTRL_OFF->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 0;     //off
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF;
    }else if(ui->RB_SW_COMP->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;        
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 0;     //complementary
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON_ATTR_SWITCHING_MODE;
    }else if(ui->RB_SW_NON_COMP->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1;     //non-complementary
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON_ATTR_SWITCHING_MODE;
    }else if(ui->RB_FRIC_ON->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1;     //on
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON_ATTR_FRICTION_COMPENSATION;
    }else if(ui->RB_FRIC_OFF->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 0;     //off
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON_ATTR_FRICTION_COMPENSATION;
    }else if(ui->RB_MODE_POS->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 0;     //position
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON_ATTR_CONTROL_MODE;
    }else if(ui->RB_MODE_PWM->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 3;     //pwm
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON_ATTR_CONTROL_MODE;
    }else if(ui->RB_ERROR_CLEAR->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;        
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 0;     // only error clear
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON_MOTION_ERROR_CLEAR;
    }else if(ui->RB_JOINT_RECOVER->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1;     // error clear + joint recovery
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON_MOTION_ERROR_CLEAR;
    }
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}




