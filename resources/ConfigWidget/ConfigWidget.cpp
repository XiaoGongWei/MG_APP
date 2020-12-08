#include "ConfigWidget.h"
#include "ui_ConfigWidget.h"

ConfigWidget::ConfigWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ConfigWidget),
    myConfTranIni("MG_APP.ini")
{
    ui->setupUi(this);
    this->setFixedSize(this->size());// fix window size
    this->setWindowModality(Qt::ApplicationModal);

    // connect
    connect(ui->pushButton_ok, SIGNAL(clicked(bool)), this, SLOT(clickOk()));
    connect(ui->pushButton_cancel, SIGNAL(clicked(bool)), this, SLOT(clickCancel()));
    // init window
    initWidgt();

}

ConfigWidget::~ConfigWidget()
{
    delete ui;
}

void ConfigWidget::initWidgt()
{
// Delete satellites
    QString deleteSats_str = myConfTranIni.getValue("/MG_APP/deleteSats");
    if(!deleteSats_str.isEmpty()) ui->plainTextEdit_delSats->setPlainText(deleteSats_str);
// SYS/#/OBS TYPES (Set the PPP dual-frequency observation type)
    QString SatOBStype;
    QStringList Sat_List;
    //GPS
    SatOBStype = myConfTranIni.getValue("/MG_APP/GPS_OBS_TYPE");
    Sat_List= SatOBStype.split(";"); Sat_List.removeAll(QString(""));
    if(Sat_List.length() >= 2){
        ui->GPS_L1->setPlainText(Sat_List.at(0)); ui->GPS_L2->setPlainText(Sat_List.at(1));
    }
    //GLONASS
    SatOBStype = myConfTranIni.getValue("/MG_APP/GLONASS_OBS_TYPE");
    Sat_List= SatOBStype.split(";"); Sat_List.removeAll(QString(""));
    if(Sat_List.length() >= 2){
        ui->GLO_L1->setPlainText(Sat_List.at(0)); ui->GLO_L2->setPlainText(Sat_List.at(1));
    }
    //BDS
    SatOBStype = myConfTranIni.getValue("/MG_APP/BDS_OBS_TYPE");
    Sat_List= SatOBStype.split(";"); Sat_List.removeAll(QString(""));
    if(Sat_List.length() >= 2){
        ui->BDS_L1->setPlainText(Sat_List.at(0)); ui->BDS_L2->setPlainText(Sat_List.at(1));
    }
    //Galileo
    SatOBStype = myConfTranIni.getValue("/MG_APP/Galileo_OBS_TYPE");
    Sat_List= SatOBStype.split(";"); Sat_List.removeAll(QString(""));
    if(Sat_List.length() >= 2){
        ui->GAL_L1->setPlainText(Sat_List.at(0)); ui->GAL_L2->setPlainText(Sat_List.at(1));
    }
// Set Parameters
    QString Qw_Str, Pk_Str;
    Qw_Str = myConfTranIni.getValue("/MG_APP/Qw");
    Pk_Str = myConfTranIni.getValue("/MG_APP/Pk");
    QStringList Qw_StrList =  Qw_Str.split(";"),
            Pk_StrList = Pk_Str.split(";");
    Qw_StrList.removeAll(QString("")); Pk_StrList.removeAll(QString(""));
    if(Qw_StrList.length() != 0)
    {
        while(Qw_StrList.length() < 5) Qw_StrList.append(QString(""));
        ui->Qw_pos->setPlainText(Qw_StrList.at(0)); ui->Qw_zwd->setPlainText(Qw_StrList.at(1));
        ui->Qw_clk->setPlainText(Qw_StrList.at(2)); ui->Qw_amb->setPlainText(Qw_StrList.at(3));
        ui->Qw_ion->setPlainText(Qw_StrList.at(4));
    }

    if(Pk_StrList.length() != 0)
    {
        while(Pk_StrList.length() < 5) Pk_StrList.append(QString(""));
        ui->Pk_pos->setPlainText(Pk_StrList.at(0)); ui->Pk_zwd->setPlainText(Pk_StrList.at(1));
        ui->Pk_clk->setPlainText(Pk_StrList.at(2)); ui->Pk_amb->setPlainText(Pk_StrList.at(3));
        ui->Pk_ion->setPlainText(Pk_StrList.at(4));
    }

    QString LP_precision = myConfTranIni.getValue("/MG_APP/LP_precision");
    QStringList LP_StrList =  LP_precision.split(";");
    LP_StrList.removeAll(QString(""));
    if(LP_StrList.length() !=0)
    {
        while(LP_StrList.length() < 2) LP_StrList.append(QString(""));
        ui->L_precision->setPlainText(LP_StrList.at(0)); ui->P_precision->setPlainText(LP_StrList.at(1));
    }

}


//slots

void ConfigWidget::clickOk()
{
// only insert string
    QJsonArray myconfArryJson;
    QJsonObject allConfig;
// Comments
    QString Comments = "xiaogongwei create this file. xiaogongwei10@163.com";
    allConfig.insert("comments", Comments);

// Delete satellites
    QString deleteSats = ui->plainTextEdit_delSats->toPlainText();
    deleteSats = deleteSats.trimmed();
    deleteSats = deleteSats.replace("；", ";");
    deleteSats = deleteSats.replace(" ", ";");
    deleteSats = deleteSats.toUpper();
    allConfig.insert("deleteSats", deleteSats.toUtf8().data());
// SYS/#/OBS TYPES (Set the PPP dual-frequency observation type)
    QString gps_obsType = ui->GPS_L1->toPlainText() + ";" + ui->GPS_L2->toPlainText();// GPS
    gps_obsType = gps_obsType.replace("；", ";"); gps_obsType = gps_obsType.replace(" ", ";");
    gps_obsType = gps_obsType.toUpper();
    QString glo_obsType = ui->GLO_L1->toPlainText() + ";" + ui->GLO_L2->toPlainText();// GLONASS
    glo_obsType = glo_obsType.replace("；", ";"); glo_obsType = glo_obsType.replace(" ", ";");
    glo_obsType = glo_obsType.toUpper();
    QString bds_obsType = ui->BDS_L1->toPlainText() + ";" + ui->BDS_L2->toPlainText();// BDS
    bds_obsType = bds_obsType.replace("；", ";"); bds_obsType = bds_obsType.replace(" ", ";");
    bds_obsType = bds_obsType.toUpper();
    QString gal_obsType = ui->GAL_L1->toPlainText() + ";" + ui->GAL_L2->toPlainText();// Galileo
    gal_obsType = gal_obsType.replace("；", ";"); gal_obsType = gal_obsType.replace(" ", ";");
    gal_obsType = gal_obsType.toUpper();

    allConfig.insert("GPS_OBS_TYPE", gps_obsType.toUtf8().data());
    allConfig.insert("GLONASS_OBS_TYPE", glo_obsType.toUtf8().data());
    allConfig.insert("BDS_OBS_TYPE", bds_obsType.toUtf8().data());
    allConfig.insert("Galileo_OBS_TYPE", gal_obsType.toUtf8().data());
// Set Parameters
    QString Qw_List = ui->Qw_pos->toPlainText() + ";" + ui->Qw_zwd->toPlainText() + ";"
            + ui->Qw_clk->toPlainText() + ";" + ui->Qw_amb->toPlainText() + ";"
            + ui->Qw_ion->toPlainText();// Qw String List
    Qw_List = Qw_List.replace("；", ";"); Qw_List = Qw_List.replace(" ", ";");

    QString Pk_List = ui->Pk_pos->toPlainText() + ";" + ui->Pk_zwd->toPlainText() + ";"
            + ui->Pk_clk->toPlainText() + ";" + ui->Pk_amb->toPlainText() + ";"
            + ui->Pk_ion->toPlainText();// Qw String List
    Pk_List = Pk_List.replace("；", ";"); Pk_List = Pk_List.replace(" ", ";");

    QString LP_precision = ui->L_precision->toPlainText() + ";" +ui->P_precision->toPlainText();

    allConfig.insert("Qw", Qw_List.toUtf8().data());
    allConfig.insert("Pk", Pk_List.toUtf8().data());
    allConfig.insert("LP_precision", LP_precision.toUtf8().data());

// add all config to array
     myconfArryJson.append(allConfig);

// write to ini file???
    QMessageBox:: StandardButton result = QMessageBox::information(this, "Configure", "Are you sure about the reconfiguration?",
                                                                   QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
    if(result == QMessageBox::Yes)
    {
        QString iniFileName = myConfTranIni.getFileName(), jsonFileName = "MG_APP.json";
        QStringList iniList = iniFileName.split(".");
        if(iniList.length() > 1) jsonFileName = iniList.at(0) + ".json";
        ConfTranIni::writeJson2Ini(myConfTranIni.getFileName(), myconfArryJson);
        ConfTranIni::writeJson2File(jsonFileName, myconfArryJson);
        initWidgt();
        this->close();
    }
}

void ConfigWidget::clickCancel()
{
    initWidgt();
    this->close();
}
