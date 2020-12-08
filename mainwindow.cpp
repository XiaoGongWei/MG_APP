#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    m_ConfigWidget(parent)
{
    ui->setupUi(this);
    // init Widget and add it
    initWindow();
//    setAttribute(Qt::WA_DeleteOnClose, true);
}

MainWindow::~MainWindow()
{
    if(m_mutiply_data.length() > 0)
        m_mutiply_data.clear();
    delete ui;
}
void MainWindow::closeEvent(QCloseEvent *e)
{
    exit(0);// exit all app processes and threads
}

void MainWindow::AboutApp()
{
    QMessageBox::information(this, tr("About"), tr("This app was creat by XiaoGongWei.\n E-mail: xiaogongwei10@163.com\n Alipay(支付宝) Donate: 270734392@qq.com\n github: github.com/xiaogongwei/MG_APP"));
}

void MainWindow::paintEvent(QPaintEvent *)
{
    QPainter tupian(this);
    QPixmap tu;
    QString bkPath = m_App_floder + "images/BackG1.jpg";
    tu.load(bkPath);
    tupian.drawPixmap(this->rect(),tu);
}


void MainWindow::initWindow()
{
    m_station_path = "C:/Users/Administrator/Desktop/wab20230";// Default directory of observation files
//    m_station_path = "D:/Testdata/KIRI0020.18o";
    m_App_floder = QCoreApplication::applicationDirPath() + PATHSEG;
    ui->textEdit_FilePath->setText(m_station_path);
    m_isRuned = false;
    m_isRunedBatch = false;
    m_Display_Max_line = 99999;
    mp_qtPlot = NULL;
    // fix windows
    setFixedSize(this->width(), this->height());
    setWindowIcon(QIcon("widget.ico"));
    setWindowTitle("MG-APPS v2.0.5");
    // connect signal to slots
    // pushButon
    connect(ui->pushButton_Select, SIGNAL(clicked(bool)), this, SLOT(selectFilePath()));
    connect(ui->pushButton_Run, SIGNAL(clicked(bool)), this, SLOT(RunPPP()));
    connect(ui->pushButton_SPP, SIGNAL(clicked(bool)), this, SLOT(RunSPP()));
    connect(ui->pushButton_RunBatch, SIGNAL(clicked(bool)), this, SLOT(RunPPPBatch()));
    connect(ui->pushButton_Plot, SIGNAL(clicked(bool)), this, SLOT(plotAllRes()));
    // menuBar
    m_otherMenu = menuBar()->addMenu("&Tools");
    // about
    QString aboutPath = m_App_floder + "images/about.ico";
    QAction *AboutAct = new QAction(tr("&About"),this);
    AboutAct->setStatusTip(tr("App was creat by David Xiao."));
    AboutAct->setIcon(QIcon(aboutPath));
    connect(AboutAct,SIGNAL(triggered()),this,SLOT(AboutApp()));
    m_otherMenu->addAction(AboutAct);

    // Configure
    QString cofPath = m_App_floder + "images/config.ico";
    QAction *configureAct = new QAction(tr("&Configure"),this);
    configureAct->setStatusTip(tr("Configure MG_APP."));
    configureAct->setIcon(QIcon(cofPath));
    connect(configureAct,SIGNAL(triggered()), &m_ConfigWidget,SLOT(show()));
    m_otherMenu->addAction(configureAct);

    // status tip
    setStatusTip("MG-APPS is runing.");
    ui->pushButton_Run->setStatusTip("Run PPP in single station.");
    ui->pushButton_RunBatch->setStatusTip("Run PPP in multiply stations.");
    ui->pushButton_SPP->setStatusTip("Run SPP in single station.");
    ui->pushButton_Plot->setStatusTip("After Run you can plot.");
}

void MainWindow::plotAllRes()
{
    if(m_mutiply_data.length() > 0 && m_isRunedBatch && ui->comboBox_RunBatch->count() > 0 )
    {// mutiply stations
        // find QComoBox select station
        QString plotStationName = ui->comboBox_RunBatch->currentText();
        int plot_flag = 0;
        for(int i = 0;i < m_mutiply_names.length();i++)
        {
            QString temp_station_name = m_mutiply_names.at(i);
            if(temp_station_name.compare(plotStationName) == 0)
            {
                plot_flag = i;
                break;
            }
        }
        // plot select station
        PlotGUIData plot_station_data = m_mutiply_data.at(plot_flag);
        plotSigleStation(plot_station_data);
    }
    else if(m_isRuned)
    {// single station
        plotSigleStation(m_single_data);
    }
    else
    {
        WaringBox("After Run, you can Plot.");
        return ;
    }
}

void MainWindow::plotSigleStation(PlotGUIData &station_data)
{
    int revDataLen = station_data.X.length();
    int wnd_dev_pix = 20;// Window deviates from pixels
    QPoint tempPos;
    // juge QWrite2File have data
    if(revDataLen == 0) return ;
    // plot ENU
    QVector< double > xAixsData;
    // get axis of x and use ( station_data.X - station_data.X(end) )
    double true_pos[3] = {0};
    true_pos[0] = station_data.X.at(revDataLen-1);
    true_pos[1] = station_data.Y.at(revDataLen-1);
    true_pos[2] = station_data.Z.at(revDataLen-1);

    PlotGUIData station_data_copy;
    for(int i = 0;i < revDataLen;i++)
    {
        if(station_data.X[i] !=0 && station_data.spp_X[i] !=0)
        {
            xAixsData.append(i);
            station_data_copy.X.append(station_data.X[i] - true_pos[0]);
            station_data_copy.Y.append(station_data.Y[i] - true_pos[1]);
            station_data_copy.Z.append(station_data.Z[i] - true_pos[2]);
            station_data_copy.spp_X.append(station_data.spp_X[i] - true_pos[0]);
            station_data_copy.spp_Y.append(station_data.spp_Y[i] - true_pos[1]);
            station_data_copy.spp_Z.append(station_data.spp_Z[i] - true_pos[2]);
            station_data_copy.clockData.append(station_data.clockData[i]);
            station_data_copy.ZTD_W.append(station_data.ZTD_W[i]);
        }
    }
    station_data_copy.save_file_path = station_data.save_file_path;
    // plot PPP pos
    int save_image_width = 800, save_image_hight = 495;
    QString save_image_folder = station_data.save_file_path + "images"+ PATHSEG;
    QVector< QVector< double > > XX, YY;
    QVector< QString > XY_Names;
    XX.append(xAixsData); YY.append(station_data_copy.X);
    XX.append(xAixsData); YY.append(station_data_copy.Y);
    XX.append(xAixsData); YY.append(station_data_copy.Z);
    XY_Names.append("filter_dX"); XY_Names.append("filter_dY"); XY_Names.append("filter_dZ");
    mp_qtPlot = new QtPlot();
    mp_qtPlot->setAttribute(Qt::WA_DeleteOnClose, true);
    mp_qtPlot->qtPlot2D(XX, YY, XY_Names, "Epoch", "Unit(m)");
    mp_qtPlot->show();
    if(isDirExist(save_image_folder))
    {// save image
        QString file_name = "ppp_dXYZ.png";
        mp_qtPlot->savePng(save_image_folder + file_name, save_image_width, save_image_hight);
    }

    // get pos and move window
    tempPos = mp_qtPlot->pos();
    mp_qtPlot->move(tempPos.x()+0*wnd_dev_pix, tempPos.y()+0*wnd_dev_pix);
    mp_qtPlot->resize(this->width(), (int) (this->width() * 0.618));
    // plot SPP pos
    QVector< QVector< double > > spp_XX, spp_YY;
    QVector< QString > spp_Names;
    spp_XX.append(xAixsData); spp_YY.append(station_data_copy.spp_X);
    spp_XX.append(xAixsData); spp_YY.append(station_data_copy.spp_Y);
    spp_XX.append(xAixsData); spp_YY.append(station_data_copy.spp_Z);
    spp_Names.append("spp_dX"); spp_Names.append("spp_dY"); spp_Names.append("spp_dZ");
    mp_qtPlot = new QtPlot();
    mp_qtPlot->setAttribute(Qt::WA_DeleteOnClose, true);
    mp_qtPlot->qtPlot2D(spp_XX, spp_YY, spp_Names, "Epoch", "Unit(m)");
    mp_qtPlot->show();
    // get pos and move window
    tempPos = mp_qtPlot->pos();
    mp_qtPlot->move(tempPos.x()+ 1*wnd_dev_pix, tempPos.y()+ 1*wnd_dev_pix);
    mp_qtPlot->resize(this->width(), (int) (this->width() * 0.618));
    if(isDirExist(save_image_folder))
    {// save image
        QString file_name = "spp_dXYZ.png";
        mp_qtPlot->savePng(save_image_folder + file_name, save_image_width, save_image_hight);
    }
    // plot Clock
    QVector< QVector< double > > xClock, YClock;
    QVector< QString > clock_Names;
    xClock.append(xAixsData); YClock.append(station_data_copy.clockData);
    clock_Names.append("Base_Receiver_Clock");
    mp_qtPlot = new QtPlot();
    mp_qtPlot->setAttribute(Qt::WA_DeleteOnClose, true);
    mp_qtPlot->qtPlot2D(xClock, YClock, clock_Names, "Epoch", "Unit(m)");
    mp_qtPlot->show();
    // get pos and move window
    tempPos = mp_qtPlot->pos();
    mp_qtPlot->move(tempPos.x()+2*wnd_dev_pix, tempPos.y()+2*wnd_dev_pix);
    mp_qtPlot->resize(this->width(), (int) (this->width() * 0.618));
    if(isDirExist(save_image_folder))
    {// save image
        QString file_name = "Base_Receiver_Clock.png";
        mp_qtPlot->savePng(save_image_folder + file_name, save_image_width, save_image_hight);
    }
    // plot Clock
    QVector< QVector< double > > xZWD, YZWD;
    QVector< QString > ZWD_Names;
    xZWD.append(xAixsData); YZWD.append(station_data_copy.ZTD_W);
    ZWD_Names.append("ZTD");
    mp_qtPlot = new QtPlot();
    mp_qtPlot->setAttribute(Qt::WA_DeleteOnClose, true);
    mp_qtPlot->qtPlot2D(xZWD, YZWD, ZWD_Names, "Epoch", "Unit(m)");
    mp_qtPlot->show();
    // get pos and move window
    tempPos = mp_qtPlot->pos();
    mp_qtPlot->move(tempPos.x()+3*wnd_dev_pix, tempPos.y()+3*wnd_dev_pix);
    mp_qtPlot->resize(this->width(), (int) (this->width() * 0.618));
    if(isDirExist(save_image_folder))
    {// save image
        QString file_name = "ZTD.png";
        mp_qtPlot->savePng(save_image_folder + file_name, save_image_width, save_image_hight);
    }
}

void MainWindow::RunPPPBatch()
{
    m_station_path = ui->textEdit_FilePath->text();
    // clear all station data
    m_isRuned = false;
    m_isRunedBatch = false;
    clearPlotGUIData(m_single_data);// clear old data
    m_mutiply_data.clear();
    m_mutiply_names.clear();
    ui->comboBox_RunBatch->clear();
    // QComoBox
    QString TropDelay = ui->comboBox_TropDelay->currentText(),
            Method = ui->comboBox_Method->currentText(),
            CutAngle_Str = ui->lineEdit_Angle->text(),
            PPPModel_Str = ui->comboBox_PPPModel->currentText();
    double CutAngle = CutAngle_Str.toDouble();
    // QCheckBox
    QString SatSystem = "";
    bool Kinematic = false;
    if(ui->checkBox_GPS->isChecked())
        SatSystem.append("G");
    if(ui->checkBox_GLONASS->isChecked())
        SatSystem.append("R");
    if(ui->checkBox_GAlieo->isChecked())
        SatSystem.append("E");
    if(ui->checkBox_BDS->isChecked())
        SatSystem.append("C");
    if(ui->checkBox_Kinematic->isChecked())
        Kinematic = true;
//    QString Smooth_Str = ui->comboBox_PPP_SMOOTH->currentText();
    QString Smooth_Str = "NoSmooth";
    QString m_Products = "igs";
    if(ui->radioButton_igs->isChecked())
        m_Products = "igs";
    else if(ui->radioButton_cnt->isChecked())
        m_Products = "cnt";

    // run batch stations
    if(!m_station_path.isEmpty())
    {
        ui->textEdit_Display->clear();// clear QTextEdit
        bool isBackBatch = ui->checkBox_Back->isChecked();
        QBatchProcess batchPPP(m_station_path, ui->textEdit_Display, Method, SatSystem, TropDelay, CutAngle, Kinematic, Smooth_Str, isBackBatch, m_Products, PPPModel_Str);
        batchPPP.Run(false);// false represent don't disply every epoch information(ENU or XYZ)
        m_isRunedBatch = batchPPP.isRuned();
        if(m_isRunedBatch)
        {
            ui->comboBox_RunBatch->clear();
            batchPPP.getStoreAllData(m_mutiply_data);// if you want store data before Run set setStoreAllData.
            m_mutiply_names = batchPPP.getStationNames();
            ui->comboBox_RunBatch->addItems(m_mutiply_names);
        }
        else
        {
            ui->comboBox_RunBatch->clear();
            m_mutiply_data.clear();
            m_mutiply_names.clear();
        }
    }
    else
    {
        WaringBox("Please select obsevation floder.");
    }
}

void MainWindow::RunPPP()
{
    m_station_path = ui->textEdit_FilePath->text();
    // clear mutiply stations
    m_isRuned = false;
    m_isRunedBatch = false;
    clearPlotGUIData(m_single_data);// clear old data
    m_mutiply_data.clear();
    m_mutiply_names.clear();
    ui->comboBox_RunBatch->clear();
    // QComoBox
    QString TropDelay = ui->comboBox_TropDelay->currentText(),
            Method = ui->comboBox_Method->currentText(),
            CutAngle_Str = ui->lineEdit_Angle->text(),
            PPPModel_Str = ui->comboBox_PPPModel->currentText();
    double CutAngle = CutAngle_Str.toDouble();
    // QCheckBox
    QString SatSystem = "";
    bool Kinematic = false;
    // Clock difference sequence of receiver
    if(ui->checkBox_GPS->isChecked())
        SatSystem.append("G");
    if(ui->checkBox_GLONASS->isChecked())
        SatSystem.append("R");
    if(ui->checkBox_GAlieo->isChecked())
        SatSystem.append("E");
    if(ui->checkBox_BDS->isChecked())
        SatSystem.append("C");
    if(ui->checkBox_Kinematic->isChecked())
        Kinematic = true;
//    QString Smooth_Str = ui->comboBox_PPP_SMOOTH->currentText();
    QString Smooth_Str = "NoSmooth";
    QString m_Products = "igs";
    if(ui->radioButton_igs->isChecked())
        m_Products = "igs";
    else if(ui->radioButton_cnt->isChecked())
        m_Products = "cnt";
// From Configure file
    // Delete satellites
    QString removeSats = m_ConfigWidget.myConfTranIni.getValue("/MG_APP/deleteSats");
    removeSats.append(";C01;C02;C03;C04;C05");// remove GEO of BeiDou
    // SYS/#/OBS TYPES (Set the PPP dual-frequency observation type)
    QVector<QStringList> ObsTypeSet = getConfObsType();
    // Set Parameters
    QString Qw_Str = m_ConfigWidget.myConfTranIni.getValue("/MG_APP/Qw"),
            Pk_Str = m_ConfigWidget.myConfTranIni.getValue("/MG_APP/Pk"),
            LP_Str = m_ConfigWidget.myConfTranIni.getValue("/MG_APP/LP_precision");
    QStringList Qw_StrList =  Qw_Str.split(";"), Pk_StrList = Pk_Str.split(";"),
            LP_List = LP_Str.split(";");
    QVector<QStringList> Qw_Pk;
    Qw_Pk.append(Qw_StrList); Qw_Pk.append(Pk_StrList); Qw_Pk.append(LP_List);

    if(!m_station_path.isEmpty())
    {
        ui->textEdit_Display->clear();
        if(ui->checkBox_Back->isChecked())
        {
            QPPPBackSmooth  myBkPPP(m_station_path, ui->textEdit_Display, Method, SatSystem, TropDelay, CutAngle, Kinematic, Smooth_Str, m_Products, PPPModel_Str);
            myBkPPP.Run(true);// true represent disply every epoch information(ENU or XYZ)
            m_isRuned = myBkPPP.isRuned();
            if(m_isRuned)
            {
                clearPlotGUIData(m_single_data);
                myBkPPP.getRunResult(m_single_data);
            }
        }
        else
        {
            QPPPModel myPPP(m_station_path, ui->textEdit_Display, Method, SatSystem, TropDelay, CutAngle, Kinematic,
                            Smooth_Str, m_Products, PPPModel_Str, removeSats, ObsTypeSet, Qw_Pk);
            myPPP.Run(true);// true represent disply every epoch information(ENU or XYZ)
            m_isRuned = myPPP.isRuned();
            if(m_isRuned)
            {
                clearPlotGUIData(m_single_data);
                myPPP.getRunResult(m_single_data);
            }
        }

    }
    else
    {
        WaringBox("Please select obsevation floder.");
    }
}

void MainWindow::RunSPP()
{
    m_station_path = ui->textEdit_FilePath->text();
    // clear mutiply stations
    m_isRuned = false;
    m_isRunedBatch = false;
    clearPlotGUIData(m_single_data);// clear old data
    m_mutiply_names.clear();
    ui->comboBox_RunBatch->clear();
    // QComoBox
    QString TropDelay = ui->comboBox_TropDelay->currentText(),
            Method = ui->comboBox_Method->currentText(),
            CutAngle_Str = ui->lineEdit_Angle->text(),
            PPPModel_Str = ui->comboBox_PPPModel->currentText();;
    double CutAngle = CutAngle_Str.toDouble();
    // QCheckBox
    QString SatSystem = "";
    bool Kinematic = false;
    if(ui->checkBox_GPS->isChecked())
        SatSystem.append("G");
    if(ui->checkBox_GLONASS->isChecked())
        SatSystem.append("R");
    if(ui->checkBox_GAlieo->isChecked())
        SatSystem.append("E");
    if(ui->checkBox_BDS->isChecked())
        SatSystem.append("C");
    if(ui->checkBox_Kinematic->isChecked())
        Kinematic = true;
    // QComoBox
    QString Smooth_Str = ui->comboBox_SPP_SMOOTH->currentText(),
            SPP_Model = ui->comboBox_SPP_LC->currentText();

    if(!m_station_path.isEmpty())
    {
        ui->textEdit_Display->clear();
        QSPPModel mySPP(m_station_path, ui->textEdit_Display, Method, SatSystem, TropDelay, CutAngle, Kinematic, Smooth_Str, SPP_Model, PPPModel_Str);
        mySPP.Run(true);// false represent disply every epoch information(ENU or XYZ)
        m_isRuned = mySPP.isRuned();
        if(m_isRuned)
        {
            clearPlotGUIData(m_single_data);
            mySPP.getRunResult(m_single_data);
        }
    }
    else
    {
        WaringBox("Please select obsevation floder.");
    }
}

QVector<QStringList> MainWindow::getConfObsType()
{
    QVector<QStringList> tempConfObs;
    QString SatOBStype;
    QStringList Sat_List;
    // GPS
    SatOBStype = m_ConfigWidget.myConfTranIni.getValue("/MG_APP/GPS_OBS_TYPE");
    Sat_List= SatOBStype.split(";");
    Sat_List.prepend("G");
    Sat_List.removeAll(QString(""));
    if(Sat_List.length() >= 2)  tempConfObs.append(Sat_List);
    //GLONASS
    SatOBStype = m_ConfigWidget.myConfTranIni.getValue("/MG_APP/GLONASS_OBS_TYPE");
    Sat_List= SatOBStype.split(";");
    Sat_List.prepend("R");
    Sat_List.removeAll(QString(""));
    if(Sat_List.length() >= 2)  tempConfObs.append(Sat_List);
    //BDS
    SatOBStype = m_ConfigWidget.myConfTranIni.getValue("/MG_APP/BDS_OBS_TYPE");
    Sat_List= SatOBStype.split(";");
    Sat_List.prepend("C");
    Sat_List.removeAll(QString(""));
    if(Sat_List.length() >= 2)  tempConfObs.append(Sat_List);
    //Galileo
    SatOBStype = m_ConfigWidget.myConfTranIni.getValue("/MG_APP/Galileo_OBS_TYPE");
    Sat_List= SatOBStype.split(";");
    Sat_List.prepend("E");
    Sat_List.removeAll(QString(""));
    if(Sat_List.length() >= 2)  tempConfObs.append(Sat_List);

    return tempConfObs;
}

void MainWindow::selectFilePath()
{
    QFileDialog fileDialog;
    fileDialog.setFileMode(QFileDialog::Directory);
    m_station_path = fileDialog.getExistingDirectory(this, "Open Directory", "", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if(m_station_path.isEmpty())
    {
        QString WarngInfo = "Can not select file path";
        WaringBox(WarngInfo);
        ui->textEdit_FilePath->setText("");
        autoScrollTextEdit(ui->textEdit_Display, WarngInfo);
    }
    else
    {
        QString Display = "Select Path:  " + m_station_path;
        ui->textEdit_FilePath->setText(m_station_path);
        autoScrollTextEdit(ui->textEdit_Display, Display);
    }
}
void MainWindow::clearPlotGUIData(PlotGUIData &station_data)
{
    station_data.X.clear();
    station_data.Y.clear();
    station_data.Z.clear();
    station_data.spp_X.clear();
    station_data.spp_Y.clear();
    station_data.spp_Z.clear();
    station_data.clockData.clear();
    station_data.ZTD_W.clear();
}

// The edit box automatically scrolls, adding one row or more lines at a time.
void MainWindow::autoScrollTextEdit(QTextEdit *textEdit,QString &add_text)
{
    //Add line character and refresh edit box.
    QString insertText = add_text + ENDLINE;
    textEdit->insertPlainText(insertText);
    //Keep the editor in the last line of the cursor.
    QTextCursor cursor=textEdit->textCursor();
    cursor.movePosition(QTextCursor::End);
    textEdit->setTextCursor(cursor);
    //If you exceed a certain number of lines, empty it.
    if(textEdit->document()->lineCount() > m_Display_Max_line)
    {
        textEdit->clear();
    }
}

void MainWindow::WaringBox(QString info)
{
    QMessageBox::warning(this, "Warning", info);
}

bool MainWindow::isDirExist(QString fullPath)
{
    if(fullPath.isEmpty()) return false;

    QDir dir(fullPath);
    if(dir.exists())
    {
      return true;
    }
    else
    {
       bool ok = dir.mkpath(fullPath);//Create a multi-level directory
       return ok;
    }
}
