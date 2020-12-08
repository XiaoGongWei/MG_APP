#include "QSPPModel.h"

QStringList QSPPModel::searchFilterFile(QString floder_path, QStringList filers)
{
    QDir floder_Dir(floder_path);
    QStringList all_file_path;
    floder_Dir.setFilter(QDir::Files | QDir::NoSymLinks);
    floder_Dir.setNameFilters(filers);
    QFileInfoList file_infos = floder_Dir.entryInfoList();
    for(int i = 0;i < file_infos.length();i++)
    {
        QFileInfo file_info = file_infos.at(i);
        if(file_info.fileName() != "." || file_info.fileName() != "..")
            all_file_path.append(file_info.absoluteFilePath());
    }
    return all_file_path;
}

//Destructed function
QSPPModel::~QSPPModel()
{

}

//Run the specified directory file
QSPPModel::QSPPModel(QString files_path,  QTextEdit *pQTextEdit, QString Method, QString Satsystem,
                     QString TropDelay, double CutAngle, bool isKinematic, QString Smooth_Str,
                     QString SPP_Model, QString pppmodel_t)
{
    //Run the specified directory file initialization variable
    initVar();
    m_run_floder = files_path + PATHSEG;
    m_App_floder = QCoreApplication::applicationDirPath() + PATHSEG;
    // Display for GUI
    mp_QTextEditforDisplay = pQTextEdit;
    // find files
    QStringList tempFilters, OFileNamesList;
    // find obs files
    tempFilters.clear();
    tempFilters.append("*.*o");
    tempFilters.append("*.*O");
    OFileNamesList = searchFilterFile(m_run_floder, tempFilters);

    // get want file
    // o file
    QString OfileName = "";
    if (!OFileNamesList.isEmpty())
    {
        OfileName = OFileNamesList.at(0);
        m_haveObsFile = true;
    }
    else
    {
        ErroTrace("QPPPModel::QPPPModel: Cant not find Obsvertion file.");
        m_haveObsFile = false;
    }


    // use defualt config
    setConfigure(Method, Satsystem, TropDelay, CutAngle, isKinematic, Smooth_Str, SPP_Model, pppmodel_t);
    // save data to QPPPModel
    initSPPModel(OfileName);
}


void QSPPModel::setConfigure(QString Method, QString Satsystem, QString TropDelay, double CutAngle, bool isKinematic, QString Smooth_Str,
                             QString SPP_Model, QString pppmodel_t)
{
    // Configure
    m_Solver_Method = Method;// m_Solver_Method value can be "SRIF" or "Kalman"
    m_CutAngle = CutAngle;// (degree)
    m_SatSystem = Satsystem;// GPS, GLONASS, BDS, and Galieo are used respectively: the letters G, R, C, E
    m_TropDelay = TropDelay;// The tropospheric model m_TropDelay can choose Sass, Hopfiled, UNB3m
    m_isKinematic = isKinematic;
    m_PPPModel_Str = pppmodel_t;
    //Setting up the file system SystemStr:"G"(Turn on the GPS system);"GR":(Turn on the GPS+GLONASS system);"GRCE"(all open), etc.
    //GPS, GLONASS, BDS, and Galieo are used respectively: the letters G, R, C, E
    setSatlitSys(Satsystem);
    m_sys_str = Satsystem;
    m_sys_num = getSystemnum();

    if("Smooth" == Smooth_Str)
    {
        m_KalmanClass.setSmoothRange(QKalmanFilter::KALMAN_SMOOTH_RANGE::SMOOTH);
        m_SRIFAlgorithm.setSmoothRange(SRIFAlgorithm::SRIF_SMOOTH_RANGE::SMOOTH);
        m_isSmoothRange = true;// Whether to use phase smoothing pseudorange for SPP
    }
    else if("NoSmooth" == Smooth_Str)
    {
        m_KalmanClass.setSmoothRange(QKalmanFilter::KALMAN_SMOOTH_RANGE::NO_SMOOTH);
        m_SRIFAlgorithm.setSmoothRange(SRIFAlgorithm::SRIF_SMOOTH_RANGE::NO_SMOOTH);
        m_isSmoothRange = false;// Whether to use phase smoothing pseudorange for SPP
    }

    if("P_IF" == SPP_Model)
    {

        if(isKinematic)
        {
            m_KalmanClass.setModel(QKalmanFilter::KALMAN_MODEL::SPP_KINEMATIC);// set Kinematic model
            m_SRIFAlgorithm.setModel(SRIFAlgorithm::SRIF_MODEL::SPP_KINEMATIC);
            m_minSatFlag = 5;// Dynamic Settings 5, Static Settings 1 in setConfigure()
        }
        else
        {
            m_KalmanClass.setModel(QKalmanFilter::KALMAN_MODEL::SPP_STATIC);// set Static model
            m_SRIFAlgorithm.setModel(SRIFAlgorithm::SRIF_MODEL::SPP_STATIC);
            m_minSatFlag = 5;// Dynamic Settings 5, Static Settings 1 in setConfigure()
        }
    }
    else if("PL_IF" == SPP_Model)
    {
        if(isKinematic)
        {
            m_KalmanClass.setModel(QKalmanFilter::KALMAN_MODEL::PPP_KINEMATIC);// set Kinematic model
            m_SRIFAlgorithm.setModel(SRIFAlgorithm::SRIF_MODEL::PPP_KINEMATIC);
        }
        else
        {
            m_KalmanClass.setModel(QKalmanFilter::KALMAN_MODEL::PPP_STATIC);// set Static model
            m_SRIFAlgorithm.setModel(SRIFAlgorithm::SRIF_MODEL::PPP_STATIC);
            m_minSatFlag = 1;// Dynamic Settings 5, Static Settings 1 in setConfigure()
        }
    }
    else
    {
        m_KalmanClass.setModel(QKalmanFilter::KALMAN_MODEL::SPP_KINEMATIC);// set Kinematic model
        m_KalmanClass.setModel(QKalmanFilter::KALMAN_MODEL::PPP_KINEMATIC);// set Kinematic model
    }

    if(Method == "KalmanOu")
    {
        m_KalmanClass.setFilterMode(QKalmanFilter::KALMAN_FILLTER::KALMAN_MrOu);
    }

    if(m_PPPModel_Str.contains("Ion", Qt::CaseInsensitive))
    {
        setPPPModel(PPP_MODEL::PPP_Combination);
        m_KalmanClass.setPPPModel(PPP_MODEL::PPP_Combination);
        m_writeFileClass.setPPPModel(PPP_MODEL::PPP_Combination);
    }
    else if(m_PPPModel_Str.contains("Uncomb", Qt::CaseInsensitive))
    {
        setPPPModel(PPP_MODEL::PPP_NOCombination);
        m_KalmanClass.setPPPModel(PPP_MODEL::PPP_NOCombination);
        m_writeFileClass.setPPPModel(PPP_MODEL::PPP_NOCombination);
    }
    else
        m_haveObsFile = false;

}

//Initialization operation
void QSPPModel::initVar()
{
    for (int i = 0;i < 3;i++)
        m_ApproxRecPos[0] = 0;
    m_OFileName = "";
    multReadOFile = 1000;
    m_leapSeconds = 0;
    FlagN = 0;
    m_run_floder = "";
    m_haveObsFile = false;
    m_isRuned = false;
    m_isInitSPP = false;
    m_isConnect = false;
    m_save_images_path = "";
    m_iswritre_file = false;
    m_minSatFlag = 4;// Dynamic Settings 5, Static Settings 1 in setConfigure()
    m_isSmoothRange = false;// Whether to use phase smoothing pseudorange for SPPz
}

//Constructor
void QSPPModel::initSPPModel(QString OFileName)
{
    if(!m_haveObsFile) return ;// if not have observation file.
//Set up multi-system data
//Setting up the file system, SystemStr:"G"(Turn on the GPS system);"GR":(Turn on GPS system to turn on GPS+GLONASS system);"GRCE"(All open), etc.
//GPS, GLONASS, BDS, and Galieo are used respectively: the letters G, R, C, E
    setSatlitSys(m_SatSystem);
//Initial various classes
    m_ReadOFileClass.setObsFileName(OFileName);
    m_ReadTropClass.setTropFileNames("./gpt2_5.grd","GMF", m_TropDelay);// Default tropospheric model projection function GMF

    QStringList tempFilters, NavFileNamesList, AtxFileNamesList;

//Save file name
    m_OFileName = OFileName;
//Various class settings
    int obsTime[5] = {0};
    double Seconds = 0.0;
    m_ReadOFileClass.getApproXYZ(m_ApproxRecPos);//Obtain the approximate coordinates of the O file
    m_ReadOFileClass.getFistObsTime(obsTime,Seconds);//Get the initial observation time

//Search products and download
    int GPS_Week = 0, GPS_Day = 0;
    qCmpGpsT.YMD2GPSTime(obsTime[0],obsTime[1],obsTime[2],obsTime[3],obsTime[4],Seconds, &GPS_Week, &GPS_Day);
    // find floder nav
    // find nav files
    QString year2Str = QString::number(obsTime[0]).mid(2,2),
            NavfileName = "";
    tempFilters.clear();
    tempFilters.append("*."+ year2Str + "N");
    tempFilters.append("*."+ year2Str + "n");
    tempFilters.append("*."+ year2Str + "p");
    tempFilters.append("*."+ year2Str + "P");
    NavFileNamesList = searchFilterFile(m_run_floder, tempFilters);
    if(!NavFileNamesList.isEmpty())
    {
       NavfileName = NavFileNamesList.at(0);
    }
    //If you don't have a Nav product, you need to make up these products
    if(NavfileName.isEmpty())
    {
        int Doy = qCmpGpsT.YearAccDay(obsTime[0], obsTime[1], obsTime[2]),
                Year = obsTime[0];
        connectHost();
        NavfileName = downNavFile(m_run_floder, Year, Doy);
    }

    if(!NavfileName.isEmpty())
    {
        // read nav
        m_QReadGPSN.setFileName(NavfileName);
        m_QReadGPSN.getAllData();
    }
    else
    {
        m_haveObsFile = false;
        ErroTrace("QPPPModel::QPPPModel: Cant not find Navigation file, try download.");
        NavfileName = "";
        return ;
    }
    //Read the required calculation file module (time consuming)
    m_ReadTropClass.getAllData();//Read grd files for later tropospheric calculations
//Get skip seconds
    m_leapSeconds = qCmpGpsT.getLeapSecond(obsTime[0],obsTime[1],obsTime[2],obsTime[3],obsTime[4],Seconds);
}

bool QSPPModel::connectHost()
{
    if(m_isConnect) return true;
    QString ftp_link = "cddis.gsfc.nasa.gov", user_name = "anonymous", user_password = "";//ftp information
    int Port = 21;
    ftpClient.FtpSetUserInfor(user_name, user_password);
    ftpClient.FtpSetHostPort(ftp_link, Port);
    m_isConnect = true;
    return true;
}

//download Get erp file
// productType: "erp"
// GPS_Week: is GPS week.
QString QSPPModel::downNavFile(QString store_floder_path, int Year, int doy)
{
    QString nav_path_name = "";
    if(!m_isConnect)
        return nav_path_name;
    QString p_short_path = "/pub/gps/data/campaign/mgex/daily/rinex3/", p_productCompany = "brdm";// set path of products
    QString n_short_path = "/pub/gps/data/daily/", n_productCompany = "brdc";// set path of products

    QString p_path_string = "", p_name = "";
    QString n_path_string = "", n_name = "";
    //net file path
    p_path_string.append(p_short_path);
    QString year_str = QString::number(Year);
    p_path_string.append(year_str);
    p_path_string.append("/");

    n_path_string.append(n_short_path);
    n_path_string.append(year_str);
    n_path_string.append("/");
    n_path_string.append(n_productCompany); //append "brdc" floder
    // doy is 3 char example 002,099
    QString doy_str = "";
    if(doy < 10)
        doy_str = "00" + QString::number(doy);
    else if(doy>=10 && doy < 100)
        doy_str = "0" + QString::number(doy);
    else
        doy_str = QString::number(doy);
    p_path_string.append(doy_str);
    p_path_string.append("/");
    // append 18p floder
    QString yearp = year_str.mid(2,2) + "p";
    QString yearn = year_str.mid(2,2) + "n";
    p_path_string.append(yearp);
    p_path_string.append("/");


    // file name
    p_name.append(p_productCompany);
    p_name.append(doy_str);
    p_name.append("0");
    QString suffix_name_p = "." + yearp + ".Z";
    p_name.append(suffix_name_p);

    n_name.append(n_productCompany);
    n_name.append(doy_str);
    n_name.append("0");
    QString suffix_name_n = "." + yearn + ".Z";
    n_name.append(suffix_name_n);
    // down load begin
    bool is_down_p = false, is_down_n = false;
    QString disPlayQTextEdit = "download start!";
    autoScrollTextEdit(mp_QTextEditforDisplay, disPlayQTextEdit);// display for QTextEdit
    QString p_net_path_name = p_path_string + p_name,
            p_temp_local_file = store_floder_path + p_name;
    QString n_net_path_name = n_path_string + n_name,
            n_temp_local_file = store_floder_path + n_name;
    if(!ftpClient.FtpGet(p_net_path_name, p_temp_local_file))
    {
        disPlayQTextEdit = "download: " + p_net_path_name + " bad!";
        autoScrollTextEdit(mp_QTextEditforDisplay, disPlayQTextEdit);// display for QTextEdit
        is_down_p = false;
        if(!ftpClient.FtpGet(n_net_path_name, n_temp_local_file))
        {
            disPlayQTextEdit = "download: " + n_net_path_name + " bad!";
            autoScrollTextEdit(mp_QTextEditforDisplay, disPlayQTextEdit);// display for QTextEdit
            is_down_n = false;
        }
        else
        {
            disPlayQTextEdit = "download: " + n_net_path_name + " success!";
            autoScrollTextEdit(mp_QTextEditforDisplay, disPlayQTextEdit);// display for QTextEdit
            is_down_n = true;
        }
    }
    else
    {
        disPlayQTextEdit = "download: " + p_net_path_name + " success!";
        autoScrollTextEdit(mp_QTextEditforDisplay, disPlayQTextEdit);// display for QTextEdit
        is_down_p = true;
    }
    // save products file to fileList
    QString temp_local_file;
    if(is_down_p)
        temp_local_file = p_temp_local_file;
    else if(is_down_n)
        temp_local_file = n_temp_local_file;
    // If the download fail
    if(!is_down_p && !is_down_n)
    {
        m_haveObsFile = false;
        disPlayQTextEdit = "download: gbm or igs products bad, Procedure will terminate!";
        autoScrollTextEdit(mp_QTextEditforDisplay, disPlayQTextEdit);// display for QTextEdit
    }

    // umcompress ".Z"
    QFileInfo fileinfo(temp_local_file);
    if(0 != fileinfo.size())
    {
        MyCompress compress;
        compress.UnCompress(temp_local_file, store_floder_path);
    }
    int extend_name = temp_local_file.lastIndexOf(".");
    // return file path + name
    nav_path_name = temp_local_file.mid(0, extend_name);
    bool is_down_nav = (is_down_p || is_down_n);
    if(!is_down_nav) nav_path_name = "";
    return nav_path_name;
}

// get matrix B and observer L(QSPPModel::Obtaining_equation() is same function as QPPPModel::Obtaining_equation() )
void QSPPModel::Obtaining_equation(QVector< SatlitData > &currEpoch, double *ApproxRecPos, MatrixXd &mat_B, VectorXd &Vct_L, MatrixXd &mat_P, bool isSmoothRange)
{
    int epochLenLB = currEpoch.length();
    MatrixXd B, P;
    VectorXd L, sys_len;

    sys_len.resize(m_sys_str.length());
    B.resize(epochLenLB,3 + m_sys_num);
    P.resize(epochLenLB,epochLenLB);
    L.resize(epochLenLB);

    sys_len.setZero();
    B.setZero();
    L.setZero();
    P.setIdentity();
    bool is_find_base_sat = false;

    for (int i = 0; i < epochLenLB;i++)
    {
        SatlitData oneSatlit = currEpoch.at(i);
        double li = 0,mi = 0,ni = 0,p0 = 0,dltaX = 0,dltaY = 0,dltaZ = 0;
        dltaX = oneSatlit.X - ApproxRecPos[0];
        dltaY = oneSatlit.Y - ApproxRecPos[1];
        dltaZ = oneSatlit.Z - ApproxRecPos[2];
        p0 = qSqrt(dltaX*dltaX+dltaY*dltaY+dltaZ*dltaZ);
        li = dltaX/p0;mi = dltaY/p0;ni = dltaZ/p0;
        //Computational B matrix
        //P3 pseudorange code matrix
        B(i, 0) = li;B(i, 1) = mi;B(i, 2) = ni; B(i, 3) = -1;// base system satlite clk must exist
        // debug by xiaogongwei 2019.04.03 for ISB
        for(int k = 1; k < m_sys_str.length();k++)
        {
            if(m_sys_str[k] == oneSatlit.SatType)
            {
                B(i,3+k) = -1;
                sys_len[k] = 1;//good no zeros cloumn in B,sys_lenmybe 0 1 1 0
            }
        }
        // is exist base system satlite clk
        if(m_sys_str[0] == oneSatlit.SatType)
            is_find_base_sat = true;

        //Calculating the L matrix
        double dlta = 0;//Correction of each
        dlta =  - oneSatlit.StaClock + oneSatlit.SatTrop - oneSatlit.Relativty -
            oneSatlit.Sagnac - oneSatlit.TideEffect - oneSatlit.AntHeight;
        //Each correction correction pseudorange code PP3
        if(isSmoothRange)
        {// add by xiaogongwei 2018.11.20
            L(i) = p0 - oneSatlit.PP3_Smooth + dlta;
            // Computing weight matrix P
//            if(oneSatlit.UTCTime.epochNum > 30 && oneSatlit.PP3_Smooth_NUM < 30 )//
//                P(i, i) = 0.0001;
//            else
                P(i, i) = 1 / oneSatlit.PP3_Smooth_Q;//Smooth pseudorange right????
        }
        else
        {
            L(i) = p0 - oneSatlit.PP3 + dlta;
            // Computing weight matrix P
            P(i, i) = oneSatlit.SatWight;//Pseudo-range right
        }

    }//B, L is calculated
    // save data to mat_B
    mat_B = B;
    Vct_L = L;
    mat_P = P;
    // debug by xiaogongwei 2019.04.04
    int no_zero = sys_len.size() - 1 - sys_len.sum();
    if(no_zero > 0 || !is_find_base_sat)
    {
        int new_hang = B.rows() + no_zero, new_lie = B.cols(), flag = 0;
        if(!is_find_base_sat) new_hang++; // check base system satlite clk is exist
        mat_B.resize(new_hang,new_lie);
        mat_P.resize(new_hang,new_hang);
        Vct_L.resize(new_hang);
        mat_B.setZero();
        Vct_L.setZero();
        mat_P.setIdentity();
        // check base system satlite clk is exist
        if(!is_find_base_sat)
        {
            for(int i = 0;i < B.rows();i++)
                B(i, 3) = 0;
            mat_B(mat_B.rows() - 1, 3) = 1;
        }
        mat_B.block(0,0,B.rows(),B.cols()) = B;
        mat_P.block(0,0,P.rows(),P.cols()) = P;
        Vct_L.head(L.rows()) = L;
        for(int i = 1; i < sys_len.size();i++)
        {
            if(0 == sys_len[i])
            {
                mat_B(epochLenLB+flag, 3+i) = 1;// 3 is conntain [dx,dy,dz,mf]
                flag++;
            }

        }
    }//if(no_zero > 0)
}

void QSPPModel::SimpleSPP(QVector < SatlitData > &prevEpochSatlitData, QVector < SatlitData > &epochSatlitData, double *spp_pos)
{
    double p_HEN[3] = {0};
    m_ReadOFileClass.getAntHEN(p_HEN);//Get the antenna high
    GPSPosTime epochTime;//Obtaining observation time
    if(epochSatlitData.length() > 0)
        epochTime = epochSatlitData.at(0).UTCTime;//Obtaining observation time
    else
        return;

    Vector3d tempPos3d, diff_3d;
    tempPos3d[0] = spp_pos[0]; tempPos3d[1] = spp_pos[1]; tempPos3d[2] = spp_pos[2];
    QVector< SatlitData > store_currEpoch;
    int max_iter = 20;
    for(int iterj = 0; iterj < max_iter; iterj++)
    {
        QVector< SatlitData > currEpoch;
        // get every satilite data
        for (int i = 0;i < epochSatlitData.length();i++)
        {
            SatlitData tempSatlitData = epochSatlitData.at(i);//Store calculated corrected satellite data
            if(!isInSystem(tempSatlitData.SatType))
                continue;
//Test whether the carrier and pseudorange are abnormal and terminate in time.
            if(!(tempSatlitData.L1&&tempSatlitData.L2&&tempSatlitData.C1&&tempSatlitData.C2))
                continue;
            //Obtain the coordinates and clock of the epoch satellite from the NAV data data
            double pXYZ[3] = {0},pdXYZ[3] = {0}, stalitClock = 0;//Unit m
            // Note: Time is the signal transmission time(m_PrnGpst - tempSatlitData.C2/M_C - tempSatlitData.StaClock/M_C)
            getNAVPos(epochTime.Year,epochTime.Month,epochTime.Day,
                      epochTime.Hours,epochTime.Minutes,epochTime.Seconds,
                      tempSatlitData.C2/M_C,tempSatlitData.PRN,
                      tempSatlitData.SatType,pXYZ,&stalitClock,pdXYZ);//Obtain the precise ephemeris coordinates of the satellite launch time (here the satellite clock error tempSatlitData.StaClock/M_C needs to be subtracted, otherwise the convergence gap is 20cm)
            tempSatlitData.X = pXYZ[0];tempSatlitData.Y = pXYZ[1];tempSatlitData.Z = pXYZ[2];
            tempSatlitData.StaClock = stalitClock;
            //Calculate the satellite's high sitting angle (as the receiver approximates the target)
            double EA[2]={0};
            getSatEA(tempSatlitData.X,tempSatlitData.Y,tempSatlitData.Z,spp_pos,EA);
            tempSatlitData.EA[0] = EA[0];tempSatlitData.EA[1] = EA[1];
            EA[0] = EA[0]*MM_PI/180;EA[1] = EA[1]*MM_PI/180;//Go to the arc to facilitate the calculation below
            tempSatlitData.SatWight = 0.01;// debug xiaogongwei 2018.11.16
            if( spp_pos[0] !=0 ) getWight(tempSatlitData);//Set the weight of the satellite . debug by xiaogongwei 2019.04.24
//Test the state of the precise ephemeris and the clock difference and whether the carrier and pseudorange are abnormal, and terminate the XYZ or the satellite with the clock difference of 0 in time.
            if (!(tempSatlitData.X&&tempSatlitData.Y&&tempSatlitData.Z&&tempSatlitData.StaClock))
                continue;
//Quality control (height angle pseudorange difference)
            if (qAbs(tempSatlitData.C1 - tempSatlitData.C2) > 50)
                continue;
            if(spp_pos[0] !=0 && tempSatlitData.EA[0] < m_CutAngle)
                continue;
//At the time of SPP, the five satellites with poor GEO orbit in front of Beidou are not removed.
//          if(tempSatlitData.SatType == 'C' && tempSatlitData.PRN <=5 )
//              continue;
//Calculate the wavelength (mainly for multiple systems)
            double F1 = tempSatlitData.Frq[0],F2 = tempSatlitData.Frq[1];
            if(F1 == 0 || F2 == 0) continue;//Frequency cannot be 0
            //Computational relativity correction
            double relative = 0;
            if(spp_pos[0] !=0 ) relative = getRelativty(pXYZ,spp_pos,pdXYZ);
            tempSatlitData.Relativty = relative;
            //Calculate the autobiographic correction of the earth
            double earthW = 0;
            earthW = getSagnac(tempSatlitData.X,tempSatlitData.Y,tempSatlitData.Z,spp_pos);
            tempSatlitData.Sagnac = 0;
            if(spp_pos[0] !=0 ) tempSatlitData.Sagnac = earthW;
            //Calculate the total tropospheric delay!!!
            double MJD = qCmpGpsT.computeJD(epochTime.Year,epochTime.Month,epochTime.Day,
                epochTime.Hours,epochTime.Minutes,epochTime.Seconds) - 2400000.5;//Simplified Julian Day
            //Calculate and save the annual accumulation date
            double TDay = qCmpGpsT.YearAccDay(epochTime.Year,epochTime.Month,epochTime.Day);
            double p_BLH[3] = {0},mf = 0, TropDelay = 0, TropZHD = 0;
            qCmpGpsT.XYZ2BLH(spp_pos[0], spp_pos[1], spp_pos[2], p_BLH);
            if(spp_pos[0] !=0 ) getTropDelay(MJD,TDay,EA[0],p_BLH,&mf, NULL, &TropDelay, &TropZHD);
            tempSatlitData.SatTrop = TropDelay;
            tempSatlitData.StaTropMap = 0;
            tempSatlitData.UTCTime.TropZHD = TropZHD; //Calculate zenith dry delay ZHD
            if(spp_pos[0] !=0 ) tempSatlitData.StaTropMap = mf;
            //Calculate antenna high offset correction  Antenna Height
            tempSatlitData.AntHeight = 0;
            if( spp_pos[0] !=0 )
                tempSatlitData.AntHeight = p_HEN[0]*qSin(EA[0]) + p_HEN[1]*qCos(EA[0])*qSin(EA[1]) + p_HEN[2]*qCos(EA[0])*qCos(EA[1]);
            //Receiver L1 L2 offset correction
            double Lamta1 = M_C/F1,Lamta2 = M_C/F2;
            tempSatlitData.L1Offset = 0;
            tempSatlitData.L2Offset = 0;
            //Satellite antenna phase center correction
            tempSatlitData.SatL1Offset = 0.0;
            tempSatlitData.SatL2Offset = 0.0;
            //Calculate tide correction
            tempSatlitData.TideEffect = 0;
            //Calculate antenna phase winding
            tempSatlitData.AntWindup = 0;
            //Computation to eliminate ionospheric pseudorange and carrier combinations (here absorbed receiver carrier deflection and WindUp)  add SatL1Offset and SatL1Offset by xiaogongwei 2019.04.12
            double alpha1 = (F1*F1)/(F1*F1 - F2*F2),alpha2 = (F2*F2)/(F1*F1 - F2*F2);
            tempSatlitData.LL1 = Lamta1*(tempSatlitData.L1 + tempSatlitData.L1Offset + tempSatlitData.SatL1Offset - tempSatlitData.AntWindup);
            tempSatlitData.LL2 = Lamta2*(tempSatlitData.L2 + tempSatlitData.L2Offset + tempSatlitData.SatL2Offset - tempSatlitData.AntWindup);
            tempSatlitData.CC1 = tempSatlitData.C1 + Lamta1*tempSatlitData.L1Offset + Lamta1*tempSatlitData.SatL1Offset;
            tempSatlitData.CC2 = tempSatlitData.C2 + Lamta2 *tempSatlitData.L2Offset + Lamta2*tempSatlitData.SatL2Offset;

            tempSatlitData.LL3 = alpha1*tempSatlitData.LL1 - alpha2*tempSatlitData.LL2;//Eliminate ionospheric carrier LL3
            tempSatlitData.PP3 = alpha1*tempSatlitData.CC1 - alpha2*tempSatlitData.CC2;//Eliminate ionospheric carrier PP3
            // save data to currEpoch
            currEpoch.append(tempSatlitData);
        }
        // judge satilite number large 4
        if(currEpoch.length() <= 4)
        {
            memset(spp_pos, 0, 4*sizeof(double));// debug by xiaogongwei 2019.09.25
            epochSatlitData = currEpoch;// debug by xiaogongwei 2019.04.10
            return ;
        }
        // get equation
        MatrixXd mat_B, mat_P;
        VectorXd Vct_L, Xk;
        Vector3d XYZ_Pos;
        Obtaining_equation( currEpoch, spp_pos, mat_B, Vct_L, mat_P, false);// debug xiaogongwei 2018.11.16
        // slover by least square
        Xk = (mat_B.transpose()*mat_P*mat_B).inverse()*mat_B.transpose()*mat_P*Vct_L;
        XYZ_Pos[0] = tempPos3d[0] + Xk[0];
        XYZ_Pos[1] = tempPos3d[1] + Xk[1];
        XYZ_Pos[2] = tempPos3d[2] + Xk[2];
        diff_3d = XYZ_Pos - tempPos3d;
        tempPos3d = XYZ_Pos;// save slover pos
        // update spp_pos
        spp_pos[0] = XYZ_Pos[0]; spp_pos[1] = XYZ_Pos[1]; spp_pos[2] = XYZ_Pos[2];
        // debug by xiaogongwei 2018.11.17
        if(diff_3d.cwiseAbs().maxCoeff() < 1)
        {
            spp_pos[3] = Xk[3];// save base clk
            store_currEpoch = currEpoch;
            break;
        }
        if(diff_3d.cwiseAbs().maxCoeff() > 2e7 || !isnormal(diff_3d[0]) || iterj == max_iter - 1)
        {
            memset(spp_pos, 0, 4*sizeof(double));
            epochSatlitData = currEpoch;// debug by xiaogongwei 2019.09.25
            return ;
        }
    }// for(int iterj = 0; iterj < 20; iterj++)
// add Pesudorange smoothed by xiaogongwei 2018.11.20
    MatrixXd mat_B, mat_P;
    VectorXd Vct_L, Xk_smooth;
    //Monitor satellite quality and cycle slip
//    getGoodSatlite(prevEpochSatlitData,store_currEpoch, m_CutAngle);
    if(store_currEpoch.length() < m_minSatFlag)
    {
        memset(spp_pos, 0, 4*sizeof(double));// debug by xiaogongwei 2019.09.25
        epochSatlitData = store_currEpoch;// debug by xiaogongwei 2019.04.10
        return ;
    }

    if(m_isSmoothRange)//
    {
        m_QPseudoSmooth.SmoothPesudoRange(prevEpochSatlitData, store_currEpoch);
        Obtaining_equation( store_currEpoch, spp_pos, mat_B, Vct_L, mat_P, true);// debug xiaogongwei 2018.11.16
        // slover by least square
        Xk_smooth = (mat_B.transpose()*mat_P*mat_B).inverse()*mat_B.transpose()*mat_P*Vct_L;
        // update spp_pos
        spp_pos[0] += Xk_smooth[0]; spp_pos[1] += Xk_smooth[1]; spp_pos[2] += Xk_smooth[2];
        // use spp_pos update  mat_B  Vct_L
        Obtaining_equation( store_currEpoch, spp_pos, mat_B, Vct_L, mat_P, true);// debug xiaogongwei 2019.03.28
    }
    else
    {// Safe operation
        for(int i = 0; i < store_currEpoch.length(); i++)
        {
            store_currEpoch[i].PP3_Smooth = store_currEpoch[i].PP3;
            store_currEpoch[i].PP3_Smooth_NUM = 1;
            store_currEpoch[i].PP3_Smooth_Q = 1 / store_currEpoch[i].SatWight;
        }
        Obtaining_equation( store_currEpoch, spp_pos, mat_B, Vct_L, mat_P, false);
    }

    // Qulity control. add by xiaogongwei 2019.05.06
    VectorXd delate_flag;
//    int max_iter1 = 10;
    // Assuming that SPP has only one gross error, use if is not while, filtering uses while loop to eliminate Debug by xiaogongwei  2019.05.06
    while(m_qualityCtrl.VtPVCtrl_C(mat_B, Vct_L, mat_P, delate_flag, store_currEpoch.length()))//
    {
        QVector<int> del_val;
        int sat_len = store_currEpoch.length();
        for(int i = sat_len - 1; i >= 0;i--)
        {
            if(0 != delate_flag[i])
                del_val.append(i);
        }
//        max_iter1--;
//        if(max_iter1 <= 0) break;
        // delete gross Errors
        int del_len = del_val.length();
        if(sat_len - del_len >= 5)
        {
            for(int i = 0; i < del_len;i++)
                store_currEpoch.remove(del_val[i]);
            sat_len = store_currEpoch.length();// update epochLenLB

            //update spp_pos
            Obtaining_equation( store_currEpoch, spp_pos, mat_B, Vct_L, mat_P, m_isSmoothRange);
            MatrixXd mat_Q = (mat_B.transpose()*mat_P*mat_B).inverse();
            VectorXd x_solver = mat_Q*mat_B.transpose()*mat_P*Vct_L;
            spp_pos[0] += x_solver[0]; spp_pos[1] += x_solver[1]; spp_pos[2] += x_solver[2];
        }
        else
        {
            memset(spp_pos, 0, 4*sizeof(double));// debug by xiaogongwei 2019.09.25
            break;
        }
    }
// change epochSatlitData !!!!!!
    epochSatlitData = store_currEpoch;
}

//Read O files, sp3 files, clk files, and various error calculations, Kalman filtering ......................
//isDisplayEveryEpoch represent is disply every epoch information?(ENU or XYZ)
void QSPPModel::Run(bool isDisplayEveryEpoch)
{
    if(!m_haveObsFile) return ;// if not have observation file.
    QTime myTime; // set timer
    myTime.start();// start timer
    //Externally initialize fixed variables to speed up calculations
    double p_HEN[3] = {0};//Get the antenna high
    m_ReadOFileClass.getAntHEN(p_HEN);
    //Traversing data one by one epoch, reading O file data
    QString disPlayQTextEdit = "";// display for QTextEdit
    QVector < SatlitData > prevEpochSatlitData;//Store the satellite data of an epoch, use the cycle hop detection (put it on the top or read multReadOFile epochs, and the life cycle will expire when reading)
    double spp_pos[4] = {0};// store SPP pos and filtter spp
    Vector3d spp_vct;// save spp pos
    spp_vct.fill(0);

    while (!m_ReadOFileClass.isEnd())
    {
        QVector< QVector < SatlitData > > multepochSatlitData;//Store multiple epochs
        m_ReadOFileClass.getMultEpochData(multepochSatlitData,multReadOFile);//Read multReadOFile epochs
//Multiple epoch cycles
        for (int n = 0; n < multepochSatlitData.length();n++)
        {
//            qDebug() << FlagN;
            if(FlagN == 5073)
            {
                int a = 0;
            }
            QVector< SatlitData > epochSatlitData;//Temporary storage of uncalculated data for each epoch satellite
            epochSatlitData = multepochSatlitData.at(n);
            if(epochSatlitData.length() == 0) continue;
            GPSPosTime epochTime = epochSatlitData.at(0).UTCTime;//Obtain the observation time (the epoch stores the observation time for each satellite)
            epochTime.epochNum = FlagN;
            //Set the epoch of the satellite
            for(int i = 0;i < epochSatlitData.length();i++)
                epochSatlitData[i].UTCTime.epochNum = FlagN;
            // use SPP for approximat position
            spp_vct.fill(0);
            spp_pos[0] = spp_vct[0]; spp_pos[1] = spp_vct[1]; spp_pos[2] = spp_vct[2];
            SimpleSPP(prevEpochSatlitData, epochSatlitData, spp_pos);// prevEpochSatlitData And calculate the zenith dry delay ZHD
            spp_vct[0] = spp_pos[0]; spp_vct[1] = spp_pos[1]; spp_vct[2] = spp_pos[2];

            //can't spp
            if(epochSatlitData.length() < m_minSatFlag || spp_pos[0] == 0)
            {
                prevEpochSatlitData.clear();
                // display clock jump
                disPlayQTextEdit = "Valid Satellite Number: " + QString::number(epochSatlitData.length()) + ENDLINE +
                        "Waring: ***************Satellite number not sufficient*****************";
                autoScrollTextEdit(mp_QTextEditforDisplay, disPlayQTextEdit);// display for QTextEdit
                // translation to ENU
                VectorXd ENU_Vct;
                Vector3d spp_vct;
                int param_len = epochSatlitData.length() + 32;
                ENU_Vct.resize(param_len);
                ENU_Vct.fill(0);
                spp_vct.fill(0);
                saveResult2Class(ENU_Vct, spp_vct, epochTime, epochSatlitData, FlagN);
                FlagN++;
                continue;
            }

            //Monitor satellite quality and cycle slip
            getGoodSatlite(prevEpochSatlitData,epochSatlitData, m_CutAngle);

            //Replace the total tilt delay with the tilt dry delay and the wet delay as the parameter estimate
            if(m_KalmanClass.getModel() == QKalmanFilter::KALMAN_MODEL::PPP_KINEMATIC || m_KalmanClass.getModel() == QKalmanFilter::KALMAN_MODEL::PPP_STATIC)//m_KalmanClass.getModel() != QKalmanFilter::KALMAN_MODEL::SPP
            {
                //Calculate the total tropospheric delay!!!
                double MJD = qCmpGpsT.computeJD(epochTime.Year,epochTime.Month,epochTime.Day,
                    epochTime.Hours,epochTime.Minutes,epochTime.Seconds) - 2400000.5;//Simplified Julian Day
                //Calculate and save the annual accumulation date
                double TDay = qCmpGpsT.YearAccDay(epochTime.Year,epochTime.Month,epochTime.Day);
                double p_BLH[3] = {0}, mf = 0;
                qCmpGpsT.XYZ2BLH(spp_pos[0], spp_pos[1], spp_pos[2], p_BLH);
                for(int i = 0; i < epochSatlitData.length();i++)
                {
                    if(spp_pos[0] !=0)
                    {
                        double TropDelay1 = epochSatlitData[i].SatTrop, EA[2] = {0};
                        EA[0] = epochSatlitData[i].EA[0]*MM_PI/180;
                        EA[1] = epochSatlitData[i].EA[1]*MM_PI/180;//Go to the arc to facilitate the calculation below
                        getTropDelay(MJD,TDay,EA[0],p_BLH,&mf, &TropDelay1, NULL);
                        epochSatlitData[i].SatTrop = TropDelay1;
                        epochSatlitData[i].StaTropMap = mf;
                    }
                }
            }

            //Satellite number not sufficient
            if(epochSatlitData.length() < m_minSatFlag)
            {
                prevEpochSatlitData.clear();
                // display clock jump
                disPlayQTextEdit = "Valid Satellite Number: " + QString::number(epochSatlitData.length()) + ENDLINE +
                        "Waring: ***************Satellite number not sufficient*****************";
                autoScrollTextEdit(mp_QTextEditforDisplay, disPlayQTextEdit);// display for QTextEdit
                // translation to ENU
                VectorXd ENU_Vct;
                Vector3d spp_vct;
                int param_len = epochSatlitData.length() + 32;
                ENU_Vct.resize(param_len);
                ENU_Vct.fill(0);
                spp_vct.fill(0);
                saveResult2Class(ENU_Vct, spp_vct, epochTime, epochSatlitData, FlagN);
                FlagN++;
                continue;
            }

            // Choose solve method Kalman or SRIF
            MatrixXd P;
            VectorXd X;//DX, dY, dZ, dVt (receiver clock difference)[dx,dy,dz,dClock]
            X.resize(5+epochSatlitData.length());
            X.setZero();
            if (!m_Solver_Method.compare("SRIF", Qt::CaseInsensitive))
                m_SRIFAlgorithm.SRIFforStatic(prevEpochSatlitData,epochSatlitData,spp_pos,X,P);
            else
                m_KalmanClass.KalmanforStatic(prevEpochSatlitData,epochSatlitData,spp_pos,X,P);
//Save the last epoch satellite data
            prevEpochSatlitData = epochSatlitData;

//Output calculation result(print result)
            // display every epoch results
            if(isDisplayEveryEpoch)
            {
                int Valid_SatNumber = epochSatlitData.length();
                // display epoch number
                disPlayQTextEdit = "Epoch Number: " + QString::number(FlagN);
                autoScrollTextEdit(mp_QTextEditforDisplay, disPlayQTextEdit);// display for QTextEdit
                //
                disPlayQTextEdit = "GPST: " + QString::number(epochTime.Hours) + ":" + QString::number(epochTime.Minutes)
                        + ":" + QString::number(epochTime.Seconds) + ENDLINE + "Satellite number: " + QString::number(epochSatlitData.length());
                autoScrollTextEdit(mp_QTextEditforDisplay, disPlayQTextEdit);// display for QTextEdit
                // display ENU or XYZ
                disPlayQTextEdit = "Valid Satellite Number: " + QString::number(Valid_SatNumber) + ENDLINE
                        + "Estimated coordinates: [ " + QString::number(spp_pos[0], 'f', 4) + "," + QString::number(spp_pos[1], 'f', 4) + ","
                        + QString::number(spp_pos[2], 'f', 4) + " ]" + ENDLINE;
                autoScrollTextEdit(mp_QTextEditforDisplay, disPlayQTextEdit);// display for QTextEdit
            }

//Save each epoch X data to prepare for writing a file
            // translation to ENU
            VectorXd ENU_Vct;
            ENU_Vct = X;// [dx,dy,dz,dTrop,dClock,N1,N2,...Nn]
            //ENU_Vct[0] = P(1,1); ENU_Vct[1] = P(2,2); ENU_Vct[2] = P(3,3);
            ENU_Vct[0] = spp_pos[0]; ENU_Vct[1] = spp_pos[1]; ENU_Vct[2] = spp_pos[2];
            saveResult2Class(ENU_Vct, spp_vct, epochTime, epochSatlitData, FlagN, &P);
            FlagN++;// epcoh number ++

        }//End of multiple epochs  (int n = 0; n < multepochSatlitData.length();n++)

        // clear multepochSatlitData
        multepochSatlitData.clear();
    }//Arrive at the end of the file End while (!m_ReadOFileClass.isEnd())

// time end
    float m_diffTime = myTime.elapsed() / 1000.0;
    if(isDisplayEveryEpoch)
    {
        disPlayQTextEdit = "The Elapse Time: " + QString::number(m_diffTime) + "s";
        autoScrollTextEdit(mp_QTextEditforDisplay, disPlayQTextEdit);// display for QTextEdit
    }
//Write result to file
    writeResult2File();
    m_isRuned = true;// Determine whether the operation is complete.
}

// The edit box automatically scrolls, adding one row or more lines at a time.
void QSPPModel::autoScrollTextEdit(QTextEdit *textEdit,QString &add_text)
{
    if(textEdit == NULL) return ;
    int m_Display_Max_line = 99999;
    //Add line character and refresh edit box.
    QString insertText = add_text + ENDLINE;
    textEdit->insertPlainText(insertText);
    //Keep the editor in the last line of the cursor.
    QTextCursor cursor=textEdit->textCursor();
    cursor.movePosition(QTextCursor::End);
    textEdit->setTextCursor(cursor);
    textEdit->repaint();
    QApplication::processEvents();
    //If you exceed a certain number of lines, empty it.
    if(textEdit->document()->lineCount() > m_Display_Max_line)
    {
        textEdit->clear();
    }
}

//Set file system SystemStr: "G" (turn on GPS system); "GR": (turn on GPS + GLONASS system); "GRCE" (all open), etc.
//GPS, GLONASS, BDS, and Galieo are used respectively: the letters G, R, C, E
bool QSPPModel::setSatlitSys(QString SystemStr)
{
    bool IsGood = QBaseObject::setSatlitSys(SystemStr);
    //Set O file reading satellite system
    m_ReadOFileClass.setSatlitSys(SystemStr);
    //Set N file reading satellite system
    m_QReadGPSN.setSatlitSys(SystemStr);
    //Set the system of m_writeFileClass
    m_writeFileClass.setSatlitSys(SystemStr);
    //Set kalman filtering system
    m_KalmanClass.setSatlitSys(SystemStr);
    //Set the system of m_writeFileClass
    m_writeFileClass.setSatlitSys(SystemStr);
    //Set the system of m_SRIFAlgorithm
    m_SRIFAlgorithm.setSatlitSys(SystemStr);
    return IsGood;
}

//Obtain the coordinates of the epoch satellite from the SP3 data data
void QSPPModel::getNAVPos(int Year,int Month,int Day,int Hours,int Minutes,double Seconds, double signal_transmission_time,
                          int PRN,char SatType,double *p_XYZ,double *StaClock, double *pdXYZ)
{
    m_QReadGPSN.getSatPos(PRN, SatType, signal_transmission_time, Year, Month, Day, Hours, Minutes, Seconds, StaClock, p_XYZ, pdXYZ);
}

//Earth autobiography correction
double QSPPModel::getSagnac(double X,double Y,double Z,double *approxRecvXYZ)
{//Earth autobiography correction
    double dltaP = M_We*((X - approxRecvXYZ[0])*Y - (Y - approxRecvXYZ[1])*X)/M_C;
    return -dltaP;//Returns the opposite number such that p = p' + dltaP; can be added directly
}

void QSPPModel::getWight(SatlitData &tempSatlitData)
{
    double E = tempSatlitData.EA[0]*MM_PI/180;
    double SatWight = qSin(E)*qSin(E) / M_Zgama_P_square;//Set the weight of the satellite
    switch (tempSatlitData.SatType) {
    case 'R': SatWight = 0.75*SatWight; break;
    case 'C': case 'E': SatWight = 0.3*SatWight; break;
    default:
        break;
    }
    //Five satellites with poor GEO orbit in front of Beidou
    if(tempSatlitData.SatType == 'C' && tempSatlitData.PRN <= 5)
        SatWight = 0.01*SatWight;
    tempSatlitData.SatWight = SatWight;//Set the weight of the satellite debug by xiaogongwei 2019.04.24
}

//Computational relativistic effect
double QSPPModel::getRelativty(double *pSatXYZ,double *pRecXYZ,double *pSatdXYZ)
{
    /*double c = 299792458.0;
    double dltaP = -2*(pSatXYZ[0]*pSatdXYZ[0] + pSatdXYZ[1]*pdXYZ[1] + pSatXYZ[2]*pSatdXYZ[2]) / c;*/
    double b[3] = {0},a = 0,R = 0,Rs = 0,Rr = 0,v_light = 299792458.0,GM=3.9860047e14,dltaP = 0;
    b[0] = pRecXYZ[0] - pSatXYZ[0];
    b[1] = pRecXYZ[1] - pSatXYZ[1];
    b[2] = pRecXYZ[2] - pSatXYZ[2];
    a = pSatXYZ[0]*pSatdXYZ[0] + pSatXYZ[1]*pSatdXYZ[1] + pSatXYZ[2]*pSatdXYZ[2];
    R=qCmpGpsT.norm(b,3);
    Rs = qCmpGpsT.norm(pSatXYZ,3);
    Rr = qCmpGpsT.norm(pRecXYZ,3);
    dltaP=-2*a/M_C + (2*M_GM/qPow(M_C,2))*qLn((Rs+Rr+R)/(Rs+Rr-R));
    return dltaP;//m
}

//Calculate EA, E: satellite elevation angle, A: azimuth
void QSPPModel::getSatEA(double X,double Y,double Z,double *approxRecvXYZ,double *EA)
{//Calculate EA// BUG occurs Since XYZ to BLH is calculated L (earth longitude) is actually opposite when y < 0, x > 0.L = -atan(y/x) error should be L = -atan(-y/x)
    double pSAZ[3] = {0};
    qCmpGpsT.XYZ2SAZ(X,Y,Z,pSAZ,approxRecvXYZ);//Bugs
    EA[0] = (MM_PI/2 - pSAZ[2])*360/(2*MM_PI);
    EA[1] = pSAZ[1]*360/(2*MM_PI);
}


//Using the Sass model There are other models and projection functions that can be used, as well as the GPT2 model.
// ZHD_s: GNSS signal direction dry delay, ZHD: station zenith direction dry delay, ZPD: GNSS signal direction total delay
void QSPPModel::getTropDelay(double MJD,int TDay,double E,double *pBLH,double *mf, double *ZHD_s, double *ZPD, double *ZHD)
{
    //double GPT2_Trop = m_ReadTropClass.getGPT2SasstaMDelay(MJD,TDay,E,pBLH,mf);//The GPT2 model only returns the dry delay estimate and the wet delay function.
    double tropDelayH = 0, tropDelayP = 0, tropDelayZHD = 0;
    if(m_TropDelay.mid(0,1).compare("U") == 0)
    {
        tropDelayH = m_ReadTropClass.getUNB3mDelay(pBLH,TDay,E,mf, &tropDelayP, &tropDelayZHD);//Total delay of the UNB3M model
    }
    else if(m_TropDelay.mid(0,1).compare("S") == 0)
    {
        tropDelayH =  m_ReadTropClass.getGPT2SasstaMDelay(MJD,TDay,E,pBLH,mf, &tropDelayP, &tropDelayZHD);//GPT2 model total delay
    }
    else
    {
        tropDelayH = m_ReadTropClass.getGPT2HopfieldDelay(MJD,TDay,E,pBLH,mf, &tropDelayP, &tropDelayZHD);//GPT2 model total delay
    }

    // juge tropDely is nan or no Meaningless

    if(qAbs(tropDelayH) > 100  || qAbs(pBLH[2]) > 50000)
        tropDelayH = 1e-6;
    else if(!isnormal(tropDelayH))
        tropDelayH = 1e-6;

    if(qAbs(tropDelayP) > 100  || qAbs(pBLH[2]) > 50000)
        tropDelayP = 1e-6;
    else if(!isnormal(tropDelayP))
        tropDelayP = 1e-6;

    if(qAbs(tropDelayZHD) > 100  || qAbs(pBLH[2]) > 50000)
        tropDelayZHD = 1e-6;
    else if(!isnormal(tropDelayZHD))
        tropDelayZHD = 1e-6;
    if(ZHD) *ZHD = tropDelayZHD;
    if(ZHD_s)  *ZHD_s = tropDelayH;
    if(ZPD)  *ZPD = tropDelayP;
    return ;
}

//Detect cycle hops: return pLP is a three-dimensional array, the first is the W-M combination (N2-N1 < 3.5) The second number ionospheric residual (<0.3) The third is (lamt2*N2-lamt1*N1 < 3.5)
bool QSPPModel::CycleSlip(const SatlitData &oneSatlitData,double *pLP)
{//
    if (oneSatlitData.L1*oneSatlitData.L2*oneSatlitData.C1*oneSatlitData.C2 == 0)//Determine whether it is dual-frequency data
        return false;
    double F1 = oneSatlitData.Frq[0],F2 = oneSatlitData.Frq[1];//Get the frequency of this satellite
    double Lamta1 = M_C/F1,Lamta2 = M_C/F2;
    //MW Combination(1)
//    double NL12 = ((F1-F2)/(F1+F2))*(oneSatlitData.C1/Lamta1 + oneSatlitData.C2/Lamta2) - (oneSatlitData.L1 - oneSatlitData.L2);
    //MW Combination(2)
    double lamdaMW = M_C/(F1 - F2);
    double NL12 = (oneSatlitData.L1 - oneSatlitData.L2) - (F1*oneSatlitData.C1 + F2*oneSatlitData.C2)/((F1+F2)*lamdaMW);
    double IocL = Lamta1*oneSatlitData.L1 - Lamta2*oneSatlitData.L2;//Ionospheric delayed residual
    double IocLP = IocL+(oneSatlitData.C1 - oneSatlitData.C2);
    pLP[0] = NL12;
    pLP[1] =IocL;
    pLP[2] = IocLP;
    return true;
}

//Repair receiver clock jump
void QSPPModel::reciveClkRapaire(QVector< SatlitData > &prevEpochSatlitData,QVector< SatlitData > &epochSatlitData)
{
    int preEpochLen = prevEpochSatlitData.length();
    int epochLen = epochSatlitData.length();
    // clock jump repair Debug by xiaogongwei 2019.03.30
    double sum_S = 0.0, k1 = 0.01*M_C, M = 0.0, jump = 0.0;// for repair clock
    int clock_num = 0, check_sat_num = 0;// for repair clock
    for (int i = 0;i < epochLen;i++)
    {
        SatlitData epochData = epochSatlitData.at(i);
        for (int j = 0;j < preEpochLen;j++)
        {
            SatlitData preEpochData = prevEpochSatlitData.at(j);
            if (epochData.PRN == preEpochData.PRN&&epochData.SatType == preEpochData.SatType)
            {
                check_sat_num++;
                double Lamta1 = M_C / preEpochData.Frq[0], Lamta2 = M_C / preEpochData.Frq[1];
                double IocL1 = Lamta1*preEpochData.L1 - Lamta2*preEpochData.L2,
                        IocL2 = Lamta1*epochData.L1 - Lamta2*epochData.L2;//Ionospheric delayed residual
                if(qAbs(IocL2 - IocL1) < M_IR)
                {
                    double dP3 = epochData.PP3 - preEpochData.PP3, dL3 = epochData.LL3 - preEpochData.LL3;
                    double Si = (dP3 - dL3);// (m)
                    // judge clock jump type
                    if(qAbs(dP3) >= 0.001*M_C && m_clock_jump_type == 0)
                        m_clock_jump_type = 1;
                    if(qAbs(dL3) >= 0.001*M_C && m_clock_jump_type == 0)
                        m_clock_jump_type = 2;
                    if(qAbs(Si) >= 0.001*M_C)
                    {
                        sum_S += Si;
                        clock_num++;
                    }
                }
            }
        }
    }
    // repair clock
    double jump_pro = (double)clock_num / check_sat_num;
    if(jump_pro > 0.8)
    {
        M = 1e3*sum_S / (M_C*clock_num);// unit:ms
        if(qAbs(qRound(M) - M) <= 1e-5)
            jump = qRound(M);
        else
            jump = 0;
    }
    if(jump != 0)
    {
        for (int i = 0;i < epochLen;i++)
        {
            SatlitData tempSatlitData = epochSatlitData[i];
            double F1 = tempSatlitData.Frq[0],F2 = tempSatlitData.Frq[1];
            double Lamta1 = M_C/F1,Lamta2 = M_C/F2;
            double alpha1 = (F1*F1)/(F1*F1 - F2*F2),alpha2 = (F2*F2)/(F1*F1 - F2*F2);
            if(m_clock_jump_type == 1)
            {// clock jump type I
                tempSatlitData.L1 = tempSatlitData.L1 + 1e-3*jump*M_C / Lamta1;
                tempSatlitData.L2 = tempSatlitData.L2 + 1e-3*jump*M_C / Lamta2;
                tempSatlitData.LL3 = alpha1*(tempSatlitData.L1 + tempSatlitData.L1Offset + tempSatlitData.SatL1Offset - tempSatlitData.AntWindup)*Lamta1
                        - alpha2*(tempSatlitData.L2 + tempSatlitData.L2Offset + tempSatlitData.SatL2Offset - tempSatlitData.AntWindup)*Lamta2;//Eliminate ionospheric carrier LL3

            }
            else if(m_clock_jump_type == 2)
            {// clock jump type II
                tempSatlitData.C1 = tempSatlitData.C1 - 1e-3*jump*M_C / Lamta1;
                tempSatlitData.C2 = tempSatlitData.C2 - 1e-3*jump*M_C / Lamta2;
                tempSatlitData.PP3 = alpha1*(tempSatlitData.C1 + Lamta1*tempSatlitData.L1Offset + Lamta1*tempSatlitData.SatL1Offset)
                        - alpha2*(tempSatlitData.C2 + Lamta2 *tempSatlitData.L2Offset + Lamta2*tempSatlitData.SatL2Offset);//Eliminate ionospheric carrier PP3
            }
            epochSatlitData[i] = tempSatlitData;
        }
    }
}

//Screening satellites that do not have missing data and detect cycle slips, high quality (height angles, ranging codes, etc.)
void QSPPModel::getGoodSatlite(QVector< SatlitData > &prevEpochSatlitData,QVector< SatlitData > &epochSatlitData,double eleAngle)
{
    int preEpochLen = prevEpochSatlitData.length();
    int epochLen = epochSatlitData.length();
    // clock jump repair Debug by xiaogongwei 2019.03.30
    double sum_S = 0.0, k1 = 1 - 5*1e-5, M = 0.0, jump = 0.0;// for repair clock
    int clock_num = 0;// for repair clock
    for (int i = 0;i < epochLen;i++)
    {
        SatlitData epochData = epochSatlitData.at(i);
        for (int j = 0;j < preEpochLen;j++)
        {
            SatlitData preEpochData = prevEpochSatlitData.at(j);
            if (epochData.PRN == preEpochData.PRN&&epochData.SatType == preEpochData.SatType)
            {
                double Lamta1 = M_C / preEpochData.Frq[0], Lamta2 = M_C / preEpochData.Frq[1];
                double IocL1 = Lamta1*preEpochData.L1 - Lamta2*preEpochData.L2,
                        IocL2 = Lamta1*epochData.L1 - Lamta2*epochData.L2;//Ionospheric delayed residual
                if(qAbs(IocL2 - IocL1) < M_IR)
                {
                    double dP3 = epochData.PP3 - preEpochData.PP3, dL3 = epochData.LL3 - preEpochData.LL3;
                    double Si = (dP3 - dL3) / (M_C*1e-3);
                    if(qAbs(Si) > k1)
                    {
                        if(qAbs(qRound(Si) - Si) > k1)
                        {
                            sum_S += Si;
                            clock_num++;
                        }
                    }

                }
            }
        }
    }
    // repair clock
    if(clock_num > 0)
    {
        M = sum_S / clock_num;
        if(qAbs(qRound(M) - M) <= 1e-5)
            jump = qRound(M);
    }
    if(jump != 0)
    {
        for (int i = 0;i < epochLen;i++)
        {
            epochSatlitData[i].L1 = epochSatlitData[i].L1 + jump*(M_C*1e-3);
            epochSatlitData[i].L2 = epochSatlitData[i].L2 + jump*(M_C*1e-3);
        }
    }
    //Cycle slip detection
    QVector< int > CycleFlag;//Record the position of the weekly jump
    CycleFlag.resize(epochLen);
    for (int i = 0;i < epochLen;i++) CycleFlag[i] = 0;
    for (int i = 0;i < epochLen;i++)
    {
        SatlitData epochData = epochSatlitData.at(i);
        //Recording the weekly jump position data is not 0
        if (!(epochData.L1&&epochData.L2&&epochData.C1&&epochData.C2)) // debug xiaogongwei 2018.11.16
            CycleFlag[i] = -1;
        //The corrections are not zero
        if (!(epochData.X&&epochData.Y&&epochData.Z&&epochData.StaClock)) // debug xiaogongwei 2018.11.16
            CycleFlag[i] = -1;
        //Quality control (height angle, pseudorange difference)
        if (epochData.EA[0] < eleAngle || qAbs(epochData.C1 - epochData.C2) > 50)
            CycleFlag[i] = -1;
        // signal intensity
        if(epochData.SigInten >0 && epochData.SigInten < 0)
            CycleFlag[i] = -1;

        //Cycle slip detection
        for (int j = 0;j < preEpochLen;j++)
        {
            SatlitData preEpochData = prevEpochSatlitData.at(j);
            if (epochData.PRN == preEpochData.PRN&&epochData.SatType == preEpochData.SatType)
            {//Need to judge the system
                double epochLP[3]={0},preEpochLP[3]={0},diffLP[3]={0};
                CycleSlip(epochData,epochLP);
                CycleSlip(preEpochData,preEpochLP);
                for (int n = 0;n < 3;n++)
                    diffLP[n] = qAbs(epochLP[n] - preEpochLP[n]);
                //Determine the weekly jump threshold based on experience.
                // diffLP[2] No longer use 2011.08.24
                if (diffLP[0] > 5 ||diffLP[1] > M_IR||diffLP[2] > 99999 || qAbs(epochData.AntWindup - preEpochData.AntWindup) > 0.3)
                {//Weekly jump
                    CycleFlag[i] = -1;//Save the weekly jump satellite logo
                }
                // Two epoch interval satellite elevation angles cannot jump 3 degrees
//                if(qAbs(epochData.EA[0] - preEpochData.EA[0]) > 3)
//                    CycleFlag[i] = -1;
                break;
            }
            else
            {
                continue;
            }
        }
    }
    //Remove low quality and weekly hop satellites
    QVector< SatlitData > tempEpochSatlitData;
    for (int i = 0;i < epochLen;i++)
    {
        if (CycleFlag.at(i) != -1)
        {
            tempEpochSatlitData.append(epochSatlitData.at(i));
        }
    }
    epochSatlitData = tempEpochSatlitData;
}


void QSPPModel::saveResult2Class(VectorXd X, Vector3d spp_vct, GPSPosTime epochTime, QVector< SatlitData > epochResultSatlitData,
                                 int epochNum, MatrixXd *P)
{
    //Store coordinate data
    RecivePos epochRecivePos;
    epochTime.epochNum = epochNum;
    epochRecivePos.UTCtime = epochTime;

    epochRecivePos.totolEpochStalitNum = epochResultSatlitData.length();
    epochRecivePos.dX = X[0];
    epochRecivePos.dY = X[1];
    epochRecivePos.dZ = X[2];
    epochRecivePos.spp_pos[0] = spp_vct[0];
    epochRecivePos.spp_pos[1] = spp_vct[1];
    epochRecivePos.spp_pos[2] = spp_vct[2];
    m_writeFileClass.allReciverPos.append(epochRecivePos);
    //Save wet delay and receiver clock error
    double epoch_ZHD = 0.0;
    if(epochResultSatlitData.length() >= m_minSatFlag) epoch_ZHD = epochResultSatlitData.at(0).UTCTime.TropZHD;
    ClockData epochRecClock;
    epochRecClock.UTCTime = epochRecivePos.UTCtime;
    // save clock
    int clk_begin = 0;
    memset(epochRecClock.clockData, 0, 6*sizeof(double));
    if(m_KalmanClass.getModel() == QKalmanFilter::KALMAN_MODEL::SPP_STATIC || m_KalmanClass.getModel() == QKalmanFilter::KALMAN_MODEL::SPP_KINEMATIC)
    {
        epochRecClock.ZTD_W = epoch_ZHD;//Only pseudo-range to do the filtering SPP, only the zenith dry delay ZHD
        clk_begin = 3;
    }
    else
    {
        int amb_begin = 4 + m_sys_num;
        epochRecClock.ZTD_W = X(3) + epoch_ZHD;//Storage wet delay + zenith dry delay ZHD
        clk_begin = 4;
        if(getPPPModel() == PPP_MODEL::PPP_NOCombination)
        {
            int sat_num = epochResultSatlitData.length();
            //Save satellite ambiguity
            Ambiguity oneSatAmb;
            for (int i = 0;i < sat_num;i++)
            {
                SatlitData oneSat = epochResultSatlitData.at(i);
                oneSatAmb.PRN = oneSat.PRN;
                oneSatAmb.SatType = oneSat.SatType;
                oneSatAmb.UTCTime = epochRecClock.UTCTime;
                oneSatAmb.isIntAmb = false;
                oneSatAmb.ionL1 = X(i+amb_begin);
                oneSatAmb.Amb1 = X(i+amb_begin+sat_num);
                oneSatAmb.Amb2 = X(i+amb_begin+2*sat_num);
                oneSatAmb.Amb = 0.0;
                oneSatAmb.UTCTime.epochNum = epochNum;
                m_writeFileClass.allAmbiguity.append(oneSatAmb);
            }
        }
        else if(getPPPModel() == PPP_MODEL::PPP_Combination)
        {
            //Save satellite ambiguity
            Ambiguity oneSatAmb;
            for (int i = 0;i < epochResultSatlitData.length();i++)
            {
                SatlitData oneSat = epochResultSatlitData.at(i);
                oneSatAmb.PRN = oneSat.PRN;
                oneSatAmb.SatType = oneSat.SatType;
                oneSatAmb.UTCTime = epochRecClock.UTCTime;
                oneSatAmb.isIntAmb = false;
                oneSatAmb.ionL1 = 0.0;
                oneSatAmb.Amb1 = 0.0;
                oneSatAmb.Amb2 = 0.0;
                oneSatAmb.Amb = X(i+amb_begin);
                oneSatAmb.UTCTime.epochNum = epochNum;
                m_writeFileClass.allAmbiguity.append(oneSatAmb);
            }
        }
    }
    //Store the receiver skew of the first system, and its relative offsets from other systems. GCRE
    for(int i = 0;i < m_sys_str.length();i++)
    {
        switch (m_sys_str.at(i).toLatin1()) {
        case 'G':
            epochRecClock.clockData[0] = X(clk_begin+i);
            break;
        case 'C':
            epochRecClock.clockData[1] = X(clk_begin+i);
            break;
        case 'R':
            epochRecClock.clockData[2] = X(clk_begin+i);
            break;
        case 'E':
            epochRecClock.clockData[3] = X(clk_begin+i);
            break;
        default:
            break;
        }
    }
    m_writeFileClass.allClock.append(epochRecClock);
    // save used satlite Information
    m_writeFileClass.allPPPSatlitData.append(epochResultSatlitData);
    // save solver X
    m_writeFileClass.allSolverX.append(X);
    // save P matrix
    if(P)
        m_writeFileClass.allSloverQ.append(*P);
    else
        m_writeFileClass.allSloverQ.append(MatrixXd::Identity(32,32));
}


void QSPPModel::writeResult2File()
{
    QString product_path = m_run_floder, ambiguit_floder;
    QString floder_name = "Products_" + m_Solver_Method + "_" +m_PPPModel_Str + "_SPP_Static_" + m_sys_str + PATHSEG;
    if(m_isKinematic)
        floder_name = "Products_" + m_Solver_Method + "_" +m_PPPModel_Str + "_SPP_Kinematic_" + m_sys_str + PATHSEG;
    product_path.append(floder_name);
    m_floder_name = floder_name;
    // save images path
    m_save_images_path = product_path;
    ambiguit_floder = product_path + QString("Ambiguity") + PATHSEG;
    m_writeFileClass.WriteEpochPRN(product_path, "Epoch_PRN.txt");
    m_writeFileClass.writeRecivePos2Txt(product_path, "position.txt");
    m_writeFileClass.writePPP2Txt(product_path, "Satellite_info.ppp");
    m_writeFileClass.writeClockZTDW2Txt(product_path, "ZTDW_Clock.txt");
    m_writeFileClass.writeAmbiguity2Txt(ambiguit_floder);//Path is.//Ambiguity//
    m_writeFileClass.writeRecivePosKML(product_path, "position.kml");// gernerate KML
}

// Get operation results( clear QWrite2File::allPPPSatlitData Because the amount of data is too large.)
void QSPPModel::getRunResult(PlotGUIData &plotData)
{
    int dataLen = m_writeFileClass.allReciverPos.length();
    if(dataLen == 0 || !m_isRuned) return ;
    if(!m_save_images_path.isEmpty())  plotData.save_file_path = m_save_images_path;

    for(int i = 0;i < dataLen;i++)
    {
        plotData.X.append(m_writeFileClass.allReciverPos.at(i).dX);
        plotData.Y.append(m_writeFileClass.allReciverPos.at(i).dY);
        plotData.Z.append(m_writeFileClass.allReciverPos.at(i).dZ);
        plotData.spp_X.append(m_writeFileClass.allReciverPos.at(i).spp_pos[0]);
        plotData.spp_Y.append(m_writeFileClass.allReciverPos.at(i).spp_pos[1]);
        plotData.spp_Z.append(m_writeFileClass.allReciverPos.at(i).spp_pos[2]);
        plotData.clockData.append(m_writeFileClass.allClock.at(i).clockData[0]);// GPS clock
        plotData.clockData_bias_1.append(m_writeFileClass.allClock.at(i).clockData[1]);//clock bias maybe is zero
        plotData.clockData_bias_2.append(m_writeFileClass.allClock.at(i).clockData[2]);//clock bias maybe is zero
        plotData.clockData_bias_3.append(m_writeFileClass.allClock.at(i).clockData[3]);//clock bias maybe is zero
        plotData.ZTD_W.append(m_writeFileClass.allClock.at(i).ZTD_W);
    }
}



