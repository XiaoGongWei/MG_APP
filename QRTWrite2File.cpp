#include "QRTWrite2File.h"


QRTWrite2File::QRTWrite2File(void)
{
    m_save_filenum = 0;
    m_ZTDW_Clock_head = false;
}


QRTWrite2File::~QRTWrite2File(void)
{
    closeSaveFile();
}

bool QRTWrite2File::setSaveFloder(QString fload_path)
{
    if(!isDirExist(fload_path))
    {
        QString infor = "can not construct floder: " + fload_path;
        ErroTrace(infor);
    }
    QVector<QString> tempallSaveFiles;
    tempallSaveFiles << "position.txt" << "bad_satellites.txt" << "Satellite_info.ppp" << "ZTDW_Clock.txt";
    QDir pathname(fload_path);
    for(int i = 0; i < tempallSaveFiles.length();i++)
    {
        QString tempfileName = tempallSaveFiles.at(i);
        QString fileName = pathname.absoluteFilePath(tempfileName);
        m_QFileVct[i].setFileName(fileName);

        if (!m_QFileVct[i].open(QFile::WriteOnly|QFile::Text))
        {
            QString erroInfo = "Open " + fileName + " File Error!";
            ErroTrace(erroInfo);
            return false;
        }
        m_QTextStreamVct[i].setDevice(&m_QFileVct[i]);
        m_QFileNames[i] = fileName;
        m_save_filenum++;
    }

}

void QRTWrite2File::closeSaveFile()
{
    for(int i = 0;i < m_save_filenum;i++)
    {
        if(m_QFileVct[i].isOpen())
        {
            m_QTextStreamVct[i].flush();
            m_QFileVct[i].flush();
            m_QFileVct[i].close();
            m_QFileNames[i] = "";
        }
    }
}

bool QRTWrite2File::writeRecivePosKML(QString fload_path, QString tempfileName)
{
    if (allReciverPos.length() == 0)//If no data is read
        return false;
    if(!isDirExist(fload_path))
    {
        QString infor = "can not construct floder: " + fload_path;
        ErroTrace(infor);
    }
    QString fileName = fload_path + tempfileName;
    QFile saveFile(fileName);//Saved file
    if (!saveFile.open(QFile::WriteOnly|QFile::Text))
    {
        QString erroInfo = "Open " + fileName + " File Error!";
        ErroTrace(erroInfo);
        return false;
    }
    QTextStream saveFileOut(&saveFile);
    // write head information
    saveFileOut << "<\?xml version=\"1.0\" encoding=\"UTF-8\"\?>" << endl;
    saveFileOut << "<kml xmlns=\"http://earth.google.com/kml/2.1\">" << endl;
    saveFileOut << "<Document>" << endl;
    saveFileOut << "<Style id=\"yellowLineGreenPoly\">" << endl;
    saveFileOut << "<LineStyle>" << endl;
    saveFileOut << "<color>ff0400ff</color>" << endl;
    saveFileOut << "<width>1.4</width>" << endl;
    saveFileOut << "</LineStyle>" << endl;
    saveFileOut << "<PolyStyle>" << endl;
    saveFileOut << "<color>7f00ff00</color>" << endl;
    saveFileOut << "</PolyStyle>" << endl;
    saveFileOut << "</Style>" << endl;
    saveFileOut << "<Placemark>" << endl;
    saveFileOut << "<name>" << endl;
    saveFileOut << "      david_kml" << endl;
    saveFileOut << "</name>" << endl;
    saveFileOut << "<description>" << endl;
    saveFileOut << "      this is create by david_xiao.Email:xiaogongwei10@163.com" << endl;
    saveFileOut << "</description>" << endl;
    saveFileOut << "<styleUrl>#yellowLineGreenPoly</styleUrl>" << endl;
    saveFileOut << "<LineString>" << endl;
    saveFileOut << "<coordinates>" << endl;

    int lenRecivePos = allReciverPos.length();
    double temp_PI = 3.1415926535897932385;
    for (int i = 0;i <lenRecivePos;i++ )
    {
        RecivePos oneRecivePos = allReciverPos.at(i);
        if(oneRecivePos.dX == 0) continue;
        double pXYZ[3] = {oneRecivePos.dX, oneRecivePos.dY, oneRecivePos.dZ},
                pBLH[3] = {0};
        m_qcmpClass.XYZ2BLH(pXYZ, pBLH);
        saveFileOut << QString::number( pBLH[1]*180 / temp_PI, 'f', 12 ) << ","
                << QString::number( pBLH[0]*180 / temp_PI, 'f', 12 ) << ","
                << QString::number( pBLH[2], 'f', 12 )
                << endl;
    }
    saveFileOut << "</coordinates>" << endl;
    saveFileOut << "</LineString>" << endl;
    saveFileOut << "</Placemark>" << endl;
    saveFileOut << "</Document>" << endl;
    saveFileOut << "</kml>" << endl;
    saveFileOut.flush();
    saveFile.close();
    return true;
}

////The calculation result is written to txt
bool QRTWrite2File::writeRecivePos2Txt(const RecivePos oneRecivePos, MatrixXd *Qmat)
{
    int _flag = findFilenameFlag("position.txt");
    if(_flag == -1) return false;

    QTextStream &saveFileOut = m_QTextStreamVct[_flag];

    saveFileOut.setFieldWidth(10);
    saveFileOut<<oneRecivePos.UTCtime.epochNum;
    saveFileOut.setFieldWidth(2);
    saveFileOut<<": ";
    //Output year, month, day, hour, minute, second
    saveFileOut.setFieldWidth(4);
    saveFileOut<<QString::number(oneRecivePos.UTCtime.Year);
    saveFileOut.setFieldWidth(1);
    saveFileOut<<"-";
    saveFileOut.setFieldWidth(2);
    saveFileOut<<QString::number(oneRecivePos.UTCtime.Month);
    saveFileOut.setFieldWidth(1);
    saveFileOut<<"-";
    saveFileOut.setFieldWidth(2);
    saveFileOut<<QString::number(oneRecivePos.UTCtime.Day);
    saveFileOut.setFieldWidth(1);
    saveFileOut<<" ";
    saveFileOut.setFieldWidth(2);
    saveFileOut<<QString::number(oneRecivePos.UTCtime.Hours);
    saveFileOut.setFieldWidth(1);
    saveFileOut<<":";
    saveFileOut.setFieldWidth(2);
    saveFileOut<<QString::number(oneRecivePos.UTCtime.Minutes);
    saveFileOut.setFieldWidth(1);
    saveFileOut<<":";
    saveFileOut.setFieldWidth(10);
    saveFileOut<<QString::number(oneRecivePos.UTCtime.Seconds,'f',7);

    //Calculate dE dN dU
    saveFileOut.setFieldWidth(15);
    saveFileOut<<QString::number(oneRecivePos.totolEpochStalitNum);
    // cout SPP Pos
    saveFileOut.setFieldWidth(2);
    saveFileOut<<"  ";
    saveFileOut.setFieldWidth(15);
    saveFileOut<<QString::number(oneRecivePos.spp_pos[0],'f',4);
    saveFileOut.setFieldWidth(2);
    saveFileOut<<"  ";
    saveFileOut.setFieldWidth(15);
    saveFileOut<<QString::number(oneRecivePos.spp_pos[1],'f',4);
    saveFileOut.setFieldWidth(2);
    saveFileOut<<"  ";
    saveFileOut.setFieldWidth(15);
    saveFileOut<<QString::number(oneRecivePos.spp_pos[2],'f',4);
    saveFileOut.setFieldWidth(2);

    // cout PPP pos
    saveFileOut.setFieldWidth(2);
    saveFileOut<<"  ";
    saveFileOut.setFieldWidth(15);
    saveFileOut<<QString::number(oneRecivePos.dX,'f',4);
    saveFileOut.setFieldWidth(2);
    saveFileOut<<"  ";
    saveFileOut.setFieldWidth(15);
    saveFileOut<<QString::number(oneRecivePos.dY,'f',4);
    saveFileOut.setFieldWidth(2);
    saveFileOut<<"  ";
    saveFileOut.setFieldWidth(15);
    saveFileOut<<QString::number(oneRecivePos.dZ,'f',4);
    saveFileOut.setFieldWidth(2);

    if(Qmat)
    {
        // write pos zigama = sqrt(Q(i,i)), i=1,2,3
        saveFileOut.setFieldWidth(2);
        saveFileOut<<"  ";
        saveFileOut.setFieldWidth(15);
        saveFileOut<<QString::number(sqrt(abs((*Qmat)(0,0))),'f',8);
        saveFileOut.setFieldWidth(2);
        saveFileOut<<"  ";
        saveFileOut.setFieldWidth(15);
        saveFileOut<<QString::number(sqrt(abs((*Qmat)(1,1))),'f',8);
        saveFileOut.setFieldWidth(2);
        saveFileOut<<"  ";
        saveFileOut.setFieldWidth(15);
        saveFileOut<<QString::number(sqrt(abs((*Qmat)(2,2))),'f',8);
        saveFileOut.setFieldWidth(2);
    }

    saveFileOut<<endl;
    saveFileOut.flush();

    return true;
}

//Various error corrections are written to the .ppp file
bool QRTWrite2File::writePPP2Txt(const QVector < SatlitData > epochSatlite)
{
    int _flag = findFilenameFlag("Satellite_info.ppp");
    if(_flag == -1) return false;
    QTextStream &saveFileOut = m_QTextStreamVct[_flag];

    int StallitNumbers = epochSatlite.length();
    if(StallitNumbers == 0) return false;
    //Output the first epoch, and start tag">"
    saveFileOut.setFieldWidth(0);
    saveFileOut << ">epoch_num:";
    saveFileOut << epochSatlite.at(0).UTCTime.epochNum;
    saveFileOut << endl;
    //Output header information
    saveFileOut<<"Satellite Number:";
    saveFileOut.setFieldWidth(3);
    saveFileOut<<StallitNumbers;
    saveFileOut<<",(yyyy-mm-dd-hh-mm-ss):";
    saveFileOut.setFieldWidth(4);
    saveFileOut<<QString::number(epochSatlite.at(0).UTCTime.Year);
    saveFileOut.setFieldWidth(1);
    saveFileOut<<"-";
    saveFileOut.setFieldWidth(2);
    saveFileOut<<QString::number(epochSatlite.at(0).UTCTime.Month);
    saveFileOut.setFieldWidth(1);
    saveFileOut<<"-";
    saveFileOut.setFieldWidth(2);
    saveFileOut<<QString::number(epochSatlite.at(0).UTCTime.Day);
    saveFileOut.setFieldWidth(1);
    saveFileOut<<" ";
    saveFileOut.setFieldWidth(2);
    saveFileOut<<QString::number(epochSatlite.at(0).UTCTime.Hours);
    saveFileOut.setFieldWidth(1);
    saveFileOut<<":";
    saveFileOut.setFieldWidth(2);
    saveFileOut<<QString::number(epochSatlite.at(0).UTCTime.Minutes);
    saveFileOut.setFieldWidth(1);
    saveFileOut<<":";
    saveFileOut.setFieldWidth(10);
    saveFileOut<<QString::number(epochSatlite.at(0).UTCTime.Seconds, 'f', 7);
    saveFileOut.setFieldWidth(6);
    saveFileOut<<",ZHD:";
    // write ZHD and Epoch Flag
    SatlitData oneStallit_t = epochSatlite.at(0);
    saveFileOut.setFieldWidth(7);
    saveFileOut<<QString::number(oneStallit_t.UTCTime.TropZHD,'f',4);
    saveFileOut.setFieldWidth(12);
    saveFileOut<<", BadFlag: ";
    saveFileOut.setFieldWidth(3);
    saveFileOut<<QString::number(oneStallit_t.EpochFlag);
    saveFileOut<<endl;
    //Output model correction information
    for (int j = 0;j < StallitNumbers;j++)
    {
        SatlitData oneStallit= epochSatlite.at(j);
        saveFileOut.setFieldWidth(1);
        saveFileOut<<oneStallit.SatType;
        saveFileOut.setFieldWidth(2);
        if (oneStallit.PRN < 10)
            saveFileOut<<"0"+QString::number(oneStallit.PRN);
        else
            saveFileOut<<oneStallit.PRN;

        saveFileOut.setFieldWidth(2);
        saveFileOut<<": ";

        if(getPPPModel() == PPP_MODEL::PPP_Combination)
        {
            // LL3
            saveFileOut.setFieldWidth(14);
            saveFileOut<<QString::number(oneStallit.VLL3,'f',8);
            saveFileOut.setFieldWidth(2);
            saveFileOut<<", ";
            // PP3
            saveFileOut.setFieldWidth(14);
            saveFileOut<<QString::number(oneStallit.VPP3,'f',8);
            saveFileOut.setFieldWidth(2);
            saveFileOut<<", ";

        }
        else if(getPPPModel() == PPP_MODEL::PPP_NOCombination)
        {
            // L1 L2
            saveFileOut.setFieldWidth(14);
            saveFileOut<<QString::number(oneStallit.VL1,'f',8);
            saveFileOut.setFieldWidth(2);
            saveFileOut<<", ";
            saveFileOut.setFieldWidth(14);
            saveFileOut<<QString::number(oneStallit.VL2,'f',8);
            saveFileOut.setFieldWidth(2);
            saveFileOut<<", ";
            // C1 C2
            saveFileOut.setFieldWidth(14);
            saveFileOut<<QString::number(oneStallit.VC1,'f',8);
            saveFileOut.setFieldWidth(2);
            saveFileOut<<", ";
            saveFileOut.setFieldWidth(14);
            saveFileOut<<QString::number(oneStallit.VC2,'f',8);
            // ION and AMB
            saveFileOut.setFieldWidth(2);
            saveFileOut<<", ";
            // ion
            saveFileOut.setFieldWidth(14);
            saveFileOut<<QString::number(oneStallit.ionL1,'f',4);
            saveFileOut.setFieldWidth(2);
            saveFileOut<<", ";
            // amb1 amb2 amb3
            saveFileOut.setFieldWidth(14);
            saveFileOut<<QString::number(oneStallit.Amb1,'f',4);
            saveFileOut.setFieldWidth(2);
            saveFileOut<<", ";
            saveFileOut.setFieldWidth(14);
            saveFileOut<<QString::number(oneStallit.Amb2,'f',4);
            saveFileOut.setFieldWidth(2);
            saveFileOut<<", ";
        }
        saveFileOut.setFieldWidth(14);
        saveFileOut<<QString::number(oneStallit.Amb,'f',4);
        saveFileOut.setFieldWidth(2);
        saveFileOut<<", ";

        saveFileOut.setFieldWidth(14);
        saveFileOut<<QString::number(oneStallit.X,'f',4);
        saveFileOut.setFieldWidth(2);
        saveFileOut<<", ";
        saveFileOut.setFieldWidth(14);
        saveFileOut<<QString::number(oneStallit.Y,'f',4);
        saveFileOut.setFieldWidth(2);
        saveFileOut<<", ";
        saveFileOut.setFieldWidth(14);
        saveFileOut<<QString::number(oneStallit.Z,'f',4);
        saveFileOut.setFieldWidth(2);
        saveFileOut<<", ";
        saveFileOut.setFieldWidth(14);
        saveFileOut<<QString::number(oneStallit.StaClock,'f',4);
        saveFileOut.setFieldWidth(2);
        saveFileOut<<", ";
        saveFileOut.setFieldWidth(14);
        saveFileOut<<QString::number(oneStallit.EA[0],'f',4);
        saveFileOut.setFieldWidth(2);
        saveFileOut<<", ";
        saveFileOut.setFieldWidth(14);
        saveFileOut<<QString::number(oneStallit.EA[1],'f',4);
        saveFileOut.setFieldWidth(2);
        saveFileOut<<", ";
        saveFileOut.setFieldWidth(14);
        saveFileOut<<QString::number(oneStallit.SatTrop,'f',4);
        saveFileOut.setFieldWidth(2);
        saveFileOut<<", ";
        saveFileOut.setFieldWidth(14);
        saveFileOut<<QString::number(oneStallit.StaTropMap,'f',4);
        saveFileOut.setFieldWidth(2);
        saveFileOut<<", ";
        saveFileOut.setFieldWidth(14);
        saveFileOut<<QString::number(oneStallit.Relativty,'f',4);
        saveFileOut.setFieldWidth(2);
        saveFileOut<<", ";
        saveFileOut.setFieldWidth(14);
        saveFileOut<<QString::number(oneStallit.Sagnac,'f',4);
        saveFileOut.setFieldWidth(2);
        saveFileOut<<", ";
        saveFileOut.setFieldWidth(14);
        saveFileOut<<QString::number(oneStallit.TideEffect,'f',4);
        saveFileOut.setFieldWidth(2);
        saveFileOut<<", ";
        saveFileOut.setFieldWidth(14);
        saveFileOut<<QString::number(oneStallit.AntHeight,'f',4);
        saveFileOut.setFieldWidth(2);
        saveFileOut<<", ";
        saveFileOut.setFieldWidth(14);
        saveFileOut<<QString::number(oneStallit.SatL1Offset,'f',4);
        saveFileOut.setFieldWidth(2);
        saveFileOut<<", ";
        saveFileOut.setFieldWidth(14);
        saveFileOut<<QString::number(oneStallit.SatL2Offset,'f',4);
        saveFileOut.setFieldWidth(2);
        saveFileOut<<", ";
        saveFileOut.setFieldWidth(14);
        saveFileOut<<QString::number(oneStallit.L1Offset,'f',4);
        saveFileOut.setFieldWidth(2);
        saveFileOut<<", ";
        saveFileOut.setFieldWidth(14);
        saveFileOut<<QString::number(oneStallit.L2Offset,'f',4);
        saveFileOut.setFieldWidth(2);
        saveFileOut<<", ";
        saveFileOut.setFieldWidth(14);
        saveFileOut<<QString::number(oneStallit.AntWindup,'f',4);
        //QString::number(oneStallit.pos[2],'f',4);

        saveFileOut<<endl;//Output coordinates of a satellite in the epoch
    }
    saveFileOut.flush();
    return true;
}

//Write the zenith wet delay and clock difference to txt, the first column wet delay second column clock difference
bool QRTWrite2File::writeClockZTDW2Txt(const ClockData epochZTDWClock)
{
    int _flag = findFilenameFlag("ZTDW_Clock.txt");
    if(_flag == -1) return false;

    QTextStream &saveFileOut = m_QTextStreamVct[_flag];

    if(!m_ZTDW_Clock_head)
    {
        // write head information
        QString head_string[10] = {"epoch", "Observation time", "ZTD(m)", "diff_clk_G(m)",
                                 "diff_clk_C(m)", "diff_clk_R(m)", "diff_clk_E(m)"};
        QString sys_str = getSatlitSys(), base_sys = "G";
        if(!sys_str.isEmpty()) base_sys = sys_str.at(0);
        switch (base_sys[0].toLatin1()) {
        case 'G':
            head_string[3] = "base_clk_G(m)";
            break;
        case 'C':
            head_string[4] = "base_clk_C(m)";
            break;
        case 'R':
            head_string[5] = "base_clk_R(m)";
            break;
        case 'E':
            head_string[6] = "base_clk_E(m)";
            break;
        default:
            break;
        }
        for(int i = 0;i < 7;i++)
        {
            saveFileOut.setFieldWidth(12);
            if(i == 1) saveFileOut.setFieldWidth(26);
            saveFileOut << head_string[i]+"|";
        }
        saveFileOut << endl;
        m_ZTDW_Clock_head = true;
    }

    saveFileOut.setFieldWidth(10);
    saveFileOut<<epochZTDWClock.UTCTime.epochNum;
    saveFileOut.setFieldWidth(2);
    saveFileOut<<": ";
    //Output year, month, day, hour, minute, second
    saveFileOut.setFieldWidth(4);
    saveFileOut<<QString::number(epochZTDWClock.UTCTime.Year);
    saveFileOut.setFieldWidth(1);
    saveFileOut<<"-";
    saveFileOut.setFieldWidth(2);
    saveFileOut<<QString::number(epochZTDWClock.UTCTime.Month);
    saveFileOut.setFieldWidth(1);
    saveFileOut<<"-";
    saveFileOut.setFieldWidth(2);
    saveFileOut<<QString::number(epochZTDWClock.UTCTime.Day);
    saveFileOut.setFieldWidth(1);
    saveFileOut<<" ";
    saveFileOut.setFieldWidth(2);
    saveFileOut<<QString::number(epochZTDWClock.UTCTime.Hours);
    saveFileOut.setFieldWidth(1);
    saveFileOut<<":";
    saveFileOut.setFieldWidth(2);
    saveFileOut<<QString::number(epochZTDWClock.UTCTime.Minutes);
    saveFileOut.setFieldWidth(1);
    saveFileOut<<":";
    saveFileOut.setFieldWidth(10);
    saveFileOut<<QString::number(epochZTDWClock.UTCTime.Seconds,'f',7);
    //Output X system receiver clock error, Xi system relative X system receiver clock offset
    saveFileOut.setFieldWidth(10);
    saveFileOut<<QString::number(epochZTDWClock.ZTD_W,'f',4);
    saveFileOut.setFieldWidth(2);
    saveFileOut<<"  ";
    for(int k = 0; k < 4;k++)
    {
        saveFileOut.setFieldWidth(10);
        saveFileOut<<QString::number(epochZTDWClock.clockData[k],'f',4);
        saveFileOut.setFieldWidth(2);
        saveFileOut<<"  ";
    }
    saveFileOut<<endl;
    saveFileOut.flush();
    return true;
}

//All satellites write txt files named "G32.txt", "C02.txt", "R08.txt", etc., the first column of ambiguity
bool QRTWrite2File::writeAmbiguity2Txt(QString fload_path)
{
    if(!isDirExist(fload_path))
    {
        QString infor = "can not construct floder: " + fload_path;
        ErroTrace(infor);
    }

    //Find the name of the satellite that observes all systems
    int lenSatAbm = allAmbiguity.length();
    QVector< char > store_SatType;
    QVector< int > store_SatPRN;
    for (int i = 0;i < lenSatAbm;i++)
    {
        Ambiguity oneSatAbm = allAmbiguity.at(i);
        char tSatType = '0';
        int tSatPRN = -1;
        bool isFind = false;
        for (int j = 0;j < store_SatPRN.length();j++)
        {
            tSatPRN = store_SatPRN.at(j);
            tSatType = store_SatType.at(j);
            if (tSatPRN == oneSatAbm.PRN && tSatType == oneSatAbm.SatType)
            {
                isFind = true;
                break;
            }
        }
        if (!isFind)
        {
            store_SatType.append(oneSatAbm.SatType);
            store_SatPRN.append(oneSatAbm.PRN);
        }
    }
    //Write all satellite names to the file
    for (int i = 0;i < store_SatPRN.length();i++)
    {
        char tSatType = '0';
        int tSatPRN = -1;
        tSatType = store_SatType.at(i);
        tSatPRN = store_SatPRN.at(i);
        WriteAmbPRN(fload_path, tSatPRN,tSatType);
    }
    return true;
}

//The ambiguity of writing to the PRN satellite
bool QRTWrite2File::WriteAmbPRN(QString temp_floder,int PRN,char SatType)
{
    if (allAmbiguity.length() == 0)//If no data is read
        return false;
    QString fileName,strPRN;
    strPRN = QString::number(PRN);
    if(PRN < 10)
        strPRN = QString::number(0) + strPRN;
    fileName = QString(SatType) + strPRN + QString(".txt");
    fileName = temp_floder + fileName; //Path of Current exec file
    QFile saveFile(fileName);//Saved file
    if (!saveFile.open(QFile::WriteOnly|QFile::Text))
    {
        QString erroInfo = "Open " + fileName + " File Error!";
        ErroTrace(erroInfo);
        return false;
    }

    QTextStream saveFileOut(&saveFile);

    int lenAbm = allAmbiguity.length();
    for (int i = 0;i <lenAbm;i++ )
    {
        Ambiguity oneSatAmb = allAmbiguity.at(i);
        if (oneSatAmb.PRN != PRN || oneSatAmb.SatType != SatType)
        {
            continue;
        }
        saveFileOut.setFieldWidth(10);
        saveFileOut<<(oneSatAmb.UTCTime.epochNum);
        saveFileOut.setFieldWidth(2);
        saveFileOut<<": ";
        //Output GPS week seconds
        saveFileOut.setFieldWidth(16);
        double GPSSecond = m_qcmpClass.YMD2GPSTime(oneSatAmb.UTCTime.Year,oneSatAmb.UTCTime.Month,oneSatAmb.UTCTime.Day,oneSatAmb.UTCTime.Hours,oneSatAmb.UTCTime.Minutes,oneSatAmb.UTCTime.Seconds);
        saveFileOut<<QString::number(GPSSecond,'f',6);
        saveFileOut.setFieldWidth(2);
        saveFileOut<<"  ";

        //Write ambiguity
        saveFileOut.setFieldWidth(10);
        saveFileOut<<QString::number(1);
        saveFileOut.setFieldWidth(2);
        saveFileOut<<"  ";
        saveFileOut.setFieldWidth(16);
        if (oneSatAmb.isIntAmb)
            saveFileOut<<QString::number(qRound(oneSatAmb.Amb));
        else
            saveFileOut<<QString::number(oneSatAmb.Amb,'f',4);

        saveFileOut.setFieldWidth(2);
        saveFileOut<<"  ";

        saveFileOut<<endl;
    }
    saveFileOut.flush();
    saveFile.close();

    return true;
}


//Write the satellite stored in the variable allAmbiguity to the file
bool QRTWrite2File::WriteEpochPRN(QString fload_path, QString tempfileName)
{
    if (allAmbiguity.length() == 0)//If no data is read
        return false;
    if(!isDirExist(fload_path))
    {
        QString infor = "can not construct floder: " + fload_path;
        ErroTrace(infor);
    }
    QString fileName = fload_path + tempfileName;

    QFile saveFile(fileName);//Saved file
    if (!saveFile.open(QFile::WriteOnly|QFile::Text))
    {
        QString erroInfo = "Open " + fileName + " File Error!";
        ErroTrace(erroInfo);
        return false;
    }
    QTextStream saveFileOut(&saveFile);
    //Write data only
    int lengAllSat = allAmbiguity.length();
    QString tQstrPRN = "";
    for (int i = 0; i < lengAllSat;i++)
    {
        Ambiguity oneSatAmb = allAmbiguity.at(i);
        //Write epoch
        saveFileOut.setFieldWidth(10);
        saveFileOut<<(oneSatAmb.UTCTime.epochNum);
        saveFileOut.setFieldWidth(2);
        saveFileOut<<": ";
        //Write satellite type and number, such as G24, C01
        saveFileOut.setFieldWidth(1);
        saveFileOut<<QString(oneSatAmb.SatType);
        saveFileOut.setFieldWidth(2);
        if (oneSatAmb.PRN < 10)
            tQstrPRN = QString::number(0) + QString::number(oneSatAmb.PRN);
        else
            tQstrPRN = QString::number(oneSatAmb.PRN);
        saveFileOut<<tQstrPRN;
        saveFileOut<<endl;
    }
    saveFileOut.flush();
    saveFile.close();

    return true;
}

bool QRTWrite2File::writeBadSatliteData(const SatlitData oneSatAmb)
{
    int _flag = findFilenameFlag("bad_satellite.txt");
    if(_flag == -1) return false;

    QTextStream &saveFileOut = m_QTextStreamVct[_flag];
    //Write data only
    QString tQstrPRN = "";
    //Write epoch
    saveFileOut.setFieldAlignment(QTextStream::AlignRight);
    saveFileOut.setFieldWidth(10);
    saveFileOut<<(oneSatAmb.UTCTime.epochNum);
    saveFileOut.setFieldWidth(2);
    saveFileOut<<": ";
    //Write satellite type and number, such as G24, C01
    saveFileOut.setFieldWidth(1);
    saveFileOut<<QString(oneSatAmb.SatType);
    saveFileOut.setFieldWidth(2);
    if (oneSatAmb.PRN < 10)
        tQstrPRN = QString::number(0) + QString::number(oneSatAmb.PRN);
    else
        tQstrPRN = QString::number(oneSatAmb.PRN);
    saveFileOut<<tQstrPRN;
    saveFileOut.setFieldWidth(2);
    saveFileOut<<"  ";

    //Output year, month, day, hour, minute, second
    saveFileOut.setFieldWidth(4);
    saveFileOut<<QString::number(oneSatAmb.UTCTime.Year);
    saveFileOut.setFieldWidth(1);
    saveFileOut<<"-";
    saveFileOut.setFieldWidth(2);
    saveFileOut<<QString::number(oneSatAmb.UTCTime.Month);
    saveFileOut.setFieldWidth(1);
    saveFileOut<<"-";
    saveFileOut.setFieldWidth(2);
    saveFileOut<<QString::number(oneSatAmb.UTCTime.Day);
    saveFileOut.setFieldWidth(1);
    saveFileOut<<" ";
    saveFileOut.setFieldWidth(2);
    saveFileOut<<QString::number(oneSatAmb.UTCTime.Hours);
    saveFileOut.setFieldWidth(1);
    saveFileOut<<":";
    saveFileOut.setFieldWidth(2);
    saveFileOut<<QString::number(oneSatAmb.UTCTime.Minutes);
    saveFileOut.setFieldWidth(1);
    saveFileOut<<":";
    saveFileOut.setFieldWidth(10);
    saveFileOut<<QString::number(oneSatAmb.UTCTime.Seconds,'f',7);

    saveFileOut.setFieldWidth(18);
    saveFileOut<<"Eliminate info: ";
    for(int n = 0;n < oneSatAmb.badMsg.length();n++)
    {
        saveFileOut.setFieldAlignment(QTextStream::AlignLeft);
        saveFileOut.setFieldWidth(132);
        saveFileOut<<oneSatAmb.badMsg.at(n);
        saveFileOut.setFieldWidth(3);
        saveFileOut<< " | ";
    }
    saveFileOut.setFieldWidth(1);
    saveFileOut<<endl;
    saveFileOut.flush();

    return true;
}

bool QRTWrite2File::isDirExist(QString fullPath)
{
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

int QRTWrite2File::findFilenameFlag(QString Filename)
{
    for(int i = 0; i < MAX_FILE_WRITE;i++)
    {
        if(m_QFileNames[i].contains(Filename))
            return i;
    }

    return -1;
}
