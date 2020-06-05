#include "QReadOFile.h"

//Initialize variables
void QReadOFile::initVar()
{
	m_OfileName = "";
	tempLine = "";
	isReadHead = false;
    m_IsCloseFile = true;
//Header information
	RinexVersion = 0;
	FileIdType = 0;
	SatelliteSys = 0;
	PGM = "";
	RUNBY = "";
	CreatFileDate = "";
	CommentInfo = "";
	MarkerName = "";
	ObserverNames = "";
	Agency = "";
	ReciverREC = "";
	ReciverType = "";
	ReciverVers = "";
	AntNumber = "";
	AntType = "";
	for (int i = 0; i < 3; i++)
	{
        ApproxXYZ[i] = 0;
        AntHEN[i] = 0;
	}
	FactL12[0] = -1; FactL12[1] = -1;
    m_TypeObservNum_2 = -1;
	IntervalSeconds = -1;
	for (int i = 0; i < 5; i++)
	{
		YMDHM[i] = 0;
	}
	ObsSeconds = 0;
	SateSystemOTime = "";
//	QString str1 = "   G 1 A 2";
//    matchHead.setPattern(".*[[GRSCE][0-9][0-9]]*.*");
    matchHead.setPattern(".*[GRSCE][0-9][0-9].*");//Simple match header file (2.X) (can be optimized)
//    QString tempLineREG = "0  0 18G06G17R01R17G24G02R09G2";
//    tempLineREG = "  G09GMMM  ";
//    if(matchHead.exactMatch(tempLineREG))
//        qDebug() << "match" << endl;
//    else
//        qDebug() << "no match" << endl;
}

QReadOFile::QReadOFile()
{
	initVar();
}

//Class initialization
QReadOFile::QReadOFile(QString OfileName)
{
	initVar();
	if (OfileName.trimmed().isEmpty())
		ErroTrace("File Name is Empty!(QReadOFile::QReadOFile(QString OfileName))");
	m_OfileName = OfileName;
	//Read file header information and parse
	getHeadInf();//The file is not closed here
	baseYear = (int)(YMDHM[0]/100)*100;
}

//Class initialization
void QReadOFile::setObsFileName(QString OfileName)
{
    initVar();
    if (OfileName.trimmed().isEmpty())
        ErroTrace("File Name is Empty!(QReadOFile::QReadOFile(QString OfileName))");
    m_OfileName = OfileName;
    //Read file header information and parse
    getHeadInf();//The file is not closed here
    baseYear = (int)(YMDHM[0]/100)*100;

}

QReadOFile::~QReadOFile(void)
{
    closeFile();
}

//Read the header file to the variable (each version O file header file is similar,The file is not closed here）
void QReadOFile::getHeadInf()
{
	if (isReadHead)	return ;
	//open a file
	m_readOFileClass.setFileName(m_OfileName);
	if (!m_readOFileClass.open(QFile::ReadOnly))
		ErroTrace("Open file bad!(QReadOFile::QReadOFile(QString OfileName))");
    m_IsCloseFile = false;
	//Read header file
	while (!m_readOFileClass.atEnd())
	{
		tempLine = m_readOFileClass.readLine();
        if (tempLine.contains("END OF HEADER",Qt::CaseInsensitive))
            break;
        if (tempLine.contains("RINEX VERSION",Qt::CaseInsensitive))
        {
            RinexVersion =  tempLine.mid(0,20).trimmed().toDouble();
            FileIdType = tempLine.at(20);
            SatelliteSys = tempLine.at(40);
        }
        else if (tempLine.contains("PGM / RUN BY / DATE",Qt::CaseInsensitive))
        {
            PGM = tempLine.mid(0,20).trimmed();
            RUNBY = tempLine.mid(20,20).trimmed();
            CreatFileDate = tempLine.mid(40,20).trimmed();
        }
        else if (tempLine.contains("COMMENT",Qt::CaseInsensitive))
        {
            CommentInfo+=tempLine.mid(0,60).trimmed() + "\n";
        }
        else if (tempLine.contains("MARKER NAME",Qt::CaseInsensitive))
        {
            MarkerName = tempLine.mid(0,60).trimmed().toUpper();
        }
        else if (tempLine.contains("MARKER NUMBER"))
        {
            MarkerNumber = tempLine.mid(0,60).trimmed();
        }
        else if (tempLine.contains("OBSERVER / AGENCY",Qt::CaseInsensitive))
        {
            ObserverNames = tempLine.mid(0,20).trimmed();
            Agency = tempLine.mid(20,40).trimmed();
        }
        else if (tempLine.contains("REC # / TYPE / VERS"))
        {
            ReciverREC = tempLine.mid(0,20).trimmed();
            ReciverType = tempLine.mid(20,20).trimmed();
            ReciverVers = tempLine.mid(40,20).trimmed();
        }
        else if (tempLine.contains("ANT # / TYPE",Qt::CaseInsensitive))
        {
            AntNumber = tempLine.mid(0,20).trimmed();
            AntType = tempLine.mid(20,20).trimmed();
        }
        else if (tempLine.contains("APPROX POSITION XYZ",Qt::CaseInsensitive))
        {
            QString tempPos = tempLine.mid(0, 59).trimmed();
            tempPos = tempPos.simplified();
            QStringList headList = tempPos.split(" ");
            for(int i = 0;i < 3;i++)
                ApproxXYZ[i] = headList.at(i).toDouble();
        }
        else if (tempLine.contains("ANTENNA: DELTA H/E/N",Qt::CaseInsensitive))
        {
            QString tempHEN = tempLine.mid(0, 59).trimmed();
            tempHEN = tempHEN.simplified();
            QStringList headList = tempHEN.split(" ");
            for(int i = 0;i < 3;i++)
                AntHEN[i] = headList.at(i).toDouble();
        }
        else if (tempLine.contains("WAVELENGTH FACT L1/2",Qt::CaseInsensitive))
        {
            FactL12[0] = tempLine.mid(0,6).trimmed().toInt();
            FactL12[1] = tempLine.mid(6,6).trimmed().toInt();
        }
        else if (tempLine.contains("TYPES OF OBSERV",Qt::CaseInsensitive))
        {//Due to the use of the global variable Static, the function has a big bug. It is forbidden to use global variables. It will not disappear in the main function.
            if (RinexVersion >= 3) continue;
            if(m_TypeObservNum_2 < 1)
                m_TypeObservNum_2 = tempLine.left(6).trimmed().toInt();
            if (m_TypeObservNum_2 != 0 )
            {
                for (int i = 0;i < 9;i++)
                {
                    QString tempObserVar = "";
                    tempObserVar = tempLine.left(6*i + 12).right(6).trimmed();
                    if (tempObserVar.isEmpty())
                        break;
                    m_ObservVarsNames_2.append(tempObserVar);
                }
            }
            else
            {
                ErroTrace("Lack TYPES OF OBSERV!(QReadOFile::QReadOFile(QString OfileName))");
            }

        }//# / TYPES OF OBSERV if End
        else if (tempLine.contains("SYS / # / OBS TYPES",Qt::CaseInsensitive))
        {//Renix 3.0 added header file
            if (RinexVersion < 3)	continue;
            obsVarNamesVer3 tempObsType;
            tempObsType.SatType = tempLine.mid(0, 1);
            tempObsType.obsNum3ver = tempLine.mid(3, 3).trimmed().toInt();
            QString obstypeName = "";
            int flag = 0, read_hang_num = 0, i = 0;
            read_hang_num = (int)(tempObsType.obsNum3ver/13.01);// tempObsType.obsNum3ver if a multiple of 13 read one less line

            for (i = 0; i < tempObsType.obsNum3ver;i++)
            {
                obstypeName = tempLine.mid(6 + flag * 4, 4).trimmed();
                flag++;
                tempObsType.obsNames3ver.append(obstypeName);
                //Exceeding 13 parsing the next line
                if (flag % 13 == 0 && read_hang_num > 0)
                {
                    tempLine = m_readOFileClass.readLine();
                    flag = 0; read_hang_num--;
                }
            }

            m_obsVarNamesVer3.append(tempObsType);//Save this system data
        }
        else if (tempLine.contains("INTERVAL",Qt::CaseInsensitive))
        {
            IntervalSeconds = tempLine.mid(0,10).trimmed().toDouble();
        }
        else if (tempLine.contains("TIME OF FIRST OBS",Qt::CaseInsensitive))
        {
            for (int i = 0;i < 5;i++)
            {
                YMDHM[i] = tempLine.left((i+1)*6).right(6).trimmed().toInt();
            }
            ObsSeconds = tempLine.left(43).right(13).trimmed().toDouble();
            SateSystemOTime = tempLine.left(51).right(8).trimmed();
        }
    }//while (!m_readOFileClass.atEnd())
	isReadHead = true;
    // process Reniex 3
    if(RinexVersion >= 3)
        PriorityRanking();
    //If the header file does not have obs time to retrieve the observed data
    if(0 == YMDHM[0])
    {
        int current_pos = m_readOFileClass.pos();
        tempLine = m_readOFileClass.readLine();
        baseYear = 2000;//Document 2.0 before 2000 does not support!!!
        if(RinexVersion < 3)
        {
            YMDHM[0] = tempLine.mid(0,3).trimmed().toInt() + baseYear;
            YMDHM[1] = tempLine.mid(3,3).trimmed().toInt();
            YMDHM[2] = tempLine.mid(6,3).trimmed().toInt();
            YMDHM[3] = tempLine.mid(9,3).trimmed().toInt();
            YMDHM[4] = tempLine.mid(12,3).trimmed().toInt();
            ObsSeconds = tempLine.mid(15,11).trimmed().toDouble();
        }
        else if(RinexVersion >= 3)
        {
            YMDHM[0] = tempLine.mid(1,5).trimmed().toInt();
            YMDHM[1] = tempLine.mid(6,3).trimmed().toInt();
            YMDHM[2] = tempLine.mid(9,3).trimmed().toInt();
            YMDHM[3] = tempLine.mid(12,3).trimmed().toInt();
            YMDHM[4] = tempLine.mid(15,3).trimmed().toInt();
            ObsSeconds = tempLine.mid(18,11).trimmed().toDouble();
        }
        m_readOFileClass.seek(current_pos);
    }
}

//Convert characters to data lines, parse them into satellite data, and obtain satellite ranging codes and carrier data
bool QReadOFile::getOneSatlitData(QString &dataString,SatlitData &oneSatlite)
{//Read Rinex 2.X and 3.X observation files can be used
	if (dataString.isEmpty())	return false;
    int obserNumbers = 0;
	//Obtain satellite ranging code and carrier data location by version
    obserNumbers = oneSatlite.obserType.length();
    for(int i = 0;i < obserNumbers; i++)
    {
        QString obsType = oneSatlite.obserType.at(i);
        double obsValue = dataString.mid(i*16, 14).trimmed().toDouble();
        oneSatlite.obserValue.append(obsValue);
        if(obsType.contains("L"))
        {
            if(oneSatlite.LL1_LOCK == -1)
                oneSatlite.LL1_LOCK = dataString.mid(i*16+14,1).trimmed().toInt();
            if(oneSatlite.SigInten == -1)
                oneSatlite.SigInten = dataString.mid(i*16+15,1).trimmed().toInt();
        }
    }
	return true;
}

void QReadOFile::getFrequency(SatlitData &oneSatlite)
{//debug:2017.07.08
	if (RinexVersion >= 3)
		getFrequencyVer3(oneSatlite);
	else if (RinexVersion < 3)
		getFrequencyVer2(oneSatlite);
	return ;
}
//Get the frequency of L1 and L2 according to the type of satellite PRN
void QReadOFile::getFrequencyVer2(SatlitData &oneSatlite)
{//debug:2017.07.08
    oneSatlite.Frq[0] = 0; oneSatlite.Frq[1] = 0;
	if (oneSatlite.SatType == 'G')
	{//There is a bug: if the GPS frequency is C2 C1
        oneSatlite.Frq[0] = g_GPSFrq[1];
        oneSatlite.Frq[1] = g_GPSFrq[2];
	}
	else if (oneSatlite.SatType == 'R')
	{
		oneSatlite.Frq[0] = M_GLONASSF1(g_GlonassK[oneSatlite.PRN - 1]);
		oneSatlite.Frq[1] = M_GLONASSF2(g_GlonassK[oneSatlite.PRN - 1]);
	} 
	else if (oneSatlite.SatType == 'C')
	{
		oneSatlite.Frq[0] = g_BDSFrq[1];
        oneSatlite.Frq[1] = g_BDSFrq[7];
	}
	else if (oneSatlite.SatType == 'E')
	{
        oneSatlite.Frq[0] = g_GalieoFrq[1];
        oneSatlite.Frq[1] = g_GalieoFrq[5];
	}

}

//Get the frequency of L1 and L2 according to the type of satellite PRN
void QReadOFile::getFrequencyVer3(SatlitData &oneSatlite)
{//debug:2017.07.08
    oneSatlite.Frq[0] = 0; oneSatlite.Frq[1] = 0; oneSatlite.Frq[2] = 0;
	if (oneSatlite.SatType == 'G')
    {
        int L1Type = oneSatlite.wantObserType.at(1).mid(1,1).toInt(),
            L2Type = oneSatlite.wantObserType.at(3).mid(1,1).toInt(),
            L3Type = oneSatlite.wantObserType.at(5).mid(1,1).toInt();
        if(L1Type >= 0) oneSatlite.Frq[0] = g_GPSFrq[L1Type];
        if(L2Type >= 0) oneSatlite.Frq[1] = g_GPSFrq[L2Type];
        if(L3Type >= 0) oneSatlite.Frq[2] = g_GPSFrq[L3Type];
	}
	else if (oneSatlite.SatType == 'R')
	{
		oneSatlite.Frq[0] = M_GLONASSF1(g_GlonassK[oneSatlite.PRN - 1]);
		oneSatlite.Frq[1] = M_GLONASSF2(g_GlonassK[oneSatlite.PRN - 1]);
        oneSatlite.Frq[2] = 1.202025e9;
	} 
	else if (oneSatlite.SatType == 'C')
	{
        int L1Type = oneSatlite.wantObserType.at(1).mid(1,1).toInt(),
            L2Type = oneSatlite.wantObserType.at(3).mid(1,1).toInt(),
            L3Type = oneSatlite.wantObserType.at(5).mid(1,1).toInt();
        if(L1Type >= 0) oneSatlite.Frq[0] = g_BDSFrq[L1Type];
        if(L2Type >= 0) oneSatlite.Frq[1] = g_BDSFrq[L2Type];
        if(L3Type >= 0) oneSatlite.Frq[2] = g_BDSFrq[L3Type];
	}
	else if (oneSatlite.SatType == 'E')
	{
        int L1Type = oneSatlite.wantObserType.at(1).mid(1,1).toInt(),
            L2Type = oneSatlite.wantObserType.at(3).mid(1,1).toInt(),
            L3Type = oneSatlite.wantObserType.at(5).mid(1,1).toInt();
        if(L1Type >= 0) oneSatlite.Frq[0] = g_GalieoFrq[L1Type];
        if(L2Type >= 0) oneSatlite.Frq[1] = g_GalieoFrq[L2Type];
        if(L3Type >= 0) oneSatlite.Frq[2] = g_GalieoFrq[L3Type];
	}

}

//Get the type of observation (version greater than 3)
void QReadOFile::getObsType(SatlitData &oneSatlite)
{
    QString satType(oneSatlite.SatType);
    if(RinexVersion >= 3)
    {
        for(int i = 0;i < m_obsVarNamesVer3.length();i++)
        {
            obsVarNamesVer3 tempObsVar= m_obsVarNamesVer3.at(i);
            if(tempObsVar.SatType.contains(satType))
            {
                oneSatlite.obserType = tempObsVar.obsNames3ver;
                break;
            }
        }
    }
}
//Read Rinec 3.X observation files (read all observations)
void QReadOFile::readEpochVer3(QVector< SatlitData > &epochData)
{
	if (!isReadHead) getHeadInf();
	tempLine = m_readOFileClass.readLine();
	//Enter data area to read
	while (!m_readOFileClass.atEnd())
	{//Read data segment
        //找到数据头部> 2015 05 02 00 00  0.0000000  0 28
        if (0 != tempLine.mid(0,1).compare(">"))
        {
            tempLine = m_readOFileClass.readLine();
            continue;
        }
		//Read header data
		SatlitData oneSatlit;//One satellite data
		int epochStalitNum = 0;
		oneSatlit.UTCTime.Year = tempLine.mid(1,5).trimmed().toInt();
        oneSatlit.UTCTime.Month = tempLine.mid(6,3).trimmed().toInt();
        oneSatlit.UTCTime.Day = tempLine.mid(9,3).trimmed().toInt();
        oneSatlit.UTCTime.Hours = tempLine.mid(12,3).trimmed().toInt();
        oneSatlit.UTCTime.Minutes = tempLine.mid(15,3).trimmed().toInt();
        oneSatlit.UTCTime.Seconds = tempLine.mid(18,11).trimmed().toDouble();
        oneSatlit.EpochFlag = tempLine.mid(30,2).trimmed().toInt();
        epochStalitNum = tempLine.mid(32,3).trimmed().toInt();//Number of satellites in this epoch
        // Debug by xiaogongwei 2019.02.19
        // Get the receiver clock deviation unit: s
        double rec_off = tempLine.mid(40,16).toDouble();
//        oneSatlit.UTCTime.Seconds += rec_off;// can't add to Obs Time Debug by xiaogongwei 2019-03-30
        // The current epoch data has an error
        if(oneSatlit.UTCTime.Year == 0 || oneSatlit.EpochFlag > 1)
            return ;
		//Reading satellite data
		char tempSatType = '0';
		QString dataString = "";
		for (int i = 0;i < epochStalitNum;i++)
		{
			tempLine = m_readOFileClass.readLine(); 
			tempSatType = *(tempLine.mid(0,1).toLatin1().data());//Types of
			if (isInSystem(tempSatType))
			{
                oneSatlit.SigInten = -1;// set -1 at 2018.07.30 store signal intensity
                oneSatlit.LL1_LOCK = -1;
				oneSatlit.SatType = tempSatType;//Types of
				oneSatlit.PRN = tempLine.mid(1,2).toInt();//PRN
                oneSatlit.obserType.clear();
                oneSatlit.obserValue.clear();
                oneSatlit.wantObserType.clear();
                getObsType(oneSatlit);//Get the type of observation
				dataString = tempLine.mid(3);
				getOneSatlitData(dataString,oneSatlit);//Obtain satellite ranging code and carrier data
                getWantData_3(oneSatlit);//Get the data you need
                getFrequency(oneSatlit);//Acquisition frequency
				epochData.append(oneSatlit);//Save a satellite in the epoch
			}
		}
		break;
	}
}

//Read Rixin 2.X observation files (read all observations)
void QReadOFile::readEpochVer2(QVector< SatlitData > &epochData)
{
	if (!isReadHead) getHeadInf();
	tempLine = m_readOFileClass.readLine();
	//Enter data area to read
	while (!m_readOFileClass.atEnd())
	{//Read data segment
		//if (!matchHead.exactMatch(tempLine)) //Find the data header>  10  4 10  0  1 30.0000000  0  9G 2G29G12G 4G13G17G10G 5G30
        if (tempLine.mid(18,1) != ".")
		{
            tempLine = m_readOFileClass.readLine();
			continue;
		}
		//Read header data
		SatlitData oneSatlit;//One satellite data
		int epochStalitNum = 0;
		oneSatlit.UTCTime.Year = tempLine.mid(0,3).trimmed().toInt() + baseYear;
		oneSatlit.UTCTime.Month = tempLine.mid(3,3).trimmed().toInt();
		oneSatlit.UTCTime.Day = tempLine.mid(6,3).trimmed().toInt();
		oneSatlit.UTCTime.Hours = tempLine.mid(9,3).trimmed().toInt();
		oneSatlit.UTCTime.Minutes = tempLine.mid(12,3).trimmed().toInt();
		oneSatlit.UTCTime.Seconds = tempLine.mid(15,11).trimmed().toDouble();
		oneSatlit.EpochFlag = tempLine.mid(26,3).trimmed().toInt();
		epochStalitNum = tempLine.mid(29,3).trimmed().toInt();//Number of satellites in this epoch
        oneSatlit.obserType = m_ObservVarsNames_2;// All satellites initialize the same type of observation
        // Debug by xiaogongwei 2019.02.19
        // Get the receiver clock deviation unit: s
        double rec_off = tempLine.mid(68,12).toDouble();
//        oneSatlit.UTCTime.Seconds += rec_off; // can't add to Obs Time Debug by xiaogongwei 2019-03-30
        // The current epoch data has an error
        if(oneSatlit.UTCTime.Year == 0 || oneSatlit.EpochFlag > 1)
            return ;
		//Judge the situation (first parse the header file)
		int headHang = epochStalitNum/12,headHangLast = epochStalitNum%12,tempPrn = 0;
		QVector< char > saveType;//Save the parsed header file type
		QVector< int > savePRN;//Save the parsed PRN
		QString tempSatName = "";//The name of a satellite
        char tempCh = 'G';
		for(int i = 0;i < headHang;i++)
		{
			for(int j = 0;j < 12;j++)
			{//Store up to 12 satellites in a row
				tempSatName = tempLine.mid(32+j*3,3);
				tempCh = *(tempSatName.mid(0,1).toLatin1().data());
                if(tempCh == 32) tempCh = 'G';
				tempPrn = tempSatName.mid(1,2).trimmed().toInt();
				saveType.append(tempCh);
				savePRN.append(tempPrn);
			}
			tempLine = m_readOFileClass.readLine();
		}
		for(int i = 0;i < headHangLast;i++)
		{//Read a satellite with less than one line (including the first line is not enough)
			tempSatName = tempLine.mid(32+i*3,3);
			tempCh = *(tempSatName.mid(0,1).toLatin1().data());
            if(tempCh == 32) tempCh = 'G';
			tempPrn = tempSatName.mid(1,2).trimmed().toInt();
			saveType.append(tempCh);
			savePRN.append(tempPrn);
		}
		if (headHangLast > 0)
		{//Read the remaining incoming data rows, different from the integer rows
			tempLine = m_readOFileClass.readLine();
		}
		if (saveType.length() != epochStalitNum||savePRN.length()!=epochStalitNum)
			ErroTrace("Data erro!(QReadOFile::readEpochVer2)");
		//Reading satellite data
		QString dataString = "";
        int dataHang = (m_TypeObservNum_2/5.0 - m_TypeObservNum_2/5 == 0)?(m_TypeObservNum_2/5):(m_TypeObservNum_2/5 + 1);//The number of rows of observation data is equivalent to the ceil (TypeObservNum) function
		for (int i = 0;i < epochStalitNum;i++)
		{
			tempCh = saveType.at(i);
			tempPrn = savePRN.at(i);
			oneSatlit.SatType = tempCh;//Types of
			oneSatlit.PRN = tempPrn;//PRN
            dataString = tempLine;//Save a row of data
            dataString.replace("\n", "");
            while(dataString.length() < 80)
                dataString.append(" ");
			for (int j = 0;j < dataHang - 1;j++)//Read extra rows of data
            {
                QString tempLine80 = m_readOFileClass.readLine();
                tempLine80.replace("\n", "");
                while(tempLine80.length() < 80)
                    tempLine80.append(" ");
                dataString += tempLine80;
            }
			if (isInSystem(tempCh))
			{
                oneSatlit.LL1_LOCK = -1;
                oneSatlit.SigInten = -1;
                oneSatlit.wantObserType.clear();
                oneSatlit.obserValue.clear();
                getOneSatlitData(dataString,oneSatlit);//Obtain satellite ranging code and carrier data
                getWantData_2(oneSatlit);//Get the data you need
                getFrequency(oneSatlit);//Acquisition frequency
                epochData.append(oneSatlit);//Save a satellite in the epoch
			}
            if (i  != epochStalitNum - 1)
			{//Do not read the last satellite, leave it as the top of this function to read
				tempLine = m_readOFileClass.readLine();
			}
		}
		break;
    }
}

//Use less than 3, get obsType observations(obsType: Rinex2:C1 P1 L1 L2)
//Debug:2018.08.03
QString QReadOFile::getObsTypeData(SatlitData &oneSatlit, QString obsType, double &obsValue)
{
    int obsLen = oneSatlit.obserType.length();
    QString satObsType = "";
    obsValue = 0;
    for(int i = 0;i < obsLen;i++)
    {
        satObsType = oneSatlit.obserType.at(i);
//        if(satObsType.contains(obsType))
        if(0 == satObsType.compare(obsType))
        {
            obsValue = oneSatlit.obserValue.at(i);
            return satObsType;
        }
    }
	return satObsType;

}

void QReadOFile::ProcessCLPos(obsVarNamesVer3 epochSystem)
{
    CLPos ObsTypePos;
    QVector< QString > obsType = epochSystem.obsNames3ver;
    int prioLen = -1;
    // init data
    QString frqenceStr[4] = {"0", "0", "0", "0"};// store frqence number
    QString prioArry[8] = {"0", "0", "0", "0", "0", "0", "0", "0"};
    ObsTypePos.SatType = epochSystem.SatType;
    // juge satalite system
    if(epochSystem.SatType.contains("G", Qt::CaseInsensitive))
    {// GPS 1 2 5 frqence
        frqenceStr[0] = "1"; frqenceStr[1] = "2"; frqenceStr[2] = "5";
        prioLen = 8;
        prioArry[0] = "P"; prioArry[1] = "W"; prioArry[2] = "C";
        prioArry[3] = "I"; prioArry[4] = "L"; prioArry[5] = "S";
        prioArry[6] = "Q"; prioArry[7] = "X";
    }
    else if(epochSystem.SatType.contains("R", Qt::CaseInsensitive))
    {// GLONASS  C1C C2C
        frqenceStr[0] = "1"; frqenceStr[1] = "2"; frqenceStr[2] = "3";
        prioLen = 5;
        prioArry[0] = "P"; prioArry[1] = "C"; prioArry[2] = "Q";
        prioArry[3] = "X"; prioArry[4] = "I";
    }
    else if(epochSystem.SatType.contains("C", Qt::CaseInsensitive))
    {// BDS 1 7 6 or 2
        // "2" -> "6" -> "7" is for BDS-III
        frqenceStr[0] = "2"; frqenceStr[1] = "6"; frqenceStr[2] = "7"; frqenceStr[3] = "1";
        prioLen = 3;
        prioArry[0] = "I"; prioArry[1] = "Q"; prioArry[2] = "X";
    }
    else if(epochSystem.SatType.contains("E", Qt::CaseInsensitive))
    {// Galileo 1 5 7 8
        frqenceStr[0] = "1"; frqenceStr[1] = "5"; frqenceStr[2] = "7"; frqenceStr[3] = "8";
        prioLen = 6;
        prioArry[0] = "C"; prioArry[1] = "Q"; prioArry[2] = "A";
        prioArry[3] = "B"; prioArry[4] = "X"; prioArry[5] = "Z";
    }
    // search System obsType
    for(int i = 0; i < prioLen;i++)
    {
        QString C1i = "C" + frqenceStr[0] + prioArry[i], L1i = "L" + frqenceStr[0] + prioArry[i],
                C2i = "C" + frqenceStr[1] + prioArry[i], L2i = "L" + frqenceStr[1] + prioArry[i],
                C3i = "C" + frqenceStr[2] + prioArry[i], L3i = "L" + frqenceStr[2] + prioArry[i];
        int tempPos1 = -1, tempPos2 = -1;
        // frequence 1
        // C1 L1
        tempPos1 = obsType.indexOf(C1i);// find pos of C1P etc
        tempPos2 = obsType.indexOf(L1i);// find pos of L1P etc
        if(tempPos1 != -1 && tempPos2 != -1)
        {
            ObsTypePos.C1Type.append(C1i);
            ObsTypePos.C1Pos.append(tempPos1);
            // L1
            ObsTypePos.L1Type.append(L1i);
            ObsTypePos.L1Pos.append(tempPos2);
        }
        // frequence 2
        // C2 L2
        tempPos1 = obsType.indexOf(C2i);// find pos of C2P etc
        tempPos2 = obsType.indexOf(L2i);// find pos of L2P etc
        if(tempPos1 != -1 && tempPos2 != -1)
        {
            ObsTypePos.C2Type.append(C2i);
            ObsTypePos.C2Pos.append(tempPos1);
            // L2
            ObsTypePos.L2Type.append(L2i);
            ObsTypePos.L2Pos.append(tempPos2);
        }
        // frequence 3
        // C3 L3
        tempPos1 = obsType.indexOf(C3i);// find pos of C3P etc
        tempPos2 = obsType.indexOf(L3i);// find pos of L3P etc
        if(tempPos1 != -1 && tempPos2 != -1)
        {
            ObsTypePos.C3Type.append(C3i);
            ObsTypePos.C3Pos.append(tempPos1);
            // L2
            ObsTypePos.L3Type.append(L3i);
            ObsTypePos.L3Pos.append(tempPos2);
        }

    }
    // if BDS
    if(epochSystem.SatType.contains("C", Qt::CaseInsensitive))
    {
        for(int i = 0; i < prioLen;i++)
        {
            QString C1i = "C" + frqenceStr[3] + prioArry[i], L1i = "L" + frqenceStr[3] + prioArry[i];
            QString C2i = "C" + frqenceStr[2] + prioArry[i], L2i = "L" + frqenceStr[2] + prioArry[i];
            int tempPos1 = -1, tempPos2 = -1;
            // If C2I is missing, use C1I instead
            // C1 L1
            tempPos1 = obsType.indexOf(C1i);// find pos of C1P etc
            tempPos2 = obsType.indexOf(L1i);// find pos of L1P etc
            if(tempPos1 != -1 && tempPos2 != -1)
            {
                ObsTypePos.C1Type.append(C1i);
                ObsTypePos.C1Pos.append(tempPos1);
                // L1
                ObsTypePos.L1Type.append(L1i);
                ObsTypePos.L1Pos.append(tempPos2);
            }
            // If C6I is missing, use C7I instead
            // C2 L2
            tempPos1 = obsType.indexOf(C2i);// find pos of C2P etc
            tempPos2 = obsType.indexOf(L2i);// find pos of L2P etc
            if(tempPos1 != -1 && tempPos2 != -1)
            {
                ObsTypePos.C2Type.append(C2i);
                ObsTypePos.C2Pos.append(tempPos1);
                // L2
                ObsTypePos.L2Type.append(L2i);
                ObsTypePos.L2Pos.append(tempPos2);
            }

        }
    }
    // juge some system maybe single frequence
    if(isInSystem(*(ObsTypePos.SatType.toLatin1().data())) && ObsTypePos.C2Pos.length() == 0)
    {
        QString warning_str = "Please check >>>SYS / # / OBS TYPES<<< in Observation file." + ENDLINE
                + "You select system maybe single frequence!"+ ENDLINE + "System ( " + ObsTypePos.SatType
                + " ) maybe single frequence";
        ErroTrace(warning_str);
    }
    m_allObsTypePos.append(ObsTypePos);
}

//Sort observations by priority (2018.08.02) Version is greater than or equal to 3
//The observation type is sorted according to P, W, C, X, S priority, that is, the end of the observation value (for example: C2P, C2W, C2C, C2X, C2S)
void QReadOFile::PriorityRanking()
{
    for(int i = 0;i < m_obsVarNamesVer3.length();i++)
    {
        obsVarNamesVer3 epochSystem = m_obsVarNamesVer3.at(i);
        ProcessCLPos(epochSystem);
    }
}

//Get the data you need (my PPP needs pseudorange and carrier, please change this function if you need other data)
//Debug:2018.08.03
void QReadOFile::getWantData_2(SatlitData &oneSatlit)
{
    // init 4 QString
    for(int i = 0;i < 4;i++)
        oneSatlit.wantObserType.append("");
    //get obs data store Carrier and Pesudorange
    double obsValue = 0;
    // get Frenquce 1 best
    // C1
    getObsTypeData(oneSatlit, "P1", obsValue);
    if(0 != obsValue)
    {
        oneSatlit.C1 = obsValue;
        oneSatlit.wantObserType[0] = "P1";
    }
    else
    {
        getObsTypeData(oneSatlit, "C1", obsValue);
        oneSatlit.C1 = obsValue;
        oneSatlit.wantObserType[0] = "C1";
    }
    // L1
    getObsTypeData(oneSatlit, "L1", obsValue);
    oneSatlit.L1 = obsValue;
    oneSatlit.wantObserType[1] = "L1";
    // get Frenquce 2 best
    // C2
    getObsTypeData(oneSatlit, "P2", obsValue);
    if(0 != obsValue)
    {
        oneSatlit.C2 = obsValue;
        oneSatlit.wantObserType[2] = "P2";
    }
    else
    {
        getObsTypeData(oneSatlit, "C2", obsValue);
        oneSatlit.C2 = obsValue;
        oneSatlit.wantObserType[2] = "C2";
    }
    // L2
    getObsTypeData(oneSatlit, "L2", obsValue);
    oneSatlit.L2 = obsValue;
    oneSatlit.wantObserType[3] = "L2";
}

//Get the data you need (my PPP needs pseudorange and carrier, please change this function if you need other data)
//The main idea: read C2P first, if C2P is 0, read C2W, and then recursively
//Debug:2018.08.23
void QReadOFile::getWantData_3(SatlitData &oneSatlit)
{
    // init 4 QString
    oneSatlit.C1 = 0; oneSatlit.L1 = 0;
    oneSatlit.C2 = 0; oneSatlit.L2 = 0;
    oneSatlit.C3 = 0; oneSatlit.L3 = 0;
    // all obstype number (C1, C2, C3, L1, L2, L3)
    int obsNumber = 6;
    for(int i = 0;i < obsNumber;i++)
        oneSatlit.wantObserType.append("");
    // find prioData
    CLPos prioData;
    for(int i = 0;i < m_allObsTypePos.length(); i++)
    {
		char ch_temp1 = *(m_allObsTypePos.at(i).SatType.toLatin1().data()),
             ch_temp2 = oneSatlit.SatType;
        if(ch_temp2  == ch_temp1)
        {
            prioData = m_allObsTypePos.at(i);
            break;
        }
    }
    //The purpose of using a for loop is to replace the idea between channels of the same frequency (ie, reading C2W data missing, replacing with C2X; L2W missing using L2X instead will produce a weekly jump)
    //Debug by xiaogongwei 2019.05.22  For missing data, satellite anomalies cannot be used as "substitute ideas"
    // get Frenquce 1 best
    double obsValue = 0;
    // C1
    for(int i = 0; i < prioData.C1Pos.length();i++)
    {
        int flag = prioData.C1Pos.at(i);
        obsValue = oneSatlit.obserValue.at(flag);
        if( 1 ) // obsValue!= 0
        {
            oneSatlit.C1 = obsValue;
            oneSatlit.wantObserType[0] = prioData.C1Type.at(i);
            break;
        }
    }
    // L1
    for(int i = 0; i < prioData.L1Pos.length();i++)
    {
        int flag = prioData.L1Pos.at(i);
        obsValue = oneSatlit.obserValue.at(flag);
        if( 1 ) // obsValue!= 0
        {
            oneSatlit.L1 = obsValue;
            oneSatlit.wantObserType[1] = prioData.L1Type.at(i);
            break;
        }
    }
    // get Frenquce 2 best
    obsValue = 0;
    // C2
    for(int i = 0; i < prioData.C2Pos.length();i++)
    {
        int flag = prioData.C2Pos.at(i);
        obsValue = oneSatlit.obserValue.at(flag);
        if( 1 )// obsValue!= 0
        {
            oneSatlit.C2 = obsValue;
            oneSatlit.wantObserType[2] = prioData.C2Type.at(i);
            break;
        }
    }
    // L2
    for(int i = 0; i < prioData.L2Pos.length();i++)
    {
        int flag = prioData.L2Pos.at(i);
        obsValue = oneSatlit.obserValue.at(flag);
        if( 1 ) // obsValue!= 0
        {
            oneSatlit.L2 = obsValue;
            oneSatlit.wantObserType[3] = prioData.L2Type.at(i);
            break;
        }
    }
    // get Frenquce 3 best
    obsValue = 0;
    // C3
    for(int i = 0; i < prioData.C3Pos.length();i++)
    {
        int flag = prioData.C3Pos.at(i);
        obsValue = oneSatlit.obserValue.at(flag);
        if( 1 )// obsValue!= 0
        {
            oneSatlit.C3 = obsValue;
            oneSatlit.wantObserType[4] = prioData.C3Type.at(i);
            break;
        }
    }
    // L3
    for(int i = 0; i < prioData.L3Pos.length();i++)
    {
        int flag = prioData.L3Pos.at(i);
        obsValue = oneSatlit.obserValue.at(flag);
        if( 1 ) // obsValue!= 0
        {
            oneSatlit.L3 = obsValue;
            oneSatlit.wantObserType[5] = prioData.L3Type.at(i);
            break;
        }
    }

}
//Read an epoch (can be expanded by version)
void QReadOFile::getEpochData(QVector< SatlitData > &epochData)
{
	if (RinexVersion >= 3)
    {
        readEpochVer3(epochData);
    }
	else if (RinexVersion < 3)
    {
        readEpochVer2(epochData);
    }
	return ;
}

//Read epochNum epoch data (premature to the bottom of the file may be less than epochNum)
void QReadOFile::getMultEpochData(QVector< QVector< SatlitData > >&multEpochData,int epochNum)
{
	if (epochNum < 1) return ;
	int i = 0;
	while(!m_readOFileClass.atEnd())
	{
		if (i++ >= epochNum) break;
		QVector< SatlitData > epochData;
		getEpochData(epochData);
        if(epochData.length() > 0)
            multEpochData.append(epochData);
	}
}
//Determine if the end of the file is reached (end)
bool QReadOFile::isEnd()
{
	return m_readOFileClass.atEnd();
}
void QReadOFile::closeFile()
{
    if(!m_IsCloseFile) m_readOFileClass.close();
}

////////////////////////////Get some header information below///////////////////////////////
//Get header file comment information
QString QReadOFile::getComment()
{
	return CommentInfo;
}
//Get approximate coordinates
void QReadOFile::getApproXYZ(double* AppXYZ)
{
	for (int i = 0;i < 3;i++)
		AppXYZ[i] = ApproxXYZ[i];
}

//Get the HEN correction of the antenna
void QReadOFile::getAntHEN(double* m_AntHEM)
{
	for (int i = 0;i < 3;i++)
		m_AntHEM[i] = AntHEN[i];
}
//Obtain the initial observation epoch time
void QReadOFile::getFistObsTime(int* m_YMDHM,double &Seconds)
{
	Seconds = ObsSeconds;
	for (int i = 0;i < 5;i++)
		m_YMDHM[i] = YMDHM[i];
}
//Get the antenna marker point name
QString QReadOFile::getMakerName()
{
	return MarkerName;
}
//Obtain the observation interval (unit s)
double QReadOFile::getInterval()
{
	return IntervalSeconds;
}
//Get the receiver antenna type
QString QReadOFile::getAntType()
{
	return AntType;

}
//Get receiver type
QString QReadOFile::getReciveType()
{
	return ReciverType;
}
