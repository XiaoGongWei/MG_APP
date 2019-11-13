#include "QReadClk.h"


QReadClk::QReadClk()
{

}

QReadClk::QReadClk(QStringList ClkFileNames)
{
	initVar();
    if(ClkFileNames.length() == 1) IsSigalFile = true;
	if (ClkFileNames.length() > 0)
	{
		m_ClkFileNames =ClkFileNames;
	}
	else
	{
		isReadAllData = true;
		return ; 
	}
}

void QReadClk::setClkFileNames(QStringList ClkFileNames)
{
    initVar();
    if(ClkFileNames.length() == 1) IsSigalFile = true;
    if (ClkFileNames.length() > 0)
    {
        m_ClkFileNames =ClkFileNames;
    }
    else
    {
        isReadAllData = true;
        return ;
    }
}


//Destructed function
QReadClk::~QReadClk(void)
{
	releaseAllData();
}

void QReadClk::initVar()
{
	isReadHead = false;
	isReadAllData = false;
    IsSigalFile = false;
	m_ClkFileName = "";
	lagrangeFact = 10;
	//Contains only a total of 32 satellites in the GPS system
	m_lastGPSTimeFlag = 0;//As a starting marker
	m_lastCmpGPSTime = 999999999;//Record the GPS time of the initial calculation
	m_EndHeadNum = 8;
	m_WeekOrder = 0;
    ACCELERATE = true;// 1：Acceleration program (time is from small to large) 0：Do not speed up the program (time chaos)
	InitStruct();
}

//Initialize the matrix inside the structure to be used (with memory saving)
void QReadClk::InitStruct()
{
    int maxSatlitNum = 64;
	if (isInSystem('G'))
	{
        epochData.MatrixDataGPS.resize(maxSatlitNum,4);
		epochData.MatrixDataGPS.setZero();
	}
	if (isInSystem('R'))
	{
        epochData.MatrixDataGlonass.resize(maxSatlitNum,4);
		epochData.MatrixDataGlonass.setZero();
	}
	if (isInSystem('C'))
	{
        epochData.MatrixDataBDS.resize(maxSatlitNum,4);
		epochData.MatrixDataBDS.setZero();
	}
	if (isInSystem('E'))
	{
        epochData.MatrixDataGalieo.resize(maxSatlitNum,4);
		epochData.MatrixDataGalieo.setZero();
	}
}


//Go to GPS time
double QReadClk::YMD2GPSTime(int Year,int Month,int Day,int HoursInt,int Minutes,double Seconds,int *GPSWeek)//,int *GPSTimeArray
{
	double Hours = HoursInt + ((Minutes * 60) + Seconds)/3600.0;
	//Get JD
	double JD = 0.0;
	if(Month<=2)
		JD = (int)(365.25*(Year-1)) + (int)(30.6001*(Month+12+1)) + Day + Hours/24.0 + 1720981.5;
	else
		JD = (int)(365.25*(Year)) + (int)(30.6001*(Month+1)) + Day + Hours/24.0 + 1720981.5;

	int Week = (int)((JD - 2444244.5) / 7);
	int N =(int)(JD + 1.5)%7;
	if (GPSWeek)	*GPSWeek = Week;
	return (N*24*3600 + HoursInt*3600 + Minutes*60 + Seconds);
}

//Read header file
void QReadClk::readAllHead()
{
	if (isReadHead)
		return ;
	tempLine = m_readCLKFileClass.readLine();//Read the first line
	QString flagHeadEnd = "END";
	QString endHeadStr = tempLine.mid(60,20).trimmed();

	while (!endHeadStr.contains(flagHeadEnd,Qt::CaseInsensitive))
	{
		//Read header file data......
		//Skip header file here

		//Read the next line
		tempLine = m_readCLKFileClass.readLine();//Read the next line
		endHeadStr = tempLine.mid(60,20).trimmed();
	}
	tempLine = m_readCLKFileClass.readLine();//Read the next line into the data area
	isReadHead = true;
}

//Read all data from multiple files
void QReadClk::readFileData2Vec(QStringList ClkFileNames)
{
	if (ClkFileNames.length() == 0)	isReadAllData = true;
	if (isReadAllData)
		return;
	m_allEpochData.clear();
	//First read the header file to read the file from small to large according to the time
	int minGPSWeek = 999999999;//Save the minimum week and reduce the int out of bounds (weeks are converted to seconds)
	QVector< int > tempGPSWeek,fileFlagSeconds;//Save the file to observe the actual time
	QVector< double > tempGPSSeconds;
	for (int i = 0;i < ClkFileNames.length();i++)
	{
		int Year =0,Month = 0,Day = 0,Hours = 0,Minutes = 0,GPSWeek = 0;
		double Seconds = 0,GPSSeconds = 0;
		QString CLKFileName = ClkFileNames.at(i);
		QFile clkfile(CLKFileName);
		if (!clkfile.open(QFile::ReadOnly))
		{
			QString erroInfo = "Open " + CLKFileName + "faild!(QReadClk::readFileData2Vec)";
			ErroTrace(erroInfo);
		}
		//Skip header file
		do 
		{
			tempLine = clkfile.readLine();//Read the first line
			if (clkfile.atEnd())
			{
				ErroTrace("Can not read clk file!(QReadClk::readFileData2Vec)");
				break;
			}
        } while (!tempLine.contains("END OF HEADER",Qt::CaseInsensitive));
		//Find the time corresponding to the AS line
		do 
		{
			tempLine = clkfile.readLine();//Read the first line
			if (clkfile.atEnd())
			{
				ErroTrace("Can not read clk file!(QReadClk::readFileData2Vec)");
				break;
			}
		} while (!tempLine.mid(0,2).contains("AS",Qt::CaseInsensitive));
		//Get Time
		Year = tempLine.mid(8,4).toInt();
		Month = tempLine.mid(13,2).toInt();
		Day = tempLine.mid(16,2).toInt();
		Hours = tempLine.mid(19,2).toInt();
		Minutes = tempLine.mid(22,2).toInt();
		Seconds = tempLine.mid(25,11).toDouble();
		GPSSeconds = YMD2GPSTime(Year,Month,Day,Hours,Minutes,Seconds,&GPSWeek);
		if (GPSWeek <= minGPSWeek)
			minGPSWeek = GPSWeek;
		tempGPSWeek.append(GPSWeek);
		tempGPSSeconds.append(GPSSeconds);
		clkfile.close();
	}
	//Convert to seconds
	QVector< int > WeekOrder;//Save data, mark whether cross-week，Cross: 1 does not span: 0 / / read multiple files need to use
	for (int i = 0;i < tempGPSWeek.length();i++)
	{
		double Seconds = (tempGPSWeek.at(i) - minGPSWeek)*604800 + tempGPSSeconds.at(i);
		WeekOrder.append(tempGPSWeek.at(i) - minGPSWeek);
		fileFlagSeconds.append(Seconds);
	}
	//Sort file names by time
	for (int i = 0; i < fileFlagSeconds.length();i++)
	{
		for (int j = i+1;j < fileFlagSeconds.length();j++)
		{
			if (fileFlagSeconds.at(j) < fileFlagSeconds.at(i))//exchange
			{
				//Exchange file name
				QString tempFileName = ClkFileNames.at(i);
				ClkFileNames[i] = ClkFileNames.at(j);
				ClkFileNames[j] = tempFileName;
				//Exchange week sign
				int tempWeek = WeekOrder.at(i);
				WeekOrder[i] = WeekOrder.at(j);
				WeekOrder[j] = tempWeek;
			}
		}
	}
	m_WeekOrder = 0;
	for (int i = 0;i < WeekOrder.length();i++)
	{
		m_WeekOrder += WeekOrder[i];//Save cross-week mark
	}
//Read all files by time
	for (int i = 0;i < ClkFileNames.length();i++)
	{
		QString CLKFileName = ClkFileNames.at(i);
//open a file
		QFile clkfile(CLKFileName);
		if (!clkfile.open(QFile::ReadOnly))
		{
			QString erroInfo = "Open " + CLKFileName + "faild!(QReadClk::readFileData2Vec)";
			ErroTrace(erroInfo);
		}
//Read header file data
		tempLine = clkfile.readLine();//Read the first line
		QString endHeadStr = tempLine.mid(60,20).trimmed();
        while (!endHeadStr.contains("END OF HEADER",Qt::CaseInsensitive))
		{
            //Do header file data read here skip header file
			tempLine = clkfile.readLine();//Read the next line
			endHeadStr = tempLine.mid(60,20).trimmed();
		}
		tempLine = clkfile.readLine();//Read the next line into the data area

//Read data area
		//First time tempLine AR......
		int Year =0,Month = 0,Day = 0,Hours = 0,Minutes = 0,Week = 0;
        double Seconds = 0, temp_gps_time = 0;// debug 2018.11.15 xiaogongwei
		while (!clkfile.atEnd())
		{
			//Skip the area that is not the beginning of the clock error
			while(tempLine.mid(0,3) != "AS ")
				tempLine = clkfile.readLine();
			//Read an epoch error
			epochData.MatrixDataGPS.setZero();//It is best to clear before use, although it increases the amount of calculation but is safe.
			epochData.MatrixDataGlonass.setZero();
			epochData.MatrixDataBDS.setZero();
            epochData.MatrixDataGalieo.setZero();
            // init temp_gps_time
            Year = tempLine.mid(8,4).toInt();
            Month = tempLine.mid(13,2).toInt();
            Day = tempLine.mid(16,2).toInt();
            Hours = tempLine.mid(19,2).toInt();
            Minutes = tempLine.mid(22,2).toInt();
            Seconds = tempLine.mid(25,9).toDouble();
            epochData.GPSTime = YMD2GPSTime(Year,Month,Day,Hours,Minutes,Seconds,&Week) + WeekOrder.at(i)*604800;//Add 604800s across the week
            epochData.GPSWeek = Week;
            temp_gps_time = epochData.GPSTime;// debug 2018.11.15 xiaogongwei

			while (tempLine.mid(0,3) == "AS ")
			{//Enter the first epoch
				int PRN = 0;
                char tempSatType = 'G';
                bool isReadLineData = false;
				tempSatType = *(tempLine.mid(3,1).toLatin1().data());
				//GPS system
				if (tempSatType == 'G'&&isInSystem(tempSatType))
				{
					//Read year, month, day, hour, minute, second
					Year = tempLine.mid(8,4).toInt();
					Month = tempLine.mid(13,2).toInt();
					Day = tempLine.mid(16,2).toInt();
					Hours = tempLine.mid(19,2).toInt();
					Minutes = tempLine.mid(22,2).toInt();
					Seconds = tempLine.mid(25,9).toDouble();
					epochData.GPSTime = YMD2GPSTime(Year,Month,Day,Hours,Minutes,Seconds,&Week) + WeekOrder.at(i)*604800;//Add 604800s across the week
					epochData.GPSWeek = Week;
                    temp_gps_time = epochData.GPSTime;// debug 2018.11.15 xiaogongwei
					//Read satellite PRN, clock error, medium error
					PRN = tempLine.mid(4,2).toInt();//PRN
					epochData.MatrixDataGPS(PRN - 1,0) = PRN;
					epochData.MatrixDataGPS(PRN - 1,1) = tempLine.mid(40,20).toDouble();//Clock error
					epochData.MatrixDataGPS(PRN - 1,2) = tempLine.mid(60,20).toDouble();//Medium error
                    isReadLineData = true;
				}
				//Glonass system
				else if (tempSatType == 'R'&&isInSystem(tempSatType))
				{
					//Read year, month, day, hour, minute, second
					Year = tempLine.mid(8,4).toInt();
					Month = tempLine.mid(13,2).toInt();
					Day = tempLine.mid(16,2).toInt();
					Hours = tempLine.mid(19,2).toInt();
					Minutes = tempLine.mid(22,2).toInt();
					Seconds = tempLine.mid(25,9).toDouble();
					epochData.GPSTime = YMD2GPSTime(Year,Month,Day,Hours,Minutes,Seconds,&Week) + WeekOrder.at(i)*604800;//Add 604800s across the week
					epochData.GPSWeek = Week;
                    temp_gps_time = epochData.GPSTime;// debug 2018.11.15 xiaogongwei
					//Read satellite PRN, clock error, medium error
					PRN = tempLine.mid(4,2).toInt();//PRN
					epochData.MatrixDataGlonass(PRN - 1,0) = PRN;
					epochData.MatrixDataGlonass(PRN - 1,1) = tempLine.mid(40,20).toDouble();//Clock error
					epochData.MatrixDataGlonass(PRN - 1,2) = tempLine.mid(60,20).toDouble();//Medium error
                    isReadLineData = true;
				}
				//BDS system
				else if (tempSatType == 'C'&&isInSystem(tempSatType))
				{
					//Read year, month, day, hour, minute, second
					Year = tempLine.mid(8,4).toInt();
					Month = tempLine.mid(13,2).toInt();
					Day = tempLine.mid(16,2).toInt();
					Hours = tempLine.mid(19,2).toInt();
					Minutes = tempLine.mid(22,2).toInt();
					Seconds = tempLine.mid(25,9).toDouble();
					epochData.GPSTime = YMD2GPSTime(Year,Month,Day,Hours,Minutes,Seconds,&Week) + WeekOrder.at(i)*604800;//Add 604800s across the week
					epochData.GPSWeek = Week;
                    temp_gps_time = epochData.GPSTime;// debug 2018.11.15 xiaogongwei
					//Read satellite PRN, clock error, medium error
					PRN = tempLine.mid(4,2).toInt();//PRN
					epochData.MatrixDataBDS(PRN - 1,0) = PRN;
					epochData.MatrixDataBDS(PRN - 1,1) = tempLine.mid(40,20).toDouble();//Clock error
					epochData.MatrixDataBDS(PRN - 1,2) = tempLine.mid(60,20).toDouble();//Medium error
                    isReadLineData = true;
				}
				//Galileo system
				else if (tempSatType == 'E'&&isInSystem(tempSatType))
				{
					//Read year, month, day, hour, minute, second
					Year = tempLine.mid(8,4).toInt();
					Month = tempLine.mid(13,2).toInt();
					Day = tempLine.mid(16,2).toInt();
					Hours = tempLine.mid(19,2).toInt();
					Minutes = tempLine.mid(22,2).toInt();
					Seconds = tempLine.mid(25,9).toDouble();
					epochData.GPSTime = YMD2GPSTime(Year,Month,Day,Hours,Minutes,Seconds,&Week) + WeekOrder.at(i)*604800;//Add 604800s across the week
					epochData.GPSWeek = Week;
                    temp_gps_time = epochData.GPSTime;// debug 2018.11.15 xiaogongwei
					//Read satellite PRN, clock error, medium error
					PRN = tempLine.mid(4,2).toInt();//PRN
					epochData.MatrixDataGalieo(PRN - 1,0) = PRN;
					epochData.MatrixDataGalieo(PRN - 1,1) = tempLine.mid(40,20).toDouble();//Clock error
                    epochData.MatrixDataGalieo(PRN - 1,2) = tempLine.mid(60,20).toDouble();//Medium error
                    isReadLineData = true;
				}
				tempLine = clkfile.readLine();//Read a row of coordinate data
                // use temp_gps_time juge epoch is become  debug 2018.11.15 xiaogongwei
                if(tempLine.mid(0,3) == "AS " && temp_gps_time != 0)
                {
                    double cureent_gps_time = 0;
                    Year = tempLine.mid(8,4).toInt();
                    Month = tempLine.mid(13,2).toInt();
                    Day = tempLine.mid(16,2).toInt();
                    Hours = tempLine.mid(19,2).toInt();
                    Minutes = tempLine.mid(22,2).toInt();
                    Seconds = tempLine.mid(25,9).toDouble();
                    cureent_gps_time = YMD2GPSTime(Year,Month,Day,Hours,Minutes,Seconds,&Week) + WeekOrder.at(i)*604800;//Add 604800s across the week
                    if(cureent_gps_time != temp_gps_time)
                        break;
                }
			}
            m_allEpochData.append(epochData);
		}//End of reading file data   while (!clkfile.atEnd())
		clkfile.close();
	}//for (int i = 0;i < ClkFileNames.length();i++)
	isReadAllData = true;
	//Determine the clock error epoch interval to determine the number of interpolation points
	CLKData epoch1 = m_allEpochData.at(20);
	CLKData epoch2 = m_allEpochData.at(21);
	if (qAbs(epoch1.GPSTime - epoch2.GPSTime) < 60)
		lagrangeFact = 2;
}

//Return data
QVector< CLKData > QReadClk::getAllData()
{
    readFileData2Vec(m_ClkFileNames);
	return m_allEpochData;
}

//Release all data
void QReadClk::releaseAllData()
{
	m_allEpochData.clear();
}

////pX,pY,pZ：lagrangeFact point coordinates ；pGPST:lagrangeFact points GPS week seconds;GPST Satellite launch time within seconds of the week
void QReadClk::get8Point(int PRN,char SatType,double *pCLKT,double *pGPST,double GPST)
{//Get the nearest lagrangeFact points
	//Check for acceleration
	int lengthEpoch = m_allEpochData.length();
	//Discover the location of GPST throughout the epoch
	int GPSTflag = m_lastGPSTimeFlag;
	int numPoint = lagrangeFact / 2;//Take numPoint points before and after
	if (qAbs(m_lastCmpGPSTime - GPST) > 0.3)//More than 0.3s indicates that the epoch interval of the observation epoch is definitely above 1 s (reducing the multiple Query position of the same epoch)
	{
		if (ACCELERATE)	m_lastCmpGPSTime = GPST;
		for (int i = m_lastGPSTimeFlag;i < lengthEpoch;i++)
		{
			CLKData epochData = m_allEpochData.at(i);
			if (epochData.GPSTime >= GPST)
				break; 
			else
				GPSTflag++;
		}
		//Two-point interpolation occurs because the satellite propagates for 0.00n seconds, causing the GPSTflag to be 1 more.
		if (numPoint == 1) GPSTflag--;
	}



//Take lagrangeFact points before and after  Consider the boundary problem

	if (GPSTflag < 0) GPSTflag = 0;
	if (ACCELERATE)	m_lastGPSTimeFlag = GPSTflag;//Save the latest location

	if ((GPSTflag >= numPoint - 1) && (GPSTflag <= lengthEpoch - numPoint - 1))
	{//In front of the middle position four contain itself, the last four do not contain itself
		for (int i = 0;i < lagrangeFact;i++)
		{
			CLKData epochData = m_allEpochData.at(GPSTflag - numPoint + 1 + i);
			//Determine which system data belongs to
			switch (SatType)
			{
			case 'G':
				pCLKT[i] = epochData.MatrixDataGPS(PRN - 1,1);
				pGPST[i] = epochData.GPSTime;
				break;
			case 'R':
				pCLKT[i] = epochData.MatrixDataGlonass(PRN - 1,1);
				pGPST[i] = epochData.GPSTime;
				break;
			case 'C':
				pCLKT[i] = epochData.MatrixDataBDS(PRN - 1,1);
				pGPST[i] = epochData.GPSTime;
				break;
			case 'E':
				pCLKT[i] = epochData.MatrixDataGalieo(PRN - 1,1);
				pGPST[i] = epochData.GPSTime;
				break;
			default:
				pCLKT[i] = 0;
				pGPST[i] = 0;
			}
		}
	}
	else if(GPSTflag < numPoint - 1)
	{//At the start position boundary
		for (int i = 0;i < lagrangeFact;i++)
		{
			CLKData epochData = m_allEpochData.at(i);
			//Determine which system data belongs to
			switch (SatType)
			{
			case 'G':
				pCLKT[i] = epochData.MatrixDataGPS(PRN - 1,1);
				pGPST[i] = epochData.GPSTime;
				break;
			case 'R':
				pCLKT[i] = epochData.MatrixDataGlonass(PRN - 1,1);
				pGPST[i] = epochData.GPSTime;
				break;
			case 'C':
				pCLKT[i] = epochData.MatrixDataBDS(PRN - 1,1);
				pGPST[i] = epochData.GPSTime;
				break;
			case 'E':
				pCLKT[i] = epochData.MatrixDataGalieo(PRN - 1,1);
				pGPST[i] = epochData.GPSTime;
				break;
			default:
				pCLKT[i] = 0;
				pGPST[i] = 0;
			}
		}
	}
	else if(GPSTflag > lengthEpoch - numPoint - 1)
	{//End position boundary
		for (int i = 0;i < lagrangeFact;i++)
		{
			//debug:2017.07.08
			CLKData epochData = m_allEpochData.at(lengthEpoch - (lagrangeFact-i));
			//Determine which system data belongs to
			switch (SatType)
			{
			case 'G':
				pCLKT[i] = epochData.MatrixDataGPS(PRN - 1,1);
				pGPST[i] = epochData.GPSTime;
				break;
			case 'R':
				pCLKT[i] = epochData.MatrixDataGlonass(PRN - 1,1);
				pGPST[i] = epochData.GPSTime;
				break;
			case 'C':
				pCLKT[i] = epochData.MatrixDataBDS(PRN - 1,1);
				pGPST[i] = epochData.GPSTime;
				break;
			case 'E':
				pCLKT[i] = epochData.MatrixDataGalieo(PRN - 1,1);
				pGPST[i] = epochData.GPSTime;
				break;
			default:
				pCLKT[i] = 0;
				pGPST[i] = 0;
			}
		}
	}
}


//Lagrangian method
void QReadClk::lagrangeMethod(int PRN,char SatType,double GPST,double *pCLKT)
{//Lagrangian interpolation, here before and after the selection - a total of 8 points for interpolation GPST satellite launch time week seconds
	*pCLKT = 0;
	//Determine whether to set up a legal system
	if (!isInSystem(SatType)) 
		return ;
	//Determine whether to read data
	if (!isReadAllData) 
	{
        readFileData2Vec(m_ClkFileNames);
	}
	double m_CLKT[12] = {0};//Save the acquired interpolation data
	double m_pGPST[12] = {0};
	get8Point(PRN,SatType,m_CLKT,m_pGPST,GPST);//Obtain the last 8 or 2 o'clock difference of the PRN satellite launch time
	//Verify data integrity (data cannot be missing in precision clock-interpolation)
	for (int i = 0;i <lagrangeFact;i++)
		if (!m_CLKT[i]) return ;
	//Lagrangian interpolation
	double sumCLK = 0;//Interpolated coordinates XYZ
	for (int k = 0; k < lagrangeFact;k++)
	{
		double lk = 1;
		for (int n = 0;n < lagrangeFact;n++)
		{
			if (k == n) continue;
			lk *= (GPST - m_pGPST[n])/(m_pGPST[k] - m_pGPST[n]);
		}
		sumCLK += m_CLKT[k]*lk;
	}
	*pCLKT = sumCLK*M_C;
}



//Obtain the satellite clock error at the time of transmission (multi-system)
void QReadClk::getStaliteClk(int PRN,char SatType,double GPST,double *pCLKT)
{
    readFileData2Vec(m_ClkFileNames);
	//If you are on the first day of GPS week, the GPST time needs to add 604800, the reason for reading the data.
	if (!IsSigalFile&&m_WeekOrder > 0)
	{
		if (GPST < 24*3600)  GPST += 604800;
	}
	//if (GPST < 24*3600)  GPST += 604800;//Consider not comprehensive
	lagrangeMethod(PRN,SatType,GPST,pCLKT);
}

bool QReadClk::setSatlitSys(QString SystemStr)
{
	bool IsGood = QBaseObject::setSatlitSys(SystemStr);
	InitStruct();
	return IsGood;
}
