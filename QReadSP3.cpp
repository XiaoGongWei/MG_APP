#include "QReadSP3.h"

const int QReadSP3::lagrangeFact = 8;//Constant initialization


void QReadSP3::initVar()
{
	tempLine = "";
	isReadHead = false;
	isReadAllData = false;
	IsSigalFile = false;
	//Contains only a total of 32 satellites in the GPS system
	m_EndHeadNum = 8;
	m_lastGPSTimeFlag = 0;//As a starting marker
	m_lastCmpGPSTime = 999999999;//Record the GPS time of the initial calculation
	m_SP3FileName = "";
	m_WeekOrder = 0;
    ACCELERATE = true;
	InitStruct();

}

//Initialize the matrix inside the structure to be used (with memory saving)
void QReadSP3::InitStruct()
{
    int maxSatlitNum = 64;
	if (isInSystem('G'))
	{
        epochData.MatrixDataGPS.resize(maxSatlitNum,5);
		epochData.MatrixDataGPS.setZero();
	}
	if (isInSystem('R'))
	{
        epochData.MatrixDataGlonass.resize(maxSatlitNum,5);
		epochData.MatrixDataGlonass.setZero();
	}
	if (isInSystem('C'))
	{
        epochData.MatrixDataBDS.resize(maxSatlitNum,5);
		epochData.MatrixDataBDS.setZero();
	}
	if (isInSystem('E'))
	{
        epochData.MatrixDataGalieo.resize(maxSatlitNum,5);
		epochData.MatrixDataGalieo.setZero();
	}
}

QReadSP3::QReadSP3()
{
	initVar();
}
// init class
QReadSP3::QReadSP3(QStringList SP3FileNames)
{
	initVar();
	if (SP3FileNames.length() > 0)
	{
		m_SP3FileNames =SP3FileNames;
	}
	else
	{
		isReadAllData = true;
		return ; 
	}
}

// init class
void QReadSP3::setSP3FileNames(QStringList SP3FileNames)
{
    initVar();
    if (SP3FileNames.length() > 0)
    {
        m_SP3FileNames =SP3FileNames;
    }
    else
    {
        isReadAllData = true;
        return ;
    }
}

QReadSP3::~QReadSP3(void)
{
	releaseAllData();
}

//Go to GPS time
int QReadSP3::YMD2GPSTime(int Year,int Month,int Day,int HoursInt,int Minutes,int Seconds,int *GPSWeek)//,int *GPSTimeArray
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
	if (GPSWeek) *GPSWeek = Week;
	return (N*24*3600 + HoursInt*3600 + Minutes*60 + Seconds);
}


//Read multiple file data
void QReadSP3::readFileData2Vec(QStringList SP3FileNames)
{
	if (SP3FileNames.length() == 0) isReadAllData = true;
	if (isReadAllData)
		return;
	m_allEpochData.clear();
//First read the header file to read the file from small to large according to the time
	int minGPSWeek = 999999999;//Save the minimum week and reduce the int out of bounds (weeks are converted to seconds)
	QVector< int > tempGPSWeek,tempGPSSeconds,fileFlagSeconds;//Save the file to observe the actual time
	for (int i = 0;i < SP3FileNames.length();i++)
	{
		int Year =0,Month = 0,Day = 0,Hours = 0,Minutes = 0,GPSWeek = 0,GPSSeconds = 0;
		double Seconds = 0;
		QString Sp3FileName = SP3FileNames.at(i);
		QFile sp3file(Sp3FileName);
		if (!sp3file.open(QFile::ReadOnly))
		{
			QString erroInfo = "Open " + Sp3FileName + "faild!";
			ErroTrace(erroInfo);
		}
		//Read header file
		tempLine = sp3file.readLine();//Read the first line
		Year = tempLine.mid(3,4).toInt();
		Month = tempLine.mid(8,2).toInt();
		Day = tempLine.mid(11,2).toInt();
		Hours = tempLine.mid(14,2).toInt();
		Minutes = tempLine.mid(17,2).toInt();
		Seconds = tempLine.mid(20,11).toDouble();
		GPSSeconds = YMD2GPSTime(Year,Month,Day,Hours,Minutes,Seconds,&GPSWeek);
		if (GPSWeek <= minGPSWeek)
			minGPSWeek = GPSWeek;
		tempGPSWeek.append(GPSWeek);
		tempGPSSeconds.append(GPSSeconds);
		sp3file.close();
	}
	//Convert to seconds
	QVector< int > WeekOrder;//Save the data, mark whether it is cross-week, Cross: 1 does not span: 0 / / read multiple files need to use
	for (int i = 0;i < tempGPSWeek.length();i++)
	{
		int Seconds = (tempGPSWeek.at(i) - minGPSWeek)*604800 + tempGPSSeconds.at(i);
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
				QString tempFileName = SP3FileNames.at(i);
				SP3FileNames[i] = SP3FileNames.at(j);
				SP3FileNames[j] = tempFileName;
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
	for (int i = 0;i < SP3FileNames.length();i++)
	{
		QString Sp3FileName = SP3FileNames.at(i);
		//打开文件
		QFile sp3file(Sp3FileName);
		if (!sp3file.open(QFile::ReadOnly))
		{
			QString erroInfo = "Open " + Sp3FileName + "faild!";
			ErroTrace(erroInfo);
		}
//Read header file
		tempLine = sp3file.readLine();//Read the first line
		QString flagHeadEnd = "*";
		QString pre3Char = tempLine.mid(0,1);
		while (pre3Char != flagHeadEnd)
		{
			tempLine = sp3file.readLine();//Read the next line
			pre3Char = tempLine.mid(0,1);
		}

//Read data part
		//When entering tempLin for the first time, the first epoch is reached. * YYYY-MM.......
		int Year =0,Month = 0,Day = 0,Hours = 0,Minutes = 0,Week = 0;
		double Seconds = 0;
		while (!sp3file.atEnd())
		{
			//Judge whether to end
			if (tempLine.mid(0,3) == "EOF")
				break;
			//Read header calendar metadata
			epochData.MatrixDataGPS.setZero();//It is best to clear before use, although it increases the amount of calculation but is safe.
			epochData.MatrixDataGlonass.setZero();
			epochData.MatrixDataBDS.setZero();
			epochData.MatrixDataGalieo.setZero();
			Year = tempLine.mid(3,4).toInt();
			Month = tempLine.mid(8,2).toInt();
			Day = tempLine.mid(11,2).toInt();
			Hours = tempLine.mid(14,2).toInt();
			Minutes = tempLine.mid(17,2).toInt();
			Seconds = tempLine.mid(20,11).toDouble();
			epochData.GPSTime = YMD2GPSTime(Year,Month,Day,Hours,Minutes,Seconds,&Week) + WeekOrder.at(i)*604800;//Add 604800s across the week
			epochData.GPSWeek = Week;
			//Read satellite coordinates // read satellite coordinates
			tempLine = sp3file.readLine();//Read a row of coordinate data
			while (tempLine.mid(0,3) != "EOF"&&tempLine.mid(0,1) == "P")
			{
				int PRN = 0;
                char tempSatType = 'G';
				//GPS system
				tempSatType = *(tempLine.mid(1,1).toLatin1().data());
                if(tempSatType == 32) tempSatType = 'G';
				PRN = tempLine.mid(2,2).toInt();
				if (tempSatType == 'G'&&isInSystem(tempSatType))
				{
					epochData.MatrixDataGPS(PRN - 1,0) = tempLine.mid(2,2).toDouble();//PRN
					epochData.MatrixDataGPS(PRN - 1,1) = 1000*tempLine.mid(5,13).toDouble();//X
					epochData.MatrixDataGPS(PRN - 1,2) = 1000*tempLine.mid(19,13).toDouble();//Y
					epochData.MatrixDataGPS(PRN - 1,3) = 1000*tempLine.mid(33,13).toDouble();//Z
                    epochData.MatrixDataGPS(PRN - 1,4) = 1e-6*tempLine.mid(47,13).toDouble()*M_C;//sp3 Clk
				}
				//Glonass system
				else if (tempSatType == 'R'&&isInSystem(tempSatType))
				{
					epochData.MatrixDataGlonass(PRN - 1,0) = tempLine.mid(2,2).toDouble();//PRN
					epochData.MatrixDataGlonass(PRN - 1,1) = 1000*tempLine.mid(5,13).toDouble();//X
					epochData.MatrixDataGlonass(PRN - 1,2) = 1000*tempLine.mid(19,13).toDouble();//Y
					epochData.MatrixDataGlonass(PRN - 1,3) = 1000*tempLine.mid(33,13).toDouble();//Z
                    epochData.MatrixDataGlonass(PRN - 1,4) = 1e-6*tempLine.mid(47,13).toDouble()*M_C;//sp3 Clk
				}
				//BDS system
				else if (tempSatType == 'C'&&isInSystem(tempSatType))
				{
					epochData.MatrixDataBDS(PRN - 1,0) = tempLine.mid(2,2).toDouble();//PRN
					epochData.MatrixDataBDS(PRN - 1,1) = 1000*tempLine.mid(5,13).toDouble();//X
					epochData.MatrixDataBDS(PRN - 1,2) = 1000*tempLine.mid(19,13).toDouble();//Y
					epochData.MatrixDataBDS(PRN - 1,3) = 1000*tempLine.mid(33,13).toDouble();//Z
                    epochData.MatrixDataBDS(PRN - 1,4) = 1e-6*tempLine.mid(47,13).toDouble()*M_C;//sp3 Clk
				}
				//Galileo system
				else if (tempSatType == 'E'&&isInSystem(tempSatType))
				{
					epochData.MatrixDataGalieo(PRN - 1,0) = tempLine.mid(2,2).toDouble();//PRN
					epochData.MatrixDataGalieo(PRN - 1,1) = 1000*tempLine.mid(5,13).toDouble();//X
					epochData.MatrixDataGalieo(PRN - 1,2) = 1000*tempLine.mid(19,13).toDouble();//Y
					epochData.MatrixDataGalieo(PRN - 1,3) = 1000*tempLine.mid(33,13).toDouble();//Z
                    epochData.MatrixDataGalieo(PRN - 1,4) = 1e-6*tempLine.mid(47,13).toDouble()*M_C;//sp3 Clk
				}
				tempLine = sp3file.readLine();//Read a row of coordinate data
			}
            if (!m_allEpochData.empty() && m_allEpochData.back().GPSTime == epochData.GPSTime)
                m_allEpochData.back() = epochData;//Replace equal GPSTime data
            else
                m_allEpochData.append(epochData);//Save a file data
		}//End of reading file
		sp3file.close();
	}//for (int i = 0;i < SP3FileNames.length();i++)//Read multiple files at the end
	isReadAllData = true;
}

////pX,pY,pZ：lagrangeFact point coordinates ；pGPST: lagrangeFact points within GPS week seconds; GPST satellite launch time within weeks
void QReadSP3::get8Point(int PRN,char SatType,double *pX,double *pY,double *pZ,int *pGPST,double GPST ,double *pClk)
{//Get the nearest lagrangeFact points
	int lengthEpoch = m_allEpochData.length();
	//Discover the location of GPST throughout the epoch
	int GPSTflag = m_lastGPSTimeFlag;
	int numPoint = lagrangeFact / 2;//Take numPoint points before and after
	if (qAbs(m_lastCmpGPSTime - GPST) > 0.3)//More than 0.3s indicates that the epoch interval of the observation epoch is definitely above 1 s (reducing the multiple Query position of the same epoch)
	{
		if (ACCELERATE)	m_lastCmpGPSTime = GPST;
		for (int i = m_lastGPSTimeFlag;i < lengthEpoch;i++)
		{
			SP3Data epochData = m_allEpochData.at(i);
			if (epochData.GPSTime >= GPST)
				break; 
			else
				GPSTflag++;
		}
	}
	if (ACCELERATE) m_lastGPSTimeFlag = GPSTflag;//Save the latest location

//Take numPoint points before and after, consider the boundary problem
	if ((GPSTflag >= numPoint - 1) && (GPSTflag <= lengthEpoch - numPoint - 1))
	{//In front of the middle position four contain itself, the last four do not contain itself
		for (int i = 0;i < lagrangeFact;i++)
		{
			SP3Data epochData = m_allEpochData.at(GPSTflag - numPoint + 1 + i);
			//Determine which system data belongs to
			switch(SatType)
			{
			case 'G':
				pX[i] = epochData.MatrixDataGPS(PRN - 1,1);
				pY[i] = epochData.MatrixDataGPS(PRN - 1,2);
				pZ[i] = epochData.MatrixDataGPS(PRN - 1,3);
                pClk[i] = epochData.MatrixDataGPS(PRN - 1,4);
				pGPST[i] = epochData.GPSTime;
				break;
			case 'R':
				pX[i] = epochData.MatrixDataGlonass(PRN - 1,1);
				pY[i] = epochData.MatrixDataGlonass(PRN - 1,2);
				pZ[i] = epochData.MatrixDataGlonass(PRN - 1,3);
                pClk[i] = epochData.MatrixDataGlonass(PRN - 1,4);
				pGPST[i] = epochData.GPSTime;
				break;
			case 'C':
				pX[i] = epochData.MatrixDataBDS(PRN - 1,1);
				pY[i] = epochData.MatrixDataBDS(PRN - 1,2);
				pZ[i] = epochData.MatrixDataBDS(PRN - 1,3);
                pClk[i] = epochData.MatrixDataBDS(PRN - 1,4);
				pGPST[i] = epochData.GPSTime;
				break;
			case 'E':
				pX[i] = epochData.MatrixDataGalieo(PRN - 1,1);
				pY[i] = epochData.MatrixDataGalieo(PRN - 1,2);
				pZ[i] = epochData.MatrixDataGalieo(PRN - 1,3);
                pClk[i] = epochData.MatrixDataGalieo(PRN - 1,4);
				pGPST[i] = epochData.GPSTime;
				break;
			default:
				pX[i] = 0;
				pY[i] = 0;
				pZ[i] = 0;
                pClk[i] = 0;
				pGPST[i] = 0;
			}
		}
	}
	else if(GPSTflag < numPoint - 1)
	{//At the start position boundary
		for (int i = 0;i < lagrangeFact;i++)
		{
			SP3Data epochData = m_allEpochData.at(i);
			//Determine which system data belongs to
			switch(SatType)
			{
			case 'G':
				pX[i] = epochData.MatrixDataGPS(PRN - 1,1);
				pY[i] = epochData.MatrixDataGPS(PRN - 1,2);
				pZ[i] = epochData.MatrixDataGPS(PRN - 1,3);
                pClk[i] = epochData.MatrixDataGPS(PRN - 1,4);
				pGPST[i] = epochData.GPSTime;
				break;
			case 'R':
				pX[i] = epochData.MatrixDataGlonass(PRN - 1,1);
				pY[i] = epochData.MatrixDataGlonass(PRN - 1,2);
				pZ[i] = epochData.MatrixDataGlonass(PRN - 1,3);
                pClk[i] = epochData.MatrixDataGlonass(PRN - 1,4);
				pGPST[i] = epochData.GPSTime;
				break;
			case 'C':
				pX[i] = epochData.MatrixDataBDS(PRN - 1,1);
				pY[i] = epochData.MatrixDataBDS(PRN - 1,2);
				pZ[i] = epochData.MatrixDataBDS(PRN - 1,3);
                pClk[i] = epochData.MatrixDataBDS(PRN - 1,4);
				pGPST[i] = epochData.GPSTime;
				break;
			case 'E':
				pX[i] = epochData.MatrixDataGalieo(PRN - 1,1);
				pY[i] = epochData.MatrixDataGalieo(PRN - 1,2);
				pZ[i] = epochData.MatrixDataGalieo(PRN - 1,3);
                pClk[i] = epochData.MatrixDataGalieo(PRN - 1,4);
				pGPST[i] = epochData.GPSTime;
				break;
			default:
				pX[i] = 0;
				pY[i] = 0;
				pZ[i] = 0;
                pClk[i] = 0;
				pGPST[i] = 0;
			}
		}
	}
	else if(GPSTflag > lengthEpoch - numPoint - 1)
	{//End position boundary
		for (int i = 0;i < lagrangeFact;i++)
		{
			//debug:2017.07.08
			SP3Data epochData = m_allEpochData.at(lengthEpoch - (lagrangeFact-i));
			//Determine which system data belongs to
			switch(SatType)
			{
			case 'G':
				pX[i] = epochData.MatrixDataGPS(PRN - 1,1);
				pY[i] = epochData.MatrixDataGPS(PRN - 1,2);
				pZ[i] = epochData.MatrixDataGPS(PRN - 1,3);
                pClk[i] = epochData.MatrixDataGPS(PRN - 1,4);
				pGPST[i] = epochData.GPSTime;
				break;
			case 'R':
				pX[i] = epochData.MatrixDataGlonass(PRN - 1,1);
				pY[i] = epochData.MatrixDataGlonass(PRN - 1,2);
				pZ[i] = epochData.MatrixDataGlonass(PRN - 1,3);
                pClk[i] = epochData.MatrixDataGlonass(PRN - 1,4);
				pGPST[i] = epochData.GPSTime;
				break;
			case 'C':
				pX[i] = epochData.MatrixDataBDS(PRN - 1,1);
				pY[i] = epochData.MatrixDataBDS(PRN - 1,2);
				pZ[i] = epochData.MatrixDataBDS(PRN - 1,3);
                pClk[i] = epochData.MatrixDataBDS(PRN - 1,4);
				pGPST[i] = epochData.GPSTime;
				break;
			case 'E':
				pX[i] = epochData.MatrixDataGalieo(PRN - 1,1);
				pY[i] = epochData.MatrixDataGalieo(PRN - 1,2);
				pZ[i] = epochData.MatrixDataGalieo(PRN - 1,3);
                pClk[i] = epochData.MatrixDataGalieo(PRN - 1,4);
				pGPST[i] = epochData.GPSTime;
				break;
			default:
				pX[i] = 0;
				pY[i] = 0;
				pZ[i] = 0;
                pClk[i] = 0;
				pGPST[i] = 0;
			}
		}
	}
}

//Lagrangian method
void QReadSP3::lagrangeMethod(int PRN,char SatType,double GPST,double *pXYZ,double *pdXYZ, double *pSp3Clk)
{//Lagrangian interpolation, here and after the selection - a total of 8 points for interpolation. GPST-satellite launch time within seconds of the week
	for (int i = 0;i < 3;i++)
	{//Initialization, security
		pXYZ[i] = 0;
		pdXYZ[i] = 0;
	}
    if(pSp3Clk) *pSp3Clk = 0;
	//Determine whether to set up a legal system
	if (!isInSystem(SatType)) 
		return ;
	//Determine whether to read data
	if (!isReadAllData)
	{
        readFileData2Vec(m_SP3FileNames);
	}
    double pX[lagrangeFact]={0},pY[lagrangeFact] = {0},pZ[lagrangeFact] = {0}, pClk[lagrangeFact] = {0};
	int pGPST[lagrangeFact] = {0};
    get8Point(PRN,SatType,pX,pY,pZ,pGPST,GPST,pClk);//Obtain the coordinates of the last 8 points of the PRN satellite launch time
	//Check if the data is missing (the data in the orbit interpolation of the precise ephemeris cannot be missing)
	for (int i = 0;i <lagrangeFact;i++)
		if (!(pX[i]&&pY[i]&&pZ[i])) return ;

    double sumX = 0,sumY = 0,sumZ = 0, sumClk = 0;;//Interpolated coordinates XYZ
	double sumDX[2] = {0},sumDY[2] = {0},sumDZ[2] = {0};//Satellite coordinates of +-0.5s before and after interpolation are used to simultaneously determine the speed
	double lk = 1,ldk[2] = {1,1};
	for (int k = 0; k < lagrangeFact;k++)
	{
		for (int n = 0;n < lagrangeFact;n++)
		{
			if (k == n) continue;
			lk = lk*(GPST - pGPST[n])/(pGPST[k] - pGPST[n]);
			ldk[0] = ldk[0]*(GPST - 0.5 - pGPST[n])/(pGPST[k] - pGPST[n]);
			ldk[1] = ldk[1]*(GPST + 0.5 - pGPST[n])/(pGPST[k] - pGPST[n]);
		}
		sumX = sumX + pX[k]*lk;sumDX[0] = sumDX[0] + pX[k]*ldk[0];sumDX[1] = sumDX[1] + pX[k]*ldk[1];
		sumY = sumY + pY[k]*lk;sumDY[0] = sumDY[0] + pY[k]*ldk[0];sumDY[1] = sumDY[1] + pY[k]*ldk[1];
		sumZ = sumZ + pZ[k]*lk;sumDZ[0] = sumDZ[0] + pZ[k]*ldk[0];sumDZ[1] = sumDZ[1] + pZ[k]*ldk[1];
        sumClk = sumClk + pClk[k]*lk;

		lk = 1;ldk[0] = 1;ldk[1] = 1;
	}
	pXYZ[0] = sumX;pXYZ[1] = sumY;pXYZ[2] = sumZ;
    if(pSp3Clk) *pSp3Clk = sumClk;
	pdXYZ[0] = sumDX[1] - sumDX[0];pdXYZ[1] = sumDY[1] - sumDY[0];pdXYZ[2] = sumDZ[1] - sumDZ[0];
}

//Get precise ephemeris coordinates and speed
void QReadSP3::getPrcisePoint(int PRN, char SatType, double GPST, double *pXYZ, double *pdXYZ, double *pSp3Clk)
{
    readFileData2Vec(m_SP3FileNames);
	//If you are on the first day of GPS week, the GPST time needs to add 604800, the reason for reading the data.
	if (!IsSigalFile&&m_WeekOrder > 0)
	{
		if (GPST < 24*3600)  GPST += 604800;
	}
	//if (GPST < 24*3600)  GPST += 604800;//Consider not comprehensive
    lagrangeMethod(PRN,SatType,GPST,pXYZ,pdXYZ, pSp3Clk);
}

//Get all the data after reading
QVector< SP3Data > QReadSP3::getAllData()
{
	if (m_SP3FileNames.isEmpty()&&m_SP3FileName.isEmpty())
		return m_allEpochData;
    readFileData2Vec(m_SP3FileNames);
	return m_allEpochData;
}

//Release all data
void QReadSP3::releaseAllData()
{
	m_allEpochData.clear();
}

bool QReadSP3::setSatlitSys(QString SystemStr)
{
	bool IsGood = QBaseObject::setSatlitSys(SystemStr);
	InitStruct();
	return IsGood;
}
