#include "QReadGPSN.h"


QReadGPSN::QReadGPSN(void)
{
}


QReadGPSN::~QReadGPSN(void)
{
}

void QReadGPSN::initVar()
{
	isReadHead = false;
    isReadAllData = false;
    m_epochDataNum_Ver2 = 7;
	m_leapSec = 0;
	m_BaseYear = 2000;
	IonAlpha[0] = 0;IonAlpha[1] = 0;IonAlpha[2] = 0;IonAlpha[3] = 0;
	IonBeta[0] = 0;IonBeta[1] = 0;IonBeta[2] = 0;IonBeta[3] = 0;
	DeltaA01[0] = 0;DeltaA01[1] = 0;
	DeltaTW[0] = 0;DeltaTW[1] = 0;
}

QReadGPSN::QReadGPSN(QString NFileName)
{
	initVar();
	if (NFileName.trimmed().isEmpty())
		ErroTrace("File Name is Empty!(QReadGPSN::QReadGPSN(QString NFileName))");
	m_NfileName = NFileName;

	//Read file header information and parse
	getHeadInf();//The file is not closed here
	//GLONASS has only three rows and 12
    if (FileIdType == 'R')	m_epochDataNum_Ver2 = 3;
}

void QReadGPSN::setFileName(QString NFileName)
{
    initVar();
    if (NFileName.trimmed().isEmpty())
        ErroTrace("File Name is Empty!(QReadGPSN::QReadGPSN(QString NFileName))");
    m_NfileName = NFileName;

    //Read file header information and parse
    getHeadInf();//The file is not closed here
    //GLONASS has only three rows and has 12 data
    if (FileIdType == 'R')	m_epochDataNum_Ver2 = 3;
}

//Read the header file of the navigation file
void QReadGPSN::getHeadInf()
{
	if (isReadHead)	return ;
	//open a file
	m_readGPSNFile.setFileName(m_NfileName);
	if (!m_readGPSNFile.open(QFile::ReadOnly))
		ErroTrace("Open file bad!(QReadOFile::QReadOFile(QString OfileName))");
	//Read header file
	while (!m_readGPSNFile.atEnd())
	{
		tempLine = m_readGPSNFile.readLine();
		if (tempLine.contains("END OF HEADER",Qt::CaseInsensitive))
			break;
		if (tempLine.contains("RINEX VERSION",Qt::CaseInsensitive))
		{

			RinexVersion =  tempLine.mid(0,10).trimmed().toDouble();
			if (tempLine.mid(20,20).contains("GPS",Qt::CaseInsensitive))
				FileIdType = 'G';
			else if (tempLine.mid(20,20).contains("CMP",Qt::CaseInsensitive))
				FileIdType = 'C';
			else if (tempLine.mid(20,20).contains("GLONASS",Qt::CaseInsensitive))
				FileIdType = 'R';
            else
                FileIdType = 'G';
		} 
		else if (tempLine.contains("PGM / RUN BY / DATE",Qt::CaseInsensitive))
		{
			PGM = tempLine.mid(0,20).trimmed();
			RUNBY = tempLine.mid(20,20).trimmed();
			CreatFileDate = tempLine.mid(40,20).trimmed();
			//qDebug()<<CreatFileDate;
		}
		else if (tempLine.contains("COMMENT",Qt::CaseInsensitive))
		{
			CommentInfo+=tempLine.mid(0,60).trimmed() + "\n";
			//qDebug()<<CommentInfo;
		}
		else if (tempLine.contains("ION ALPHA",Qt::CaseInsensitive))
		{
			for(int i = 0;i < 4;i++)
			{
				QString strReplaceTemp = tempLine.mid(2+i*12,12).replace('D','E').trimmed();
				IonAlpha[i] = strReplaceTemp.toDouble();
			}
		}
		else if (tempLine.contains("ION BETA",Qt::CaseInsensitive))
		{
			for(int i = 0;i < 4;i++)
			{
				QString strReplaceTemp = tempLine.mid(2+i*12,12).replace('D','E').trimmed();
				IonBeta[i] = strReplaceTemp.toDouble();
			}
		}
		else if (tempLine.contains("DELTA-UTC",Qt::CaseInsensitive))
		{
			for(int i = 0;i < 2;i++)
			{
				QString strReplaceTemp = tempLine.mid(3+i*19,19).replace('D','E').trimmed();
				DeltaA01[i] = strReplaceTemp.toDouble();
				DeltaTW[i] = tempLine.mid(41+9*i,9).trimmed().toInt();
			}
		}
	}//End of read header file
	isReadHead = true;
}

//Read Rilex 2.X broadcast ephemeris data
void QReadGPSN::readNFileVer2(QVector< BrdData > &allBrdData)
{
	if (!isReadHead) getHeadInf();
	if (isReadAllData) return ;	
	tempLine = m_readGPSNFile.readLine();
	//Enter data area to read
	while (!m_readGPSNFile.atEnd())
	{
		BrdData epochBrdData;
		if (!tempLine.mid(0,2).trimmed().isEmpty())
		{
			tempLine.replace('D','E');
			epochBrdData.PRN = tempLine.mid(0,2).toInt();
			epochBrdData.SatType = FileIdType;
			epochBrdData.UTCTime.Year = tempLine.mid(3,2).toInt() + m_BaseYear;//m_BaseYear is set to 2000
			epochBrdData.UTCTime.Month = tempLine.mid(6,2).toInt();
			epochBrdData.UTCTime.Day = tempLine.mid(9,2).toInt();
			epochBrdData.UTCTime.Hours = tempLine.mid(12,2).toInt();
			epochBrdData.UTCTime.Minutes = tempLine.mid(15,2).toInt();
			epochBrdData.UTCTime.Seconds = tempLine.mid(17,5).toDouble();
			epochBrdData.TimeDiv = tempLine.mid(22,19).toDouble();
			epochBrdData.TimeMove = tempLine.mid(41,19).toDouble();
            epochBrdData.TimeMoveSpeed = 0;
            if(epochBrdData.SatType != 'R')
            {
                epochBrdData.TimeMoveSpeed = tempLine.mid(60,19).toDouble();
            }


			//Read the next row of data
			tempLine = m_readGPSNFile.readLine();
			tempLine.replace('D','E');
			double tempdb = 0.0;
            for (int i = 0; i < m_epochDataNum_Ver2;i++)
			{
				for (int j = 0;j < 4;j++)
				{
					tempdb = tempLine.mid(3 + j*19,19).toDouble();
					epochBrdData.epochNData.append(tempdb);
				}
				tempLine = m_readGPSNFile.readLine();
				tempLine.replace('D','E');
			}
			allBrdData.append(epochBrdData);//Save a data segment
		}
		else
		{
			continue;
		}
	}//while (!m_readGPSNFile.atEnd())Read to the end of the file
	isReadAllData = true;
	//Calculating leap seconds
	BrdData fistEpoch = allBrdData.at(0);
	m_leapSec = getLeapSecond(fistEpoch.UTCTime.Year,fistEpoch.UTCTime.Month,fistEpoch.UTCTime.Day,
		fistEpoch.UTCTime.Hours,fistEpoch.UTCTime.Minutes,fistEpoch.UTCTime.Seconds);
}

//Read Rilex 3.X broadcast ephemeris data
void QReadGPSN::readNFileVer3(QVector< BrdData > &allBrdData)
{
    int m_epochDataNum_Ver3 = 7;//Store a data segment (GPS and BDS are 28 7 rows GLONASS is 12 3 rows)
    if (!isReadHead) getHeadInf();
    if (isReadAllData) return ;
    tempLine = m_readGPSNFile.readLine();
    //Enter data area to read
    while (!m_readGPSNFile.atEnd())
    {
        BrdData epochBrdData;
        if (!tempLine.mid(0,3).trimmed().isEmpty())
        {
            tempLine.replace('D','E');
            epochBrdData.SatType = *(tempLine.mid(0,1).toLatin1().data());
            epochBrdData.PRN = tempLine.mid(1,2).toInt();
            epochBrdData.UTCTime.Year = tempLine.mid(4,4).toInt();
            epochBrdData.UTCTime.Month = tempLine.mid(9,2).toInt();
            epochBrdData.UTCTime.Day = tempLine.mid(12,2).toInt();
            epochBrdData.UTCTime.Hours = tempLine.mid(15,2).toInt();
            epochBrdData.UTCTime.Minutes = tempLine.mid(18,2).toInt();
            epochBrdData.UTCTime.Seconds = tempLine.mid(21,2).toDouble();
            epochBrdData.TimeDiv = tempLine.mid(23,19).toDouble();
            epochBrdData.TimeMove = tempLine.mid(42,19).toDouble();
            epochBrdData.TimeMoveSpeed = 0;
            if(epochBrdData.SatType == 'R')
                m_epochDataNum_Ver3 = 3;
            else
            {
                m_epochDataNum_Ver3 = 7;
                epochBrdData.TimeMoveSpeed = tempLine.mid(61,19).toDouble();
            }

            //Read the next row of data
            tempLine = m_readGPSNFile.readLine();
            tempLine.replace('D','E');
            double tempdb = 0.0;
            for (int i = 0; i < m_epochDataNum_Ver3;i++)
            {
                for (int j = 0;j < 4;j++)
                {
                    tempdb = tempLine.mid(4 + j*19,19).toDouble();
                    epochBrdData.epochNData.append(tempdb);
                }
                tempLine = m_readGPSNFile.readLine();
                tempLine.replace('D','E');
            }
            allBrdData.append(epochBrdData);//Save a data segment
        }
        else
        {
            continue;
        }
    }//while (!m_readGPSNFile.atEnd())Read to the end of the file
    isReadAllData = true;
    //Calculating leap seconds
    BrdData fistEpoch = allBrdData.at(0);
    m_leapSec = getLeapSecond(fistEpoch.UTCTime.Year,fistEpoch.UTCTime.Month,fistEpoch.UTCTime.Day,
        fistEpoch.UTCTime.Hours,fistEpoch.UTCTime.Minutes,fistEpoch.UTCTime.Seconds);
}

//Read all broadcast ephemeris data to allBrdData
QVector< BrdData > QReadGPSN::getAllData()
{
	if (isReadAllData) return m_allBrdData;	
	if (RinexVersion < 3.0)
	{
		readNFileVer2(m_allBrdData);
	}
    if (RinexVersion >= 3.0)
    {
        readNFileVer3(m_allBrdData);
    }

	return m_allBrdData;
}

//Search for the most recent navigation data
int QReadGPSN::SearchNFile(int PRN,char SatType,double GPSOTime)
{//Return matching N navigation message
	//Find the distance 2h before the navigation message
	int flag = 0;
	int lenNHead = m_allBrdData.length();

	QVector< int > Fileflag;
	QVector< double > FileflagTime;
	for (int i = 0;i < lenNHead;i++)
	{
		BrdData epochNData = m_allBrdData.at(i);
		if (PRN != epochNData.PRN || SatType != epochNData.SatType)
			continue;
		else
		{
			double GPSNTime = YMD2GPSTime(epochNData.UTCTime.Year,epochNData.UTCTime.Month,epochNData.UTCTime.Day,
				epochNData.UTCTime.Hours,epochNData.UTCTime.Minutes,epochNData.UTCTime.Seconds);
			Fileflag.append(i);
			FileflagTime.append(GPSNTime);
		}
	}
	//Found a minimum time difference <2h
	int FileflagLen = Fileflag.length();
	if (FileflagLen != FileflagTime.length())
		return -1;
	int flagMin = 0;
	if(FileflagLen == 0)
		return -1;
	double GPSNTime = FileflagTime.at(0);
    double Min = qAbs(GPSOTime - GPSNTime);
	for (int i = 0;i < FileflagLen;i++)
	{
		double tGPSNTime = FileflagTime.at(i);
		if (Min > qAbs((GPSOTime - tGPSNTime)))
		{
			flagMin = i;
			Min = GPSOTime - tGPSNTime;
		}
	}
	flag = Fileflag.at(flagMin);
	//The following should be added with the absolute value qAbs()
	if (qAbs(GPSOTime - FileflagTime.at(flagMin)) > 2*3600+120)
	{
		flag = -1;
	}
	return flag;//-1 means not found
}

//PRN: satellite number, SatType: satellite type (G, C, R, E), year, month, day, minute, minute, second, observation time, UTC time (internal automatic conversion of BDS and GLONASS functions)
void QReadGPSN::getSatPos(int PRN,char SatType,double signal_transmission_time,int Year,int Month,int Day,int Hours,int Minutes,
                          double Seconds,double *StaClock, double *pXYZ,double *pdXYZ)
{
	pXYZ[0] = 0;pXYZ[1] = 0;pXYZ[2] = 0;
	pdXYZ[0] = 0;pdXYZ[1] = 0;pdXYZ[2] = 0;
	if (!isReadAllData)  getAllData();
	double GPSOTime = YMD2GPSTime(Year,Month,Day,Hours,Minutes,Seconds);
	int flag = -1;
	flag = SearchNFile(PRN,SatType,GPSOTime);//Match navigation file
	if (flag < 0) return ;

	BrdData epochBrdData = m_allBrdData.at(flag);

	//Calculate multi-system (GPS+BDS+GLONASS) coordinates
	double X = 0,Y = 0,Z = 0;//Satellite coordinates
	double dX = 0,dY = 0,dZ = 0;//Satellite speed
	double t = 0;//GPS time at the time of satellite signal transmission
	double Ek = 0;//E to be calculated

    if (SatType == 'G' || SatType == 'C' || SatType == 'E')
	{
		double n0 = qSqrt(M_GM)/qPow(epochBrdData.epochNData.at(7),3);
		double n = n0 + epochBrdData.epochNData.at(2);
        t = GPSOTime - signal_transmission_time;//The time at which the satellite signals the second of the GPS week
		if (SatType == 'C')
		{
			t -= 14;
		}
		double dltt_toe = t - epochBrdData.epochNData.at(8);
		if (dltt_toe > 302400)
			dltt_toe-=604800;
		else if (dltt_toe < -302400)
			dltt_toe+=604800;
		double M = epochBrdData.epochNData.at(3) + n*dltt_toe;
		if (M < 0)
			M = M + 2*MM_PI;
		//Use iteration to calculate E
		double eps = 1e-13;
		double dv = 9999;
		double E0 = M;
		while(dv > eps)
		{
			Ek = M + epochBrdData.epochNData.at(5)*qSin(E0);
			dv = qAbs(Ek - E0);
			E0 = Ek;
		}
		//Calculate f
		double cosf = (qCos(Ek) - epochBrdData.epochNData.at(5))/(1 - epochBrdData.epochNData.at(5)*qCos(Ek));
		double sinf = (qSin(Ek)*qSqrt(1-epochBrdData.epochNData.at(5)*epochBrdData.epochNData.at(5)))/(1 - epochBrdData.epochNData.at(5)*qCos(Ek));
		double f = qAtan2(qSin(Ek)*qSqrt(1-epochBrdData.epochNData.at(5)*epochBrdData.epochNData.at(5)), qCos(Ek) - epochBrdData.epochNData.at(5));
		//double f = qAtan((qSin(Ek)*qSqrt(1-epochBrdData.epochNData.at(5)*epochBrdData.epochNData.at(5)))/(qCos(Ek) - epochBrdData.epochNData.at(5)));
		//Calculate ud
		double ud = epochBrdData.epochNData.at(14) + f;
		//Calculate perturbation correction
		double Cuc = epochBrdData.epochNData.at(4);
		double Cus = epochBrdData.epochNData.at(6);
		double Crc = epochBrdData.epochNData.at(13);
		double Crs = epochBrdData.epochNData.at(1);
		double Cic = epochBrdData.epochNData.at(9);
		double Cis = epochBrdData.epochNData.at(11);
		double dltaU = Cuc*qCos(2*ud) + Cus*qSin(2*ud);
		double dltaR = Crc*qCos(2*ud) + Crs*qSin(2*ud);
		double dltaI = Cic*qCos(2*ud) + Cis*qSin(2*ud);
		double U = ud + dltaU;
		double R = epochBrdData.epochNData.at(7)*epochBrdData.epochNData.at(7)*(1 - epochBrdData.epochNData.at(5)*qCos(Ek)) + dltaR;
		double I = epochBrdData.epochNData.at(12) + dltaI + epochBrdData.epochNData.at(16)*(t - epochBrdData.epochNData.at(8));
		//Calculate the coordinates of the satellite orbital plane
		//double  dU = qCos(U);
		double x = R*qCos(U);
		double y = R*qSin(U);
		//Calculate the accuracy of the instantaneous ascending point L
		//double L = epochNData[10] + (epochNData[15] - M_We)*t - epochNData[15]*epochNData[8];
		double L = epochBrdData.epochNData.at(10) + epochBrdData.epochNData.at(15)*(dltt_toe) - M_We*t;
		if (SatType == 'C' && PRN < 6)
		{//Convert to L below the inertial coordinate system
			L = epochBrdData.epochNData.at(10) + epochBrdData.epochNData.at(15)*(dltt_toe) - M_We*epochBrdData.epochNData.at(8);
		}
		if (L<0)
			L = L + 2*MM_PI;
		//Calculate satellite instantaneous coordinates
		X = x*qCos(L) - y*qCos(I)*qSin(L);
		Y = x*qSin(L) + y*qCos(I)*qCos(L);
		Z = y*qSin(I);
		//Calculating satellite speed
		double dM = n;
		double dE = dM/(1-epochBrdData.epochNData.at(5)*qCos(Ek));
		double df = qSqrt(1-epochBrdData.epochNData.at(5)*epochBrdData.epochNData.at(5))*dE/(1-epochBrdData.epochNData.at(5)*qCos(Ek));
		double du = df;
		double DdltaU = 2*du*(Cus*qCos(2*ud) - Cuc*qSin(2*ud));
		double DdltaR = 2*du*(Crs*qCos(2*ud) - Crc*qSin(2*ud));
		double DdltaI = 2*du*(Cis*qCos(2*ud) - Cic*qSin(2*ud));
		double dU = du + DdltaU;
		double dR = epochBrdData.epochNData.at(7)*epochBrdData.epochNData.at(7)*epochBrdData.epochNData.at(5)*dE*qSin(Ek)+DdltaR;
		double dI = epochBrdData.epochNData.at(16) + DdltaI;
		double dL = epochBrdData.epochNData.at(15) - M_We;
		if (SatType == 'C' && PRN < 6)
		{//Convert to L below the inertial coordinate system
			dL = epochBrdData.epochNData.at(15);
		}
		double dx = dR*qCos(U) - R*dU*qSin(U);
		double dy = dR*qSin(U) + R*dU*qCos(U);
		//Calculation speed
		dX = -Y*dL - (dy*qCos(I) - Z*dI)*qSin(L) + dx*qCos(L);
		dY = X*dL + (dy*qCos(I) - Z*dI)*qCos(L) + dx*qSin(L);
		dZ = dy*qSin(I) + dy*dI*qCos(I);
		//Determine if it is BDS 1-5 satellite
		if (SatType == 'C' && PRN < 6)
		{
			//L===============
			double Wet_toe = M_We*(dltt_toe);
			//Convert coordinates via Rz(Wet_toe) Rx(-5)
			Matrix3d Rz,Rx,dRz;
			Vector3d Vx3,dVx3;//Position and speed before coordinate conversion
			Vector3d VX,dVX;//Post-conversion position and speed
			Vx3<<X,Y,Z;
			dVx3<<dX,dY,dZ;
			Rz<<qCos(Wet_toe),qSin(Wet_toe),0,
				-qSin(Wet_toe),qCos(Wet_toe),0,
				0,0,1;
			dRz<<-qSin(Wet_toe),qCos(Wet_toe),0,
				-qCos(Wet_toe),-qSin(Wet_toe),0,
				0,0,0;
			Rx<<1,0,0,
				0,qCos(-5*MM_PI/180),qSin(-5*MM_PI/180),
				0,-qSin(-5*MM_PI/180),qCos(-5*MM_PI/180);
			VX = Rz*Rx*Vx3;
			dVX = Rz*Rx*dVx3 + M_We*dRz*Rx*Vx3;
			X = VX(0);Y = VX(1);Z = VX(2);
			dX = dVX(0);dY = dVX(1);dZ = dVX(2);
		}
	}
	else if (SatType == 'R')
	{
        t = GPSOTime - signal_transmission_time;//The time at which the satellite signals the second of the GPS week
        double t0 = YMD2GPSTime(epochBrdData.UTCTime.Year,epochBrdData.UTCTime.Month,epochBrdData.UTCTime.Day,
			epochBrdData.UTCTime.Hours,epochBrdData.UTCTime.Minutes,epochBrdData.UTCTime.Seconds);
		////Calculate GLONASS satellite coordinates using Runge-Kutta/////
		Vector3d GLOXYZ,GLOdX;
		t = t - m_leapSec;//Convert GPS time to GLONASS time (G file is not GLONASS time, but standard UTC so only one integer hop second)
		GLOXYZ = RungeKuttaforGlonass(epochBrdData,t,t0,GLOdX);//Calculate G file coordinates
		//Convert to WGS84 based on experience seven parameters
		GLOXYZ = GLOXYZ*1000;
		GLOdX = GLOdX*1000;
		Matrix3d CM;
		Vector3d dltaX;
		dltaX<<-0.47,-0.51,-1.56;
		CM<<1,1.728e-6,-0.0178e-6,
			1.728e-6,1,0.076e-6,
			0.0178e-6,-0.076e-6,1;
//        GLOXYZ = dltaX + (1+22e-9)*CM*GLOXYZ;//PZ-90 is converted to WGS84 coordinates (can be ignored)
		X = GLOXYZ(0);Y = GLOXYZ(1);Z = GLOXYZ(2); 
		dX = GLOdX(0);dY = GLOdX(1);dZ = GLOdX(2);
		//Calculating the GLONASS relativistic effect
		//GlonassRel = -2*(GLOXYZ(0)*GLOdX(0)+GLOXYZ(1)*GLOdX(1)+GLOXYZ(2)*GLOdX(2))/(M_C);
	}

    //Calculate satellite clock error
    double A[3] ={epochBrdData.TimeDiv,epochBrdData.TimeMove,epochBrdData.TimeMoveSpeed};
    double t0 = YMD2GPSTime(epochBrdData.UTCTime.Year, epochBrdData.UTCTime.Month, epochBrdData.UTCTime.Day,
                                   epochBrdData.UTCTime.Hours, epochBrdData.UTCTime.Minutes,epochBrdData.UTCTime.Seconds);
    // is jump week
    double dltt_t = t - t0, tk = t;
    if (dltt_t > 302400)
        tk-=604800;
    else if (dltt_t < -302400)
        tk+=604800;
    if(StaClock) *StaClock = computeSatClock(A,tk,t0);//Multiply by the speed of light to become the distance m
	pXYZ[0] = X;pXYZ[1] = Y;pXYZ[2] = Z;
	pdXYZ[0] = dX;pdXYZ[1] = dY;pdXYZ[2] = dZ;
}

double QReadGPSN::computeSatClock(double *A,double t,double t0)
{//Calculation clock error
    double dltaT = 0;
    double c = 299792458.0;
    dltaT = A[0] + A[1]*(t - t0) + A[2]*(t-t0)*(t-t0);// - (2*qSqrt(a*u)/(c*c))*(e*qSin(E));//The second half considers that the satellite orbit is not circular
    return dltaT*c;
}

//Calculate GPS time
double QReadGPSN::YMD2GPSTime(int Year, int Month, int Day, int HoursInt, int Minutes, double Seconds, int *WeekN)//,int *GPSTimeArray
{
	double Hours = HoursInt + ((Minutes * 60) + Seconds)/3600.0;
	//Get JD
	double JD = 0.0;
	if(Month<=2)
		JD = (int)(365.25*(Year-1)) + (int)(30.6001*(Month+12+1)) + Day + Hours/24.0 + 1720981.5;
	else
		JD = (int)(365.25*(Year)) + (int)(30.6001*(Month+1)) + Day + Hours/24.0 + 1720981.5;
	//Get GPS Week and Days
	int Week = (int)((JD - 2444244.5) / 7);
	int N =(int)(JD + 1.5)%7;
	if (WeekN) *WeekN = Week;
	return (N*24*3600 + HoursInt*3600 + Minutes*60 + Seconds);
}

//Calculate Julian Day
double QReadGPSN::computeJD(int Year,int Month,int Day,int HoursInt,int Minutes,double Seconds)
{
	double Hours = HoursInt + ((Minutes * 60) + Seconds)/3600.0;
	//Get JD
	double JD = 0.0;
	if(Month<=2)
		JD = (int)(365.25*(Year-1)) + (int)(30.6001*(Month+12+1)) + Day + Hours/24.0 + 1720981.5;
	else
		JD = (int)(365.25*(Year)) + (int)(30.6001*(Month+1)) + Day + Hours/24.0 + 1720981.5;
	return JD;
}

//Calculating leap seconds
// Leap file: ftp://hpiers.obspm.fr/iers/bul/bulc/Leap_Second.dat
double QReadGPSN::getLeapSecond(int Year, int Month, int Day, int Hours/* =0 */, int Minutes/* =0 */, double Seconds/* =0 */)
{// Debug by xiaogongwei 2019.04.03
    double jd = computeJD(Year,Month,Day,Hours, Minutes, Seconds);
	double leapseconds=0;
	double Leap_seconds[50]={0};
	double TAImUTCData[50]={0};
	Leap_seconds[0]=10;
	TAImUTCData[0]=computeJD(1972,1,1,0);
	Leap_seconds[1]=11;
	TAImUTCData[1]=computeJD(1972,7,1,0);
	Leap_seconds[2]=12;
	TAImUTCData[2]=computeJD(1973,1,1,0);
	Leap_seconds[3]=13;
	TAImUTCData[3]=computeJD(1974,1,1,0);
	Leap_seconds[4]=14;
	TAImUTCData[4]=computeJD(1975,1,1,0);
	Leap_seconds[5]=15;
	TAImUTCData[5]=computeJD(1976,1,1,0);
	Leap_seconds[6]=16;
	TAImUTCData[6]=computeJD(1977,1,1,0);
	Leap_seconds[7]=17;
	TAImUTCData[7]=computeJD(1978,1,1,0);
	Leap_seconds[8]=18;
	TAImUTCData[8]=computeJD(1979,1,1,0);
	Leap_seconds[9]=19;
	TAImUTCData[9]=computeJD(1980,1,1,0);
	Leap_seconds[10]=20;
	TAImUTCData[10]=computeJD(1981,7,1,0);
	Leap_seconds[11]=21;
	TAImUTCData[11]=computeJD(1982,7,1,0);
	Leap_seconds[12]=22;
	TAImUTCData[12]=computeJD(1983,7,1,0);
	Leap_seconds[13]=23;
	TAImUTCData[13]=computeJD(1985,7,1,0);
	Leap_seconds[14]=24;
	TAImUTCData[14]=computeJD(1988,1,1,0);
	Leap_seconds[15]=25;
	TAImUTCData[15]=computeJD(1990,1,1,0);
	Leap_seconds[16]=26;
	TAImUTCData[16]=computeJD(1991,1,1,0);
	Leap_seconds[17]=27;
	TAImUTCData[17]=computeJD(1992,7,1,0);
	Leap_seconds[18]=28;
	TAImUTCData[18]=computeJD(1993,7,1,0);
	Leap_seconds[19]=29;
	TAImUTCData[19]=computeJD(1994,7,1,0);
	Leap_seconds[20]=30;
	TAImUTCData[20]=computeJD(1996,1,1,0);
	Leap_seconds[21]=31;
	TAImUTCData[21]=computeJD(1997,7,1,0);
	Leap_seconds[22]=32;
	TAImUTCData[22]=computeJD(1999,1,1,0);
	Leap_seconds[23]=33;
	TAImUTCData[23]=computeJD(2006,1,1,0);
	Leap_seconds[24]=34;
	TAImUTCData[24]=computeJD(2009,1,1,0);
	Leap_seconds[25]=35;
	TAImUTCData[25]=computeJD(2012,7,1,0);
	Leap_seconds[26]=36;
	TAImUTCData[26]=computeJD(2015,7,1,0);
	Leap_seconds[27]=37;
    TAImUTCData[27]=computeJD(2017,1,1,0);
	if (jd<TAImUTCData[0])
	{
		leapseconds=0;
	}
    else if (jd>TAImUTCData[27])
	{
        leapseconds=Leap_seconds[27];
	}
	else
	{
		int iter=0;
		for (int i=1;i<28;i++)
		{
			if (jd<=TAImUTCData[i] && jd>TAImUTCData[i-1])
			{
				iter=i;
				break;
			}
		}
		leapseconds=Leap_seconds[iter-1];
	}
	return (leapseconds-19);
}

//Fourth-order Runge-Kutta method
Vector3d QReadGPSN::RungeKuttaforGlonass(const BrdData &epochBrdData,double tk,double t0,Vector3d &dX)
{
	Vector3d X0,dX0,ddX0;
	X0<<epochBrdData.epochNData.at(0),epochBrdData.epochNData.at(4),epochBrdData.epochNData.at(8);
	dX0<<epochBrdData.epochNData.at(1),epochBrdData.epochNData.at(5),epochBrdData.epochNData.at(9);
	ddX0<<epochBrdData.epochNData.at(2),epochBrdData.epochNData.at(6),epochBrdData.epochNData.at(10);

    // is jump week
    double dltt_t = tk - t0;
    if (dltt_t > 302400)
        tk-=604800;
    else if (dltt_t < -302400)
        tk+=604800;
	double dh = 0;//Defining step size
	if (tk > t0)
		dh = 30;//(秒s)
	else
		dh = -30;
	if (qAbs(tk - t0) < 30)
		dh = tk - t0;
	//Calculated using RungeKutta
	int n = (int)((tk - t0)/dh);
	Vector3d Xtn,Ztn,Xtn1,Ztn1;
	Xtn = X0;
	Ztn = dX0;
	Xtn1.setZero();
	Ztn1.setZero();
	if (qAbs(tk - t0) < 30)
		n = 0;
	for (int i = 0;i < n + 1;i++)
	{
		double h1 = tk - t0 - i*dh;//Judging the remaining step size???????? positive and negative relationship
		if (qAbs(h1) < qAbs(dh))
			dh = h1;
		//Calculate the Runge-Kutta coefficient (refer to the fourth edition of Numerical Analysis, P133, the second order is solved by the equations)
		Vector3d L1,L2,L3,L4;
		L1 = GlonassFun(Xtn,Ztn,ddX0);
		L2 = GlonassFun(Xtn+(dh/2)*Ztn,Ztn+(dh/2)*L1,ddX0);
		L3 = GlonassFun(Xtn+(dh/2)*Ztn+(dh*dh/4)*L1,Ztn+(dh/2)*L2,ddX0);
		L4 = GlonassFun(Xtn+dh*Ztn+(dh*dh/2)*L2,Ztn+dh*L3,ddX0);
		Xtn1 = Xtn + dh*Ztn + (dh*dh/6)*(L1 + L2 + L3);
		Ztn1 = Ztn + (dh/6)*(L1 + 2*L2 + 2*L3 + L4);
		Xtn = Xtn1;
		Ztn = Ztn1;
	}
	dX = Ztn;
	return Xtn;
}

//GLONASS equation of motion
Vector3d QReadGPSN::GlonassFun(Vector3d Xt,Vector3d dXt,Vector3d ddX0)
{
	Vector3d ddXt;
	Vector3d Xrz;
	Vector3d Xwt;
	double r = qSqrt(Xt(0)*Xt(0)+Xt(1)*Xt(1)+Xt(2)*Xt(2));
	Xrz<<Xt(0)*(1 - 5*Xt(2)*Xt(2)/(r*r)),Xt(1)*(1 - 5*Xt(2)*Xt(2)/(r*r)),Xt(2)*(3 - 5*Xt(2)*Xt(2)/(r*r));
	Xwt<<M_We*M_We*Xt(0) + 2*M_We*dXt(1),M_We*M_We*Xt(1) - 2*M_We*dXt(0),0;
	ddXt = (-M_GMK/(r*r*r))*Xt + ((1.5*M_GMK*M_ReK*M_ReK*M_C20)*Xrz/(r*r*r*r*r))+ ddX0 + Xwt;
	return ddXt;
}
