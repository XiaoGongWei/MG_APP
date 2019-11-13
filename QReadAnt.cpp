#include "QReadAnt.h"


QReadAnt::QReadAnt(QString AntFileName,QString AnTypeName,double ObvJD)
{
	initVar();
    if (!AnTypeName.isEmpty())
        m_AntFileName = AntFileName;
	m_AntType = AnTypeName;
	m_ObeservTime = ObvJD;
}

void QReadAnt::setAntFileName(QString AntFileName,QString AnTypeName,double ObvJD)
{
    initVar();
    if (!AntFileName.isEmpty())
        m_AntFileName = AntFileName;
    m_AntType = AnTypeName;
    m_ObeservTime = ObvJD;
}

QReadAnt::~QReadAnt(void)
{
}

//Initialize variables
void QReadAnt::initVar()
{
	m_AntFileName = "antmod.atx";
	isReadAllData = false;
	isReadHead = false;
	isReadSatData = false;
	isReadRecvData = false;
	m_ObeservTime = 0;
	m_RecvData.IsSat = false;
    m_RecvData.isFrqData = false;//Initialization hypothesis can find the antenna
	m_RecvData.ValidJD = 0;
	m_RecvData.EndJD = 999999999;
	m_pi = 3.1415926535897932385;
	for (int i = 0;i < 3;i++)
	{
		m_sunpos[i] = 0;
		m_moonpos[i] = 0;
	}
	m_gmst = 0;
	m_sunSecFlag = -1;
}

//Set Julian time
void QReadAnt::setObsJD(QString AnTypeName,double ObsJD)
{
	m_AntType = AnTypeName;
	m_ObeservTime = ObsJD;
}

//Calculate Julian Day
double QReadAnt::computeJD(int Year,int Month,int Day,int HoursInt,int Minutes,int Seconds)
{//Calculate Julian Day
	double Hours = HoursInt + ((Minutes * 60) + Seconds)/3600.0;
	//Get JD
	double JD = 0.0;
	if(Month<=2)
		JD = (int)(365.25*(Year-1)) + (int)(30.6001*(Month+12+1)) + Day + Hours/24.0 + 1720981.5;
	else
		JD = (int)(365.25*(Year)) + (int)(30.6001*(Month+1)) + Day + Hours/24.0 + 1720981.5;
	return JD;
}

//open a file
bool QReadAnt::openFiles(QString AntFileName)
{
    if(!AntFileName.isEmpty())
    {
        m_ReadFileClass.setFileName(AntFileName);
        if (!m_ReadFileClass.open(QFile::ReadOnly))//If the file fails to open...
        {
            m_ReadFileClass.setFileName("antmod.atx");//Open to current directory
            if (!m_ReadFileClass.open(QFile::ReadOnly))
            {
                isReadHead = true;
                isReadRecvData = true;
                isReadSatData = true;
                isReadAllData = true;
                ErroTrace("Can not find antmod.atx.");
                return false;
            }
            else
            {
                return true;
            }
        }
        else
            return true;
    }
    else
    {
        isReadHead = true;
        isReadRecvData = true;
        isReadSatData = true;
        isReadAllData = true;
        ErroTrace("AntFileName is None");
        return false;
    }

	return false;
}

//Read header file
bool QReadAnt::readFileAntHead()
{
	if (isReadHead)
		return true;
	m_tempLine = m_ReadFileClass.readLine();//Read the first line
	QString flagHeadEnd = "HEADER";
	QString endHeadStr = m_tempLine.mid(60,20).trimmed();
	while (!endHeadStr.contains(flagHeadEnd,Qt::CaseInsensitive))
	{
		//Read header file data......
		//Skip header file here

		//Read the next line
		m_tempLine = m_ReadFileClass.readLine();//Read the next line
		endHeadStr = m_tempLine.mid(60,20).trimmed();
	}
	m_tempLine = m_ReadFileClass.readLine();//Read the next line into the data area
	isReadHead = true;
	return true;
}

//Read satellite + receiver antenna data
bool QReadAnt::getAllData()
{
	if (m_AntFileName.isEmpty())
		isReadAllData = true;
	if (isReadAllData) return true;
	openFiles(m_AntFileName);
	readSatliData();
	readRecvData();
	isReadAllData = true;
    m_ReadFileClass.close();
	return true;
}

//Read receiver antenna data
bool QReadAnt::readRecvData()
{
	if (!isReadHead) readFileAntHead();
	if (isReadRecvData) return true;
	isReadRecvData = true;//Read receiver antenna data only once (no more search than search)
	long int currPos = m_ReadFileClass.pos();//Save current pointer position
	//Search antenna type
	while(!m_tempLine.mid(0,20).contains(m_AntType))
	{
		m_tempLine = m_ReadFileClass.readLine();
		if (m_ReadFileClass.atEnd())
		{//Prevents inadvertent loops at the end of the read file, and flags false to indicate that the antenna was not found
            m_RecvData.isFrqData = false;
			return false;
		}
	}
	//Read receiver antenna data
	while (!m_tempLine.mid(60).contains("END OF ANTENNA"))
	{
		if (m_tempLine.mid(60).contains("TYPE / SERIAL NO"))//TYPE / SERIAL NO
		{
			m_RecvData.StrAntType = m_tempLine.mid(0,20);
			m_RecvData.SatliCNN = m_tempLine.mid(20,20);
			m_tempLine = m_ReadFileClass.readLine();//Read the next row of data

		}
		else if (m_tempLine.mid(60).contains("DAZI"))//DAZI
		{
			m_RecvData.DAZI = m_tempLine.mid(0,8).toDouble();
			m_tempLine = m_ReadFileClass.readLine();//Read the next row of data
		}
		else if (m_tempLine.mid(60).contains("ZEN1 / ZEN2 / DZEN"))//ZEN1 / ZEN2 / DZEN
		{
			m_RecvData.ZEN_12N[0] = m_tempLine.mid(2,6).toDouble();
			m_RecvData.ZEN_12N[1] = m_tempLine.mid(8,6).toDouble();
			m_RecvData.ZEN_12N[2] = m_tempLine.mid(14,6).toDouble();
			m_tempLine = m_ReadFileClass.readLine();//Read the next row of data
		}
		else if (m_tempLine.mid(60).contains("FREQUENCIES"))//# OF FREQUENCIES    
		{
			m_RecvData.NumFrqu = m_tempLine.mid(0,6).toDouble();
			m_tempLine = m_ReadFileClass.readLine();//Read the next row of data
		}
		else if (m_tempLine.mid(60).contains("VALID FROM"))//VALID FROM
		{
			int Year = m_tempLine.mid(0,6).toInt();
			int Month = m_tempLine.mid(6,6).toInt();
			int Day = m_tempLine.mid(12,6).toInt();
			int Hour = m_tempLine.mid(18,6).toInt();
			int Minuts = m_tempLine.mid(24,6).toInt();
			int Seconds = m_tempLine.mid(30,13).toDouble();
			m_RecvData.ValidJD = computeJD(Year,Month,Day,Hour,Minuts,Seconds);
			m_tempLine = m_ReadFileClass.readLine();//Read the next row of data
		}
		else if (m_tempLine.mid(60).contains("VALID UNTIL"))//VALID UNTIL
		{
			int Year = m_tempLine.mid(0,6).toInt();
			int Month = m_tempLine.mid(6,6).toInt();
			int Day = m_tempLine.mid(12,6).toInt();
			int Hour = m_tempLine.mid(18,6).toInt();
			int Minuts = m_tempLine.mid(24,6).toInt();
			int Seconds = m_tempLine.mid(30,13).toDouble();
			m_RecvData.EndJD = computeJD(Year,Month,Day,Hour,Minuts,Seconds);
			m_tempLine = m_ReadFileClass.readLine();//Read the next row of data
		}
		else if (m_tempLine.mid(60).contains("START OF FREQUENCY"))//START OF FREQUENCY
		{//Read receiver PCO and PCV
			FrqData tempFrqData;
			tempFrqData.PCO[0] = 0;tempFrqData.PCO[1] = 0;tempFrqData.PCO[2] = 0;
			tempFrqData.FrqFlag = m_tempLine.mid(3,3);
			m_tempLine = m_ReadFileClass.readLine();
			if (m_tempLine.mid(60).contains("NORTH / EAST / UP"))//NORTH / EAST / UP
			{
				tempFrqData.PCO[0] = m_tempLine.mid(0,10).toDouble();
				tempFrqData.PCO[1] = m_tempLine.mid(10,10).toDouble();
				tempFrqData.PCO[2] = m_tempLine.mid(20,10).toDouble();
			}
			//Read NOZAI data
			m_tempLine = m_ReadFileClass.readLine();
			int Hang = 0,lie = 0;
			if (m_RecvData.DAZI !=  0)
				Hang = (int)(360/m_RecvData.DAZI) + 1;
			lie = (int)((m_RecvData.ZEN_12N[1] - m_RecvData.ZEN_12N[0])/m_RecvData.ZEN_12N[2]) + 1;
			tempFrqData.Hang = Hang;
			tempFrqData.Lie = lie;
            if (m_tempLine.mid(0,8).contains("NOAZI"))
                for (int i = 0;i < lie;i++)
                    tempFrqData.PCVNoAZI.append(m_tempLine.mid(8+i*8,8).toDouble());
            //Hang does not equal 0 to read ZAI data
            if (tempFrqData.Hang != 0)
            {
                for (int i = 0;i < tempFrqData.Hang;i++)
                {
                    m_tempLine = m_ReadFileClass.readLine();//Read a row of data
                    for (int j = 0;j < tempFrqData.Lie;j++)
                    {
                        tempFrqData.PCVAZI.append(m_tempLine.mid(8+j*8,8).toDouble());
                    }
                }
            }
			m_RecvData.PCOPCV.append(tempFrqData);
            m_RecvData.isFrqData = true; // debug by xiaogongwei 2019.04.12
            m_RecvData.IsSat = false; // debug by xiaogongwei 2019.04.12
            m_tempLine = m_ReadFileClass.readLine();//Read the next row of data
		}
		else if (m_tempLine.mid(60).contains("END OF FREQUENCY"))//END OF FREQUENCY
		{
			m_tempLine = m_ReadFileClass.readLine();//Read the next row of data
		}
		else
		{
			m_tempLine = m_ReadFileClass.readLine();//Read the next row of data
		}
	}

	m_ReadFileClass.seek(currPos);//Set as the starting search coordinate
	return true;
}

//Read satellite system antenna data
bool QReadAnt::readSatliData()
{
	if (!isReadHead) readFileAntHead();
    if(isReadSatData) return false;
	long int currPos = m_ReadFileClass.pos();//Save current pointer position
	//Find to start reading data
	while(!m_tempLine.mid(60).contains("START OF ANTENNA"))
		m_tempLine = m_ReadFileClass.readLine();
	//Read satellite antenna data
    //debug: commit next 2 line (2018.07.13)
//	int SatNum = 0;//Record the number of valid satellites, which must be less than MaxSatlitNum
//	bool IsGPS = false;//判Whether the broken PRN enters the GPS area and is interrupted when it leaves again, preventing the infinite loop.
	char tempSatType = '0';
	while (!m_ReadFileClass.atEnd())
	{
		AntDataType tempSatData;
		tempSatData.IsSat = true;
		tempSatData.ValidJD = 0;
		tempSatData.EndJD = m_ObeservTime+1;
		//Read a satellite antenna data
		while (!m_tempLine.mid(60).contains("END OF ANTENNA"))
		{
			if (m_tempLine.mid(60).contains("START OF ANTENNA"))//START OF ANTENNA
			{
				m_tempLine = m_ReadFileClass.readLine();
			} 
			else if (m_tempLine.mid(60).contains("TYPE / SERIAL NO"))//TYPE / SERIAL NO
			{
				tempSatData.StrAntType = m_tempLine.mid(0,20);
				tempSatData.SatliCNN = m_tempLine.mid(20,20).trimmed();//Satellite system PRN
				m_tempLine = m_ReadFileClass.readLine();//Read the next row of data

			}
			else if (m_tempLine.mid(60).contains("DAZI"))//DAZI
			{
				tempSatData.DAZI = m_tempLine.mid(0,8).toDouble();
				m_tempLine = m_ReadFileClass.readLine();//Read the next row of data
			}
			else if (m_tempLine.mid(60).contains("ZEN1 / ZEN2 / DZEN"))//ZEN1 / ZEN2 / DZEN
			{
				tempSatData.ZEN_12N[0] = m_tempLine.mid(2,6).toDouble();
				tempSatData.ZEN_12N[1] = m_tempLine.mid(8,6).toDouble();
				tempSatData.ZEN_12N[2] = m_tempLine.mid(14,6).toDouble();
				m_tempLine = m_ReadFileClass.readLine();//Read the next row of data
			}
			else if (m_tempLine.mid(60).contains("FREQUENCIES"))//# OF FREQUENCIES    
			{
				tempSatData.NumFrqu = m_tempLine.mid(0,6).toInt();
				m_tempLine = m_ReadFileClass.readLine();//Read the next row of data
			}
			else if (m_tempLine.mid(60).contains("VALID FROM"))//VALID FROM
			{
				int Year = m_tempLine.mid(0,6).toInt();
				int Month = m_tempLine.mid(6,6).toInt();
				int Day = m_tempLine.mid(12,6).toInt();
				int Hour = m_tempLine.mid(18,6).toInt();
				int Minuts = m_tempLine.mid(24,6).toInt();
				int Seconds = m_tempLine.mid(30,13).toDouble();
				tempSatData.ValidJD = computeJD(Year,Month,Day,Hour,Minuts,Seconds);
				m_tempLine = m_ReadFileClass.readLine();//Read the next row of data
			}
			else if (m_tempLine.mid(60).contains("VALID UNTIL"))//VALID UNTIL
			{
				int Year = m_tempLine.mid(0,6).toInt();
				int Month = m_tempLine.mid(6,6).toInt();
				int Day = m_tempLine.mid(12,6).toInt();
				int Hour = m_tempLine.mid(18,6).toInt();
				int Minuts = m_tempLine.mid(24,6).toInt();
				int Seconds = m_tempLine.mid(30,13).toDouble();
				tempSatData.EndJD = computeJD(Year,Month,Day,Hour,Minuts,Seconds);
				m_tempLine = m_ReadFileClass.readLine();//Read the next row of data
			}
			else if (m_tempLine.mid(60).contains("START OF FREQUENCY"))//START OF FREQUENCY
			{//Read the satellite's PCO and PCV
				FrqData tempFrqData;
				tempFrqData.PCO[0] = 0;tempFrqData.PCO[1] = 0;tempFrqData.PCO[2] = 0;
				tempFrqData.FrqFlag = m_tempLine.mid(3,3);
				m_tempLine = m_ReadFileClass.readLine();
				if (m_tempLine.mid(60).contains("NORTH / EAST / UP"))//NORTH / EAST / UP
				{
					tempFrqData.PCO[0] = m_tempLine.mid(0,10).toDouble();
					tempFrqData.PCO[1] = m_tempLine.mid(10,10).toDouble();
					tempFrqData.PCO[2] = m_tempLine.mid(20,10).toDouble();
				}
				//Read NOZAI data
				m_tempLine = m_ReadFileClass.readLine();
				int Hang = 0,lie = 0;
				if (tempSatData.DAZI !=  0)
					Hang = (int)(360/tempSatData.DAZI) + 1;
				lie = (int)((tempSatData.ZEN_12N[1] - tempSatData.ZEN_12N[0])/tempSatData.ZEN_12N[2]) + 1;
				tempFrqData.Hang = Hang;
				tempFrqData.Lie = lie;
				if (m_tempLine.mid(0,8).contains("NOAZI"))
					for (int i = 0;i < lie;i++)
						tempFrqData.PCVNoAZI.append(m_tempLine.mid(8+i*8,8).toDouble());
				//Hang does not equal 0 to read ZAI data
				if (tempFrqData.Hang != 0)
				{
					for (int i = 0;i < tempFrqData.Hang;i++)
					{
						m_tempLine = m_ReadFileClass.readLine();//Read a row of data
						for (int j = 0;j < tempFrqData.Lie;j++)
						{
							tempFrqData.PCVAZI.append(m_tempLine.mid(8+j*8,8).toDouble());
						}
					}
				}
				tempSatData.PCOPCV.append(tempFrqData);
                tempSatData.isFrqData = true; // debug by xiaogongwei 2019.04.12
				m_tempLine = m_ReadFileClass.readLine();//Read the next row of data
			}
			else if (m_tempLine.mid(60).contains("END OF FREQUENCY"))//END OF FREQUENCY
			{
				m_tempLine = m_ReadFileClass.readLine();//Read the next row of data
			}
			else
			{
				m_tempLine = m_ReadFileClass.readLine();//Read the next row of data
			}//If elseif structure
			if (m_ReadFileClass.atEnd())	break;
		}//while (!m_tempLine.mid(60).contains("END OF ANTENNA"))

		//Determine if it is valid data
		tempSatType = *(tempSatData.SatliCNN.mid(0,1).toLatin1().data());
		if (tempSatData.ValidJD < m_ObeservTime&&tempSatData.EndJD > m_ObeservTime&&isInSystem(tempSatType))
		{
			m_SatData.append(tempSatData);
		}
		//Not jumping out of the four-system satellite antenna area
		if(tempSatType!='G'&&tempSatType!='R'&&tempSatType!='C'&&tempSatType!='E')
			break;
		m_tempLine = m_ReadFileClass.readLine();//Read the next row of data
	}//while (!m_ReadFileClass.atEnd())
	m_ReadFileClass.seek(currPos);
    isReadSatData = true;//Read satellite antenna only once (no more searches than search)
	return true;
}

//Bilinear interpolation calculation PCV
bool QReadAnt::getPCV(const AntDataType &tempAntData, char SatType, double *PCV12, double Ztop, double AZI, QVector<QString> FrqFlag)
{
    PCV12[0] = 0; PCV12[1] = 0;
	QVector< double > XZen;//Azimuth angle is equivalent to interpolation X coordinate
	QVector< double > XAzi;//The height angle is equivalent to the interpolated X coordinate
	int flagZ = 0,flagA = 0;//Interpolation left (smaller) nearest neighbor
	for (double i = tempAntData.ZEN_12N[0];i <= tempAntData.ZEN_12N[1];i+=tempAntData.ZEN_12N[2])
	{
		XZen.append(i*m_pi/180);
		if (i*m_pi/180 < Ztop)
			flagZ++;
	}
	flagZ--;
    if(flagZ < 0) flagZ = 0;// Azimuth crossing setting 0
	if (tempAntData.DAZI !=0)
	{//PCV value table in the form of a matrix
		for (int i = 0;i <= 360;i+=tempAntData.DAZI)
		{
			XAzi.append(i*m_pi/180);
			if (i*m_pi/180 < AZI)
				flagA++;
		}
		flagA--;
        if(flagA < 0) flagA = 0;// Azimuth crossing setting 0，For Geileo
	}
    //Calculate PCV12 (calculate the receiver antenna correction needs, search for the corresponding frequency according to the satellite frequency, search for the first two corrections
	FrqData PCOPCV1,PCOPCV2;
    QString FrqStr1 = "C1C", FrqStr2 = "C2C";
    if(FrqFlag.length() > 0) FrqStr1 = FrqFlag.at(0);
    if(FrqFlag.length() > 2) FrqStr2 = FrqFlag.at(2);

    QString destFlag1 = QString(SatType) + "0" + FrqStr1.mid(1,1),
            destFlag2 = QString(SatType) + "0" + FrqStr2.mid(1,1);
	bool isFind1 = false,isFind2 = false;
	for (int i = 0;i < tempAntData.PCOPCV.length();i++)
	{//This only applies to GPS L1 and L2, as well as GLONASS G1 and G2
		QString LFlag = tempAntData.PCOPCV.at(i).FrqFlag.trimmed();

		if (LFlag == destFlag1)
		{
			PCOPCV1 = tempAntData.PCOPCV.at(i);
			isFind1 = true;
		}
		else if (LFlag == destFlag2)
		{
			PCOPCV2 = tempAntData.PCOPCV.at(i);
			isFind2 = true;
		}
	}
    //Can't find GPS correction, for example, without Geileo and BDS, use GPS correction
    if(!isFind1 && tempAntData.PCOPCV.length() > 0) PCOPCV1 = tempAntData.PCOPCV.at(0);
    if(!isFind2 && tempAntData.PCOPCV.length() > 1) PCOPCV2 = tempAntData.PCOPCV.at(1);

	//Calculate PCV1
    int PCOPCV1_len = PCOPCV1.PCVAZI.length(),
        PCOPCV2_len = PCOPCV2.PCVAZI.length();
    if(tempAntData.IsSat)
    {//The PCV calculation of the satellite is not currently in accordance with the azimuth. Only the Geileo provided by the ESA unit has an azimuth, which is not necessarily accurate, and the satellite antenna azimuth is difficult to determine.
        PCOPCV1_len = 0;
        PCOPCV2_len = 0;
    }

    if (PCOPCV1_len == 0)
	{//Single linear interpolation
        if(PCOPCV1.PCVNoAZI.length() == 0) return false;

		if (flagZ < XZen.length() - 1)
		{
			double x1 = XZen.at(flagZ),x2 = XZen.at(flagZ+1);
			double y1 = PCOPCV1.PCVNoAZI.at(flagZ),y2 = PCOPCV1.PCVNoAZI.at(flagZ+1);
			PCV12[0] = y1+(Ztop - x1)*(y2-y1)/(x2 -x1);
		}
		else
			PCV12[0] = PCOPCV1.PCVNoAZI.at(flagZ);
	}
	else
	{//Bilinear interpolation
		if (flagZ >= XZen.length() - 1)		flagZ--;//Judging the out of bounds, that is, the height angle is greater than 90 degrees, generally not
		if (flagA >= XAzi.length() - 1)		flagA--;//Judging the out of bounds, that is, the azimuth is greater than 360 degrees, generally not
		double x1 = XZen.at(flagZ),x2 = XZen.at(flagZ+1);
		double y1 = XAzi.at(flagA),y2 = XAzi.at(flagA+1);

		int HangNum = 0,LieNum = 0;//Calculate the number of rows and columns in the matrix
		if (tempAntData.DAZI !=  0)
			HangNum = (int)(360/m_RecvData.DAZI) + 1;
		LieNum = (int)((tempAntData.ZEN_12N[1] - tempAntData.ZEN_12N[0])/tempAntData.ZEN_12N[2]) + 1;
		double z11 = PCOPCV1.PCVAZI.at(LieNum*flagA+flagZ),z12 = PCOPCV1.PCVAZI.at(LieNum*flagA+flagZ+1),
			z21 = PCOPCV1.PCVAZI.at(LieNum*(flagA+1)+flagZ),z22 = PCOPCV1.PCVAZI.at(LieNum*(flagA+1)+flagZ+1);

		double z1_2 = z11 + (Ztop - x1)*(z12 - z11)/(x2 - x1),
			   z2_2 = z21 + (Ztop - x1)*(z22 - z21)/(x2 - x1);

		PCV12[0] = z1_2 + (AZI - y1)*(z2_2 - z1_2)/(y2 - y1);
	}
	//Calculate PCV2
    if (PCOPCV2_len == 0)
	{//Single linear interpolation
		if (flagZ < XZen.length() - 1)
		{
			double x1 = XZen.at(flagZ),x2 = XZen.at(flagZ+1);
			double y1 = PCOPCV2.PCVNoAZI.at(flagZ),y2 = PCOPCV2.PCVNoAZI.at(flagZ+1);
			PCV12[1] = y1+(Ztop - x1)*(y2-y1)/(x2 -x1);
		}
		else
			PCV12[1] = PCOPCV2.PCVNoAZI.at(flagZ);
		
	}
	else
	{//Bilinear interpolation
		if (flagZ >= XZen.length() - 1)		flagZ--;//Judging the out of bounds, that is, the height angle is greater than 90 degrees, generally not
		if (flagA >= XAzi.length() - 1)		flagA--;//Judging the out of bounds, that is, the azimuth is greater than 360 degrees, generally not
		double x1 = XZen.at(flagZ),x2 = XZen.at(flagZ+1);
		double y1 = XAzi.at(flagA),y2 = XAzi.at(flagA+1);
		int HangNum = 0,LieNum = 0;//Calculate the number of rows and columns in the matrix
		if (tempAntData.DAZI !=  0)
			HangNum = (int)(360/m_RecvData.DAZI) + 1;
		LieNum = (int)((tempAntData.ZEN_12N[1] - tempAntData.ZEN_12N[0])/tempAntData.ZEN_12N[2]) + 1;
		double z11 = PCOPCV2.PCVAZI.at(LieNum*flagA+flagZ),z12 = PCOPCV2.PCVAZI.at(LieNum*flagA+flagZ+1),
			z21 = PCOPCV2.PCVAZI.at(LieNum*(flagA+1)+flagZ),z22 = PCOPCV2.PCVAZI.at(LieNum*(flagA+1)+flagZ+1);
		double z1_2 = z11 + (Ztop - x1)*(z12 - z11)/(x2 - x1),
			z2_2 = z21 + (Ztop - x1)*(z22 - z21)/(x2 - x1);
		PCV12[1] = z1_2 + (AZI - y1)*(z2_2 - z1_2)/(y2 - y1);

	}
	return true;
}

//Calculate the correction of the receiver antenna and correct the PCO and PCV to the direction of the satellite signal
bool QReadAnt::getRecvL12(double E, double A, char SatType, double &L1Offset, double &L2Offset, QVector<QString> FrqFlag)
{
	L1Offset = 0;L2Offset = 0;
	if (!isInSystem(SatType))	return false;
    if (!m_RecvData.isFrqData) return false;
	if (!isReadRecvData) readRecvData(); 
	FrqData PCOPCV1,PCOPCV2;
    QString FrqStr1 = "C1C", FrqStr2 = "C2C";
    if(FrqFlag.length() > 0) FrqStr1 = FrqFlag.at(0);
    if(FrqFlag.length() > 2) FrqStr2 = FrqFlag.at(2);

    QString destFlag1 = QString(SatType) + "0" + FrqStr1.mid(1,1),
            destFlag2 = QString(SatType) + "0" + FrqStr2.mid(1,1);
	bool IsFind1 = false,IsFind2 = false;
	for (int i = 0;i < m_RecvData.PCOPCV.length();i++)
	{/*This applies only to L1 and L2 of GPS and G1 and G2 of GLONASS. But calculate the BDS and Galieo satellite antennas!!!!!!!!!!!!!
		There is no 01 and 02, but the satellite's PCO and PCV are the same in any frequency band. It is not perfect.*/
		QString LFlag = m_RecvData.PCOPCV.at(i).FrqFlag.trimmed();
		
		if (LFlag == destFlag1)
		{
			PCOPCV1 = m_RecvData.PCOPCV.at(i);
			IsFind1 = true;
		}
		else if (LFlag == destFlag2)
		{
			PCOPCV2 = m_RecvData.PCOPCV.at(i);
			IsFind2 = true;
		}
	}
    // Can't find GPS correction, for example, without Geileo and BDS, use GPS correction
    if(!IsFind1 && m_RecvData.PCOPCV.length() > 0) PCOPCV1 = m_RecvData.PCOPCV.at(0);
    if(!IsFind2 && m_RecvData.PCOPCV.length() > 1) PCOPCV2 = m_RecvData.PCOPCV.at(1);

	//Get PCO
    double *PCO1 = PCOPCV1.PCO,*PCO2 = PCOPCV2.PCO;
	//Calculate PCV below
	double PCV12[2]={0};
	double Ztop = m_pi/2 - E;//Altitude angle converted to zenith angle
    getPCV(m_RecvData,SatType,PCV12,Ztop,A, FrqFlag);
	//Calculate the offset of L1 L2 in the incident direction of the satellite signal mm
	L1Offset = -PCV12[0] + PCO1[0]*qCos(E)*qCos(A)+PCO1[1]*qCos(E)*qSin(A)+PCO1[2]*qSin(E);
	L2Offset = -PCV12[1] + PCO2[0]*qCos(E)*qCos(A)+PCO2[1]*qCos(E)*qSin(A)+PCO2[2]*qSin(E);
	L1Offset = L1Offset/1000;
	L2Offset = L2Offset/1000;//Return meter
	return true;
}

//Calculate the correction of the receiver antenna, correct the PCO and PCV to the satellite signal direction, and search for the first two frequencies according to QVector<QString> FrqFlag. The L12Offset dimension must be greater than 2.
bool QReadAnt::getSatOffSet(int Year,int Month,int Day,int Hours,int Minuts,double Seconds,int PRN,char SatType,double *StaPos,double *RecPos,
                              double *L12Offset, QVector<QString> FrqFlag)
{
    L12Offset[0] = 0; L12Offset[1] = 0;
    if (!isReadSatData) return false;//Determine if the data has been read
    if (!isInSystem(SatType))	return false;	//Determine if it is within the set system
    if (qAbs(m_sunSecFlag - (Hours*3600+Minuts*60+Seconds)) > 0.3)//Two differences greater than 0.8s are considered to replace the epoch to recalculate the sun coordinates
    {//Calculate the sun coordinates once per epoch, without repeating a lot of calculations
        m_CmpClass.getSunMoonPos(Year,Month,Day,Hours,Minuts,Seconds,m_sunpos,m_moonpos,&m_gmst);//An epoch only needs to be calculated once, no need to calculate multiple times
        m_sunSecFlag = Hours*3600+Minuts*60+Seconds;
    }
	//Calculate the coordinate system of the star-solid system in the inertial system. The unit vector indicates the ex, ey, and ez axis.
	double ex[3]={0},ey[3]={0},ez[3]={0};
	double lenStaPos = qSqrt(m_CmpClass.InnerVector(StaPos,StaPos));//Calculate the length of the satellite vector
	//Computing satellite and solar cross product
	double vectSunSat[3] = {0},lenSunSta = 0.0;
	m_CmpClass.OutVector(StaPos,m_sunpos,vectSunSat);
	lenSunSta = qSqrt(m_CmpClass.InnerVector(vectSunSat,vectSunSat));
	//Calculate the X-axis length vector
	double vectEZ[3] = {0},lenEZ = 0;
	m_CmpClass.OutVector(StaPos,vectSunSat,vectEZ);
	lenEZ = qSqrt(m_CmpClass.InnerVector(vectEZ,vectEZ));
	for (int i = 0;i < 3;i++)
	{
		ez[i] = -StaPos[i]/lenStaPos;
		ey[i] = -vectSunSat[i]/lenSunSta;
		ex[i] = -vectEZ[i]/lenEZ;
	}
	//Obtain the satellite's PCO and PCV (need to determine the type of satellite)
	AntDataType m_SatTemp;
	int tempPRN = 0;
	char tempSatType = '0';
	bool Isfind = false;
	for (int i = 0;i < m_SatData.length();i++)
	{//Traversing the satellite antenna data required for the query
		m_SatTemp = m_SatData.at(i);
		tempPRN = m_SatTemp.SatliCNN.mid(1,2).toInt();
		tempSatType = *(m_SatTemp.SatliCNN.mid(0,1).toLatin1().data());
		if (tempPRN == PRN&&tempSatType == SatType)
		{
			Isfind = true;
			break;
		}
	}
	//The satellite antenna data was not found
    if (!Isfind)	return false;
    // find PCO data add by xiaogongwei 2019.04.12
    double PCOL1ecef[3] = {0},PCOL1[3] = {0},
            PCOL2ecef[3] = {0},PCOL2[3] = {0};
    QString FrqStr1 = "C1C", FrqStr2 = "C2C";
    if(FrqFlag.length() > 0) FrqStr1 = FrqFlag.at(0);
    if(FrqFlag.length() > 2) FrqStr2 = FrqFlag.at(2);
    findPCO(PCOL1, m_SatTemp.PCOPCV,  SatType,  FrqStr1);
    findPCO(PCOL2, m_SatTemp.PCOPCV,  SatType,  FrqStr2);
    //PCO conversion to ground fixed coordinate system ecef
	for (int i = 0;i < 3;i++)
	{
		PCOL1ecef[i] = ex[i]*PCOL1[0] + ey[i]*PCOL1[1] + ez[i]*PCOL1[2];
        PCOL2ecef[i] = ex[i]*PCOL2[0] + ey[i]*PCOL2[1] + ez[i]*PCOL2[2];
	}
	//Calculate the approximate coordinate vector of the satellite and receiver
	double sat2recV[3] = {RecPos[0] - StaPos[0],RecPos[1] - StaPos[1],RecPos[2] - StaPos[2]};
	double lenSat2RecV = qSqrt(m_CmpClass.InnerVector(sat2recV,sat2recV));
	double sat2recVE[3] = {sat2recV[0]/lenSat2RecV,sat2recV[1]/lenSat2RecV,sat2recV[2]/lenSat2RecV};
    double AntHigPCOL1 = m_CmpClass.InnerVector(PCOL1ecef,sat2recVE),
            AntHigPCOL2 = m_CmpClass.InnerVector(PCOL2ecef,sat2recVE);

	//Calculate PCV below
	double PCVAngle = qAcos(m_CmpClass.InnerVector(ez,sat2recVE));
    if (PCVAngle < 0|| PCVAngle > 0.244346095) //0.244346095 is 14 degree
	{
		PCVAngle = m_pi - PCVAngle;
	}
	double PCV12[2]={0}; 
    getPCV(m_SatTemp,SatType,PCV12,PCVAngle,0, FrqFlag);// 0 is degree of satlite Azimuth
    L12Offset[0] = (AntHigPCOL1 - PCV12[0])/1000;//Return unit（m）
    L12Offset[1] = (AntHigPCOL2 - PCV12[1])/1000;//Return unit（m）
    return true;
}
// Pco dimension must be greater than 3
void QReadAnt::findPCO(double *pco, QVector<FrqData> &PCOPCV, char SatType, QString frqStr)
{
    memset(pco, 0, 3*sizeof(double));
    QString frqFlag = QString(SatType) + "0" + frqStr.mid(1,1);
    for(int i = 0; i < PCOPCV.length();i++)
    {
        if(PCOPCV.at(i).FrqFlag == frqFlag)
        {
            pco[0] = PCOPCV.at(i).PCO[0];
            pco[1] = PCOPCV.at(i).PCO[1];
            pco[2] = PCOPCV.at(i).PCO[2];
        }
    }
}

