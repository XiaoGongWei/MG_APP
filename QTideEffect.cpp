#include "QTideEffect.h"

const double  QTideEffect::args[][5]={
	{1.40519E-4, 2.0,-2.0, 0.0, 0.00},  /* M2 */
	{1.45444E-4, 0.0, 0.0, 0.0, 0.00},  /* S2 */
	{1.37880E-4, 2.0,-3.0, 1.0, 0.00},  /* N2 */
	{1.45842E-4, 2.0, 0.0, 0.0, 0.00},  /* K2 */
	{0.72921E-4, 1.0, 0.0, 0.0, 0.25},  /* K1 */ 
	{0.67598E-4, 1.0,-2.0, 0.0,-0.25},  /* O1 */
	{0.72523E-4,-1.0, 0.0, 0.0,-0.25},  /* P1 */
	{0.64959E-4, 1.0,-3.0, 1.0,-0.25},  /* Q1 */
	{0.53234E-5, 0.0, 2.0, 0.0, 0.00},  /* Mf */
	{0.26392E-5, 0.0, 1.0,-1.0, 0.00},  /* Mm */
	{0.03982E-5, 2.0, 0.0, 0.0, 0.00}   /* Ssa */
};

void QTideEffect::initVar()
{
	//Initialize the star GM parameter
	m_GMi[0] = 3.986004415e14;
	m_GMi[1] = 4.9027890e12;
	m_GMi[2] = 1.3271250e20;
	//Love and Shadi parameters
	loveShida2[0] = 0;loveShida2[1] = 0;
	isgetLoveShida2 = false;
	loveShida3[0] = 0.2920;loveShida3[1] = 0.0150;
	m_SationBLH[0] = 0;m_SationBLH[1] = 0;m_SationBLH[2] = 0;
	//Initialize erp_t data
	m_erpData.n = 0;
	m_erpData.nmax = 10;
	m_erpData.data = (erpd_t*)malloc(sizeof(erpd_t)*m_erpData.nmax);
	isPoleEffect = true;
	isSolidTide = true;
	isOCEANTide = true;
	m_SecondFlag = -1;
	m_AllTideENU[0] = 0;m_AllTideENU[1] = 0;m_AllTideENU[2] = 0;
	for (int i = 0;i < 3;i++)
	{
		m_pSolidENU[i] = 0;m_pPoleENU[i] = 0;m_pOCEANENU[i] = 0;
		m_sunpos[i] = 0;m_moonpos[i] = 0;
	}
	m_gmst = 0;isGetPos = false;
	for (int i = 0;i < 5;i++)
		m_erpV[i] = 0;
	isReadErp = false;
	isReadOCEAN = false;
	LeapSeconds = 0;
    m_OCEANFileName = "OCEAN-GOT48.blq";
    m_erpFileName = "";
	m_OCEANData.isRead = false;//unavailable
	m_StationName = "";
}

//Set sun, moon, gmst data
void QTideEffect::setSunMoonPos(double *psun,double *pmoon,double gmst/* =0 */)
{
	for (int i = 0;i < 3;i++)
	{
		m_sunpos[i] = psun[i];
		m_moonpos[i] = pmoon[i];
	}
	m_gmst = gmst;
	isGetPos = true;
}

//Set station name for extreme tide search data
void QTideEffect::setStationName(QString StationName /* = "" */)
{
    if(!StationName.isEmpty()) m_StationName = StationName.mid(0,4).trimmed();// Debug by xiaogongwei 2019.03.13
}

//Incoming ocean data, erp file path, otherwise search from the current directory, otherwise it will not apply to ocean tides and extreme tide correction
QTideEffect::QTideEffect(QString OCEANFileName,QString erpFileName)
{
    initVar();
    if(!erpFileName.isEmpty()) m_erpFileName = erpFileName.trimmed();
    if(!OCEANFileName.isEmpty()) m_OCEANFileName = OCEANFileName.trimmed();
	
}

//Incoming ocean data, erp file path, otherwise search from the current directory, otherwise it will not apply to ocean tides and extreme tide correction
void QTideEffect::setTideFileName(QString OCEANFileName,QString erpFileName)
{
    initVar();
    if(!erpFileName.isEmpty()) m_erpFileName = erpFileName.trimmed();
    if(!OCEANFileName.isEmpty()) m_OCEANFileName = OCEANFileName.trimmed();
}

QTideEffect::~QTideEffect(void)
{
    if(m_erpData.data != NULL) free(m_erpData.data);
}

bool QTideEffect::readRepFile()
{
	if (isReadErp)	return true;

	if (!m_erpFileName.isEmpty())
	{//First determine if the user is passing the erp path

        isReadErp = m_cmpClass.readerp(m_erpFileName.toLatin1().data(),&m_erpData);
        if (isReadErp)
            isPoleEffect = true;
        else
            isPoleEffect = false;
	}
    else
    {
        isReadErp = false;
        isPoleEffect = false;
	}

    return isReadErp;
}

//double *erpv       O   erp values {xp,yp,ut1_utc,lod} (rad,rad,s,s/d)
bool  QTideEffect::getErpV(gtime_t obsGPST,double *erpV)
{
	if(!m_cmpClass.geterp(&m_erpData,obsGPST,erpV)) return false;
	return true;
}

void QTideEffect::tide_pole(const double *pos, const double *erpv, double *denu)
{
	double xp,yp,cosl,sinl;

	//trace(3,"tide_pole: pos=%.3f %.3f\n",pos[0]*R2D,pos[1]*R2D);
	xp=erpv[0]/AS2R; /* rad -> arcsec */
	yp=erpv[1]/AS2R;
	cosl=cos(pos[1]); sinl=sin(pos[1]);
	denu[0]=  9E-3*sin(pos[0])    *(xp*sinl+yp*cosl);
	denu[1]= -9E-3*cos(2.0*pos[0])*(xp*cosl-yp*sinl);
	denu[2]=-32E-3*sin(2.0*pos[0])*(xp*cosl-yp*sinl);
}

//Provides a very humid interface, the day will not change, as long as the BLH is accurate, pay attention to the code to reduce the amount of calculation
void QTideEffect::getPoleTide(int Year,int Month,int Day,int Hours,int Minuts,double Seconds,double *pBLH,double *pTideENU)
{
	pTideENU[0] = 0;pTideENU[1] = 0;pTideENU[2] = 0;
	if (!isPoleEffect) return ;
	double tSeconds = 0;
	int tWeek;
	gtime_t obsGPST;
	pTideENU[0] = 0;pTideENU[1] = 0;pTideENU[2] = 0;
	tSeconds = m_cmpClass.YMD2GPSTime(Year,Month,Day,Hours,Minuts,Seconds,&tWeek);
	obsGPST = m_cmpClass.gpst2time(tWeek,Seconds);
	//Get erp parameter
	//if (!isgetErpV)
	if(!getErpV(obsGPST,m_erpV))
		for (int i = 0;i < 5;i++)
			m_erpV[i] = 0;
	//Calculate the extreme tide
	tide_pole(pBLH,m_erpV,pTideENU);
}

void QTideEffect::subSolidTide(double *sunmoonPos,double *pXYZ,double *pTideXYZ,int flag)
{//Seeking a second tide
	//Seeking a second tide
	double esunmoon[3]={0},erec[3]={0};//Sun or moon, unit vector of station
	double lensunmoon = 0,lenrec = 0;
	//Find the length of the vector
	lensunmoon = m_cmpClass.norm(sunmoonPos,3);
	lenrec = m_cmpClass.norm(pXYZ,3);
	//Unitization
	for (int i = 0; i < 3;i++)
	{
		esunmoon[i] = sunmoonPos[i]/lensunmoon;
		erec[i] = pXYZ[i]/lenrec;
	}
	//Secondary solid tide effect
	if (flag<0||flag>1 ) return;
	double GMe = m_GMi[0],GMj = m_GMi[flag+1];//Get celestial gravity parameters
	double K1 = 0,K2 = 0,K3 = 0,dotsunRec = 0;
	dotsunRec = m_cmpClass.dot(esunmoon,erec,3);
    double R_earth = 6378136;// r of earth unit m
    K1 = GMj*R_earth*R_earth*R_earth*R_earth/(GMe*lensunmoon*lensunmoon*lensunmoon);
    K1 = GMj*lenrec*lenrec*lenrec*lenrec/(GMe*lensunmoon*lensunmoon*lensunmoon);

	K2 = 3*loveShida2[1]*dotsunRec;
	K3 = 3*(loveShida2[0]/2 - loveShida2[1])*(dotsunRec*dotsunRec) - loveShida2[0]/2;

	pTideXYZ[0] = K1*(K2*esunmoon[0] + K3*erec[0]);
	pTideXYZ[1] = K1*(K2*esunmoon[1] + K3*erec[1]);
	pTideXYZ[2] = K1*(K2*esunmoon[2] + K3*erec[2]);
}

void QTideEffect::getSoildTide(int Year,int Month,int Day,int Hours,int Minuts,double Seconds,double *pXYZ,double *pTideENU,bool isElimate/* = false*/)
{
	pTideENU[0] = 0;pTideENU[1] = 0;pTideENU[2] = 0;
	if (!isSolidTide)	return ; 
	double sunpos[3]={0},moonpos[3] = {0},gmst = 0;
	double pBLH[3] = {0},pmoon2XYZ[3] = {0},psun2XYZ[3]={0},pmoon3XYZ[3] = {0},sinb2 = 0;
	//As the station calculation accuracy increases, BLH also needs to be updated in real time (increased calculation amount)
    m_cmpClass.XYZ2BLH(pXYZ,pBLH);
    m_SationBLH[0] = pBLH[0];m_SationBLH[1] = pBLH[1];m_SationBLH[2] = pBLH[2];
    sinb2 = ( 3*qSin(pBLH[0])*qSin(pBLH[0]) - 1 )/2;
    loveShida2[0] = 0.6078 - 0.0006*sinb2;
    loveShida2[1] = 0.0847 + 0.0002*sinb2;
	if (isGetPos)//External incoming sun and moon data
	{
		sunpos[0] = m_sunpos[0];sunpos[1] = m_sunpos[1];sunpos[2] = m_sunpos[2];
		moonpos[0] = m_moonpos[0];moonpos[1] = m_moonpos[1];moonpos[2] = m_moonpos[2];
		gmst = m_gmst;
	}
	else
		m_cmpClass.getSunMoonPos(Year,Month,Day,Hours,Minuts,Seconds,sunpos,moonpos,&gmst);
	//Secondary solid tide calculation
	subSolidTide(moonpos,pXYZ,pmoon2XYZ,0);//moon
	subSolidTide(sunpos,pXYZ,psun2XYZ,1);//sun
	//Three solid tides only consider the moon U and N directions
	double emoon[3] = {0},erec[3] = {0};
	double lenmoon = 0,lenrec = 0;
	double K1 = 0,K2 = 0,K3 = 0,dotmoonRec = 0;
	double GMe = m_GMi[0],GMm = m_GMi[1];//Get celestial gravity parameters
	lenmoon = m_cmpClass.norm(moonpos,3);
	lenrec = m_cmpClass.norm(pXYZ,3);
	//Unitization
	for (int i = 0; i < 3;i++)
	{
		emoon[i] = moonpos[i]/lenmoon;
		erec[i] = pXYZ[i]/lenrec;
	}
	dotmoonRec = m_cmpClass.dot(emoon,erec,3);
	K1 = GMm*lenrec*lenrec*lenrec*lenrec*lenrec/(GMe*lenmoon*lenmoon*lenmoon*lenmoon);
	K2 = loveShida3[1]*( 7.5*dotmoonRec*dotmoonRec - 1.5);
	K3 = 2.5*(loveShida3[0] - 3*loveShida3[1])*dotmoonRec*dotmoonRec*dotmoonRec + 1.5*(loveShida3[1] - loveShida3[0])*dotmoonRec;
	pmoon3XYZ[0] = K1*(K3*erec[0] + K2*emoon[0]);
	pmoon3XYZ[1] = K1*(K3*erec[1] + K2*emoon[1]);
	pmoon3XYZ[2] = K1*(K3*erec[2] + K2*emoon[2]);
	//Influence result overlay
	double pTideXYZ[3]={0};
	pTideXYZ[0] = pmoon2XYZ[0] + psun2XYZ[0] + pmoon3XYZ[0];
	pTideXYZ[1] = pmoon2XYZ[1] + psun2XYZ[1] + pmoon3XYZ[1];
	pTideXYZ[2] = pmoon2XYZ[2] + psun2XYZ[2] + pmoon3XYZ[2];	
	/* step2: frequency domain, only K1 radial */
	double sin2l=qSin(2.0*m_SationBLH[0]);
	double du=-0.012*sin2l*qSin(gmst+m_SationBLH[1]);
	pTideXYZ[0] += du*erec[0];pTideXYZ[1] += du*erec[1];pTideXYZ[2] += du*erec[2];
	//Component elimination of constant tidal deformation requires projection to vertical and north directions
	if (isElimate)
	{
		double dltaU = 0,dltaN = 0;
		double  eN[3]= {-qSin(m_SationBLH[0])*qCos(m_SationBLH[1]),-qSin(m_SationBLH[0])*qSin(m_SationBLH[1]),qCos(m_SationBLH[0])};
		dltaU = -0.0603*(3*qSin(m_SationBLH[0])*qSin(m_SationBLH[0]) - 1);
		dltaN = -0.0252*qSin(2*m_SationBLH[0]);
		pTideXYZ[0]-= (dltaU*erec[0] + dltaN*eN[0]);
		pTideXYZ[1]-= (dltaU*erec[1] + dltaN*eN[1]);
		pTideXYZ[2]-= (dltaU*erec[2] + dltaN*eN[2]);
	}
	//Convert ENU direction
	//Convert pTideXYZ to station coordinates ENU direction
	pTideXYZ[0]+=pXYZ[0];pTideXYZ[1]+=pXYZ[1];pTideXYZ[2]+=pXYZ[2];
	m_cmpClass.XYZ2ENU(pTideXYZ,pTideENU,pXYZ);
	
}

//void Read the tide file (you can only read the data of the IGS station, you can not read any other point, you can also replace the receiver name with the adjacent station)
bool QTideEffect::readOCEANFile(QString  StationName,OCEANData &oceaData,QString  OCEANFileName)
{
	if (isReadOCEAN) return true;
	OCEANData tempOCEANData;
	tempOCEANData.isRead = false;
	oceaData.isRead = false;
	StationName = StationName.trimmed();//Eliminate the file name space, and the file name in the form of a space becomes empty
	StationName = StationName.toUpper();
	if (StationName.isEmpty())
		if (!m_StationName.isEmpty())
			StationName = m_StationName;
		else
		{
			isOCEANTide = false;
			return false;
		}
			
	//Eliminate the file name space, and the file name in the form of a space becomes empty.
    //Search current directory .blq file
	QDir m_dir(".");
	QStringList m_fliterList;
	//Search current directory .blq file
	m_fliterList.append("*.blq");
	QStringList OCEANFileNameList = m_dir.entryList(m_fliterList); 
	QString dirOCEANfileName ="";
	if (!OCEANFileNameList.isEmpty())
        dirOCEANfileName = OCEANFileNameList.at(0);\
	if (OCEANFileName.isEmpty())
	{
		if (!m_OCEANFileName.isEmpty())
			OCEANFileName = m_OCEANFileName;
		else if (!dirOCEANfileName.isEmpty())
			OCEANFileName = dirOCEANfileName;
		else
			return false;
	}
	m_readOCEANClass.setFileName(OCEANFileName);
    if(!m_readOCEANClass.open(QFile::ReadOnly))
    {
        ErroTrace("QTideEffect::readOCEANFile, Can not open BLQ file.");
        isReadOCEAN = true;
        return false;
    }

	//Skip header file
	QString tempLine="";
	while (!tempLine.contains("END"))
		tempLine = m_readOCEANClass.readLine();
	//Read data start symbol（$$）
	QString tempStationName;
	while (!m_readOCEANClass.atEnd())
	{
		//2-6 per line is empty, it means the data will start "$$" and the length is 4
		while (4 != tempLine.length())
		{
			tempLine = m_readOCEANClass.readLine();
			if (m_readOCEANClass.atEnd()) 
			{
				isOCEANTide = false;
				oceaData.isRead = false;
				break;	
			}
		}
			
		//Read station data
		tempLine = m_readOCEANClass.readLine();//Read header file line
		if (m_readOCEANClass.atEnd()) 
		{
			isOCEANTide = false;
			oceaData.isRead = false;
			break;	
		}
		tempStationName = tempLine.mid(2,4).trimmed().toUpper();
		if (tempStationName != StationName) continue;
		//Find the station
		tempOCEANData.StationName = tempStationName;
		//Skip comments (can read station BLH, not read here)
		tempLine = m_readOCEANClass.readLine();//Read comment
		while (tempLine.mid(0,2).contains("$"))
			tempLine = m_readOCEANClass.readLine();//Read comment
		//Analytical data
		for (int i = 0;i < 6;i++)
		{
			for (int j = 0;j < 11;j++)
			{
				if (i < 3)
					tempOCEANData.amp[i][j] = tempLine.mid(2+j*7,6).toDouble();
				else
					tempOCEANData.phasedats[i-3][j] = tempLine.mid(2+j*7,6).toDouble();
			}
			tempLine = m_readOCEANClass.readLine();//Read comment
		}
		tempOCEANData.isRead = true;
		break;//Read out data
	}
	isReadOCEAN = true;//I have read the file and cannot be sure that the station was found.
	oceaData = tempOCEANData;//save data
	if (tempOCEANData.isRead = false)	isOCEANTide = false;
	m_readOCEANClass.close();
	return true;
}

//Calculate the impact of the tide
void QTideEffect::getOCEANTide(int Year,int Month,int Day,int Hours,int Minuts,double Seconds,double *pXYZ,double *pTideENU,QString StationName)
{
	pTideENU[0]=0;pTideENU[1]=0;pTideENU[2]=0;
	if (!isOCEANTide)	return ;
	if (!isReadOCEAN)
		if (!readOCEANFile(StationName,m_OCEANData,m_OCEANFileName))
			return ; 
	if (!m_OCEANData.isRead) return ; //Determine if the data has been read and is available
	//Reference RTKLIB ARG
	const double ep1975[]={1975,1,1,0,0,0};
	double ep[6] = {0},fday = 0,days = 0,t = 0,t2 = 0,t3 = 0,a[5] = {0},ang = 0,dp[3]={0};
	int i = 0,j = 0;

	/* angular argument: see subroutine arg.f for reference [1] */
	//m_cmpClass.time2epoch(tut,ep);
	//UT1 used by RTKLIB tut requires erp to increase the amount of computation for the file. UT1 is ignored here and can be obtained using the getErp function.
	ep[0] = Year;ep[1] = Month;ep[2] = Day;ep[3] = Hours;ep[4] = Minuts;ep[5] = Seconds;
	//Convert to UTC time
	if (LeapSeconds <= 0)//Jump seconds are only obtained once, which is a number greater than zero
		LeapSeconds = m_cmpClass.getLeapSecond(Year,Month,Day,Hours,Minuts,Seconds);
	ep[5]-=LeapSeconds;
	fday=ep[3]*3600.0+ep[4]*60.0+ep[5];
	ep[3]=ep[4]=ep[5]=0.0;
	days=m_cmpClass.timediff(m_cmpClass.epoch2time(ep),m_cmpClass.epoch2time(ep1975))/86400.0;
	t=(27392.500528+1.000000035*days)/36525.0;
	t2=t*t; t3=t2*t;

	a[0]=fday;
	a[1]=(279.69668+36000.768930485*t+3.03E-4*t2)*D2R; /* H0 */
	a[2]=(270.434358+481267.88314137*t-0.001133*t2+1.9E-6*t3)*D2R; /* S0 */
	a[3]=(334.329653+4069.0340329577*t+0.010325*t2-1.2E-5*t3)*D2R; /* P0 */
	a[4]=2.0*MM_PI;
	/* displacements by 11 constituents */
	for (i=0;i<11;i++) {
		ang=0.0;
		for (j=0;j<5;j++) ang+=a[j]*args[i][j];
		for (j=0;j<3;j++) dp[j]+=m_OCEANData.amp[j][i]*cos(ang-m_OCEANData.phasedats[j][i]*D2R);
	}
	pTideENU[0]=-dp[1];
	pTideENU[1]=-dp[2];
	pTideENU[2]= dp[0];
}

//Get the impact of all tides on the ENU direction
void QTideEffect::getAllTideEffectENU(int Year,int Month,int Day,int Hours,int Minuts,double Seconds,double *pXYZ,double *pENU,double *psunpos/* =NULL */, double *pmoonpos /* = NULL */,double gmst /* = 0 */,QString StationName /* = "" */)
{
	pENU[0] = 0;pENU[1] = 0;pENU[2] = 0;
    isGetPos = false;
	if (psunpos&&pmoonpos)
		setSunMoonPos(psunpos,pmoonpos,gmst);
	//Get a tide of solids
	getSoildTide(Year,Month,Day,Hours,Minuts,Seconds,pXYZ,m_pSolidENU);//Adopt default does not consider permanent corrosion
	//Get the ocean tide
	if (!StationName.isEmpty())
		getOCEANTide(Year,Month,Day,Hours,Minuts,Seconds,pXYZ,m_pOCEANENU,StationName);
	else if (!m_StationName.isEmpty())
		getOCEANTide(Year,Month,Day,Hours,Minuts,Seconds,pXYZ,m_pOCEANENU,m_StationName);
	//Get the tide
    getPoleTide(Year,Month,Day,Hours,Minuts,Seconds,m_SationBLH,m_pPoleENU);

	//Tidal effect superposition
	pENU[0] = m_pSolidENU[0] + m_pOCEANENU[0] + m_pPoleENU[0];
	pENU[1] = m_pSolidENU[1] + m_pOCEANENU[1] + m_pPoleENU[1];
	pENU[2] = m_pSolidENU[2] + m_pOCEANENU[2] + m_pPoleENU[2];
	
}

double QTideEffect::getAllTideEffect(int Year,int Month,int Day,int Hours,int Minuts,double Seconds,
                                     double *pXYZ,double *EA,double *psunpos, double *pmoonpos,double gmst,QString StationName)
{
    double effectDistance = 0;
    isGetPos = false;
    if (qAbs(m_SecondFlag - (Hours*3600+Minuts*60+Seconds)) > 0.3)//Two differences greater than 0.3s are considered to replace the epoch to recalculate the sun coordinates (a single epoch does not repeatedly calculate the sun coordinates)
    {
        if(psunpos&&pmoonpos)
            setSunMoonPos(psunpos,pmoonpos,gmst);
        else
            m_cmpClass.getSunMoonPos(Year,Month,Day,Hours,Minuts,Seconds,m_sunpos,m_moonpos,&m_gmst);//An epoch only needs to be calculated once, no need to calculate multiple times
        getAllTideEffectENU(Year,Month,Day,Hours,Minuts,Seconds,pXYZ,m_AllTideENU,m_sunpos,m_moonpos,m_gmst,StationName);
        m_SecondFlag = Hours*3600+Minuts*60+Seconds;
	}
	effectDistance = m_AllTideENU[0]*qCos(EA[0])*qSin(EA[1]) + m_AllTideENU[1]*qCos(EA[0])*qCos(EA[1]) + m_AllTideENU[2]*qSin(EA[0]);
	return effectDistance;
}

void QTideEffect::getAllData()
{
	readRepFile();//Read erp file to variable during initialization m_erpData
	readOCEANFile(m_StationName,m_OCEANData,m_OCEANFileName);
}
