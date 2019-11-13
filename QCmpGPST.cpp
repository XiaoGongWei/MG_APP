#include "QCmpGPST.h"

QCmpGPST::QCmpGPST()
{
	//WGS-84 coordinate parameters
	elipsePara[0] = 6378137.0;
	elipsePara[1] = 6356752.3142;
	elipsePara[2] = 6399593.6258;
	elipsePara[3] = 1/298.257223563;
	elipsePara[4] = 0.00669437999014132;
	elipsePara[5] = 0.00673949674227;
	//Read erp data
	//Initialize erp_t data
	m_erpData.n = 0;
	m_erpData.nmax = 10;
	m_erpData.data = (erpd_t*)malloc(sizeof(erpd_t)*m_erpData.nmax);
    isuseErp = false;
}

bool QCmpGPST::readRepFile(QString m_erpFileName)
{
	if (!m_erpFileName.isEmpty())
	{//First determine if the user is passing the erp path
        isuseErp = readerp(m_erpFileName.toLatin1().data(),&m_erpData);
	}
	else
	{
		isuseErp = false;
	}
	return true;
}

QCmpGPST::~QCmpGPST()
{
	free(m_erpData.data);
}

//Calculate Julian Day
double QCmpGPST::computeJD(int Year,int Month,int Day,int HoursInt,int Minutes,double Seconds)
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

//Computational simplified Julian day
double QCmpGPST::computeMJD(int Year,int Month,int Day,int HoursInt,int Minutes,double Seconds)
{
	double MJD = computeJD(Year,Month,Day,HoursInt,Minutes,Seconds)  - 2400000.5; 
	return MJD;
}
//
double QCmpGPST::YMD2GPSTime(int Year,int Month,int Day,int HoursInt,int Minutes,double Seconds,int *WeekN, int *day)//,int *GPSTimeArray
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
    if (day) *day = N;
	return (N*24*3600 + HoursInt*3600 + Minutes*60 + Seconds);
}

void QCmpGPST::XYZ2BLH(double X,double Y,double Z,double *m_pBLH,double *ellipseCoeff)
{//
	if (ellipseCoeff == NULL)
	{//WGS84 coordinate system is used by default
		ellipseCoeff = elipsePara;
	}
	if (X == 0)
		return;
	m_pBLH[1] = MyAtanL(X,Y);//L appears bug

	//Compute B
	if(X == 0&&Y == 0)
		return;
	double t0 = Z/qSqrt(X*X + Y*Y);
	double P = (ellipseCoeff[2]*ellipseCoeff[4])/qSqrt(X*X + Y*Y);
	double k = 1 + ellipseCoeff[5];
	double eps = 1e-12;
	double ti = t0;
	double ti1 = 0;
	double Bi = qAtan(ti);
	double Bi1 = 0;
	do 
	{
		ti1 = t0 + P*ti/qSqrt(k + ti*ti);
		Bi = qAtan(ti);
		Bi1 = qAtan(ti1);
		ti = ti1;
	} while (qAbs(Bi1-Bi) > eps);
	m_pBLH[0] = Bi1;
	double W = qSqrt(1 - ellipseCoeff[4]*qSin(Bi1)*qSin(Bi1));
	double N = ellipseCoeff[0]/W;
	m_pBLH[2] = Z/qSin(Bi1) - N*(1 - ellipseCoeff[4]);
}

void QCmpGPST::XYZ2SAE(double X,double Y,double Z,double *m_pSAZ,double *PX)
{
	double dx = X - PX[0];
	double dy = Y - PX[1];
	double dz = Z - PX[2];
	double PX_BLH[3] = {0};
	
	XYZ2BLH(PX[0],PX[1],PX[2],PX_BLH,elipsePara);//Calculate L to appear bug
	double B = PX_BLH[0];
	double L = PX_BLH[1];
	double xx = (-qSin(B)*qCos(L)*dx - qSin(B)*qSin(L)*dy + qCos(B)*dz);
	double yy = (-qSin(L)*dx + qCos(L)*dy + 0*dz);
	double zz = (qCos(B)*qCos(L)*dx + qCos(B)*qSin(L)*dy + qSin(B)*dz);

	m_pSAZ[0] = qSqrt(xx*xx + yy*yy + zz*zz);//S
	if (xx == 0)
		return;
	m_pSAZ[1] = MyAtanA(xx,yy);//A

	if (m_pSAZ[0] == 0)
		return;
	m_pSAZ[2] = qAcos(qAbs(zz/m_pSAZ[0]));//Z(0,pi/2)appear bug
}

void QCmpGPST::XYZ2ENU(double X,double Y,double Z,double *m_pENU,double *PX)
{
	
	double dx = X - PX[0];
	double dy = Y - PX[1];
	double dz = Z - PX[2];
	double PX_BLH[3] = {0};
	XYZ2BLH(PX[0],PX[1],PX[2],PX_BLH,elipsePara);//Calculate L to appear bug
	double B = PX_BLH[0];
	double L = PX_BLH[1];
	m_pENU[0] = (-qSin(L)*dx + qCos(L)*dy);
	m_pENU[1] = (-qSin(B)*qCos(L)*dx - qSin(B)*qSin(L)*dy + qCos(B)*dz);
	m_pENU[2] = (qCos(B)*qCos(L)*dx + qCos(B)*qSin(L)*dy + qSin(B)*dz);
}
//XYZ ： Receiver approximate coordinates. m_SAZ（radian）Returned calculation result  PX station coordinates
void QCmpGPST::XYZ2SAE(double *pXYZ,double *m_pSAZ,double *PX)
{
	XYZ2SAE(pXYZ[0],pXYZ[1],pXYZ[2],m_pSAZ,PX);
}
void QCmpGPST::XYZ2BLH(double *pXYZ,double *m_pBLH)
{
	XYZ2BLH(pXYZ[0],pXYZ[1],pXYZ[2],m_pBLH);
}
void QCmpGPST::XYZ2ENU(double *pXYZ,double *m_pENU,double *PX)
{
	XYZ2ENU(pXYZ[0],pXYZ[1],pXYZ[2],m_pENU,PX);
}

double QCmpGPST::MyAtanA(double x,double y)
{
	double angel = 0;
	if (x > 0&&y>0)
	{
		angel = atan(y/x);
	}
	else if(x < 0&&y > 0)
	{
        angel = atan(y/x) + MM_PI;
	}
	else if (x < 0&&y < 0)
	{
        angel = atan(y/x) + MM_PI;
	}
	else
	{
        angel = atan(y/x) + 2*MM_PI;
	}
	return angel;
}

double QCmpGPST::MyAtanL(double x,double y)
{//Calculate the longitude according to (x, y) (radian means [-pi, pi])
	double angel = 0;
	if (x > 0&&y>0)
	{
		angel = atan(y/x);
	}
	else if(x < 0&&y > 0)
	{
        angel = atan(y/x) + MM_PI;
	}
	else if (x < 0&&y < 0)
	{
        angel = -(MM_PI - atan(y/x));
	}
	else
	{
		angel = atan(y/x);
	}
	return angel;
}

double QCmpGPST::InnerVector(double *a,double *b,int Vectorlen)
{
	double resultAB = 0;
	for(int i = 0;i < Vectorlen;i++)
		resultAB+=a[i]*b[i];
	return resultAB;
}

bool QCmpGPST::OutVector(double *a,double *b,double *c)
{
	c[0]=a[1]*b[2]-a[2]*b[1];
	c[1]=a[2]*b[0]-a[0]*b[2];
	c[2]=a[0]*b[1]-a[1]*b[0];
	return true;
}


// Leap file: ftp://hpiers.obspm.fr/iers/bul/bulc/Leap_Second.dat
double QCmpGPST::getLeapSecond(int Year, int Month, int Day, int Hours/* =0 */, int Minutes/* =0 */, double Seconds/* =0 */)
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

/* outer product of 3d vectors -------------------------------------------------
* outer product of 3d vectors 
* args   : double *a,*b     I   vector a,b (3 x 1)
*          double *c        O   outer product (a x b) (3 x 1)
* return : none
*-----------------------------------------------------------------------------*/
void QCmpGPST::cross3(const double *a, const double *b, double *c)
{
	c[0]=a[1]*b[2]-a[2]*b[1];
	c[1]=a[2]*b[0]-a[0]*b[2];
	c[2]=a[0]*b[1]-a[1]*b[0];
}

/* ecef to local coordinate transfromation matrix ------------------------------
* compute ecef to local coordinate transfromation matrix
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *E        O   ecef to local coord transformation matrix (3x3)
* return : none
* notes  : matirix stored by column-major order (fortran convention)
*-----------------------------------------------------------------------------*/
void QCmpGPST::xyz2enu(const double *pos, double *E)
{
	double sinp=sin(pos[0]),cosp=cos(pos[0]),sinl=sin(pos[1]),cosl=cos(pos[1]);

	E[0]=-sinl;      E[3]=cosl;       E[6]=0.0;
	E[1]=-sinp*cosl; E[4]=-sinp*sinl; E[7]=cosp;
	E[2]=cosp*cosl;  E[5]=cosp*sinl;  E[8]=sinp;
}

/* euclid norm -----------------------------------------------------------------
* euclid norm of vector
* args   : double *a        I   vector a (n x 1)
*          int    n         I   size of vector a
* return : || a ||
*-----------------------------------------------------------------------------*/
double QCmpGPST::norm(const double *a, int n)
{
	return sqrt(dot(a,a,n));
}


/* normalize 3d vector ---------------------------------------------------------
* normalize 3d vector
* args   : double *a        I   vector a (3 x 1)
*          double *b        O   normlized vector (3 x 1) || b || = 1
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
int QCmpGPST::normv3(const double *a, double *b)
{
	double r;
	if ((r=norm(a,3))<=0.0) return 0;
	b[0]=a[0]/r;
	b[1]=a[1]/r;
	b[2]=a[2]/r;
	return 1;
}

int QCmpGPST::getSatPRN(QString StaliteName)
{
	int PRN = -1;
	if(StaliteName.contains("G",Qt::CaseInsensitive))
		PRN =  StaliteName.right(2).toInt();
	else if (StaliteName.contains("C",Qt::CaseInsensitive))
		PRN = StaliteName.right(2).toInt() + 100;//北斗PRN=100+RRN
	else if (StaliteName.contains("R",Qt::CaseInsensitive))
		PRN = StaliteName.right(2).toInt() + 200;//GLONASS PRN=200+RRN
	return PRN;
}

//Calculate the annual accumulation date
int QCmpGPST::YearAccDay(int Year, int Month, int Day)
{
	//int YearDay = qFloor(Month*275/9) - qFloor((Month+9)/12) * ((qFloor(qCos(Year%100*0.01)) * (Year/100-qFloor(Year/400)*4)+2)/3)+1+Day-30;
	if(Month == 2 &&(Day <0 || Day > 29))
		return -1;
	if (Month < 1||Month > 12)
		return -1;
	if(Day< 1 || Day > 31)
		return -1;
	int YearDay = 0;
	int YearArray[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
	if((Year%4==0 && Year%100!=0) || Year%400==0)
		YearArray[1]++;
	for(int i = 0;i < Month-1;i++)
	{
		YearDay+=YearArray[i];
	}
	YearDay+=Day;
	return YearDay;
}

//The following reference RTKLAB to obtain the sun coordinates
/* get earth rotation parameter values -----------------------------------------
* get earth rotation parameter values
* args   : erp_t  *erp        I   earth rotation parameters
*          gtime_t time       I   time (gpst)
*          double *erpv       O   erp values {xp,yp,ut1_utc,lod} (rad,rad,s,s/d)
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
int QCmpGPST::geterp(const erp_t *erp, gtime_t time, double *erpv)
{
	const double ep[]={2000,1,1,12,0,0};
	double mjd,day,a;
	int i=0,j,k;

	//trace(4,"geterp:\n");

	if (erp->n<=0) return 0;

	mjd=51544.5+(timediff(gpst2utc(time),epoch2time(ep)))/86400.0;

	if (mjd<=erp->data[0].mjd) {
		day=mjd-erp->data[0].mjd;
		erpv[0]=erp->data[0].xp     +erp->data[0].xpr*day;
		erpv[1]=erp->data[0].yp     +erp->data[0].xpr*day;
		erpv[2]=erp->data[0].ut1_utc-erp->data[0].lod*day;
		erpv[3]=erp->data[0].lod;
		return 1;
	}
	if (mjd>=erp->data[erp->n-1].mjd) {
		day=mjd-erp->data[erp->n-1].mjd;
		erpv[0]=erp->data[erp->n-1].xp     +erp->data[erp->n-1].xpr*day;
		erpv[1]=erp->data[erp->n-1].yp     +erp->data[erp->n-1].ypr*day;
		erpv[2]=erp->data[erp->n-1].ut1_utc-erp->data[erp->n-1].lod*day;
		erpv[3]=erp->data[erp->n-1].lod;
		return 1;
	}
	for (j=0,k=erp->n-1;j<=k;) {
		i=(j+k)/2;
		if (mjd<erp->data[i].mjd) k=i-1; else j=i+1;
	}
	//If crossing the border
	if (i >= erp->n - 1)
		i--;

	if (erp->data[i].mjd==mjd-erp->data[i+1].mjd) {
		a=0.5;
	}
	else {
		//a=(mjd-erp->data[i+1].mjd)/(erp->data[i].mjd-mjd-erp->data[i+1].mjd);
		a = (mjd - erp->data[i].mjd)/(erp->data[i+1].mjd - erp->data[i].mjd);
	}
	erpv[0]=(1.0-a)*erp->data[i].xp     +a*erp->data[i+1].xp;
	erpv[1]=(1.0-a)*erp->data[i].yp     +a*erp->data[i+1].yp;
	erpv[2]=(1.0-a)*erp->data[i].ut1_utc+a*erp->data[i+1].ut1_utc;
	erpv[3]=(1.0-a)*erp->data[i].lod    +a*erp->data[i+1].lod;
	return 1;
}

/* read earth rotation parameters ----------------------------------------------
* read earth rotation parameters
* args   : char   *file       I   IGS ERP file (IGS ERP ver.2)
*          erp_t  *erp        O   earth rotation parameters
* return : status (1:ok,0:file open error)
*-----------------------------------------------------------------------------*/
int QCmpGPST::readerp(const char *file, erp_t *erp)
{
	FILE *fp;
	erpd_t *erp_data;
	double v[14]={0};
	char buff[256];

	//trace(3,"readerp: file=%s\n",file);

	if (!(fp=fopen(file,"r"))) {
		//trace(2,"erp file open error: file=%s\n",file);
		return 0;
	}
	while (fgets(buff,sizeof(buff),fp)) {
		if (sscanf(buff,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
			v,v+1,v+2,v+3,v+4,v+5,v+6,v+7,v+8,v+9,v+10,v+11,v+12,v+13)<5) {
				continue;
		}
		if (erp->n>=erp->nmax) {
			erp->nmax=erp->nmax<=0?128:erp->nmax*2;
			erp_data=(erpd_t *)realloc(erp->data,sizeof(erpd_t)*erp->nmax);
			if (!erp_data) {
				free(erp->data); erp->data=NULL; erp->n=erp->nmax=0;
				fclose(fp);
				return 0;
			}
			erp->data=erp_data;
		}
		erp->data[erp->n].mjd=v[0];
		erp->data[erp->n].xp=v[1]*1E-6*AS2R;
		erp->data[erp->n].yp=v[2]*1E-6*AS2R;
		erp->data[erp->n].ut1_utc=v[3]*1E-6;
		erp->data[erp->n].lod=v[4]*1E-6;
		erp->data[erp->n].xpr=v[12]*1E-6*AS2R;
		erp->data[erp->n++].ypr=v[13]*1E-6*AS2R;
	}
	fclose(fp);
	return 1;
}

//Reference RTKlab

/* add time --------------------------------------------------------------------
* add time to gtime_t struct
* args   : gtime_t t        I   gtime_t struct
*          double sec       I   time to add (s)
* return : gtime_t struct (t+sec)
*-----------------------------------------------------------------------------*/
gtime_t QCmpGPST::timeadd(gtime_t t, double sec)
{
	double tt;

	t.sec+=sec; tt=floor(t.sec); t.time+=(int)tt; t.sec-=tt;
	return t;
}

/* time difference -------------------------------------------------------------
* difference between gtime_t structs
* args   : gtime_t t1,t2    I   gtime_t structs
* return : time difference (t1-t2) (s)
*-----------------------------------------------------------------------------*/
double QCmpGPST::timediff(gtime_t t1, gtime_t t2)
{
	double temp = difftime(t1.time,t2.time);
	return difftime(t1.time,t2.time)+t1.sec-t2.sec;
}

//double ep = {1970,1,1,0,0,0}Convert to gtime_t;
gtime_t QCmpGPST::epoch2time(const double *ep)
{
	const int doy[]={1,32,60,91,121,152,182,213,244,274,305,335};
	gtime_t time={0};
	int days,sec,year=(int)ep[0],mon=(int)ep[1],day=(int)ep[2];

	if (year<1970||2099<year||mon<1||12<mon) return time;

	/* leap year if year%4==0 in 1901-2099 */
	days=(year-1970)*365+(year-1969)/4+doy[mon-1]+day-2+(year%4==0&&mon>=3?1:0);
	sec=(int)floor(ep[5]);
	time.time=(time_t)days*86400+(int)ep[3]*3600+(int)ep[4]*60+sec;
	time.sec=ep[5]-sec;
	return time;
}

/* utc to gpstime --------------------------------------------------------------
* convert utc to gpstime considering leap seconds
* args   : gtime_t t        I   time expressed in utc
* return : time expressed in gpstime
* notes  : ignore slight time offset under 100 ns
*-----------------------------------------------------------------------------*/
gtime_t QCmpGPST::utc2gpst(gtime_t t)
{
	int i;

	for (i=0;i<(int)sizeof(leaps)/(int)sizeof(*leaps);i++) {
		if (timediff(t,epoch2time(leaps[i]))>=0.0) return timeadd(t,-leaps[i][6]);
	}
	return t;
}

/* time to calendar day/time ---------------------------------------------------
* convert gtime_t struct to calendar day/time
* args   : gtime_t t        I   gtime_t struct
*          double *ep       O   day/time {year,month,day,hour,min,sec}
* return : none
* notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
*-----------------------------------------------------------------------------*/
void QCmpGPST::time2epoch(gtime_t t, double *ep)
{
	const int mday[]={ /* # of days in a month */
		31,28,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31,
		31,29,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31
	};
	int days,sec,mon,day;

	/* leap year if year%4==0 in 1901-2099 */
	days=(int)(t.time/86400);
	sec=(int)(t.time-(time_t)days*86400);
	for (day=days%1461,mon=0;mon<48;mon++) {
		if (day>=mday[mon]) day-=mday[mon]; else break;
	}
	ep[0]=1970+days/1461*4+mon/12; ep[1]=mon%12+1; ep[2]=day+1;
	ep[3]=sec/3600; ep[4]=sec%3600/60; ep[5]=sec%60+t.sec;
}

/* time to gps time ------------------------------------------------------------
* convert gtime_t struct to week and tow in gps time
* args   : gtime_t t        I   gtime_t struct
*          int    *week     IO  week number in gps time (NULL: no output)
* return : time of week in gps time (s)
*-----------------------------------------------------------------------------*/
double QCmpGPST::time2gpst(gtime_t t, int *week)
{
	gtime_t t0=epoch2time(gpst0);
	time_t sec=t.time-t0.time;
	int w=(int)(sec/(86400*7));

	if (week) *week=w;
	return (double)(sec-w*86400*7)+t.sec;
}

gtime_t QCmpGPST::gpst2time(int week, double sec)
{
	gtime_t t=epoch2time(gpst0);

	if (sec<-1E9||1E9<sec) sec=0.0;
	t.time+=86400*7*week+(int)sec;
	t.sec=sec-(int)sec;
	return t;
}

/* time to day and sec -------------------------------------------------------*/
double QCmpGPST::time2sec(gtime_t time, gtime_t *day)
{
	double ep[6],sec;
	time2epoch(time,ep);
	sec=ep[3]*3600.0+ep[4]*60.0+ep[5];
	ep[3]=ep[4]=ep[5]=0.0;
	*day=epoch2time(ep);
	return sec;
}

/* utc to gmst -----------------------------------------------------------------
* convert utc to gmst (Greenwich mean sidereal time)
* args   : gtime_t t        I   time expressed in utc
*          double ut1_utc   I   UT1-UTC (s)
* return : gmst (rad)
*-----------------------------------------------------------------------------*/
double QCmpGPST::utc2gmst(gtime_t t, double ut1_utc)
{
	const double ep2000[]={2000,1,1,12,0,0};
	gtime_t tut,tut0;
	double ut,t1,t2,t3,gmst0,gmst;

	tut=timeadd(t,ut1_utc);
	ut=time2sec(tut,&tut0);
	t1=timediff(tut0,epoch2time(ep2000))/86400.0/36525.0;
	t2=t1*t1; t3=t2*t1;
	gmst0=24110.54841+8640184.812866*t1+0.093104*t2-6.2E-6*t3;
	gmst=gmst0+1.002737909350795*ut;

    return fmod(gmst,86400.0)*MM_PI/43200.0; /* 0 <= gmst <= 2*MM_PI */
}

/* gpstime to utc --------------------------------------------------------------
* convert gpstime to utc considering leap seconds
* args   : gtime_t t        I   time expressed in gpstime
* return : time expressed in utc
* notes  : ignore slight time offset under 100 ns
*-----------------------------------------------------------------------------*/
gtime_t QCmpGPST::gpst2utc(gtime_t t)
{
	gtime_t tu;
	int i;
	for (i=0;i<(int)sizeof(leaps)/(int)sizeof(*leaps);i++) {
		tu=timeadd(t,leaps[i][6]);
		if (timediff(tu,epoch2time(leaps[i]))>=0.0) return tu;
	}
	return t;
}

/* inner product ---------------------------------------------------------------
* inner product of vectors
* args   : double *a,*b     I   vector a,b (n x 1)
*          int    n         I   size of vector a,b
* return : a'*b
*-----------------------------------------------------------------------------*/
double QCmpGPST::dot(const double *a, const double *b, int n)
{
	double c=0.0;

	while (--n>=0) c+=a[n]*b[n];
	return c;
}

/* astronomical arguments: f={l,l',F,D,OMG} (rad) ----------------------------*/
void QCmpGPST::ast_args(double t, double *f)
{
	static const double fc[][5]={ /* coefficients for iau 1980 nutation */
		{ 134.96340251, 1717915923.2178,  31.8792,  0.051635, -0.00024470},
		{ 357.52910918,  129596581.0481,  -0.5532,  0.000136, -0.00001149},
		{  93.27209062, 1739527262.8478, -12.7512, -0.001037,  0.00000417},
		{ 297.85019547, 1602961601.2090,  -6.3706,  0.006593, -0.00003169},
		{ 125.04455501,   -6962890.2665,   7.4722,  0.007702  -0.00005939}
	};
	double tt[4];
	int i,j;

	for (tt[0]=t,i=1;i<4;i++) tt[i]=tt[i-1]*t;
	for (i=0;i<5;i++) {
		f[i]=fc[i][0]*3600.0;
		for (j=0;j<4;j++) f[i]+=fc[i][j+1]*tt[j];
        f[i]=fmod(f[i]*AS2R,2.0*MM_PI);
	}
}

void QCmpGPST::matmul(const char *tr, int n, int k, int m, double alpha,
	const double *A, const double *B, double beta, double *C)
{
	double d;
	int i,j,x,f=tr[0]=='N'?(tr[1]=='N'?1:2):(tr[1]=='N'?3:4);

	for (i=0;i<n;i++) for (j=0;j<k;j++) {
		d=0.0;
		switch (f) {
		case 1: for (x=0;x<m;x++) d+=A[i+x*n]*B[x+j*m]; break;
		case 2: for (x=0;x<m;x++) d+=A[i+x*n]*B[j+x*k]; break;
		case 3: for (x=0;x<m;x++) d+=A[x+i*m]*B[x+j*m]; break;
		case 4: for (x=0;x<m;x++) d+=A[x+i*m]*B[j+x*k]; break;
		}
		if (beta==0.0) C[i+j*n]=alpha*d; else C[i+j*n]=alpha*d+beta*C[i+j*n];
	}
}

/* sun and moon position in eci (ref [4] 5.1.1, 5.2.1) -----------------------*/
void QCmpGPST::sunmoonpos_eci(gtime_t tut, double *rsun, double *rmoon)
{
	const double ep2000[6]={2000,1,1,12,0,0};
	double t = 0.0,f[5]={0.0},eps = 0.0,Ms = 0.0,ls = 0.0,rs = 0.0,lm = 0.0,
		pm = 0.0,rm = 0.0,sine = 0.0,cose = 0.0,sinp = 0.0,cosp = 0.0,sinl = 0.0,cosl = 0.0;

	//trace(3,"sunmoonpos_eci: tut=%s\n",time_str(tut,3));
	t=timediff(tut,epoch2time(ep2000))/86400.0/36525.0;

	/* astronomical arguments */
	ast_args(t,f);
	/* obliquity of the ecliptic */
	eps=23.439291-0.0130042*t;
	sine=sin(eps*D2R); cose=cos(eps*D2R);

	/* sun position in eci */
	if (rsun) {
		Ms=357.5277233+35999.05034*t;
		ls=280.460+36000.770*t+1.914666471*sin(Ms*D2R)+0.019994643*sin(2.0*Ms*D2R);
		rs=AU*(1.000140612-0.016708617*cos(Ms*D2R)-0.000139589*cos(2.0*Ms*D2R));
		sinl=sin(ls*D2R); cosl=cos(ls*D2R);
		rsun[0]=rs*cosl;
		rsun[1]=rs*cose*sinl;
		rsun[2]=rs*sine*sinl;

		//trace(5,"rsun =%.3f %.3f %.3f\n",rsun[0],rsun[1],rsun[2]);
	}
	/* moon position in eci */
	if (rmoon) {
		lm=218.32+481267.883*t+6.29*sin(f[0])-1.27*sin(f[0]-2.0*f[3])+
			0.66*sin(2.0*f[3])+0.21*sin(2.0*f[0])-0.19*sin(f[1])-0.11*sin(2.0*f[2]);
		pm=5.13*sin(f[2])+0.28*sin(f[0]+f[2])-0.28*sin(f[2]-f[0])-
			0.17*sin(f[2]-2.0*f[3]);
		rm=RE_WGS84/sin((0.9508+0.0518*cos(f[0])+0.0095*cos(f[0]-2.0*f[3])+
			0.0078*cos(2.0*f[3])+0.0028*cos(2.0*f[0]))*D2R);
		sinl=sin(lm*D2R); 
		cosl=cos(lm*D2R);
		sinp=sin(pm*D2R); 
		cosp=cos(pm*D2R);
		rmoon[0]=rm*cosp*cosl;
		rmoon[1]=rm*cose*cosp*sinl - rm*sine*sinp;//rmoon[1]=rm*(cose*cosp*sinl - sine*sinp); release  Optimized under mode (fastest speed)
		rmoon[2]=rm*sine*cosp*sinl + rm*cose*sinp;//rmoon[2]=rm*(sine*cosp*sinl + cose*sinp);release  Optimized under mode (fastest speed)
		//trace(5,"rmoon=%.3f %.3f %.3f\n",rmoon[0],rmoon[1],rmoon[2]);
	}
	return ;
}

/* transform ecef to geodetic postion ------------------------------------------
* transform ecef position to geodetic position
* args   : double *r        I   ecef position {x,y,z} (m)
*          double *pos      O   geodetic position {lat,lon,h} (rad,m)
* return : none
* notes  : WGS84, ellipsoidal height
*-----------------------------------------------------------------------------*/
void QCmpGPST::ecef2pos(const double *r, double *pos)
{
	double e2=FE_WGS84*(2.0-FE_WGS84),r2=dot(r,r,2),z,zk,v=RE_WGS84,sinp;

	for (z=r[2],zk=0.0;fabs(z-zk)>=1E-4;) {
		zk=z;
		sinp=z/sqrt(r2+z*z);
		v=RE_WGS84/sqrt(1.0-e2*sinp*sinp);
		z=r[2]+v*e2*sinp;
	}
    pos[0]=r2>1E-12?atan(z/sqrt(r2)):(r[2]>0.0?MM_PI/2.0:-MM_PI/2.0);
	pos[1]=r2>1E-12?atan2(r[1],r[0]):0.0;
	pos[2]=sqrt(r2+z*z)-v;
}

/* iau 1980 nutation ---------------------------------------------------------*/
void QCmpGPST::nut_iau1980(double t, const double *f, double *dpsi, double *deps)
{
	static const double nut[106][10]={
		{   0,   0,   0,   0,   1, -6798.4, -171996, -174.2, 92025,   8.9},
		{   0,   0,   2,  -2,   2,   182.6,  -13187,   -1.6,  5736,  -3.1},
		{   0,   0,   2,   0,   2,    13.7,   -2274,   -0.2,   977,  -0.5},
		{   0,   0,   0,   0,   2, -3399.2,    2062,    0.2,  -895,   0.5},
		{   0,  -1,   0,   0,   0,  -365.3,   -1426,    3.4,    54,  -0.1},
		{   1,   0,   0,   0,   0,    27.6,     712,    0.1,    -7,   0.0},
		{   0,   1,   2,  -2,   2,   121.7,    -517,    1.2,   224,  -0.6},
		{   0,   0,   2,   0,   1,    13.6,    -386,   -0.4,   200,   0.0},
		{   1,   0,   2,   0,   2,     9.1,    -301,    0.0,   129,  -0.1},
		{   0,  -1,   2,  -2,   2,   365.2,     217,   -0.5,   -95,   0.3},
		{  -1,   0,   0,   2,   0,    31.8,     158,    0.0,    -1,   0.0},
		{   0,   0,   2,  -2,   1,   177.8,     129,    0.1,   -70,   0.0},
		{  -1,   0,   2,   0,   2,    27.1,     123,    0.0,   -53,   0.0},
		{   1,   0,   0,   0,   1,    27.7,      63,    0.1,   -33,   0.0},
		{   0,   0,   0,   2,   0,    14.8,      63,    0.0,    -2,   0.0},
		{  -1,   0,   2,   2,   2,     9.6,     -59,    0.0,    26,   0.0},
		{  -1,   0,   0,   0,   1,   -27.4,     -58,   -0.1,    32,   0.0},
		{   1,   0,   2,   0,   1,     9.1,     -51,    0.0,    27,   0.0},
		{  -2,   0,   0,   2,   0,  -205.9,     -48,    0.0,     1,   0.0},
		{  -2,   0,   2,   0,   1,  1305.5,      46,    0.0,   -24,   0.0},
		{   0,   0,   2,   2,   2,     7.1,     -38,    0.0,    16,   0.0},
		{   2,   0,   2,   0,   2,     6.9,     -31,    0.0,    13,   0.0},
		{   2,   0,   0,   0,   0,    13.8,      29,    0.0,    -1,   0.0},
		{   1,   0,   2,  -2,   2,    23.9,      29,    0.0,   -12,   0.0},
		{   0,   0,   2,   0,   0,    13.6,      26,    0.0,    -1,   0.0},
		{   0,   0,   2,  -2,   0,   173.3,     -22,    0.0,     0,   0.0},
		{  -1,   0,   2,   0,   1,    27.0,      21,    0.0,   -10,   0.0},
		{   0,   2,   0,   0,   0,   182.6,      17,   -0.1,     0,   0.0},
		{   0,   2,   2,  -2,   2,    91.3,     -16,    0.1,     7,   0.0},
		{  -1,   0,   0,   2,   1,    32.0,      16,    0.0,    -8,   0.0},
		{   0,   1,   0,   0,   1,   386.0,     -15,    0.0,     9,   0.0},
		{   1,   0,   0,  -2,   1,   -31.7,     -13,    0.0,     7,   0.0},
		{   0,  -1,   0,   0,   1,  -346.6,     -12,    0.0,     6,   0.0},
		{   2,   0,  -2,   0,   0, -1095.2,      11,    0.0,     0,   0.0},
		{  -1,   0,   2,   2,   1,     9.5,     -10,    0.0,     5,   0.0},
		{   1,   0,   2,   2,   2,     5.6,      -8,    0.0,     3,   0.0},
		{   0,  -1,   2,   0,   2,    14.2,      -7,    0.0,     3,   0.0},
		{   0,   0,   2,   2,   1,     7.1,      -7,    0.0,     3,   0.0},
		{   1,   1,   0,  -2,   0,   -34.8,      -7,    0.0,     0,   0.0},
		{   0,   1,   2,   0,   2,    13.2,       7,    0.0,    -3,   0.0},
		{  -2,   0,   0,   2,   1,  -199.8,      -6,    0.0,     3,   0.0},
		{   0,   0,   0,   2,   1,    14.8,      -6,    0.0,     3,   0.0},
		{   2,   0,   2,  -2,   2,    12.8,       6,    0.0,    -3,   0.0},
		{   1,   0,   0,   2,   0,     9.6,       6,    0.0,     0,   0.0},
		{   1,   0,   2,  -2,   1,    23.9,       6,    0.0,    -3,   0.0},
		{   0,   0,   0,  -2,   1,   -14.7,      -5,    0.0,     3,   0.0},
		{   0,  -1,   2,  -2,   1,   346.6,      -5,    0.0,     3,   0.0},
		{   2,   0,   2,   0,   1,     6.9,      -5,    0.0,     3,   0.0},
		{   1,  -1,   0,   0,   0,    29.8,       5,    0.0,     0,   0.0},
		{   1,   0,   0,  -1,   0,   411.8,      -4,    0.0,     0,   0.0},
		{   0,   0,   0,   1,   0,    29.5,      -4,    0.0,     0,   0.0},
		{   0,   1,   0,  -2,   0,   -15.4,      -4,    0.0,     0,   0.0},
		{   1,   0,  -2,   0,   0,   -26.9,       4,    0.0,     0,   0.0},
		{   2,   0,   0,  -2,   1,   212.3,       4,    0.0,    -2,   0.0},
		{   0,   1,   2,  -2,   1,   119.6,       4,    0.0,    -2,   0.0},
		{   1,   1,   0,   0,   0,    25.6,      -3,    0.0,     0,   0.0},
		{   1,  -1,   0,  -1,   0, -3232.9,      -3,    0.0,     0,   0.0},
		{  -1,  -1,   2,   2,   2,     9.8,      -3,    0.0,     1,   0.0},
		{   0,  -1,   2,   2,   2,     7.2,      -3,    0.0,     1,   0.0},
		{   1,  -1,   2,   0,   2,     9.4,      -3,    0.0,     1,   0.0},
		{   3,   0,   2,   0,   2,     5.5,      -3,    0.0,     1,   0.0},
		{  -2,   0,   2,   0,   2,  1615.7,      -3,    0.0,     1,   0.0},
		{   1,   0,   2,   0,   0,     9.1,       3,    0.0,     0,   0.0},
		{  -1,   0,   2,   4,   2,     5.8,      -2,    0.0,     1,   0.0},
		{   1,   0,   0,   0,   2,    27.8,      -2,    0.0,     1,   0.0},
		{  -1,   0,   2,  -2,   1,   -32.6,      -2,    0.0,     1,   0.0},
		{   0,  -2,   2,  -2,   1,  6786.3,      -2,    0.0,     1,   0.0},
		{  -2,   0,   0,   0,   1,   -13.7,      -2,    0.0,     1,   0.0},
		{   2,   0,   0,   0,   1,    13.8,       2,    0.0,    -1,   0.0},
		{   3,   0,   0,   0,   0,     9.2,       2,    0.0,     0,   0.0},
		{   1,   1,   2,   0,   2,     8.9,       2,    0.0,    -1,   0.0},
		{   0,   0,   2,   1,   2,     9.3,       2,    0.0,    -1,   0.0},
		{   1,   0,   0,   2,   1,     9.6,      -1,    0.0,     0,   0.0},
		{   1,   0,   2,   2,   1,     5.6,      -1,    0.0,     1,   0.0},
		{   1,   1,   0,  -2,   1,   -34.7,      -1,    0.0,     0,   0.0},
		{   0,   1,   0,   2,   0,    14.2,      -1,    0.0,     0,   0.0},
		{   0,   1,   2,  -2,   0,   117.5,      -1,    0.0,     0,   0.0},
		{   0,   1,  -2,   2,   0,  -329.8,      -1,    0.0,     0,   0.0},
		{   1,   0,  -2,   2,   0,    23.8,      -1,    0.0,     0,   0.0},
		{   1,   0,  -2,  -2,   0,    -9.5,      -1,    0.0,     0,   0.0},
		{   1,   0,   2,  -2,   0,    32.8,      -1,    0.0,     0,   0.0},
		{   1,   0,   0,  -4,   0,   -10.1,      -1,    0.0,     0,   0.0},
		{   2,   0,   0,  -4,   0,   -15.9,      -1,    0.0,     0,   0.0},
		{   0,   0,   2,   4,   2,     4.8,      -1,    0.0,     0,   0.0},
		{   0,   0,   2,  -1,   2,    25.4,      -1,    0.0,     0,   0.0},
		{  -2,   0,   2,   4,   2,     7.3,      -1,    0.0,     1,   0.0},
		{   2,   0,   2,   2,   2,     4.7,      -1,    0.0,     0,   0.0},
		{   0,  -1,   2,   0,   1,    14.2,      -1,    0.0,     0,   0.0},
		{   0,   0,  -2,   0,   1,   -13.6,      -1,    0.0,     0,   0.0},
		{   0,   0,   4,  -2,   2,    12.7,       1,    0.0,     0,   0.0},
		{   0,   1,   0,   0,   2,   409.2,       1,    0.0,     0,   0.0},
		{   1,   1,   2,  -2,   2,    22.5,       1,    0.0,    -1,   0.0},
		{   3,   0,   2,  -2,   2,     8.7,       1,    0.0,     0,   0.0},
		{  -2,   0,   2,   2,   2,    14.6,       1,    0.0,    -1,   0.0},
		{  -1,   0,   0,   0,   2,   -27.3,       1,    0.0,    -1,   0.0},
		{   0,   0,  -2,   2,   1,  -169.0,       1,    0.0,     0,   0.0},
		{   0,   1,   2,   0,   1,    13.1,       1,    0.0,     0,   0.0},
		{  -1,   0,   4,   0,   2,     9.1,       1,    0.0,     0,   0.0},
		{   2,   1,   0,  -2,   0,   131.7,       1,    0.0,     0,   0.0},
		{   2,   0,   0,   2,   0,     7.1,       1,    0.0,     0,   0.0},
		{   2,   0,   2,  -2,   1,    12.8,       1,    0.0,    -1,   0.0},
		{   2,   0,  -2,   0,   1,  -943.2,       1,    0.0,     0,   0.0},
		{   1,  -1,   0,  -2,   0,   -29.3,       1,    0.0,     0,   0.0},
		{  -1,   0,   0,   1,   1,  -388.3,       1,    0.0,     0,   0.0},
		{  -1,  -1,   0,   2,   1,    35.0,       1,    0.0,     0,   0.0},
		{   0,   1,   0,   1,   0,    27.3,       1,    0.0,     0,   0.0}
	};
	double ang;
	int i,j;

	*dpsi=*deps=0.0;

	for (i=0;i<106;i++) {
		ang=0.0;
		for (j=0;j<5;j++) ang+=nut[i][j]*f[j];
		*dpsi+=(nut[i][6]+nut[i][7]*t)*sin(ang);
		*deps+=(nut[i][8]+nut[i][9]*t)*cos(ang);
	}
	*dpsi*=1E-4*AS2R; /* 0.1 mas -> rad */
	*deps*=1E-4*AS2R;
}

/* eci to ecef transformation matrix -------------------------------------------
* compute eci to ecef transformation matrix
* args   : gtime_t tutc     I   time in utc
*          double *erpv     I   erp values {xp,yp,ut1_utc,lod} (rad,rad,s,s/d)
*          double *U        O   eci to ecef transformation matrix (3 x 3)
*          double *gmst     IO  greenwich mean sidereal time (rad)
*                               (NULL: no output)
* return : none
* note   : see ref [3] chap 5
*          not thread-safe
*-----------------------------------------------------------------------------*/
void QCmpGPST::eci2ecef(gtime_t tutc, const double *erpv, double *U, double *gmst)
{
	const double ep2000[]={2000,1,1,12,0,0};
	static gtime_t tutc_;
	static double U_[9],gmst_;
	gtime_t tgps;
	double eps,ze,th,z,t,t2,t3,dpsi,deps,gast,f[5];
	double R1[9],R2[9],R3[9],R[9],W[9],N[9],P[9],NP[9];
	int i;

	//trace(3,"eci2ecef: tutc=%s\n",time_str(tutc,3));

	if (fabs(timediff(tutc,tutc_))<0.01) { /* read cache */
		for (i=0;i<9;i++) U[i]=U_[i];
		if (gmst) *gmst=gmst_; 
		return;
	}
	tutc_=tutc;

	/* terrestrial time */
	tgps=utc2gpst(tutc_);
	t=(timediff(tgps,epoch2time(ep2000))+19.0+32.184)/86400.0/36525.0;
	t2=t*t; t3=t2*t;

	/* astronomical arguments */
	ast_args(t,f);

	/* iau 1976 precession */
	ze=(2306.2181*t+0.30188*t2+0.017998*t3)*AS2R;
	th=(2004.3109*t-0.42665*t2-0.041833*t3)*AS2R;
	z =(2306.2181*t+1.09468*t2+0.018203*t3)*AS2R;
	eps=(84381.448-46.8150*t-0.00059*t2+0.001813*t3)*AS2R;
    myRz(-z,R1); myRy(th,R2); myRz(-ze,R3);
	matmul("NN",3,3,3,1.0,R1,R2,0.0,R);
	matmul("NN",3,3,3,1.0,R, R3,0.0,P); /* P=Rz(-z)*Ry(th)*Rz(-ze) */

	/* iau 1980 nutation */
	nut_iau1980(t,f,&dpsi,&deps);
    myRx(-eps-deps,R1); myRz(-dpsi,R2); myRx(eps,R3);
	matmul("NN",3,3,3,1.0,R1,R2,0.0,R);
	matmul("NN",3,3,3,1.0,R ,R3,0.0,N); /* N=Rx(-eps)*Rz(-dspi)*Rx(eps) */

	/* greenwich aparent sidereal time (rad) */
	gmst_=utc2gmst(tutc_,erpv[2]);
	gast=gmst_+dpsi*cos(eps);
	gast+=(0.00264*sin(f[4])+0.000063*sin(2.0*f[4]))*AS2R;

	/* eci to ecef transformation matrix */
    myRy(-erpv[0],R1); myRx(-erpv[1],R2); myRz(gast,R3);
	matmul("NN",3,3,3,1.0,R1,R2,0.0,W );
	matmul("NN",3,3,3,1.0,W ,R3,0.0,R ); /* W=Ry(-xp)*Rx(-yp) */
	matmul("NN",3,3,3,1.0,N ,P ,0.0,NP);
	matmul("NN",3,3,3,1.0,R ,NP,0.0,U_); /* U=W*Rz(gast)*N*P */

	for (i=0;i<9;i++) U[i]=U_[i];
	if (gmst) *gmst=gmst_; 

	//trace(5,"gmst=%.12f gast=%.12f\n",gmst_,gast);
	//trace(5,"P=\n"); tracemat(5,P,3,3,15,12);
	//trace(5,"N=\n"); tracemat(5,N,3,3,15,12);
	//trace(5,"W=\n"); tracemat(5,W,3,3,15,12);
	//trace(5,"U=\n"); tracemat(5,U,3,3,15,12);
}

//Calculate solar coordinates  Reference RTKlab
/* sun and moon position -------------------------------------------------------
* get sun and moon position in ecef
* args   : gtime_t tut      I   time in ut1
*          double *erpv     I   erp value {xp,yp,ut1_utc,lod} (rad,rad,s,s/d)
*          double *rsun     IO  sun position in ecef  (m) (NULL: not output)
*          double *rmoon    IO  moon position in ecef (m) (NULL: not output)
*          double *gmst     O   gmst (rad)
* return : none
*-----------------------------------------------------------------------------*/

void QCmpGPST::sunmoonpos(gtime_t tutc, const double *erpv, double *rsun,
	double *rmoon, double *gmst)
{
	gtime_t tut;
	double rs[3]={0},rm[3]={0},U[9]={0},gmst_ = 0;

	//trace(3,"sunmoonpos: tutc=%s\n",time_str(tutc,3));

	tut=timeadd(tutc,erpv[2]); /* utc -> ut1 */

	/* sun and moon position in eci */
	sunmoonpos_eci(tut,rsun?rs:NULL,rmoon?rm:NULL);

	/* eci to ecef transformation matrix */
    eci2ecef(tutc,erpv,U,&gmst_);

	/* sun and moon postion in ecef */
    if (rsun ) matmul("NN",3,1,3,1.0,U,rs,0.0,rsun );
	if (rmoon) matmul("NN",3,1,3,1.0,U,rm,0.0,rmoon);
	if (gmst ) *gmst=gmst_;
}



bool QCmpGPST::getSunMoonPos(int Year,int Month,int Day,int HoursInt,int Minutes,double Seconds,double *sunpos,double *moonpos,double *gmst)
{	
	////Calculate GPS week and week seconds
	int tWeekN = 0;
    double tSecond = 0.0;
	tSecond = YMD2GPSTime(Year,Month,Day,HoursInt,Minutes,Seconds,&tWeekN);
	gtime_t obsGPST,obsUTC;
	obsGPST = gpst2time(tWeekN,tSecond);
 	//Debug
//  	double ep[6] = {2010,4,10,0,0,30};
//  	obsUTC = epoch2time(ep);
//	double JD = computeJD(2010,4,10,0,0,30);
	//The GPST at the origin of the time of 1970.1.1 is converted to the origin of the UTC time of 1970.1.1.
	obsUTC = gpst2utc(obsGPST);
	//Convert UTC to UT1 using erp file
	double erpv[5] = {0};
    if (isuseErp)
	{
		geterp(&m_erpData,obsGPST,erpv);
	}
	//Calculate UT1
	//gtime_t obsUT1 = timeadd(obsUTC,erpv[2]);
	//Calculate solar moon coordinates
	sunmoonpos(obsUTC,erpv,sunpos,moonpos,gmst);
	return true;
}
