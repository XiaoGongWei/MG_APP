#include "QTropDelay.h"


QTropDelay::QTropDelay()
{
}
// setTropFileNames as QTropDelay
void QTropDelay::setTropFileNames(QString GrdFileName,QString ProjectionFun, QString tropMode)
{
    initVar();
    if(!GrdFileName.isEmpty())  m_GrdFileName = GrdFileName;
    if(!tropMode.contains("UNB3", Qt::CaseInsensitive)) isGPT2 = true;
    // jugle ProjectionFun
    m_ProjectionFun = ProjectionFun;
    if (m_ProjectionFun.contains("N",Qt::CaseInsensitive))
        m_ProjectFunFlag = 1;
    else if (m_ProjectionFun.contains("V",Qt::CaseInsensitive))
        m_ProjectFunFlag = 2;
    else
        m_ProjectFunFlag = 3;
}

QTropDelay::~QTropDelay(void)
{
	m_allGrdFile.clear();
}

void QTropDelay::initVar()
{
	m_PI = 3.1415926535897932385;//Pi, radians
	m_GrdFileName = "gpt2_5.grd";//Path + file name
	tempLine = "";
	isReadAllData = false;
	m_ProjectionFun = "GMF1";
	m_ProjectFunFlag = 1;
	isGPT2 = false;
	double tlat[5] ={15,30,45,60,75};
	double tAvgad[5] = {1.2769934e-3,1.2683230e-3,1.2465397e-3,1.2196049e-3,1.2045996e-3},
		tAvgbd[5] = {2.9153695e-3,2.9152299e-3,2.9288445e-3,2.9022565e-3,2.9024912e-3},
		tAvgcd[5] = {62.620505e-3,62.837393e-3,63.721774e-3,63.824265e-3,64.258455e-3};
	double tAmpad[5] = {0,1.2709626e-5,2.6523662e-5,3.4000452e-5,4.1202191e-5},
		tAmpbd[5] ={0,2.1414979e-5,3.0160779e-5,7.2562722e-5,11.723375e-5},
		tAmpcd[5] = {0,9.0128400e-5,4.3497037e-5,84.795348e-5,170.37206e-5};
	double tAvgaw[5] = {5.8021879e-4,5.6794847e-4,5.8118019e-4,5.9727542e-4,6.1641693e-4},
		tAvgbw[5] = {1.4275268e-3,1.5138625e-3,1.4572572e-3,1.5007428e-3,1.7599082e-3},
		tAvgcw[5] = {4.3472961e-2,4.6729510e-2,4.3908931e-2,4.4626982e-2,5.4736039e-2};
	for (int i = 0;i < 5;i++)
	{
		lat[i] = tlat[i]*m_PI/180;
		Avgad[i] = tAvgad[i];Avgbd[i] = tAvgbd[i];Avgcd[i] = tAvgcd[i];
		Ampad[i] = tAmpad[i];Ampbd[i] = tAmpbd[i];Ampcd[i] = tAmpcd[i];
		Avgaw[i] = tAvgaw[i];Avgbw[i] = tAvgbw[i];Avgcw[i] = tAvgcw[i];
	}
}

bool QTropDelay::openGrdFile(QString GrdFileName)
{
	if (!GrdFileName.isEmpty())
	{
        m_ReadFileClass.setFileName(GrdFileName);
		if (!m_ReadFileClass.open(QFile::ReadOnly))//If the file fails to open......
		{
			isReadAllData = true;//Open only once
			isGPT2 = false;//Unable to use GPT2 model
			return false;
		}
	}
    else
    {
        isReadAllData = true;//Open only once
        isGPT2 = false;//Unable to use GPT2 model
    }
	return true;
}

void QTropDelay::readGrdFile(QString grdFileName)
{
	if (isReadAllData) return;
	//Skip the investment bank title
	tempLine = m_ReadFileClass.readLine();
	while (!m_ReadFileClass.atEnd())
	{
		GrdFileVar GrdvarT;
		tempLine = m_ReadFileClass.readLine();
		GrdvarT.lat = tempLine.mid(0,6).toDouble();
		GrdvarT.lon = tempLine.mid(7,6).toDouble();
		GrdvarT.pgrid[0] = tempLine.mid(14,6).toDouble();
		GrdvarT.pgrid[1] = tempLine.mid(21,5).toDouble();
		GrdvarT.pgrid[2] = tempLine.mid(27,4).toDouble();
		GrdvarT.pgrid[3] = tempLine.mid(32,4).toDouble();
		GrdvarT.pgrid[4] = tempLine.mid(37,4).toDouble();

		GrdvarT.Tgrid[0] = tempLine.mid(42,5).toDouble();
		GrdvarT.Tgrid[1] = tempLine.mid(48,5).toDouble();
		GrdvarT.Tgrid[2] = tempLine.mid(54,4).toDouble();
		GrdvarT.Tgrid[3] = tempLine.mid(59,4).toDouble();
		GrdvarT.Tgrid[4] = tempLine.mid(64,4).toDouble();

		GrdvarT.Qgrid[0] = tempLine.mid(69,5).toDouble() / 1000;
		GrdvarT.Qgrid[1] = tempLine.mid(75,5).toDouble() / 1000;
		GrdvarT.Qgrid[2] = tempLine.mid(81,5).toDouble() / 1000;
		GrdvarT.Qgrid[3] = tempLine.mid(87,5).toDouble() / 1000;
		GrdvarT.Qgrid[4] = tempLine.mid(93,5).toDouble() / 1000;

		GrdvarT.dTgrid[0] = tempLine.mid(99,5).toDouble() / 1000;
		GrdvarT.dTgrid[1] = tempLine.mid(105,5).toDouble() / 1000;
		GrdvarT.dTgrid[2] = tempLine.mid(111,4).toDouble() / 1000;
		GrdvarT.dTgrid[3] = tempLine.mid(116,4).toDouble() / 1000;
		GrdvarT.dTgrid[4] = tempLine.mid(121,4).toDouble() / 1000;

		GrdvarT.u = tempLine.mid(126,7).toDouble();
		GrdvarT.Hs = tempLine.mid(134,8).toDouble();

		GrdvarT.ahgrid[0] = tempLine.mid(143,6).toDouble() / 1000;
		GrdvarT.ahgrid[1] = tempLine.mid(150,7).toDouble() / 1000;
		GrdvarT.ahgrid[2] = tempLine.mid(158,7).toDouble() / 1000;
		GrdvarT.ahgrid[3] = tempLine.mid(166,7).toDouble() / 1000;
		GrdvarT.ahgrid[4] = tempLine.mid(174,7).toDouble() / 1000;

		GrdvarT.awgrid[0] = tempLine.mid(182,7).toDouble() / 1000;
		GrdvarT.awgrid[1] = tempLine.mid(190,7).toDouble() / 1000;
		GrdvarT.awgrid[2] = tempLine.mid(198,7).toDouble() / 1000;
		GrdvarT.awgrid[3] = tempLine.mid(206,7).toDouble() / 1000;
		GrdvarT.awgrid[4] = tempLine.mid(214,7).toDouble() / 1000;
		m_allGrdFile.append(GrdvarT);
	}
	m_ReadFileClass.close();//Close file
	isReadAllData = true;
}

//Read all grd files
void QTropDelay::getAllData()
{
    if(isGPT2&&(!isReadAllData))
    {
        openGrdFile(m_GrdFileName);
        readGrdFile(m_GrdFileName);
    }
}

//Calculate the dry (md) and wet (mw) projection functions of the Neil projection
//Calculating the dry (md) and wet (mw) projection functions of the Neil projection is suitable for mid-latitude, and the high-latitude causes the deviation of the elevation direction.
void QTropDelay::getNeilParm(double E,double H,double Lat,int TDay,double &md,double &mw)
{
	double aht = 2.53e-5,bht = 5.49e-3,cht = 1.14e-3;
	double ad = 0,bd = 0,cd = 0;//Dry component abc
	double aw = 0,bw = 0,cw = 0;//Wet component abc
	double T0 = 28;
	//Find location
	int flag = 0;
	for(int i = 0;i < 5;i++)
		if (Lat > lat[i])
			flag++;
		else
			break;
	//Calculate the dry and wet component abc
	if (Lat > 15*m_PI/180&&Lat < 75*m_PI/180)
	{
		ad = Avgad[flag - 1] + (Avgad[flag] - Avgad[flag - 1])*(Lat - lat[flag - 1])/(lat[flag] - lat[flag - 1])
			+ Ampad[flag - 1] + (Ampad[flag] - Ampad[flag - 1])*(Lat - lat[flag - 1])*qCos(2*m_PI*(TDay - T0)/365.25)/(lat[flag] - lat[flag - 1]);
		bd = Avgbd[flag - 1] + (Avgbd[flag] - Avgbd[flag - 1])*(Lat - lat[flag - 1])/(lat[flag] - lat[flag - 1])
			+ Ampbd[flag - 1] + (Ampbd[flag] - Ampbd[flag - 1])*(Lat - lat[flag - 1])*qCos(2*m_PI*(TDay - T0)/365.25)/(lat[flag] - lat[flag - 1]);
		cd = Avgcd[flag - 1] + (Avgcd[flag] - Avgcd[flag - 1])*(Lat - lat[flag - 1])/(lat[flag] - lat[flag - 1])
			+ Ampcd[flag - 1] + (Ampcd[flag] - Ampcd[flag - 1])*(Lat - lat[flag - 1])*qCos(2*m_PI*(TDay - T0)/365.25)/(lat[flag] - lat[flag - 1]);
		aw = Avgaw[flag - 1] + (Avgaw[flag] - Avgaw[flag - 1])*(Lat - lat[flag - 1])/(lat[flag] - lat[flag - 1]);
		bw = Avgbw[flag - 1] + (Avgbw[flag] - Avgbw[flag - 1])*(Lat - lat[flag - 1])/(lat[flag] - lat[flag - 1]);
		cw = Avgcw[flag - 1] + (Avgcw[flag] - Avgcw[flag - 1])*(Lat - lat[flag - 1])/(lat[flag] - lat[flag - 1]);
	}
	else if (Lat <= 15*m_PI/180)
	{
		ad = Avgad[0] + Avgad[0]*qCos(2*m_PI*(TDay - T0)/365.25);
		bd = Avgbd[0] + Avgbd[0]*qCos(2*m_PI*(TDay - T0)/365.25);
		cd = Avgcd[0] + Avgcd[0]*qCos(2*m_PI*(TDay - T0)/365.25);
		aw = Avgaw[0] + Avgaw[0]*(Lat - lat[flag - 1])/(lat[flag] - lat[flag - 1]);
		bw = Avgbw[0] + Avgbw[0]*(Lat - lat[flag - 1])/(lat[flag] - lat[flag - 1]);
		cw = Avgcw[0] + Avgcw[0]*(Lat - lat[flag - 1])/(lat[flag] - lat[flag - 1]);
	}
	else if (Lat >= 75*m_PI/180)
	{
		ad = Avgad[4] + Avgad[4]*qCos(2*m_PI*(TDay - T0)/365.25);
		bd = Avgbd[4] + Avgbd[4]*qCos(2*m_PI*(TDay - T0)/365.25);
		cd = Avgcd[4] + Avgcd[4]*qCos(2*m_PI*(TDay - T0)/365.25);
		aw = Avgaw[4] + Avgaw[4]*(Lat - lat[flag - 1])/(lat[flag] - lat[flag - 1]);
		bw = Avgbw[4] + Avgbw[4]*(Lat - lat[flag - 1])/(lat[flag] - lat[flag - 1]);
		cw = Avgcw[4] + Avgcw[4]*(Lat - lat[flag - 1])/(lat[flag] - lat[flag - 1]);
	}
	//（Li Zhenghang second edition, GPS measurement and data processing errors）
	/*md = (1/(1+ad/(1+bd/(1+cd))))/(1/(qSin(E)+ad/(qSin(E)+bd/(qSin(E)+cd))))
	+(1/qSin(E) - (1/(1+aht/(1+bht/(1+cht)))/(1/(qSin(E)+(aht/(qSin(E)+bht/(qSin(E)+cht)))))))*H/1000;
	mw = (1/(1+aw/(1+bw/(1+cw))))/(1/(qSin(E)+aw/(qSin(E)+bw/(qSin(E)+cw))));*/
	//Calculate the projection function
	md = (1+ad/(1+bd/(1+cd)))/(qSin(E)+ad/(qSin(E)+bd/(qSin(E)+cd)))
		+(1/qSin(E) - (1+aht/(1+bht/(1+cht)))/(qSin(E)+(aht/(qSin(E)+bht/(qSin(E)+cht)))))*H/1000;
	mw = (1+aw/(1+bw/(1+cw)))/(qSin(E)+aw/(qSin(E)+bw/(qSin(E)+cw)));
}

//Calculate the dry (md) and wet (mw) projection functions of the VMF1 projection
void QTropDelay::getVMF1Parm(double ah,double aw,double E,double Lat,double H,int TDay,double &md,double &mw)
{
	double bd = 0.0029,cd = 0;//Dry projection coefficient
	double bw = 0.00146,cw = 0.04391;
	double aht = 2.53e-5,bht = 5.49e-3,cht = 1.14e-3;
	if(Lat < 0)
	{
		double c0 = 0.062,c11 = 0.001,c10 = 0.006,ph = m_PI;//MM_PI
		cd = c0 + ( (qCos(2*m_PI*(TDay - 28)/365 + ph) + 1)*c11/2 + c10 )*(1 - qCos(-Lat));
	}
	else
	{
		double c0 = 0.062,c11 = 0.0,c10 = 0.006,ph = 0;
		cd = c0 + ( (qCos(2*m_PI*(TDay - 28)/365 + ph) + 1)*c11/2 + c10 )*(1 - qCos(Lat));
	}
	//Calculate the projection function
	md = (1+ah/(1+bd/(1+cd)))/(qSin(E)+ah/(qSin(E)+bd/(qSin(E)+cd)))
		+(1/qSin(E) - (1+aht/(1+bht/(1+cht)))/(qSin(E)+(aht/(qSin(E)+bht/(qSin(E)+cht)))))*H/1000;
	mw = (1+aw/(1+bw/(1+cw)))/(qSin(E)+aw/(qSin(E)+bw/(qSin(E)+cw)));
}

//Calculate the dry (md) and wet (mw) projection functions of the VMF1 projection
void QTropDelay::getGMFParm(double MJD,double *pBLH,double E,double &md,double &mw)
{
	trop_map_gmf(MJD,pBLH[0],pBLH[1],pBLH[2],m_PI/2 - E,&md,&mw);
}

//Sass empirical model for calculating troposphere (dry delay)
double QTropDelay::getSassDelay(double &ZHD,double &ZWD,double B, double H,double E)
{
	//No actual measurement of meteorological data
	//Paper calculation parameters
	double T0 = 288.15;
	double P0 = 1013.25;
	double e0 = 11.691;
	double Ps = P0*qPow((1-0.0068*H/T0),5);
	double Ts = T0 - 0.0068 * H;
	double Es =0;//Water pressure
	if (H < 11000)
		Es = e0 * qPow((1 - 0.0068 * H / T0), 4);
	else
		Es = 0.0;
	double f = 1 - 0.00266 * qCos(2 * B) - 0.00028 * H/1000;
	ZHD = 0.002277 * Ps / f;
	ZWD = 0.002277*(1255/Ts + 0.05)*Es/f;
	return 0;
}

//Hopfield empirical model to calculate troposphere (total delay)
double QTropDelay::getHopfieldDelay(double &SD,double &SW, double H,double E)
{
	//No actual measurement of meteorological data
	//Paper calculation parameters
	double T0 = 288.15;
	double P0 = 1013.25;
	double e0 = 11.691;
	double Ps = P0*qPow((1-0.0068*H/T0),5);
	double Ts = T0 - 0.0068 * H;
	double Es =0;//Water pressure
	if (H < 11000)
		Es = e0 * qPow((1 - 0.0068 * H /   T0), 4);
	else
		Es = 0.0;
	double dltS = 0;
	double Sd = 0;//Dryness
	double Sw = 0;//humidity
	double hd = 40136 + 148.72*(Ts - 273.16);
	double hw = 11000;
	double Kd = (155.2e-7) * Ps * (hd - H) / Ts;
	double Kw = (155.2e-7) * 4810 * Es *(hw - H) / (Ts*Ts);
	Sd = Kd/qSin(qSqrt(E*E + 6.25)*m_PI/180);
	Sw = Kw/qSin(qSqrt(E*E + 2.25)*m_PI/180);
	SD = Sd;
	Sw = Sw;
	return (Sd + Sw);
}

//Humidity adopts GPT2+Hopfield (zenith dry and wet delay)
GPT2Result QTropDelay::HopfieldDelay(double &ZHD,double &ZWD,double dmjd,double dlat,double dlon,double hell,double it /* = 0 */)
{//E:Satellite elevation angle
	GPT2Result m_PTe = getGPT2Model(dmjd,dlat,dlon,hell);
	m_PTe.T +=273.15;//Convert to open Celsius
	double hd = 40136 + 148.72*(m_PTe.T - 273.16);
	ZHD = (155.2e-7) * m_PTe.p * (hd - hell) / m_PTe.T;
	ZWD = (155.2e-7) * 4810 * m_PTe.e *(11000 - hell) / (m_PTe.T*m_PTe.T);
	m_PTe.T -= 273.15;//Change to Chinese Celsius
	return(m_PTe);
}

//GPT2 Estimate + Simplified Saastamoinen Model
GPT2Result QTropDelay::SassstaMDelay(double &ZHD,double &ZWD,double dmjd,double dlat,double dlon,double hell,double it /* = 0 */)
{
	GPT2Result m_PTe = getGPT2Model(dmjd,dlat,dlon,hell);
	m_PTe.T +=273.15;//Convert to open Celsius
    double f = 1 - 0.00266 * qCos(2 * dlat) - 0.00028 * hell/1000;
	ZHD = 0.002277*m_PTe.p/f;
	ZWD = 0.002277*(1255/m_PTe.T + 0.05)*m_PTe.e/f;
	m_PTe.T -= 273.15;//Change to Chinese Celsius
	return m_PTe;
}

//Use GPT2+Hopfield + Zenill+VMF1 projection function
double QTropDelay::getGPT2HopfieldDelay(double MJD, int TDay, double E, double *pBLH, double *mf, double *ZPD, double *tZHD)
{
	if (!isReadAllData)
	{
		isGPT2 = true;
		getAllData();
	}
	double ZHD = 0,ZWD = 0;//Zen top dry and wet content
	double mD = 0,mW = 0;//Projection function
	GPT2Result tempGPT2 = HopfieldDelay(ZHD,ZWD,MJD,pBLH[0],pBLH[1],pBLH[2]);
	if (m_ProjectFunFlag == 1)
		getNeilParm(E,pBLH[2],pBLH[0],TDay,mD,mW);
	else if (m_ProjectFunFlag == 2)
		getVMF1Parm(tempGPT2.ah,tempGPT2.aw,E,pBLH[0],pBLH[2],TDay,mD,mW);
	else
		getGMFParm(MJD,pBLH,E,mD,mW);	
	if (mf) *mf = mW;//Return only wet delay projection here
    if(ZPD) *ZPD = ZHD*mD + ZWD*mW;
    if(tZHD) *tZHD = ZHD;// dbug by xiaogongwei 2018.12.24
	return (ZHD*mD);//Return only dry delay!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//return (ZHD*mD+ZWD*mW);//Total delay in adding projection function
}

//Adopt GPT2+Sassta (zenith dry and wet delay）+Neill+VMF1 Projection function
double QTropDelay::getGPT2SasstaMDelay(double MJD, int TDay, double E, double *pBLH, double *mf, double *ZPD, double *tZHD)
{
	if (!isReadAllData)
	{
		isGPT2 = true;
		getAllData();
	}
	double ZHD = 0,ZWD = 0;//Zen top dry and wet content
	double mD = 0,mW = 0;//Projection function
	GPT2Result tempGPT2 = SassstaMDelay(ZHD,ZWD,MJD,pBLH[0],pBLH[1],pBLH[2]);
	if (m_ProjectFunFlag == 1)
		getNeilParm(E,pBLH[2],pBLH[0],TDay,mD,mW);
	else if (m_ProjectFunFlag == 2)
		getVMF1Parm(tempGPT2.ah,tempGPT2.aw,E,pBLH[0],pBLH[2],TDay,mD,mW);
	else
		getGMFParm(MJD,pBLH,E,mD,mW);

	if (mf) *mf = mW;//Return only wet delay projection here
    if(ZPD) *ZPD = ZHD*mD + ZWD*mW;
    if(tZHD) *tZHD = ZHD;// dbug by xiaogongwei 2018.12.24
	return (ZHD*mD);//Return only dry delay!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//return (ZHD*mD+ZWD*mW);//Total delay in adding projection function
}

//GPTdata stores air pressure (hPa), temperature (C), and ellipsoid height difference（Geoid undulation）（m）
void QTropDelay::getGPTModel(double dmjd,double dlat,double dlon,double hell,double *GPTdata)
{
	double pres = 0,temp = 0,umdo = 0;
	trop_gpt(dmjd,dlat,dlon,hell,&pres,&temp,&umdo);
	GPTdata[0] = pres;
	GPTdata[1] = temp;
	GPTdata[2] = umdo;
}

//GPT2 calculates temperature, air pressure, humidity deceleration rate, specific humidity, ah, aw of VMF1
/*
% dmjd:  modified Julian date (scalar, only one epoch per call is possible)
% dlat:  ellipsoidal latitude in radians [-pi/2:+pi/2] (vector)
% dlon:  longitude in radians [-pi:pi] or [0:2pi] (vector)
% hell:  ellipsoidal height in m (vector)
% it:    case 1: no time variation but static quantities
%        case 0: with time variation (annual and semiannual terms)
*/

GPT2Result QTropDelay::getGPT2Model(double dmjd,double dlat,double dlon,double hell,double it)
{
	GPT2Result m_resualt;
    if(!isGPT2) return m_resualt;
	//% change the reference epoch to January 1 2000
	double dmjd1 = dmjd-51544.5;
	//% mean gravity in m/s**2
	double gm = 9.80665;
	//% molar mass of dry air in kg/mol
	double dMtr = 28.965e-3;
	//% universal gas constant in J/K/mol
	double Rg = 8.3143;
	double cosfy = 0,coshy = 0,sinfy = 0,sinhy = 0;
	if (it == 1) //% then  constant parameters
	{
		cosfy = 0;
		coshy = 0;
		sinfy = 0;
		sinhy = 0;
	}	
	else 
	{
		cosfy = cos(dmjd1/365.25*2*m_PI);
		coshy = cos(dmjd1/365.25*4*m_PI);
		sinfy = sin(dmjd1/365.25*2*m_PI);
		sinhy = sin(dmjd1/365.25*4*m_PI);
	}
	//Read grd file
	//getAllData();
	double plon = 0,ppod = 0;
	//% only positive longitude in degrees
	if (dlon < 0)
		plon = (dlon + 2*m_PI)*180/m_PI;
	else
		plon = dlon*180/m_PI;
	// % transform to polar distance in degrees
	ppod = (-dlat + m_PI/2)*180/m_PI;
	//% find the index (line in the grid file) of the nearest point
	double ipod = qFloor((ppod+5)/5); 
	double ilon = qFloor((plon+5)/5);
	//% normalized (to one) differences, can be positive or negative
	double diffpod = (ppod - (ipod*5 - 2.5))/5;
	double difflon = (plon - (ilon*5 - 2.5))/5;
	// % added by HCY
	if (ipod == 37)
		ipod = 36;
	//% get the number of the corresponding line
	double indx1 = (ipod - 1)*72 + ilon - 1;
	//% near the poles: nearest neighbour interpolation, otherwise: bilinear
	double bilinear = 0;
	if (ppod > 2.5 && ppod < 177.5 )
		bilinear = 1;               
	//% case of nearest neighbourhood
	if (bilinear == 0)
	{
		double ix = indx1;
		GrdFileVar tempGrdVar = m_allGrdFile.at(ix);
		// % transforming ellipsoidial height to orthometric height
		m_resualt.undu = tempGrdVar.u;
		double hgt = hell - tempGrdVar.u;
		// % pressure, temperature at the heigtht of the grid
		double T0 = tempGrdVar.Tgrid[0] + 
			tempGrdVar.Tgrid[1]*cosfy + tempGrdVar.Tgrid[2]*sinfy +
			tempGrdVar.Tgrid[3]*coshy + tempGrdVar.Tgrid[4]*sinhy;
		double p0 = tempGrdVar.pgrid[0] + 
			tempGrdVar.pgrid[1]*cosfy + tempGrdVar.pgrid[2]*sinfy+
			tempGrdVar.pgrid[3]*coshy + tempGrdVar.pgrid[4]*sinhy;
		//% specific humidity
		double Q = tempGrdVar.Qgrid[0] + 
			tempGrdVar.Qgrid[1]*cosfy + tempGrdVar.Qgrid[2]*sinfy+ 
			tempGrdVar.Qgrid[3]*coshy + tempGrdVar.Qgrid[4]*sinhy;

		//% lapse rate of the temperature
		m_resualt.dT = tempGrdVar.dTgrid[0] + 
			tempGrdVar.dTgrid[1]*cosfy + tempGrdVar.dTgrid[2]*sinfy+ 
			tempGrdVar.dTgrid[3]*coshy + tempGrdVar.dTgrid[4]*sinhy; 
		//% station height - grid height
		double redh = hgt - tempGrdVar.Hs;
		//  % temperature at station height in Celsius
		m_resualt.T = T0 + m_resualt.dT*redh - 273.15;
		// % temperature lapse rate in degrees / km
		m_resualt.dT = m_resualt.dT*1000;
		// % virtual temperature in Kelvin
		double Tv = T0*(1+0.6077*Q);

		double c = gm*dMtr/(Rg*Tv);

		//% pressure in hPa
		m_resualt.p = (p0*qExp(-c*redh))/100;

		//% water vapour pressure in hPa
		m_resualt.e = (Q*m_resualt.p)/(0.622+0.378*Q);

		//% hydrostatic coefficient ah 
		m_resualt.ah = tempGrdVar.ahgrid[0] + 
			tempGrdVar.ahgrid[1]*cosfy + tempGrdVar.ahgrid[2]*sinfy+ 
			tempGrdVar.ahgrid[3]*coshy + tempGrdVar.ahgrid[4]*sinhy;

		//% wet coefficient aw
		m_resualt.aw = tempGrdVar.awgrid[0] + 
			tempGrdVar.awgrid[1]*cosfy + tempGrdVar.awgrid[2]*sinfy + 
			tempGrdVar.awgrid[3]*coshy + tempGrdVar.awgrid[4]*sinhy;

	}
	else if(bilinear == 1)
	{// % bilinear interpolation
		double signpod = 0;
		double signlon = 0;
		if (diffpod > 0)
			signpod = 1;
		else if(diffpod < 0)
			signpod = -1;
		if (difflon > 0)
			signlon = 1;
		else if(difflon < 0)
			signlon = -1;

		double ipod1 = ipod + signpod;// sign(diffpod);
		double ilon1 = ilon + signlon;// sign(difflon);
		if (ilon1 == 73)
			ilon1 = 1;
		else if(ilon1 == 0) 
			ilon1 = 72;
		// % get the number of the line
		double indx2 = (ipod1 - 1)*72 + ilon -1;//  % along same longitude
		double indx3 = (ipod  - 1)*72 + ilon1 - 1;//% along same polar distance
		double indx4 = (ipod1 - 1)*72 + ilon1 - 1;// % diagonal
		//Defining variables
		double undul[4] = {0},Ql[4] = {0},dTl[4] = {0},Tl[4] = {0},
			pl[4] = {0},ahl[4] = {0},awl[4] = {0};
		double Indexflag[4] = {indx1,indx2,indx3,indx4};
		for (int l = 0;l < 4;l++)
		{
			GrdFileVar tempGrdVar = m_allGrdFile.at(Indexflag[l]);
			// % transforming ellipsoidial height to orthometric height:
			//% Hortho = -N + Hell
			undul[l] = tempGrdVar.u;
			double hgt = hell -undul[l];
			//% pressure, temperature at the heigtht of the grid
			double T0 = tempGrdVar.Tgrid[0] + 
				tempGrdVar.Tgrid[1]*cosfy + tempGrdVar.Tgrid[2]*sinfy + 
				tempGrdVar.Tgrid[3]*coshy + tempGrdVar.Tgrid[4]*sinhy;
			double p0 = tempGrdVar.pgrid[0] + 
				tempGrdVar.pgrid[1]*cosfy + tempGrdVar.pgrid[2]*sinfy + 
				tempGrdVar.pgrid[3]*coshy + tempGrdVar.pgrid[4]*sinhy;
			//% humidity 
			Ql[l] = tempGrdVar.Qgrid[0] + 
				tempGrdVar.Qgrid[1]*cosfy + tempGrdVar.Qgrid[2]*sinfy + 
				tempGrdVar.Qgrid[3]*coshy + tempGrdVar.Qgrid[4]*sinhy;
			// % reduction = stationheight - gridheight
			double Hs1 = tempGrdVar.Hs;
			double redh = hgt - Hs1;
			//% lapse rate of the temperature in degree / m
			dTl[l] = tempGrdVar.dTgrid[0] + 
				tempGrdVar.dTgrid[1]*cosfy + tempGrdVar.dTgrid[2]*sinfy + 
				tempGrdVar.dTgrid[3]*coshy + tempGrdVar.dTgrid[4]*sinhy; 
			// % temperature reduction to station height
			Tl[l] = T0 + dTl[l]*redh - 273.15;
			// % virtual temperature
			double Tv = T0*(1+0.6077*Ql[l]);  
			double c = gm*dMtr/(Rg*Tv);
			//% pressure in hPa
			pl[l] = (p0*qExp(-c*redh))/100;
			// % hydrostatic coefficient ah
			ahl[l] = tempGrdVar.ahgrid[0] + 
				tempGrdVar.ahgrid[1]*cosfy + tempGrdVar.ahgrid[2]*sinfy + 
				tempGrdVar.ahgrid[3]*coshy + tempGrdVar.ahgrid[4]*sinhy;
			//% wet coefficient aw
			awl[l] = tempGrdVar.awgrid[0] + 
				tempGrdVar.awgrid[1]*cosfy + tempGrdVar.awgrid[2]*sinfy + 
				tempGrdVar.awgrid[3]*coshy + tempGrdVar.awgrid[4]*sinhy;
		}//for (int l = 0;l < 3;l++)
		double dnpod1 = qAbs(diffpod); //% distance nearer point
		double dnpod2 = 1 - dnpod1; //  % distance to distant point
		double dnlon1 = qAbs(difflon);
		double dnlon2 = 1 - dnlon1;
		// % pressure
		double R1 = dnpod2*pl[0]+dnpod1*pl[1];
		double R2 = dnpod2*pl[2]+dnpod1*pl[3];
		m_resualt.p = dnlon2*R1+dnlon1*R2;
		//  % temperature
		R1 = dnpod2*Tl[0]+dnpod1*Tl[1];
		R2 = dnpod2*Tl[2]+dnpod1*Tl[3];
		m_resualt.T = dnlon2*R1+dnlon1*R2;
		// % temperature in degree per km
		R1 = dnpod2*dTl[0]+dnpod1*dTl[1];
		R2 = dnpod2*dTl[2]+dnpod1*dTl[3];
		m_resualt.dT = (dnlon2*R1+dnlon1*R2)*1000;
		// % humidity
		R1 = dnpod2*Ql[0]+dnpod1*Ql[1];
		R2 = dnpod2*Ql[2]+dnpod1*Ql[3];
		double Q = dnlon2*R1+dnlon1*R2;
		m_resualt.e = (Q*m_resualt.p)/(0.622+0.378*Q);

		//% hydrostatic
		R1 = dnpod2*ahl[0]+dnpod1*ahl[1];
		R2 = dnpod2*ahl[2]+dnpod1*ahl[3];
		m_resualt.ah = dnlon2*R1+dnlon1*R2;

		//% wet
		R1 = dnpod2*awl[0]+dnpod1*awl[1];
		R2 = dnpod2*awl[2]+dnpod1*awl[3];
		m_resualt.aw = dnlon2*R1+dnlon1*R2;

		//% undulation
		R1 = dnpod2*undul[0]+dnpod1*undul[1];
		R2 = dnpod2*undul[2]+dnpod1*undul[3];
		m_resualt.undu = dnlon2*R1+dnlon1*R2;
	}
	return m_resualt;
}

//Refer to someone else's GPT model as follows

/*****************************************************************************
 * Name        : trop_gpt
 * 
 * Description : Calcaulate Global Pressure and Temperature based on 
 *               Spherical Harmonics up to degree and order 9
 * 
 * Parameters  :
 * Name                           |Da|Unit|Description
 * double  dmjd                    I  N/A  Modified julian date
 * double  dlat                    I  rad  Ellipsoidal latitude
 * double  dlon                    I  rad  Ellipsoidal longitude
 * double  dhgt                    I  m    Height
 * double  *pres                   O  hPa  Pressure
 * double  *temp                   O  C    Temperature
 * double  *undu                   O  m    Geoid undulation (from a 9x9 EGM based model)
 * 
 * Reference:
 *    J. B鰄m, R. Heinkelmann, H. Schuh, Short Note: A Global Model of 
 *    Pressure and Temperature for Geodetic Applications, 
 *    Journal of Geodesy, doi:10.1007/s00190-007-0135-3, 2007.
 * 
 * Author: Feng Zhou
 * 
 * Originally written by Feng Zhou on 15/12/2015 @ GFZ
 * 
 * Email: fzhou@geodesy.cn; fzhou@gfz-potsdam.de; zhouforme@gmail.com
 * 
 * Section 1.1, GPS/Galileo technologies, GFZ German Research Centre for Geosciences
 * 
 *****************************************************************************/
void QTropDelay::trop_gpt(double dmjd,double dlat,double dlon,double dhgt,double *pres,double *temp,double *undu) 
{
	double doy,sinlat;
	int i,j,k,n,m,im,ir;
	double dfac[20],p[10][10],ap[55],bp[55];
	double sum1,apm,apa,atm,ata,hort,pres0,temp0;

	// reference day is 28 January
	// this is taken from Niell (1996) to be consistent
	doy = dmjd-44239+1-28;

	// initialized data
	static double a_geoid[55] = {
		-5.6195e-001,-6.0794e-002,-2.0125e-001,-6.4180e-002,-3.6997e-002,
		+1.0098e+001,+1.6436e+001,+1.4065e+001,+1.9881e+000,+6.4414e-001,
		-4.7482e+000,-3.2290e+000,+5.0652e-001,+3.8279e-001,-2.6646e-002,
		+1.7224e+000,-2.7970e-001,+6.8177e-001,-9.6658e-002,-1.5113e-002,
		+2.9206e-003,-3.4621e+000,-3.8198e-001,+3.2306e-002,+6.9915e-003,
		-2.3068e-003,-1.3548e-003,+4.7324e-006,+2.3527e+000,+1.2985e+000,
		+2.1232e-001,+2.2571e-002,-3.7855e-003,+2.9449e-005,-1.6265e-004,
		+1.1711e-007,+1.6732e+000,+1.9858e-001,+2.3975e-002,-9.0013e-004,
		-2.2475e-003,-3.3095e-005,-1.2040e-005,+2.2010e-006,-1.0083e-006,
		+8.6297e-001,+5.8231e-001,+2.0545e-002,-7.8110e-003,-1.4085e-004,
		-8.8459e-006,+5.7256e-006,-1.5068e-006,+4.0095e-007,-2.4185e-008      
	};

	static double b_geoid[55] = {
		+0.0000e+000,+0.0000e+000,-6.5993e-002,+0.0000e+000,+6.5364e-002,
		-5.8320e+000,+0.0000e+000,+1.6961e+000,-1.3557e+000,+1.2694e+000,
		+0.0000e+000,-2.9310e+000,+9.4805e-001,-7.6243e-002,+4.1076e-002,
		+0.0000e+000,-5.1808e-001,-3.4583e-001,-4.3632e-002,+2.2101e-003,
		-1.0663e-002,+0.0000e+000,+1.0927e-001,-2.9463e-001,+1.4371e-003,
		-1.1452e-002,-2.8156e-003,-3.5330e-004,+0.0000e+000,+4.4049e-001,
		+5.5653e-002,-2.0396e-002,-1.7312e-003,+3.5805e-005,+7.2682e-005,
		+2.2535e-006,+0.0000e+000,+1.9502e-002,+2.7919e-002,-8.1812e-003,
		+4.4540e-004,+8.8663e-005,+5.5596e-005,+2.4826e-006,+1.0279e-006,
		+0.0000e+000,+6.0529e-002,-3.5824e-002,-5.1367e-003,+3.0119e-005,
		-2.9911e-005,+1.9844e-005,-1.2349e-006,-7.6756e-009,+5.0100e-008
	};

	static double ap_mean[55] = {
		+1.0108e+003,+8.4886e+000,+1.4799e+000,-1.3897e+001,+3.7516e-003,
		-1.4936e-001,+1.2232e+001,-7.6615e-001,-6.7699e-002,+8.1002e-003,
		-1.5874e+001,+3.6614e-001,-6.7807e-002,-3.6309e-003,+5.9966e-004,
		+4.8163e+000,-3.7363e-001,-7.2071e-002,+1.9998e-003,-6.2385e-004,
		-3.7916e-004,+4.7609e+000,-3.9534e-001,+8.6667e-003,+1.1569e-002,
		+1.1441e-003,-1.4193e-004,-8.5723e-005,+6.5008e-001,-5.0889e-001,
		-1.5754e-002,-2.8305e-003,+5.7458e-004,+3.2577e-005,-9.6052e-006,
		-2.7974e-006,+1.3530e+000,-2.7271e-001,-3.0276e-004,+3.6286e-003,
		-2.0398e-004,+1.5846e-005,-7.7787e-006,+1.1210e-006,+9.9020e-008,
		+5.5046e-001,-2.7312e-001,+3.2532e-003,-2.4277e-003,+1.1596e-004,
		+2.6421e-007,-1.3263e-006,+2.7322e-007,+1.4058e-007,+4.9414e-009
	};

	static double bp_mean[55] = {
		+0.0000e+000,+0.0000e+000,-1.2878e+000,+0.0000e+000,+7.0444e-001,
		+3.3222e-001,+0.0000e+000,-2.9636e-001,+7.2248e-003,+7.9655e-003,
		+0.0000e+000,+1.0854e+000,+1.1145e-002,-3.6513e-002,+3.1527e-003,
		+0.0000e+000,-4.8434e-001,+5.2023e-002,-1.3091e-002,+1.8515e-003,
		+1.5422e-004,+0.0000e+000,+6.8298e-001,+2.5261e-003,-9.9703e-004,
		-1.0829e-003,+1.7688e-004,-3.1418e-005,+0.0000e+000,-3.7018e-001,
		+4.3234e-002,+7.2559e-003,+3.1516e-004,+2.0024e-005,-8.0581e-006,
		-2.3653e-006,+0.0000e+000,+1.0298e-001,-1.5086e-002,+5.6186e-003,
		+3.2613e-005,+4.0567e-005,-1.3925e-006,-3.6219e-007,-2.0176e-008,
		+0.0000e+000,-1.8364e-001,+1.8508e-002,+7.5016e-004,-9.6139e-005,
		-3.1995e-006,+1.3868e-007,-1.9486e-007,+3.0165e-010,-6.4376e-010
	};

	static double ap_amp[55] = {
		-1.0444e-001,+1.6618e-001,-6.3974e-002,+1.0922e+000,+5.7472e-001,
		-3.0277e-001,-3.5087e+000,+7.1264e-003,-1.4030e-001,+3.7050e-002,
		+4.0208e-001,-3.0431e-001,-1.3292e-001,+4.6746e-003,-1.5902e-004,
		+2.8624e+000,-3.9315e-001,-6.4371e-002,+1.6444e-002,-2.3403e-003,
		+4.2127e-005,+1.9945e+000,-6.0907e-001,-3.5386e-002,-1.0910e-003,
		-1.2799e-004,+4.0970e-005,+2.2131e-005,-5.3292e-001,-2.9765e-001,
		-3.2877e-002,+1.7691e-003,+5.9692e-005,+3.1725e-005,+2.0741e-005,
		-3.7622e-007,+2.6372e+000,-3.1165e-001,+1.6439e-002,+2.1633e-004,
		+1.7485e-004,+2.1587e-005,+6.1064e-006,-1.3755e-008,-7.8748e-008,
		-5.9152e-001,-1.7676e-001,+8.1807e-003,+1.0445e-003,+2.3432e-004,
		+9.3421e-006,+2.8104e-006,-1.5788e-007,-3.0648e-008,+2.6421e-010
	};

	static double bp_amp[55] = {
		+0.0000e+000,+0.0000e+000,+9.3340e-001,+0.0000e+000,+8.2346e-001,
		+2.2082e-001,+0.0000e+000,+9.6177e-001,-1.5650e-002,+1.2708e-003,
		+0.0000e+000,-3.9913e-001,+2.8020e-002,+2.8334e-002,+8.5980e-004,
		+0.0000e+000,+3.0545e-001,-2.1691e-002,+6.4067e-004,-3.6528e-005,
		-1.1166e-004,+0.0000e+000,-7.6974e-002,-1.8986e-002,+5.6896e-003,
		-2.4159e-004,-2.3033e-004,-9.6783e-006,+0.0000e+000,-1.0218e-001,
		-1.3916e-002,-4.1025e-003,-5.1340e-005,-7.0114e-005,-3.3152e-007,
		+1.6901e-006,+0.0000e+000,-1.2422e-002,+2.5072e-003,+1.1205e-003,
		-1.3034e-004,-2.3971e-005,-2.6622e-006,+5.7852e-007,+4.5847e-008,
		+0.0000e+000,+4.4777e-002,-3.0421e-003,+2.6062e-005,-7.2421e-005,
		+1.9119e-006,+3.9236e-007,+2.2390e-007,+2.9765e-009,-4.6452e-009
	};


	static double at_mean[55] = {
		+1.6257e+001,+2.1224e+000,+9.2569e-001,-2.5974e+001,+1.4510e+000,
		+9.2468e-002,-5.3192e-001,+2.1094e-001,-6.9210e-002,-3.4060e-002,
		-4.6569e+000,+2.6385e-001,-3.6093e-002,+1.0198e-002,-1.8783e-003,
		+7.4983e-001,+1.1741e-001,+3.9940e-002,+5.1348e-003,+5.9111e-003,
		+8.6133e-006,+6.3057e-001,+1.5203e-001,+3.9702e-002,+4.6334e-003,
		+2.4406e-004,+1.5189e-004,+1.9581e-007,+5.4414e-001,+3.5722e-001,
		+5.2763e-002,+4.1147e-003,-2.7239e-004,-5.9957e-005,+1.6394e-006,
		-7.3045e-007,-2.9394e+000,+5.5579e-002,+1.8852e-002,+3.4272e-003,
		-2.3193e-005,-2.9349e-005,+3.6397e-007,+2.0490e-006,-6.4719e-008,
		-5.2225e-001,+2.0799e-001,+1.3477e-003,+3.1613e-004,-2.2285e-004,
		-1.8137e-005,-1.5177e-007,+6.1343e-007,+7.8566e-008,+1.0749e-009
	};


	static double bt_mean[55] = {
		+0.0000e+000,+0.0000e+000,+1.0210e+000,+0.0000e+000,+6.0194e-001,
		+1.2292e-001,+0.0000e+000,-4.2184e-001,+1.8230e-001,+4.2329e-002,
		+0.0000e+000,+9.3312e-002,+9.5346e-002,-1.9724e-003,+5.8776e-003,
		+0.0000e+000,-2.0940e-001,+3.4199e-002,-5.7672e-003,-2.1590e-003,
		+5.6815e-004,+0.0000e+000,+2.2858e-001,+1.2283e-002,-9.3679e-003,
		-1.4233e-003,-1.5962e-004,+4.0160e-005,+0.0000e+000,+3.6353e-002,
		-9.4263e-004,-3.6762e-003,+5.8608e-005,-2.6391e-005,+3.2095e-006,
		-1.1605e-006,+0.0000e+000,+1.6306e-001,+1.3293e-002,-1.1395e-003,
		+5.1097e-005,+3.3977e-005,+7.6449e-006,-1.7602e-007,-7.6558e-008,
		+0.0000e+000,-4.5415e-002,-1.8027e-002,+3.6561e-004,-1.1274e-004,
		+1.3047e-005,+2.0001e-006,-1.5152e-007,-2.7807e-008,+7.7491e-009
	};

	static double at_amp[55] = {
		-1.8654e+000,-9.0041e+000,-1.2974e-001,-3.6053e+000,+2.0284e-002,
		+2.1872e-001,-1.3015e+000,+4.0355e-001,+2.2216e-001,-4.0605e-003,
		+1.9623e+000,+4.2887e-001,+2.1437e-001,-1.0061e-002,-1.1368e-003,
		-6.9235e-002,+5.6758e-001,+1.1917e-001,-7.0765e-003,+3.0017e-004,
		+3.0601e-004,+1.6559e+000,+2.0722e-001,+6.0013e-002,+1.7023e-004,
		-9.2424e-004,+1.1269e-005,-6.9911e-006,-2.0886e+000,-6.7879e-002,
		-8.5922e-004,-1.6087e-003,-4.5549e-005,+3.3178e-005,-6.1715e-006,
		-1.4446e-006,-3.7210e-001,+1.5775e-001,-1.7827e-003,-4.4396e-004,
		+2.2844e-004,-1.1215e-005,-2.1120e-006,-9.6421e-007,-1.4170e-008,
		+7.8720e-001,-4.4238e-002,-1.5120e-003,-9.4119e-004,+4.0645e-006,
		-4.9253e-006,-1.8656e-006,-4.0736e-007,-4.9594e-008,+1.6134e-009
	};

	static double bt_amp[55] = {
		+0.0000e+000,+0.0000e+000,-8.9895e-001,+0.0000e+000,-1.0790e+000,
		-1.2699e-001,+0.0000e+000,-5.9033e-001,+3.4865e-002,-3.2614e-002,
		+0.0000e+000,-2.4310e-002,+1.5607e-002,-2.9833e-002,-5.9048e-003,
		+0.0000e+000,+2.8383e-001,+4.0509e-002,-1.8834e-002,-1.2654e-003,
		-1.3794e-004,+0.0000e+000,+1.3306e-001,+3.4960e-002,-3.6799e-003,
		-3.5626e-004,+1.4814e-004,+3.7932e-006,+0.0000e+000,+2.0801e-001,
		+6.5640e-003,-3.4893e-003,-2.7395e-004,+7.4296e-005,-7.9927e-006,
		-1.0277e-006,+0.0000e+000,+3.6515e-002,-7.4319e-003,-6.2873e-004,
		-8.2461e-005,+3.1095e-005,-5.3860e-007,-1.2055e-007,-1.1517e-007,
		+0.0000e+000,+3.1404e-002,+1.5580e-002,-1.1428e-003,+3.3529e-005,
		+1.0387e-005,-1.9378e-006,-2.7327e-007,+7.5833e-009,-9.2323e-009
	};

	// sin(latitude)
	sinlat = sin(dlat);

	// degree n and order m
	n = 9;
	m = 9;

	// determine n! (faktorielle) moved by 1
	dfac[0] = 1.0;
	for (i = 1; i <= 2*n+1; i++) {
		dfac[i] = dfac[i-1]*i;
	}

	// determine Legendre functions (Heiskanen and Moritz, Physical Geodesy, 1967, eq. 1-62)
	for (i = 0; i <= n; i++) {
        im = M_MIN(i,m);
		for (j = 0; j <= im; j++) {
			ir = (i-j)/2;
			sum1 = 0.0;
			for (k = 0; k <= ir; k++) {
				sum1 += ipow(-1,k)*dfac[2*i-2*k]/dfac[k]/dfac[i-k]/dfac[i-j-2*k]*qPow(sinlat,i-j-2*k);
			}
			// Legendre functions moved by 1
            p[i][j] = 1.0/ipow(2,i)*M_SQRT(qPow(1.0-sinlat*sinlat,j))*sum1;
		}
	}

	// calculate spherical harmonics
	i = 0;
	for (n = 0; n <= 9; n++) {
		for (m = 0; m <= n; m++) {
			i++;
			ap[i-1] = p[n][m]*qCos(m*dlon);
			bp[i-1] = p[n][m]*qSin(m*dlon);
		}
	}

	// Geoidal height
	*undu = 0.0;
	for (i = 1; i <= 55; i++) {
		*undu += (a_geoid[i-1]*ap[i-1]+b_geoid[i-1]*bp[i-1]);      
	}

	// orthometric height
	hort = dhgt-*undu;

	// Surface pressure on the geoid
	apm = 0.0;
	apa = 0.0;
	for (i = 1; i <= 55; i++) {
		apm += (ap_mean[i-1]*ap[i-1]+bp_mean[i-1]*bp[i-1]);
		apa += (ap_amp[i-1]*ap[i-1]+bp_amp[i-1]*bp[i-1]);
	}
	pres0 = apm+apa*qCos(doy/365.25*2.0*m_PI);

	// height correction for pressure
	*pres = pres0*qPow(1.0-0.0000226*hort,5.225);

	// Surface temperature on the geoid
	atm = 0.0;
	ata = 0.0;
	for (i = 1; i <= 55; i++) {
		atm += (at_mean[i-1]*ap[i-1]+bt_mean[i-1]*bp[i-1]);
		ata += (at_amp[i-1]*ap[i-1]+bt_amp[i-1]*bp[i-1]);
	}
	temp0 = atm+ata*qCos(doy/365.25*2.0*m_PI);

	// height correction for pressure
	*temp = temp0-0.0065*hort;
}

//Called by trop_gpt
int QTropDelay::ipow(int base,int exp)
{
	int result = 1;

	while (exp) {
		if (exp & 1) result *= base;
		exp >>= 1;
		base *= base;
	}
	return result;
}

//Calculate Global Mapping Functions (GMF)
/*****************************************************************************
 * Name        : trop_map_gmf
 * 
 * Description : Calcaulate the Global Mapping Functions (GMF)
 * 
 * Parameters  :
 * Name                           |Da|Unit|Description
 * double  dmjd                    I  N/A  Modified julian date
 * double  dlat                    I  rad  Ellipsoidal latitude
 * double  dlon                    I  rad  Ellipsoidal longitude
 * double  dhgt                    I  m    Height
 * double  zd                      I  rad  Zenith distance
 * double  *gmfh                   O  N/A  Hydrostatic mapping function
 * double  *gmfw                   O  N/A  Wet mapping function
 * 
 * Reference: 
 *    Boehm, J., A.E. Niell, P. Tregoning, H. Schuh (2006), 
 *    Global Mapping Functions (GMF): A new empirical mapping function based on numerical weather model data,
 *    Geoph. Res. Letters, Vol. 33, L07304, doi:10.1029/2005GL025545.
 * 
 * Author: Feng Zhou
 * 
 * Originally written by Feng Zhou on 14/12/2015 @ GFZ
 * 
 * Email: fzhou@geodesy.cn; fzhou@gfz-potsdam.de; zhouforme@gmail.com
 * 
 * Section 1.1, GPS/Galileo technologies, GFZ German Research Centre for Geosciences
 * 
 *****************************************************************************/
void QTropDelay::trop_map_gmf(double dmjd,double dlat,double dlon,double dhgt,double zd,double *gmfh,double *gmfw) 
{
   double doy,sinlat;
   int i,j,k,n,m,im,ir;
   double p[10][10],dfac[20],ap[55],bp[55];
   double sum1,ah,bh,ch,aw,bw,cw,c0h,c10h,c11h,phh,ahm,aha,awm,awa,sine,beta,gamma,
       a_ht,b_ht,c_ht,hs_km,topcon,ht_corr_coef,ht_corr;

   // reference day is 28 January
   // this is taken from Niell (1996) to be consistent
   doy = dmjd-44239+1-28;

   // initialized data
   static double ah_mean[55] = {
     +1.2517e+02, +8.503e-01, +6.936e-02, -6.760e+00, +1.771e-01,
      +1.130e-02, +5.963e-01, +1.808e-02, +2.801e-03, -1.414e-03,
      -1.212e+00, +9.300e-02, +3.683e-03, +1.095e-03, +4.671e-05,
      +3.959e-01, -3.867e-02, +5.413e-03, -5.289e-04, +3.229e-04,
      +2.067e-05, +3.000e-01, +2.031e-02, +5.900e-03, +4.573e-04,
      -7.619e-05, +2.327e-06, +3.845e-06, +1.182e-01, +1.158e-02,
      +5.445e-03, +6.219e-05, +4.204e-06, -2.093e-06, +1.540e-07,
      -4.280e-08, -4.751e-01, -3.490e-02, +1.758e-03, +4.019e-04,
      -2.799e-06, -1.287e-06, +5.468e-07, +7.580e-08, -6.300e-09,
      -1.160e-01, +8.301e-03, +8.771e-04, +9.955e-05, -1.718e-06,
      -2.012e-06, +1.170e-08, +1.790e-08, -1.300e-09, +1.000e-10
   };
   
   static double bh_mean[55] = {
      +0.000e+00, +0.000e+00, +3.249e-02, +0.000e+00, +3.324e-02,
      +1.850e-02, +0.000e+00, -1.115e-01, +2.519e-02, +4.923e-03,
      +0.000e+00, +2.737e-02, +1.595e-02, -7.332e-04, +1.933e-04,
      +0.000e+00, -4.796e-02, +6.381e-03, -1.599e-04, -3.685e-04,
      +1.815e-05, +0.000e+00, +7.033e-02, +2.426e-03, -1.111e-03,
      -1.357e-04, -7.828e-06, +2.547e-06, +0.000e+00, +5.779e-03,
      +3.133e-03, -5.312e-04, -2.028e-05, +2.323e-07, -9.100e-08,
      -1.650e-08, +0.000e+00, +3.688e-02, -8.638e-04, -8.514e-05,
      -2.828e-05, +5.403e-07, +4.390e-07, +1.350e-08, +1.800e-09,
      +0.000e+00, -2.736e-02, -2.977e-04, +8.113e-05, +2.329e-07,
      +8.451e-07, +4.490e-08, -8.100e-09, -1.500e-09, +2.000e-10
   };

   static double ah_amp[55] = {
      -2.738e-01, -2.837e+00, +1.298e-02, -3.588e-01, +2.413e-02,
      +3.427e-02, -7.624e-01, +7.272e-02, +2.160e-02, -3.385e-03,
      +4.424e-01, +3.722e-02, +2.195e-02, -1.503e-03, +2.426e-04,
      +3.013e-01, +5.762e-02, +1.019e-02, -4.476e-04, +6.790e-05,
      +3.227e-05, +3.123e-01, -3.535e-02, +4.840e-03, +3.025e-06,
      -4.363e-05, +2.854e-07, -1.286e-06, -6.725e-01, -3.730e-02,
      +8.964e-04, +1.399e-04, -3.990e-06, +7.431e-06, -2.796e-07,
      -1.601e-07, +4.068e-02, -1.352e-02, +7.282e-04, +9.594e-05,
      +2.070e-06, -9.620e-08, -2.742e-07, -6.370e-08, -6.300e-09,
      +8.625e-02, -5.971e-03, +4.705e-04, +2.335e-05, +4.226e-06,
      +2.475e-07, -8.850e-08, -3.600e-08, -2.900e-09, +0.000e+00
   };
   
   static double bh_amp[55] = {
      +0.000e+00, +0.000e+00, -1.136e-01, +0.000e+00, -1.868e-01,
      -1.399e-02, +0.000e+00, -1.043e-01, +1.175e-02, -2.240e-03,
      +0.000e+00, -3.222e-02, +1.333e-02, -2.647e-03, -2.316e-05,
      +0.000e+00, +5.339e-02, +1.107e-02, -3.116e-03, -1.079e-04,
      -1.299e-05, +0.000e+00, +4.861e-03, +8.891e-03, -6.448e-04,
      -1.279e-05, +6.358e-06, -1.417e-07, +0.000e+00, +3.041e-02,
      +1.150e-03, -8.743e-04, -2.781e-05, +6.367e-07, -1.140e-08,
      -4.200e-08, +0.000e+00, -2.982e-02, -3.000e-03, +1.394e-05,
      -3.290e-05, -1.705e-07, +7.440e-08, +2.720e-08, -6.600e-09,
      +0.000e+00, +1.236e-02, -9.981e-04, -3.792e-05, -1.355e-05,
      +1.162e-06, -1.789e-07, +1.470e-08, -2.400e-09, -4.000e-10
   };
   
   static double aw_mean[55] = {
      +5.640e+01, +1.555e+00, -1.011e+00, -3.975e+00, +3.171e-02,
      +1.065e-01, +6.175e-01, +1.376e-01, +4.229e-02, +3.028e-03,
      +1.688e+00, -1.692e-01, +5.478e-02, +2.473e-02, +6.059e-04,
      +2.278e+00, +6.614e-03, -3.505e-04, -6.697e-03, +8.402e-04,
      +7.033e-04, -3.236e+00, +2.184e-01, -4.611e-02, -1.613e-02,
      -1.604e-03, +5.420e-05, +7.922e-05, -2.711e-01, -4.406e-01,
      -3.376e-02, -2.801e-03, -4.090e-04, -2.056e-05, +6.894e-06,
      +2.317e-06, +1.941e+00, -2.562e-01, +1.598e-02, +5.449e-03,
      +3.544e-04, +1.148e-05, +7.503e-06, -5.667e-07, -3.660e-08,
      +8.683e-01, -5.931e-02, -1.864e-03, -1.277e-04, +2.029e-04,
      +1.269e-05, +1.629e-06, +9.660e-08, -1.015e-07, -5.000e-10
   };
   
   static double bw_mean[55] = {
      +0.000e+00, +0.000e+00, +2.592e-01, +0.000e+00, +2.974e-02,
      -5.471e-01, +0.000e+00, -5.926e-01, -1.030e-01, -1.567e-02,
      +0.000e+00, +1.710e-01, +9.025e-02, +2.689e-02, +2.243e-03,
      +0.000e+00, +3.439e-01, +2.402e-02, +5.410e-03, +1.601e-03,
      +9.669e-05, +0.000e+00, +9.502e-02, -3.063e-02, -1.055e-03,
      -1.067e-04, -1.130e-04, +2.124e-05, +0.000e+00, -3.129e-01,
      +8.463e-03, +2.253e-04, +7.413e-05, -9.376e-05, -1.606e-06,
      +2.060e-06, +0.000e+00, +2.739e-01, +1.167e-03, -2.246e-05,
      -1.287e-04, -2.438e-05, -7.561e-07, +1.158e-06, +4.950e-08,
      +0.000e+00, -1.344e-01, +5.342e-03, +3.775e-04, -6.756e-05,
      -1.686e-06, -1.184e-06, +2.768e-07, +2.730e-08, +5.700e-09
   };
   
   static double aw_amp[55] = {
      +1.023e-01, -2.695e+00, +3.417e-01, -1.405e-01, +3.175e-01,
      +2.116e-01, +3.536e+00, -1.505e-01, -1.660e-02, +2.967e-02,
      +3.819e-01, -1.695e-01, -7.444e-02, +7.409e-03, -6.262e-03,
      -1.836e+00, -1.759e-02, -6.256e-02, -2.371e-03, +7.947e-04,
      +1.501e-04, -8.603e-01, -1.360e-01, -3.629e-02, -3.706e-03,
      -2.976e-04, +1.857e-05, +3.021e-05, +2.248e+00, -1.178e-01,
      +1.255e-02, +1.134e-03, -2.161e-04, -5.817e-06, +8.836e-07,
      -1.769e-07, +7.313e-01, -1.188e-01, +1.145e-02, +1.011e-03,
      +1.083e-04, +2.570e-06, -2.140e-06, -5.710e-08, +2.000e-08,
      -1.632e+00, -6.948e-03, -3.893e-03, +8.592e-04, +7.577e-05,
      +4.539e-06, -3.852e-07, -2.213e-07, -1.370e-08, +5.800e-09
   };

   static double bw_amp[55] = {
      +0.000e+00, +0.000e+00, -8.865e-02, +0.000e+00, -4.309e-01,
      +6.340e-02, +0.000e+00, +1.162e-01, +6.176e-02, -4.234e-03,
      +0.000e+00, +2.530e-01, +4.017e-02, -6.204e-03, +4.977e-03,
      +0.000e+00, -1.737e-01, -5.638e-03, +1.488e-04, +4.857e-04,
      -1.809e-04, +0.000e+00, -1.514e-01, -1.685e-02, +5.333e-03,
      -7.611e-05, +2.394e-05, +8.195e-06, +0.000e+00, +9.326e-02,
      -1.275e-02, -3.071e-04, +5.374e-05, -3.391e-05, -7.436e-06,
      +6.747e-07, +0.000e+00, -8.637e-02, -3.807e-03, -6.833e-04,
      -3.861e-05, -2.268e-05, +1.454e-06, +3.860e-07, -1.068e-07,
      +0.000e+00, -2.658e-02, -1.947e-03, +7.131e-04, -3.506e-05,
      +1.885e-07, +5.792e-07, +3.990e-08, +2.000e-08, -5.700e-09
   };

   // sin(latitude)
   sinlat = qSin(dlat);

   // degree n and order m
   n = 9;
   m = 9;

   // determine n! (faktorielle) moved by 1
   dfac[0] = 1.0;
   for (i = 1; i <= 2*n+1; i++) {
      dfac[i] = dfac[i-1]*i;
   }

   // determine Legendre functions (Heiskanen and Moritz, Physical Geodesy, 1967, eq. 1-62)
   for (i = 0; i <= n; i++) {
      im = M_MIN(i,m);
      for (j = 0; j <= im; j++) {
         ir = (i-j)/2;
         sum1 = 0.0;
         for (k = 0; k <= ir; k++) {
            sum1 += ipow(-1,k)*dfac[2*i-2*k]/dfac[k]/dfac[i-k]/dfac[i-j-2*k]*qPow(sinlat,i-j-2*k);
         }
         // Legendre functions moved by 1
         p[i][j] = 1.0/ipow(2,i)*M_SQRT(qPow(1.0-sinlat*sinlat,j))*sum1;
      }
   }
   
   // calculate spherical harmonics
   i = 0;
   for (n = 0; n <= 9; n++) {
      for (m = 0; m <= n; m++) {
         i++;
         ap[i-1] = p[n][m]*qCos(m*dlon);
         bp[i-1] = p[n][m]*qSin(m*dlon);
      }
   }
   
   // Compute hydrostatic mapping function
   bh = 0.0029;
   c0h = 0.062;
   if (dlat < 0.0) {   // southern hemisphere
      phh = m_PI;
      c11h = 0.007;
      c10h = 0.002;
   }
   else {   // northern hemisphere
      phh = 0;
      c11h = 0.005;
      c10h = 0.001;
   }
   ch = c0h+((qCos(doy/365.25*2.0*m_PI+phh)+1.0)*c11h/2.0+c10h)*(1.0-qCos(dlat));
   
   ahm = 0.0;
   aha = 0.0;
   for (i = 1; i <= 55; i++) {
      ahm += (ah_mean[i-1]*ap[i-1]+bh_mean[i-1]*bp[i-1])*1e-5;
      aha += (ah_amp[i-1]*ap[i-1]+bh_amp[i-1]*bp[i-1])*1e-5;
   }
   ah = ahm+aha*qCos(doy/365.25*2.0*m_PI);
   
   sine   = qSin(m_PI/2.0-zd);
   beta   = bh/(sine+ch);
   gamma  = ah/(sine+beta);
   topcon = (1.0+ah/(1.0+bh/(1.0+ch)));
   *gmfh  = topcon/(sine+gamma);
   
   // height correction for hydrostatic mapping function from Niell (1996)
   a_ht = 2.53e-5;
   b_ht = 5.49e-3;
   c_ht = 1.14e-3;
   hs_km = dhgt/1000.0;
   
   beta = b_ht/(sine+c_ht);
   gamma = a_ht/(sine+beta);
   topcon = (1.0+a_ht/(1.0 + b_ht/(1.0 + c_ht)));
   ht_corr_coef = 1.0/sine-topcon/(sine + gamma);
   ht_corr = ht_corr_coef*hs_km;
   *gmfh += ht_corr;
   
   // Compute wet mapping function
   bw = 0.00146;
   cw = 0.04391;
   
   awm = 0.0;
   awa = 0.0;
   for (i = 1; i <= 55; i++) {
      awm += (aw_mean[i-1]*ap[i-1]+bw_mean[i-1]*bp[i-1])*1e-5;
      awa += (aw_amp[i-1]*ap[i-1]+bw_amp[i-1]*bp[i-1])*1e-5;
   }      
   aw = awm+awa*qCos(doy/365.25*2.0*m_PI);
   beta = bw/(sine+cw);
   gamma = aw/(sine+beta);
   topcon = (1.0+aw/(1.0+bw/(1.0+cw)));
   *gmfw = topcon/(sine+gamma);
}

//UNN3M model
#ifdef EIGEN_CORE_H
//pBLH:[rad.rad,m];doy：Tday（int）； Satellite altitude angle E ：（rad）
//resulat =[Total delay, zenith dry delay, dry delay projection function, zenith wet delay, wet delay projection function];
double QTropDelay::getUNB3mDelay(double *pBLH, double TDay, double E, double *mf, double *ZPD, double *tZHD)
{
	Vector3d BLH;
	BLH[0] = pBLH[0];BLH[1] = pBLH[1];BLH[2] = pBLH[2];
	VectorXd resulat = UNB3M(BLH,TDay,E);
	if(mf) *mf = resulat[4];//Wet delay function
    if(ZPD) *ZPD = resulat[0];
    if(tZHD) *tZHD = resulat[1];// dbug by xiaogongwei 2018.12.24
	return resulat[1]*resulat[2];//Return only dry delay！！！！！！！
	//return resulat[0];//Return total delay
}

/*
 * aigs   :   BLH        I     Latitude, longitude, height
 *            DAYOYEAR   I     Day of year
 *            ELEVRAD    I     Elevation angle (radians)
 * return :   HZD     Hydrostatic zenith delay (m)
              HMF     Hydrostatic Niell mapping function
              WZD     Non-hyd. zenith delay (m)
              WMF     Non-hyd. Niell mapping function
              RTROP   Total slant delay (m)
VectorXd =[RTROP,HZD,HMF,WZD,WMF];[Total delay, zenith dry delay, dry delay projection function, zenith wet delay, wet delay projection function];
*/
VectorXd QTropDelay::UNB3M(Vector3d &BLH, double DAYOYEAR, double ELEVRAD)
{
    double LATRAD=BLH[0];
    double HEIGHTM=BLH[2];

    MatrixXd AVG(5,6);

    AVG<<15.0  ,1013.25  ,299.65  ,75.00  ,6.30  ,2.77,
            30.0  ,1017.25  ,294.15  ,80.00 , 6.05  ,3.15,
            45.0  ,1015.75 , 283.15  ,76.00  ,5.58 , 2.57,
            60.0 , 1011.75  ,272.15  ,77.50  ,5.39  ,1.81,
            75.0  ,1013.00 , 263.65 , 82.50 , 4.53 , 1.55;
    MatrixXd AMP(5,6);
    AMP<< 15.0  , 0.00  , 0.00 ,  0.00  ,0.00,  0.00,
            30.0  ,-3.75  , 7.00   ,0.00 , 0.25 , 0.33,
            45.0  ,-2.25  ,11.00  ,-1.00 , 0.32 , 0.46,
            60.0  ,-1.75  ,15.00  ,-2.50  ,0.81 , 0.74,
            75.0  ,-0.50,  14.50 ,  2.50  ,0.62 , 0.30;

    double EXCEN2 = 6.6943799901413e-03;
    double MD     = 28.9644;
    double MW     = 18.0152;
    double K1     = 77.604;
    double K2     = 64.79;
    double K3     = 3.776e5;
    double R      = 8314.34;
    double C1     = 2.2768e-03;
    double K2PRIM = K2 - K1*(MW/MD);
    double RD     = R / MD;

    double DOY2RAD=(0.31415926535897935601e01)*2/365.25;
    MatrixXd ABC_AVG(5,4);
    ABC_AVG<<15.0 ,1.2769934e-3 ,2.9153695e-3 ,62.610505e-3,
            30.0, 1.2683230e-3 ,2.9152299e-3 ,62.837393e-3,
            45.0, 1.2465397e-3 ,2.9288445e-3 ,63.721774e-3,
            60.0 ,1.2196049e-3 ,2.9022565e-3, 63.824265e-3,
            75.0 ,1.2045996e-3 ,2.9024912e-3 ,64.258455e-3;
    MatrixXd ABC_AMP(5,4);
    ABC_AMP<<15.0, 0.0    ,      0.0      ,    0.0   ,
            30.0 ,1.2709626e-5 ,2.1414979e-5, 9.0128400e-5 ,
            45.0, 2.6523662e-5 ,3.0160779e-5 ,4.3497037e-5 ,
            60.0 ,3.4000452e-5 ,7.2562722e-5 ,84.795348e-5 ,
            75.0 ,4.1202191e-5 ,11.723375e-5 ,170.37206e-5;

    double A_HT = 2.53e-5;
    double B_HT= 5.49e-3;
    double C_HT = 1.14e-3;
    double HT_TOPCON = 1 + A_HT/(1 + B_HT/(1 + C_HT));

    MatrixXd ABC_W2P0(5,4);
    ABC_W2P0<< 15.0 ,5.8021897e-4 ,1.4275268e-3 ,4.3472961e-2,
            30.0 ,5.6794847e-4 ,1.5138625e-3 ,4.6729510e-2,
            45.0, 5.8118019e-4 ,1.4572752e-3, 4.3908931e-2,
            60.0 ,5.9727542e-4 ,1.5007428e-3 ,4.4626982e-2,
            75.0 ,6.1641693e-4 ,1.7599082e-3, 5.4736038e-2;

    double LATDEG = LATRAD * 180.0 / m_PI;
    double  TD_O_Y = DAYOYEAR;
    if (LATDEG<0)
    {
        TD_O_Y = TD_O_Y + 182.625;
    }
    double COSPHS = cos((TD_O_Y - 28) * DOY2RAD );
    double LAT = abs( LATDEG );
    double P1=0,P2=0,M=0;
    if (LAT>=75)
    {
        P1=5;
        P2=5;
        M=0;
    }
    else if (LAT<=15)
    {
        P1=1;
        P2=1;
        M=0;
    }
    else
    {
        P1 = int((LAT - 15)/15) + 1;
        P2 = P1 + 1;
        double aa=LAT - AVG(P1-1,0);
        double bb=AVG(P2-1,0) - AVG(P1-1,0);
        M = (aa ) / ( bb );
    }

    double PAVG = M * ( AVG(P2-1,1) - AVG(P1-1,1) ) + AVG(P1-1,1);
    double TAVG = M * ( AVG(P2-1,2) - AVG(P1-1,2) ) + AVG(P1-1,2);
    double EAVG = M * ( AVG(P2-1,3) - AVG(P1-1,3) ) + AVG(P1-1,3);
    double BETAAVG   = M * ( AVG(P2-1,4) - AVG(P1-1,4 ) )+ AVG(P1-1,4);
    double LAMBDAAVG = M * ( AVG(P2-1,5) - AVG(P1-1,5) ) + AVG(P1-1,5);

    double PAMP = M * ( AMP(P2-1,1) - AMP(P1-1,1) ) + AMP(P1-1,1);
    double TAMP = M * ( AMP(P2-1,2) - AMP(P1-1,2 ) )+ AMP(P1-1,2);
    double EAMP = M * ( AMP(P2-1,3) - AMP(P1-1,3) ) + AMP(P1-1,3);
    double BETAAMP   = M * (AMP(P2-1,4) - AMP(P1-1,4)) + AMP(P1-1,4);
    double LAMBDAAMP = M * ( AMP(P2-1,5) - AMP(P1-1,5) ) + AMP(P1-1,5);

    double P0 = PAVG - PAMP * COSPHS;
    double T0 = TAVG - TAMP * COSPHS;
    double E0 = EAVG - EAMP * COSPHS;
    double BETA = BETAAVG - BETAAMP * COSPHS;
    BETA   = BETA / 1000;
    double LAMBDA = LAMBDAAVG - LAMBDAAMP * COSPHS;

    double ES = 0.01 * exp(1.2378847e-5 * (pow(T0 ,2) )- 1.9121316e-2 * T0 + 3.393711047e1 - 6.3431645e3 * (pow(T0 ,-1)));
    double FW = 1.00062 + 3.14e-6 * P0 + 5.6e-7 * (pow((T0 - 273.15) , 2));
    E0 = (E0 / 1.00e2) * ES * FW;

    double EP = 9.80665 / 287.054 / BETA;

    double T = T0 - BETA * HEIGHTM;
    double P = P0 * pow(( T / T0 ) , EP);
    double E = E0 *pow(( T / T0 ),( EP * (LAMBDA+1) ));

    double GEOLAT = atan((1.0-EXCEN2)*tan(LATRAD));
    double DGREF = 1.0 - 2.66e-03*cos(2.0*GEOLAT) - 2.8e-07*HEIGHTM;
    double GM    = 9.784 * DGREF;
    double DEN   = ( LAMBDA + 1.0 ) * GM;

    double TM  = T * (1 - BETA * RD / DEN);

    double HZD = C1 / DGREF * P;

    double WZD = 1.0e-6 * ( K2PRIM + K3/TM) * RD * E/DEN;

    double A_AVG = M * ( ABC_AVG(P2-1,1) - ABC_AVG(P1-1,1) ) + ABC_AVG(P1-1,1);
    double B_AVG = M * ( ABC_AVG(P2-1,2) - ABC_AVG(P1-1,2) ) + ABC_AVG(P1-1,2);
    double C_AVG = M * ( ABC_AVG(P2-1,3) - ABC_AVG(P1-1,3) ) + ABC_AVG(P1-1,3);

    double A_AMP = M * ( ABC_AMP(P2-1,1) - ABC_AMP(P1-1,1) ) + ABC_AMP(P1-1,1);
    double B_AMP = M * ( ABC_AMP(P2-1,2) - ABC_AMP(P1-1,2) ) + ABC_AMP(P1-1,2);
    double C_AMP = M * ( ABC_AMP(P2-1,3) - ABC_AMP(P1-1,3) ) + ABC_AMP(P1-1,3);

    double A = A_AVG - A_AMP * COSPHS;
    double B = B_AVG - B_AMP * COSPHS;
    double C = C_AVG - C_AMP * COSPHS;

    double SINE = sin(ELEVRAD);

    double ALPHA  = B/(SINE + C );
    double GAMMA  = A/(SINE + ALPHA);
    double TOPCON = (1 + A/(1 + B/(1 + C)));
    double HMF    = TOPCON / ( SINE + GAMMA );

    ALPHA  = B_HT/( SINE + C_HT );
    GAMMA  = A_HT/( SINE + ALPHA);
    double HT_CORR_COEF = 1/SINE - HT_TOPCON/(SINE + GAMMA);
    double HT_CORR      = HT_CORR_COEF * HEIGHTM / 1000;
    HMF          = HMF + HT_CORR;

    A = M * ( ABC_W2P0(P2-1,1) - ABC_W2P0(P1-1,1) ) + ABC_W2P0(P1-1,1);
    B = M * ( ABC_W2P0(P2-1,2) - ABC_W2P0(P1-1,2) ) + ABC_W2P0(P1-1,2);
    C = M * ( ABC_W2P0(P2-1,3) - ABC_W2P0(P1-1,3)) + ABC_W2P0(P1-1,3);

    ALPHA = B/( SINE + C );
    GAMMA = A/( SINE + ALPHA);
    TOPCON = (1 + A/(1 + B/(1 + C)));
    double WMF    = TOPCON / ( SINE + GAMMA );
    double RTROP=HZD*HMF+WZD*WMF;

    VectorXd rr(5);
    rr<<RTROP,HZD,HMF,WZD,WMF;
    return rr;
}

#endif
