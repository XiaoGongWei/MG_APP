/*************************************************************************
**
**  MG-APPS----Multi-GNSS-Automatic Precise Positioning Software
**  Copyright (C) 2016-2019 XiaoGongWei
**  This file is part of MG-APPS.
**
**  GNU Lesser General Public License Usage
**  Alternatively, this file may be used under the terms of the GNU Lesser
**  General Public License version 3 as published by the Free Software
**  Foundation and appearing in the file LICENSE.LGPL3 included in the
**  packaging of this file. Please review the following information to
**  ensure the GNU Lesser General Public License version 3 requirements
**  will be met: https://www.gnu.org/licenses/lgpl-3.0.html.
**
**  MPL License Usage
**  This Source Code Form is subject to the terms of the Mozilla Public
**  License, v. 2.0. If a copy of the MPL was not distributed with this
**  file, You can obtain one at http://mozilla.org/MPL/2.0/.
**
**  GPLv3.0 License Usage
**  This program is free software: you can redistribute it and/or modify
**  it under the terms of the GNU General Public License as published by
**  the Free Software Foundation, either version 3 of the License, or
**  (at your option) any later version.
**
**  This program is distributed in the hope that it will be useful,
**  but WITHOUT ANY WARRANTY; without even the implied warranty of
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
**  GNU General Public License for more details.
**
**  You should have received a copy of the GNU General Public License
**  along with this program.  If not, see http://www.gnu.org/licenses/.
**
**************************************************************************
**           Author: XiaoGongWei
**  Website/Contact: http://github.com/xiaogongwei
**             Date: 26.04.2019
****************************************************************************/

#ifndef QCMPGPST_H
#define QCMPGPST_H

/*
1. This class is commonly used in PPP function calculation library,
which provides a large number of calculation methods.Commonly used
coordinate and time conversion in satellite calculation, as well as
coordinate calculation under the sun and moon WGS84, need to read.
ERP file parameters.
*/

#include "QGlobalDef.h"


const  double gpst0[]={1980,1, 6,0,0,0}; /* gps time reference */
const  double gst0 []={1999,8,22,0,0,0}; /* galileo system time reference */
const  double bdt0 []={2006,1, 1,0,0,0}; /* beidou time reference */

//{2015,7,1,0,0,0,-17},
const static double leaps[][7]={ /* leap seconds {y,m,d,h,m,s,utc-gpst,...} */
    {2017,1,1,0,0,0,-18},
	{2015,7,1,0,0,0,-17},
	{2012,7,1,0,0,0,-16},
	{2009,1,1,0,0,0,-15},
	{2006,1,1,0,0,0,-14},
	{1999,1,1,0,0,0,-13},
	{1997,7,1,0,0,0,-12},
	{1996,1,1,0,0,0,-11},
	{1994,7,1,0,0,0,-10},
	{1993,7,1,0,0,0, -9},
	{1992,7,1,0,0,0, -8},
	{1991,1,1,0,0,0, -7},
	{1990,1,1,0,0,0, -6},
	{1988,1,1,0,0,0, -5},
	{1985,7,1,0,0,0, -4},
	{1983,7,1,0,0,0, -3},
	{1982,7,1,0,0,0, -2},
	{1981,7,1,0,0,0, -1},
	{1980,1,6,0,0,0, 0}
};

/* coordinate rotation matrix ------------------------------------------------*/
#define myRx(t,X) do { \
	(X)[0]=1.0; (X)[1]=(X)[2]=(X)[3]=(X)[6]=0.0; \
	(X)[4]=(X)[8]=cos(t); (X)[7]=sin(t); (X)[5]=-(X)[7]; \
} while (0)

#define myRy(t,X) do { \
	(X)[4]=1.0; (X)[1]=(X)[3]=(X)[5]=(X)[7]=0.0; \
	(X)[0]=(X)[8]=cos(t); (X)[2]=sin(t); (X)[6]=-(X)[2]; \
} while (0)

#define myRz(t,X) do { \
	(X)[8]=1.0; (X)[2]=(X)[5]=(X)[6]=(X)[7]=0.0; \
	(X)[0]=(X)[4]=cos(t); (X)[3]=sin(t); (X)[1]=-(X)[3]; \
} while (0)

class QCmpGPST
{
//public functions
public:
	QCmpGPST();
	~QCmpGPST();
    int getSatPRN(QString StaliteName);//Acquisition of satellite PRN
    int YearAccDay(int Year, int Month, int Day);//Calculate DOY
    double YMD2GPSTime(int Year,int Month,int Day,int Hours,int Minutes,double Seconds,int *WeekN = NULL, int *day = NULL);//YMD Change to GPST
    void XYZ2SAE(double X,double Y,double Z,double *m_pSAZ,double *PX);//XYZ: Receiver approximate coordinates; m_SAZ (radian): Return calculation results; PX: station coordinates
	void XYZ2BLH(double X,double Y,double Z,double *m_pBLH,double *ellipseCoeff = NULL);
	void XYZ2ENU(double X,double Y,double Z,double *m_pENU,double *PX);
    void XYZ2SAE(double *pXYZ,double *m_pSAZ,double *PX);//XYZ: Receiver approximate coordinates; m_SAZ (radian): Return calculation results; PX: station coordinates
	void XYZ2BLH(double *pXYZ,double *m_pBLH);
	void XYZ2ENU(double *pXYZ,double *m_pENU,double *PX);
    double computeJD(int Year,int Month,int Day,int HoursInt=0,int Minutes=0,double Seconds=0);//Calculating Julian Day
    double computeMJD(int Year,int Month,int Day,int HoursInt=0,int Minutes=0,double Seconds=0);//Calculating Simplified Julian Day
    bool getSunMoonPos(int Year,int Month,int Day,int HoursInt,int Minutes,double Seconds,double *sunpos,double *moonpos,double *gmst = NULL);//Calculating WGS84 coordinates of the Sun and Moon and GMST Pingrini Time
	void sunmoonpos(gtime_t tutc, const double *erpv, double *rsun,
        double *rmoon, double *gmst);//Calculating Solar Coordinate Reference RTKlab
    double getLeapSecond(int Year,int Month,int Day,int Hours=0,int Minutes=0,double Seconds=0);//Get a leap seconds
    //Common Vector Mathematical Functions
    double InnerVector(double *a,double *b,int Vectorlen = 3);//Calculate the inner product of a and b vectors
    bool OutVector(double *a,double *b,double *c);//Calculate the three-dimensional a, b vector outer product and return the result using c
	double norm(const double *a, int n);
	void cross3(const double *a, const double *b, double *c);
	int normv3(const double *a, double *b);
	double dot(const double *a, const double *b, int n);
    //RTKLIB Time Conversion
	gtime_t gpst2utc(gtime_t t);
	gtime_t epoch2time(const double *ep);
	gtime_t gpst2time(int week, double sec);
	void time2epoch(gtime_t t, double *ep);
	double timediff(gtime_t t1, gtime_t t2);
    //RTKLIB Coordinate Conversion
	void ecef2pos(const double *r, double *pos);
	void xyz2enu(const double *pos, double *E);
    //Read ERP files
	int readerp(const char *file, erp_t *erp);
	int geterp(const erp_t *erp, gtime_t time, double *erpv);
    bool readRepFile(QString m_erpFileName);//Initialize reading ERP file once
private:
    double MyAtanA(double x,double y);//tan function
    double MyAtanL(double x,double y);//tan function
    // Solar coordinates obtained by RTKLab
	gtime_t timeadd(gtime_t t, double sec);
	void ast_args(double t, double *f);
	gtime_t utc2gpst(gtime_t t);
	double time2gpst(gtime_t t, int *week);
	double time2sec(gtime_t time, gtime_t *day);
	double utc2gmst(gtime_t t, double ut1_utc);
	void sunmoonpos_eci(gtime_t tut, double *rsun, double *rmoon);
	void nut_iau1980(double t, const double *f, double *dpsi, double *deps);
	void eci2ecef(gtime_t tutc, const double *erpv, double *U, double *gmst);
	void matmul(const char *tr, int n, int k, int m, double alpha,
		const double *A, const double *B, double beta, double *C);
//Variable part
public:
    double elipsePara[6];//Ellipsoid parameter, default WGS84 parameter
private:
    erp_t m_erpData;//Save ERP file data
    bool isuseErp;//Read ERP file data
};

#endif // QCMPGPST_H
