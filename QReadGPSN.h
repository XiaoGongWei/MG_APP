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

#ifndef QREADGPSN_H
#define QREADGPSN_H

#include "QGlobalDef.h"



class QReadGPSN: public::QBaseObject
{
// function part
public:
	QReadGPSN(void);
	~QReadGPSN(void);
    QReadGPSN(QString NFileName);
    void setFileName(QString NFileName);
    QVector< BrdData > getAllData();// read all broadcast ephemeris data to allBrdData
    //PRN: satellite, SatType: satellite type (G,C,R,E), UTC (BDS and GLONASS function internal automatic conversion), CA: pseudo-range observation value
    void getSatPos(int PRN, char SatType, double signal_transmission_time, int Year, int Month, int Day, int Hours,
                   int Minutes, double Seconds, double *StaClock, double *pXYZ, double *pdXYZ);
private:
    void initVar();// initializes variables
    void getHeadInf();// read header information
    void readNFileVer2(QVector< BrdData > &allBrdData);// read Rinex 2.x broadcast ephemeris data
    void readNFileVer3(QVector< BrdData > &allBrdData);// read Rinex 2.x broadcast ephemeris data
    int SearchNFile(int PRN,char SatType,double GPSOTime);// search for recent navigation data
    double YMD2GPSTime(int Year, int Month, int Day, int Hours, int Minutes, double Seconds, int *WeekN = NULL);//YMD Change to GPST //GPSTimeArray[4]=GPSWeek GPS_N
    double getLeapSecond(int Year,int Month,int Day,int Hours=0,int Minutes=0,double Seconds = 0.0);// gey leap seconds
	double computeJD(int Year,int Month,int Day,int HoursInt,int Minutes = 0,double Seconds = 0.0);
    double computeSatClock(double *A, double t, double t0);
	Vector3d GlonassFun(Vector3d Xt,Vector3d dXt,Vector3d ddX0);
	Vector3d RungeKuttaforGlonass(const BrdData &epochBrdData,double tk,double t0,Vector3d &dX);
// data section
public:

private:
    QFile m_readGPSNFile;// read the broadcast ephemeris file
    QString m_NfileName;// save the name of the broadcast ephemeris file

    int m_BaseYear;// basic year is defined as 2000
    double m_leapSec;// save the skip second
	QVector< BrdData > m_allBrdData;
    bool isReadHead;// determines whether to read the header file
    bool isReadAllData;// determine whether to read all data
    QString tempLine;// cache one line of string
    int m_epochDataNum_Ver2;// store one data segment (28 7 rows for GPS and BDS and 12 3 rows for GLONASS)
// The following is the header data section
	//RINEX VERSION / TYPE
	double RinexVersion;
    char FileIdType;//G,C and R represent GPS,BDS and GLONASS systems respectively
	//PGM / RUN BY / DATE
	QString PGM;
	QString RUNBY;
	QString CreatFileDate;
	//COMMENT
	QString CommentInfo;
	//ION ALPHA
	double IonAlpha[4];
	double IonBeta[4];
	//DELTA-UTC: A0,A1,T,W
	double DeltaA01[2];
	int DeltaTW[2];
	//	IsReadHeadInfo
	bool IsReadHeadInfo;
};
#endif
