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

#ifndef QREADSP3_H
#define QREADSP3_H


#include "QGlobalDef.h"

/*
1. Optimization calculation is required here: 8-point interpolation does not need frequent interpolation for each satellite
2. Abandon the imprecise coordinates of the later stage
3. Return the velocity of the satellite at the interpolation time
4. Examples:

(1) class initialization
QReadSP3 readSP3("D:\\igs15786.sp3");The path must be specified

(2) it needs to read data once before calling the class, and call getAllData() if it wants to read data; A longer time
readSP3.setSatlitSys("GC");// The letters G,R,C and E are used for GPS,GLONASS,BDS and Galieo respectively
readSP3.getAllData();

(3)And then call getPrcisePoint(int PRN,double GPST,double *pXYZ,double *pdXYZ = NULL);
double pXYZ[3] ={0}，pdXYZ[3] = {0};// Define coordinates and velocities
readSP3.getPrcisePoint(32,514800,pXYZ,pdXYZ);// You get the coordinates and the velocity
or
readSP3.getPrcisePoint(32,514800,pXYZ);// Get the coordinates

5. Note:
(1) this class can read RINEX VERSION/TYPE: 3.00 / C (currently only read GPS satellite; if necessary, you can modify readFileData2Vec and readAllData functions by yourself.)
(2) getPrcisePoint is recommended to be called from small to large according to GPS time (GPST) to "improve the running speed", otherwise it is not obvious. Change ACCELERATE to 0 if you don't want to call by time
*/

// the coordinate system comes with SP3
typedef struct _SP3Data
{//SP3 file data format
    int GPSWeek;// GPS week
    int GPSTime;// GPS week  seconds
    MatrixXd MatrixDataGPS;// the ith column of the GPS system stores PRN X,Y and Z respectively
	MatrixXd MatrixDataGlonass;//Glonass
	MatrixXd MatrixDataBDS;//BDS
	MatrixXd MatrixDataGalieo;//Galieo
}SP3Data;

class QReadSP3: public QBaseObject
{
// function part
public:
	QReadSP3();
    QReadSP3(QStringList SP3FileNames);// read multiple files including one
    void setSP3FileNames(QStringList SP3FileNames);// read multiple files including one
	~QReadSP3(void);
    QVector< SP3Data > getAllData();// return and read all data (time consuming)
    bool setSatlitSys(QString SystemStr);// Where, GPS,GLONASS,BDS,Galieo are respectively used: letters G,R,C,E (override parent class)
    void getPrcisePoint(int PRN,char SatType,double GPST,double *pXYZ,double *pdXYZ = NULL, double *pSp3Clk = NULL);// obtain precise ephemeris coordinates pXYZ and velocity pDXYZ(3d)
	void releaseAllData();
private:
    void initVar();// initializes variables
    void InitStruct();// initialize the internal matrix of the structure to be used (to save memory)
    void readFileData2Vec(QStringList SP3FileNames);// read multiple file data to m_allEpochData
    void get8Point(int PRN, char SatType, double *pX, double *pY, double *pZ, int *pGPST, double GPST, double *pClk);//pX,pY,pZ: 8 coordinates; PGPST :8 GPS points per second; GPST satellite launch time within weeks seconds
    void lagrangeMethod(int PRN,char SatType,double GPST,double *pXYZ,double *pdXYZ = NULL, double *pSp3Clk = NULL);// the seventh-order Lagrangian interpolation is performed by interpolating 8 points before and after the launch time of GPST satellite to return the coordinates of pXYZ: sp3 format (WGS84) within seconds of the week; PdXYZsp3 format (WGS84) speed
    int YMD2GPSTime(int Year,int Month,int Day,int Hours,int Minutes,int Seconds,int *GPSWeek = NULL);// calculate GPS time. GPS week :GPS week
// data section
public:
    bool  ACCELERATE;// 1: getPrcisePoint function time from small to large
	
private:
    QString m_SP3FileName;// save individual file names
	QStringList m_SP3FileNames;
    QVector< SP3Data > m_allEpochData;// coordinate data for each epoch. PRN X,Y and Z are stored in the I column of time, minute and second
    SP3Data epochData;// store each epoch metadata
    QString tempLine;// Temporary storage of read row data
    bool isReadHead;// whether to read header file
    bool isReadAllData;// whether to read the entire file data
    bool IsSigalFile;// whether multiple files or a single file
    int m_WeekOrder;// save the number of cross-week files
	static const int lagrangeFact;
// variables used to optimize looking for clock differences prevent looking all the way from 0
    int m_lastGPSTimeFlag;// save the first GPS time data obtained by get8Point last time
    double m_lastCmpGPSTime;// save the last calculated GPST to prevent one epoch
    int m_EndHeadNum;// the first and last clock difference between the number of read blocks
};

#endif
