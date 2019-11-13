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

#ifndef QTIDEEFFECT_H
#define QTIDEEFFECT_H

#include "QCmpGPST.h"
/*
The current tides are corrected to:
1, extremely tide: need to download erp files, use RTKLIB C language standard library to read data erp files
2. Earth tide: consider the 2nd and 3rd order tide, frequency domain correction, and default does not consider permanent deformation
3. Ocean tides: the name of the station is set to four characters, or the name of the station is passed in using the getAllTideEffectENU and getAllTideEffect functions

4. Examples:
(1) class initialization:
QString OCEANFileName = "",QString erpFileName = "";// none can be empty, the program will automatically search under the current execution directory
QString StationName = "BJFS";// define station name search Marine data (4 bytes)

QTideEffect tideEffect(OCEANFileName,erpFileName);
tideEffect.setStationName(StationName);// if you want to use ocean tides, you must set the name of the station
tideEffect.getAllData();// select read data

(2) function call method:
// calculate all tidal corrections
// pXYZ: station coordinates EA: altitude Angle and azimuth, psunpos solar coordinates
double pXYZ[3]={99999,999999,99999},EA[2] = {0.9,0.9},psunpos[3] = {99999,9999,9999},pmoonpos[3] = {99999,9999,9999},gmst = 0;
QString StationName = "BJFS";// define station name search Marine data (4 bytes)
doubel result = 0;// effect of tides on line of sight distance (unit m)
result = tideEffect.getAllTideEffect(Year,Month,Day,Hours,Minutes,Seconds,pXYZ,EA,psunpos,pmoonpos,gmst,StationName);

double pENU[3] = {0};// preserve the influence of tides on the direction of ENU
tideEffect.getAllTideEffectENU(Year,Month,Day,Hours,Minutes,Seconds,pXYZ,pENU,psunpos,pmoonpos,gmst,StationName);

Note: the getAllTideEffect function can be used to pass in the values of sun, moon and GMST, which can reduce the calculation amount without repeated calculation of the solar and moon coordinates (setSunMoonPos function can also be used in real time).
These values can be obtained from the QReadAnt class by calculating satellite antenna corrections.
*/

#include <QFile>

typedef struct _OCEANData
{
    QString StationName;// uppercase station name
	double amp[3][11];
	double phasedats[3][11];
    bool isRead;// to determine whether the data is read correctly, it may fail to read, true is correct, and flase fails
}OCEANData;

// tidal correction class
class QTideEffect
{
// function part
public:
    QTideEffect(QString OCEANFileName = "",QString erpFileName = "");// incoming ocean data,erp file path, otherwise search from current directory otherwise do not apply ocean tide and polar tide correction
    void setTideFileName(QString OCEANFileName = "",QString erpFileName = "");// incoming ocean data,erp file path, otherwise search from current directory otherwise do not apply ocean tide and polar tide correction
	~QTideEffect(void);
    // get line of sight conversion to distance correction
    void getAllTideEffectENU(int Year,int Month,int Day,int Hours,int Minuts,double Seconds,double *pXYZ,double *pENU,double *psunpos=NULL,
        double *pmoonpos=NULL,double gmst = 0,QString StationName = "");// obtain the influence of all sea tide effects on the ENU direction
    // get correction of ENU direction
    double getAllTideEffect(int Year,int Month,int Day,int Hours,int Minuts,double Seconds,double *pXYZ,double *EA,double *psunpos=NULL,
        double *pmoonpos=NULL,double gmst = 0,QString StationName = "");// pXYZ: station WGS84 coordinate system, EA satellite line of sight height Angle and azimuth (radians), 2d array EA[2]
    // ENU correction of polar tide, earth tide and ocean tide is obtained separately
    void getPoleTide(int Year,int Month,int Day,int Hours,int Minuts,double Seconds,double *pBLH,double *pTideENU);// input observation time (year, month, day, hour, minute and second), pXYZ: I is the coordinates of station WGS84, and output station XYZ correction *pTideXYZ: O correction
    void getSoildTide(int Year,int Month,int Day,int Hours,int Minuts,double Seconds,double *pXYZ,double *pTideENU,bool isElimate = false);// Input observation time, year, month, day, time, second, pXYZ:I is WGS84 coordinate of the station, output station XYZ correction * pTideXYZ:O correction isElimate: Whether to consider permanent deformation, permanent deformation model seems inaccurate, no significant impact, optional.
    void getOCEANTide(int Year,int Month,int Day,int Hours,int Minuts,double Seconds,double *pXYZ,double *pTideENU,QString StationName = "");// Station Name: The name of the station (e.g. BJFS or BJFS). Station Name searches the current directory if it is empty. Otherwise, tide correction is not used.
    // the name of the station should be set before reading the data
    void setStationName(QString StationName = "");// the station name must be set before initialization
	void getAllData();
    // read ocean files must be station name, if there is no file name current directory search variable. BLQ file
    bool readOCEANFile(QString  StationName,OCEANData &oceaData,QString  OCEANFileName = "");// Station Name: Station Name: Station name (e.g. BJFS or BJFS), read by QFile (can only read IGS station data at any other point can not be read, can also replace the receiver name with the adjacent station) OCEAN File Name empty use class initialization file, otherwise search the current directory.dat file, otherwise do not use the sea. Ocean Tides
    void setSunMoonPos(double *psun,double *pmoon,double gmst=0);// Set-Update Coordinates (Accelerate calculation as getAllTideEffectENU passes in Sun-Moon coordinates)
private:
	void initVar();
    bool readRepFile();// Read the ERP file if it exists
    bool getErpV(gtime_t obsGPST,double *erpV);// Read ERP file data to private variables
	void tide_pole(const double *pos, const double *erpv, double *denu);
    void subSolidTide(double *sunmoonPos,double *pXYZ,double *pTideXYZ,int flag);// Flag:0 represents the calculation of the moon and 1 represents the sun.
// Variable part
public:
    // Keeping the Sun, Moon and GMST values can avoid calculating the Sun and Moon coordinates repeatedly.
    double m_sunpos[3],m_moonpos[3],m_gmst;// Each epoch is updated once to provide coordinates for other programs to reduce repeated calculations
    QCmpGPST m_cmpClass;// QCmpGPST class is needed to calculate calculation function
private:
	QString m_erpFileName;
	erp_t m_erpData;
	bool isOCEANTide;
    bool isPoleEffect;// Whether to use erp tide default true
    bool isSolidTide;// Whether to use  solid tide the default true
    // Solid Tide Required Parameters
    double m_GMi[3];// The gravitational constants of the earth, moon and sun are stored separately.
    double loveShida2[2];// Initialization parameters of Love and Shadoi parameters of secondary solid tide
    bool isgetLoveShida2;// Whether to Obtain Love and Shadoi Parameters of Secondary Solid Tide
    double loveShida3[2];// Love and Shadoi parameters of three solid tides
    double m_SationBLH[3];// Station BLH
    double m_SecondFlag;// Save epoch seconds, prevent multiple calculations, reduce the amount of calculation
    double m_AllTideENU[3];// Preserve the ENU of this epoch to prevent multiple calculations and reduce the amount of calculation
    double m_pSolidENU[3];// Storage Computation of Solid Tide
    double m_pPoleENU[3];// The Tide of Storage Computing
    double m_pOCEANENU[3];// Storage Computing Tides
    double m_erpV[5];// Storing polar shift parameters of the first epoch
    bool isReadErp;// Determine whether to read the ERP file (read only once, even if it fails, it is true, not read, the corresponding tide will be closed, no effect)
    bool isReadOCEAN;// Determine whether OCEAN data has been read (read only once, even if it fails, it is true, and if it is not read, the corresponding tide will be closed, without effect)
    // Reading marine data classes
    QString m_OCEANFileName;// Name of Ocean Data File
	QFile m_readOCEANClass;
    OCEANData m_OCEANData;// Preserving station data
	QString m_StationName;
	static const double args[][5];
    double LeapSeconds;// Hop seconds for the current year, used to transfer to UTC on the day of the year, month and day
    bool isGetPos;// Determine whether or not to obtain the above data
};

#endif
