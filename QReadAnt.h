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

#ifndef QREADANT_H
#define QREADANT_H



#include "QCmpGPST.h"


/* program description
If the "receiver" antenna type and Julian obvJD at the "start" moment of observation cannot be passed in during initialization, setObsJD function must be set
Antenna type and Julian day otherwise cannot match the data of satellite start effective time antenna
1. Class initialization
QString AntFileName = "D:\\antmod.atx", AntType = "ASH700936B_M    SNOW";//AntFileName: antenna file path (empty will go to the current directory to open "antmod.atx") AntType:O file (observation file) to obtain the antenna type
double ObsJD = 2455296.50;// start time (approximately) Julian day
QReadAnt readAnt(AntFileName,AntType,ObsJD);
or
QReadAnt readAnt(AntFileName);
readAnt.setObsJD(AntType,ObsJD);
2. Read data files
Read data only once at initialization (time taken)
readAnt.getAllData();// read all data from satellites and receivers
3. Function call method
double E = 0.7,A = 0.3;// altitude Angle and azimuth Angle
double L1Offset = 0, L2Offset = 0;// save the correction of L1 and L2 line of sight (unit m)
readAnt.getRecvL12(E,A,&L1Offset,&L2Offset);// calculate the correction of L1 and L2 line of sight (unit m)

int Year = 2010,Month =  4,Day = 10,Hours = 0,Minuts = 0,PRN = 32;
double Seconds = 0.0;
double StaPos[3] = {9999,999,9999},RecPos[3] = {9999,9999,9999};
double L12OffSet = 0;
// time is UTC, month, year, day, hour and second, StaPos: satellite XYZ coordinate, RecPos: receiver XYZ coordinate, SatAntH: satellite antenna correction (since the PCO and PCV of each satellite antenna frequency are the same, it only needs to return a correction distance) to return m
gL12OffSet = readAnt.getSatOffSet(Year,Month,Day,Hours,Minuts,Seconds,PRN,StaPos,RecPos);

*/

// define the storage antenna data structure. Only commonly used data is saved here
typedef struct _FrqunceData
{// define a PCO and PCV data format for the frequency band
	QString FrqFlag;//START OF FREQUENCY
	double PCO[3];
    //PCV data
    short int Hang;// PCVAZI line number
    short int Lie;// number of PCVAZI and PCVNoAZI columns
	QVector< double > PCVNoAZI;
    QVector< double > PCVAZI;// the matrix is converted to a 1-dimensional vector
}FrqData;

typedef struct _AntDataType
{
    QString StrAntType;// antenna type
    QString SatliCNN;//CNN satellite Numbers G,R,E...
    double DAZI;// store azimuth interval 360/DAZI
    double ZEN_12N[3];// store high Angle range and interval (ZEN_12N [1] - ZEN_12N [0])/ZEN_12N [2]
    short NumFrqu;// number of frequency bands of satellites
    double ValidJD;// valid start time
    double EndJD;// valid end time
    QVector <FrqData> PCOPCV;// PCO and PCV data for multiple frequency bands are stored
    bool IsSat;//true represents satellite false represents receiver data
    bool isFrqData;// if all antenna files are not found, mark false; // initialization hypothesis can be found that the antenna is true
}AntDataType;


class QReadAnt:public QBaseObject
{
// function part
public:
	QReadAnt(QString AntFileName = "",QString AnTypeName = "",double ObvJD = 0);
    void setAntFileName(QString AntFileName = "",QString AnTypeName = "",double ObvJD = 0);
	~QReadAnt(void);
    bool getRecvL12(double E, double A, char SatType, double &L1Offset, double &L2Offset, QVector<QString> FrqFlag);//E: satellite altitude Angle; A: satellite azimuth (unit radian), L1Offset and L2Offset wavelength correction (unit m)
    bool getSatOffSet(int Year,int Month,int Day,int Hours,int Minuts,double Seconds,int PRN,char SatType,double *StaPos,double *RecPos, double *L12Offset,
                      QVector<QString> FrqFlag);// StaPos: satellite XYZ coordinate, RecPos: receiver XYZ coordinate,L12Offset:FrqFlag first two frequency value correction return m
    void setObsJD(QString AnTypeName,double ObsJD);// a Julian date for observation must be set
    bool getAllData();// read valid satellite antenna data and receiver antenna data
private:
    void initVar();// initializes variables
    bool openFiles(QString AntTypeName);// open the file
    bool readFileAntHead();// read the header file
    bool readAntFile(QString AntTypeName);// read the antenna file
    bool readSatliData();// read the data needed by the satellite
    bool readRecvData();// read the documents needed by the satellite
    double computeJD(int Year,int Month,int Day,int HoursInt=0,int Minutes=0,int Seconds=0);// calculate Julian days
    bool getPCV(const AntDataType &tempAntData,char SatType,double *PCV12,double Ztop,double AZI, QVector<QString> FrqFlag);// calculate the PCV; Z: fixed Angle of satellite = PI /2 - altitude Angle; A: satellite azimuth (unit radians)
    void findPCO(double *pco, QVector<FrqData> &PCOPCV, char SatType = 'G', QString frqStr = "C1C");// add by xiaogongwei 2019.04.12
// variable section
public:
    double m_sunpos[3],m_moonpos[3],m_gmst;// each epoch is updated once to provide coordinates for other programs, reducing repeated calculations
    QCmpGPST m_CmpClass;// compute library public for compute sun and moon position
private:
    QString m_AntFileName;// save the antenna data file name
    QString m_AntType;// save the receiver antenna type name
    QFile m_ReadFileClass;// read the file class
    bool isReadAllData;// whether to read all data flags
    bool isReadHead;// whether to read header file
    bool isReadSatData;// whether to read satellite antenna data
    bool isReadRecvData;// whether to read receiver antenna data
    QString m_tempLine;// cache read row data
    double m_ObeservTime;// Julian days to store observation O files to match the effective time of the satellite antenna
    AntDataType m_RecvData;// data of receiver antenna PCO and PCV are saved
    QVector< AntDataType > m_SatData;// data of multiple satellite antenna PCO and PCV are saved
    double m_pi;// PI radian
    double m_sunSecFlag;// if the reference epoch changes, the solar coordinates need to be recalculated to store the reference epoch in seconds
};

#endif
