/*************************************************************************
**
**  MG-APP----Multi-GNSS-Automatic Precise Positioning Software
**  Copyright (C) 2016-2020 XiaoGongWei
**  This file is part of MG-APP.
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
**  BSD 2-clause
**  Copyright(c) 2007‐2013, T.Takasu, All rights reserved
**  Redistribution and use in source and binary forms, with or without modification,
**  permitted provided that the following conditions are met:
**  1. Redistributions of source code must retain the above copyright notice,
**  this list of conditions and the following disclaimer.
**  2. Redistributions in binary form must reproduce the above copyright notice,
**  this list of conditions and the following disclaimer in the documentation and/or
**  other materials provided with the distribution.
**
**  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
**  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
**  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
**  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
**  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
**  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
**  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
**  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
**  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
**  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**************************************************************************
**           Author: XiaoGongWei
**   Website: github.com/xiaogongwei/MG_APP
** Download link (The GPS Toolbox): https://www.ngs.noaa.gov/gps-toolbox/
**             Date: 06.02.2020
****************************************************************************/


#ifndef QRTWRITE2FILE_H
#define QRTWRITE2FILE_H



#include "QCmpGPST.h"

#define MAX_FILE_WRITE 10

class QRTWrite2File:public QBaseObject
{
// Functional part
public:
    QRTWrite2File(void);
    ~QRTWrite2File(void);
    bool setSaveFloder(QString fload_path);
    void closeSaveFile();
    bool writeRecivePos2Txt(const RecivePos oneRecivePos, MatrixXd *Qmat=NULL);// The result of calculating ENU direction is written to TXT
    bool writePPP2Txt(const QVector<SatlitData> epochSatlite);// Corrections to errors are written to. PPP file
    bool writeClockZTDW2Txt(const ClockData epochZTDWClock);// Write zenith wet delay and clock difference to TXT first column wet delay second column clock difference
    bool writeAmbiguity2Txt(QString fload_path);// Write the satellite stored in all Ambiguity variable into TXT file named "G32.txt", "C02.txt", "R08.txt" and so on. The first column is ambiguity.
    bool WriteEpochPRN(QString fload_path, QString tempfileName);// Write the satellite stored in variable allAmbiguity to the file, the first column epoch, the second column satellite number
    bool writeRecivePosKML(QString fload_path, QString tempfileName);// generate KML format
    bool writeBadSatliteData(const SatlitData oneSatAmb);// Write the bad satellite stored in variable allBadSatlitData to the file
private:
    bool isDirExist(QString fullPath);
    bool WriteAmbPRN(QString temp_floder, int PRN,char SatType);// Fuzziness of Writing to PRN Satellite
    int findFilenameFlag(QString Filename);
    QFile m_QFileVct[MAX_FILE_WRITE];
    QTextStream m_QTextStreamVct[MAX_FILE_WRITE];
    QString  m_QFileNames[MAX_FILE_WRITE];
    int m_save_filenum;
    bool m_ZTDW_Clock_head;


// Variable part
public:
    QVector< RecivePos > allReciverPos;// Save the position result to TXT KML
    QVector< QVector < SatlitData > > allPPPSatlitData;// Save the data calculated by PPP to write to. PPP file
    QVector< Ambiguity> allAmbiguity;// Ambiguity of Storage Satellite
    QVector< ClockData > allClock;// Store Clock Data
    QVector< VectorXd > allSolverX;// Storage Solution X
    QVector< MatrixXd > allSloverQ;// Storage Covariance Matrix
    QVector< SatlitData > allBadSatlitData;// Storage Elimination Satellites
private:
    QCmpGPST m_qcmpClass;// Function Computing Library Class
};

#endif
