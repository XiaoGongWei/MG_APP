/*************************************************************************
**
**  MG-APP----Multi-GNSS-Automatic Precise Positioning Software
**  Copyright (C) 2016-2019 XiaoGongWei
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
**************************************************************************
**           Author: XiaoGongWei
**  Website/Contact: http://github.com/xiaogongwei
**             Date: 26.04.2019
****************************************************************************/

#ifndef QBATCHPROCESS_H
#define QBATCHPROCESS_H

#include "QPPPModel.h"
#include "QPPPBackSmooth.h"


/*
 *illustrate:
 * ObsFiles_Path store multiply obsevation data. We will make " ObsFiles_Path " dir in ObsFiles_Path.
 *
 * Example:
 * QString ObsFiles_Path = "C:\\OFiles\\";
 * QBatchProcess process_stations;
 * process_stations.Run(ObsFiles_Path);
 *
 */

class QBatchProcess
{
public:
    QBatchProcess(QString files_path, QTextEdit *pQTextEdit = NULL, QString Method = "Kalman", QString Satsystem = "G", QString TropDelay = "Sass",
                  double CutAngle = 10, bool isKinematic = false, QString Smooth_Str = "NoSmooth", bool isBackBatch = false);
    ~QBatchProcess();
    bool Run(bool isDisplayEveryEpoch = false);//isDisplayEveryEpoch represent is disply every epoch information?(ENU or XYZ)
    void getStoreAllData(QVector<PlotGUIData> &all_SationData);
    QStringList getStationNames();
    bool isRuned(){ return m_isRuned; }
private:
    bool isDirExist(QString fullPath);
    bool distribute(QString ofile_path, QString destin_floder);
    bool isFileExist(QString fullFileName);
    void autoScrollTextEdit(QTextEdit *textEdit,QString &add_text);
private:
    QString m_mkdir_name;
    QTextEdit *mp_QTextEdit;
    QString M_ObsFiles_Path;// M_ObsFiles_Path is a floder which store multiply .O files
    QStringList m_AllStations;// all Stations names
    QVector< PlotGUIData > m_AllStationsData;// use pointer store for External data
    bool m_isRuned;
    // configure flag
    QString m_Method;
    QString m_TropDelay;
    QString m_Satsystem;
    QString m_Smooth_Str;
    bool m_isKinematic;
    bool m_isBackBatch;
    double m_CutAngle;

};

#endif // QBATCHPROCESS_H
