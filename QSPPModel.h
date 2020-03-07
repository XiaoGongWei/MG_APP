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
**************************************************************************
**           Author: XiaoGongWei
**   Website: github.com/xiaogongwei/MG_APP
** Download link (The GPS Toolbox): https://www.ngs.noaa.gov/gps-toolbox/
**             Date: 06.02.2020
****************************************************************************/

#ifndef QSPPMODEL_H
#define QSPPMODEL_H

#include "QCmpGPST.h"
#include "QReadOFile.h"
#include "QReadGPSN.h"
#include "QTropDelay.h"
#include "QKalmanFilter.h"
#include "QWrite2File.h"
#include "SRIFAlgorithm.h"
#include "QualityCtrl.h"
#include "QPseudoSmooth.h"
#include "FtpClient.h"
#include "MyCompress.h"
#include "QReadAnt.h"

// next include model is for GUI
#include <QTextEdit>
#include <QApplication>
#include <QTextCursor>
#include <QTime>
#include <QCoreApplication>

// use ionospheric free SPP model

class QSPPModel:public QBaseObject
{
// function part
public:
    // Configure PPP parameters
    QSPPModel(QString files_path, QTextEdit *pQTextEdit = NULL, QString Method = "Kalman", QString Satsystem = "G",
              QString TropDelay = "Sass", double CutAngle = 10, bool isKinematic = false, QString Smooth_Str = "Smooth", QString SPP_Model = "P_IF");
    ~QSPPModel();
    void initSPPModel(QString OFileName);
    void Run(bool isDisplayEveryEpoch = true);//isDisplay Every Epoch represent is disply every epoch information?(ENU or XYZ)
    // set SystemStr:"G"(turn on GPS system); GR":(turn on GPS+GLONASS system);" GRCE"(all turned on), etc
    bool setSatlitSys(QString SystemStr);// The letters G,R,C and E are used for GPS,GLONASS,BDS and Galieo respectively
// next public function is for GUI
    // configure model
    void setConfigure(QString Method = "Kalman", QString Satsystem = "G", QString TropDelay = "Sass", double CutAngle = 10,
                      bool isKinematic = false, QString Smooth_Str = "Smooth", QString SPP_Model = "P_IF");
    // Get operation results( clear QWrite2File::allPPPSatlitData Because the amount of data is too large.)
    void getRunResult(PlotGUIData &plotData);
    bool isRuned(){return m_isRuned;}
private:
    void initVar();
    void getNAVPos(int Year, int Month, int Day, int Hours, int Minutes, double Seconds, double signal_transmission_time,
                   int PRN, char SatType, double *p_XYZ, double *StaClock, double *pdXYZ = NULL);//YMDHMS satellite p_XYZ returns WGS84 coordinates and velocity in seconds per week
    void getSatEA(double X,double Y,double Z,double *approxRecvXYZ,double *EA);// Calculate height Angle and azimuth EA
    double getSagnac(double X,double Y,double Z,double *approxRecvXYZ);// the autobiography of the earth(m)
    double getRelativty(double *pSatXYZ,double *pRecXYZ,double *pSatdXYZ);// Calculate relativistic effects
    void getWight(SatlitData &tempSatlitData);// obtain weights of different satellite systems
    void getTropDelay(double MJD,int TDay,double E,double *pBLH,double *mf = NULL, double *ZHD_s = NULL, double *ZPD = NULL, double *ZHD = NULL);// MJD: simplified Julian day, TDay: annual product day, E: altitude Angle (rad) pBLH: geodetic coordinate system, *mf: projection function
    bool CycleSlip(const SatlitData &oneSatlitData,double *pLP);// cycle slip detection. The pLP is a three-dimensional array. The first is the w-m combination (n2-n1 < 5), the second is the ionospheric residual (<0.3), and the third is (lamt2* n2-lamt1 *N1 < 5).
    double getPreEpochWindUp(QVector < SatlitData > &prevEpochSatlitData,int PRN,char SatType);// WindUp of previous epoch, does not return 0
    // the satellites with high quality are obtained, including: whether the cycle skip height Angle data are missing, c1-p2 <50 adjacent epoch WindUp < 0.3; EpochSatlitData: previous epochSatlitData: current epochSatlitData (automatically delete low-quality satellite); EleAngle: high Angle
    void getGoodSatlite(QVector < SatlitData > &prevEpochSatlitData,QVector < SatlitData > &epochSatlitData,double eleAngle = 10);
    void saveResult2Class(VectorXd X, Vector3d spp_vct, GPSPosTime epochTime, QVector< SatlitData > epochResultSatlitData, int epochNum, MatrixXd *P = NULL);// save Filter Result to m_writeFileClass (QWrite2File)
    void writeResult2File();// save the class m_writeFileClass (QWrite2File) to the file
    QStringList searchFilterFile(QString floder_path, QStringList filers);// serch files by filter
    void SimpleSPP(QVector < SatlitData > &prevEpochSatlitData, QVector < SatlitData > &epochSatlitData, double *spp_pos);// spp for Few corrections
    void Obtaining_equation( QVector< SatlitData > &currEpoch, double *ApproxRecPos, MatrixXd &mat_B, VectorXd &Vct_L, MatrixXd &mat_P, bool isSmoothRange = false);// get Observation equation B*X = L~(P)
    void reciveClkRapaire(QVector< SatlitData > &prevEpochSatlitData, QVector< SatlitData > &epochSatlitData);
    // next function is for GUI
    void autoScrollTextEdit(QTextEdit *textEdit,QString &add_text);
    // next function is for FTP
    bool connectHost();
    QString downNavFile(QString store_path = "./", int Year = 0, int doy = 0);
// data section
public:

private:
    QTextEdit *mp_QTextEditforDisplay;
    QString m_OFileName;// O file path + file name
    QString m_run_floder;// run the specified directory file
    QString m_App_floder;// executable directory
    QString m_Solver_Method;// m_Solver_Method value can be "SRIF" or "Kalman"
    bool m_isKinematic;// judge is Kinematic
    bool m_isInitSPP; // judge is init possition use SimpleSPP
    double m_ApproxRecPos[3];// Approximate coordinates of the station
    double m_CutAngle;// cut-off height Angle (degree)
    QString m_SatSystem;// for GPS,GLONASS,BDS, and Galieo, use the letters G,R,C, and e. to set the file system m_SatSystem:"G"(turn on the GPS system); GR":(turn on GPS+GLONASS system);" GRCE"(all turned on), etc
    QString m_TropDelay;// the tropospheric model m_TropDelay can be selected as Sass, hopfield and UNB3m
    QString m_sys_str;// satellite system for short ('G', 'R', 'C', 'E')
    bool m_isSmoothRange;// Whether to use phase smoothing pseudo-distance for SPP
    int m_sys_num;// number of satellite systems
    int multReadOFile;// each buffer O file epoch metadata (the larger the number of memory occupied, the higher the speed is relatively fast... The default 1000)
    int m_leapSeconds;// get leap seconds
    bool m_haveObsFile;// jugue have obsvertion file
    bool m_isRuned;// Determine whether the operation is complete.
    int FlagN;
    bool m_isConnect;// determine whether the network is connected
    // various libraries are used to calculate error correction, kalman filtering and file operation
    QCmpGPST qCmpGpsT;// function library for calculating GPS time, coordinate transformation, etc
    QReadOFile m_ReadOFileClass;// reads the O file class
    QTropDelay m_ReadTropClass;// reading the troposphere requires files
    QKalmanFilter m_KalmanClass;// kalman filtering
    SRIFAlgorithm m_SRIFAlgorithm;
    QWrite2File m_writeFileClass;// write to the file class
    QualityCtrl m_qualityCtrl;// Quality control class
    QPseudoSmooth m_QPseudoSmooth;// smoothed Pesudorange
    QReadGPSN m_QReadGPSN;// read nav data
    FtpCLient ftpClient;// ftp Dowload
    // for save plot image
    QString m_save_images_path;
    bool m_iswritre_file;
    int m_clock_jump_type;// 1 is Pseudo range jump, 2 is carrier jump
    // min flag
    int m_minSatFlag;// the minimum number of satellites required

};


#endif // QSPPMODEL_H
