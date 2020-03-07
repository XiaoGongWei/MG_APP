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

#ifndef QTROPDELAY_H
#define QTROPDELAY_H

#include "QGlobalDef.h"

/*
1. Description:
This class includes the GPT2 UNB3M model and the methods of obtaining GPT2 and GPT parameters.
Tropospheric model: Sass model, Hofield model,
Tropospheric Projection Function Model: Neill, VMF1, GMF
# ifdef EIGEN_CORE_H (Eigen library flag, no Eigen automatic masking UNB3M model)
...
#endif

2. Examples:
(1) Class initialization:
QTropDelay tropDelay;// No GPT2-related models (only UNB3M models)
QTropDelay tropDelay("D:\\gpt2_5.grd","Neil","GPT2")The GPT2-related model must have a gpt2_5.grd file path, and the latter two can choose neither default Neil nor UNB3M

All data needs to be read before using GPT2 and its associated Sass and Hofile models
tropDelay.getAllData();

(2) GPT2 + Sass model (VMF1 projection can be set when the default projection function Neil is initialized):
double MJD = 852221.0,E = 0.72,pBLH[3] ={0.3,1.5,88.3};// Reduced Julian day, altitude angle (rad), B longitude (rad), L latitude (rad) of BLH, H (m) of BLH.
int TDay = 132;// DOY
double TropDelay = 0,mf = 0;// Tropospheric dry delay (m), projection function (m)
TropDelay = tropDelay.getGPT2SasstaMDelay(MJD,TDay,E,pBLH,&mf);// GPT2+Sass calculation results are saved to TropDelay and MF

(3) UNB3M model, projection function Neil:
int TDay = 132;//DOY
double pBLH[3] ={0.3,1.5,88.3},E = 0.72;// BLH (rad, rad, m) and satellite altitude angle E (rad)
double TropDelay = 0,mf = 0;// Tropospheric dry delay (m), projection function (m)
TropDelay = tropDelay.getUNB3mDelay(pBLH, TDay, E,&mf);

(4) GPT parameter acquisition:
double MJD = 852221.0,Lat = 0.3,Lon = 1.5,Hell = 88.3;// MJD reduces Julian day, Lat is B longitude (rad) of BLH, Lon is L latitude (rad) of BLH, Hell is H (m) of BLH.
GPT2Result tempGPT2;// Computational results
tempGPT2 = tropDelay.getGPT2Model(MJD,Lat,Lon,Hell);// Computing GPT2 to GPT2 Result Structure Preservation

double GPTresult[3] = {0};// Storage of pressure (hPa) temperature (C) Geoid undulation (m) (the first two are useful)
tropDelay.getGPTModel(MJD,Lat,Lon,Hell，GPTresult);// Calculate GPT to GPTresult variable save
(5) Tropospheric projection function
double MJD = 852221.0,pBLH[3] ={0.3,1.5,88.3},E = 0.72;
double md = 0,mw = 0;// Projection Functions of Dry (md) and Wet (mw)
tropDelay.getGMFParm(MJD,pBLH,E,&md,&mw);// Computing GMF projection function to MD and MW

GPT2Result tempGPT2;// Computational results
int TDay = 132;// Year by year
tempGPT2 = tropDelay.getGPT2Model(MJD,pBLH[0],pBLH[1],pBLH[2]);
getVMF1Parm(tempGPT2.ah,tempGPT2.aw,E,pBLH[0],pBLH[2],TDay,&md,&mw);// Computing VMF1 projection function to MD and MW

tropDelay.getNeilParm(E,pBLH[2],pBLH[0],TDay,&md,&mw);// Computing Neil projection function to MD and MW

3. Note: The tropospheric delays returned in this class are only dry delays (if you need to make simple changes to the "tail" of the corresponding function with exclamation marks)
4. Reference website: http://ggosatm.hg.tuwien.ac.at/DELAY/SOURCE/(including MATLAB and data)

*/

#define d2r  (Pi/180.0)
#define r2d  (180.0/Pi)

#define M_MIN(x,y)    ((x)<(y)?(x):(y))
#define M_SQRT(x)     ((x)<=0.0?0.0:sqrt(x))

typedef struct _GPT2Result
{// Preserving the results of GPT2
	double p;//pressure in hPa
	double T;//temperature in degrees
	double dT;//temperature lapse rate in degrees per km
	double e;// water vapour pressure in hPa
	double ah;//hydrostatic mapping function coefficient at zero height (VMF1)
	double aw;//wet mapping function coefficient (VMF1)
	double undu;//geoid undulation in m
}GPT2Result;

typedef struct _GrdFileVar
{// Store GRD file data for calculation
	double lat;//latitude
	double lon;//lontitude
	double pgrid[5];//pressure in Pascal
	double Tgrid[5];// temperature in Kelvin
	double Qgrid[5];//specific humidity in kg/kg
	double dTgrid[5];//temperature lapse rate in Kelvin/m
	double u;//geoid undulation in m
	double Hs;//orthometric grid height in m
	double ahgrid[5];//hydrostatic mapping function coefficient, dimensionless
	double awgrid[5];// wet mapping function coefficient, dimensionless
}GrdFileVar;

class QTropDelay
{
// Functional part
public:
    QTropDelay();
    ~QTropDelay(void);
    void setTropFileNames(QString GrdFileName = "", QString ProjectionFun = "GMF", QString tropMode = "UNB3m");
    // The following two functions are the tropospheric model estimated by GPT2 (need to read. GRD file)
    double getGPT2SasstaMDelay(double MJD, int TDay, double E, double *pBLH, double *mf = NULL, double *ZPD = NULL, double *tZHD = NULL);// Obtain the zenith ZHD+ZWD total delay MJD of GPT2+Sassta+VMF1: Simplified Julian T-year cumulative day Lat, lon, Hell: longitude and latitude, station height T: annual cumulative day E altitude angle (rad), * mf: projection function
    double getGPT2HopfieldDelay(double MJD, int TDay, double E, double *pBLH, double *mf = NULL, double *ZPD = NULL, double *tZHD = NULL);// Obtain the zenith ZHD+ZWD total delay MJD of GPT2+Hopfield+VMF1: Simplified Julian T-year cumulative day Lat, lon, Hell: longitude and latitude, station height T: annual cumulative day E altitude angle (rad), * mf: projection function
    // The following function is the tropospheric model estimated by UNB3 (no need to read. GRD files)
#ifdef EIGEN_CORE_H
    double getUNB3mDelay(double *pBLH, double TDay, double E, double *mf = NULL, double *ZPD = NULL, double *tZHD = NULL);
#endif
    // Here's how to get GPT2 and GPT parameters
    GPT2Result getGPT2Model(double dmjd,double dlat,double dlon,double hell,double it = 0);// Using GPT2 to calculate the detailed input, see the function body it:1 for no time change 0:time change
    void getGPTModel(double dmjd,double dlat,double dlon,double hell,double *GPTdata);// GPT data [3] is calculated using GPT and stored in the gas pressure (hPa) temperature (C) ellipsoid height difference (m).
    // Here's how to get the projection function
    void getVMF1Parm(double ah,double aw,double E,double Lat,double H,int TDay,double &md,double &mw);// Calculating VMF1 dry weight (md) and wet weight (mw) projection function E as elevation angle (rad) H as positive height (m) T as annual accumulated day Lat as geodetic latitude (rad north latitude greater than zero south latitude less than zero) awbw from GPT 2 model estimation
    void getNeilParm(double E,double H,double Lat,int TDay,double &md,double &mw);// Calculating Neil dry (md) and wet (mw) projection function E as elevation angle (rad) H as high (m) pH as terrestrial latitude (rad) T-year product day
    void getGMFParm(double MJD,double *pBLH,double E,double &md,double &mw);// MJD: Reduced Julian Day, pBLH Geodetic Coordinate System (rad, rad, m), E as Projection Function of Height Angle (rad) Dry Quantity (md) and Moisture Quantity (mw)
    // Sasssta model of experience (zenith direction)
	double getSassDelay(double &ZHD,double &ZWD,double B, double H,double E);
    // Experience Hopfield Model (Line of Sight)
    double getHopfieldDelay(double &SD,double &SW, double H,double E);// Empirical model is used to calculate the direction of sight (H: BLH, unit m; E: satellite altitude angle, unit (radian)).
    void getAllData();// Reading all data can only be computed if the data is read (it can be read when the program is initialized)
private:
	void initVar();
	bool openGrdFile(QString GrdFileName);
    void readGrdFile(QString grdFileName);// Read GRD file data for calculation
    GPT2Result HopfieldDelay(double &ZHD,double &ZWD,double dmjd,double dlat,double dlon,double hell,double it = 0);// Using GPT2 estimation + Hopfield (zenith direction) it:1 to indicate no time change 0:time change
    GPT2Result SassstaMDelay(double &ZHD,double &ZWD,double dmjd,double dlat,double dlon,double hell,double it = 0);// Using GPT2 estimation + simplified Sastamoinen model (zenith direction) it:1 indicates no time change 0:time change
	void trop_map_gmf(double dmjd,double dlat,double dlon,double dhgt,double zd,double *gmfh,double *gmfw);
	void trop_gpt(double dmjd,double dlat,double dlon,double dhgt,double *pres,double *temp,double *undu);
    int ipow(int base,int exp);// Called by trop_gpt
#ifdef EIGEN_CORE_H
    VectorXd UNB3M(Vector3d &BLH, double DAYOYEAR, double ELEVRAD);// UNB3 Model (Dependent on Eigen Library)
#endif
// Variable part
public:

private:
    // Neil model parameters are defined below
    // Average dry component and fluctuation amplitude table of ad, bd, CD 15-75 degrees
	double lat[5];
	double Avgad[5],Avgbd[5],Avgcd[5];
	double Ampad[5],Ampbd[5],Ampcd[5];
    // Define the wet component coefficient table below
	double Avgaw[5],Avgbw[5],Avgcw[5];
    double m_PI;// circumference
	QString m_GrdFileName;
	QFile m_ReadFileClass;
	QVector< GrdFileVar > m_allGrdFile;
	QString tempLine;
	bool isReadAllData;
    bool isGPT2;// Determine whether it is a GPT2 model: 1 or choose GPT:0
    QString m_ProjectionFun;// Selected projection function
	int m_ProjectFunFlag;//1:Neil 2:VMF1 3:GMF
};

#endif

