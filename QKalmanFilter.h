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

#ifndef QKALMANFILTER_H
#define QKALMANFILTER_H


// Kalman filter class, considering the satellite replacement, initialization of Kalman parameters

#include "QGlobalDef.h"
#include "QualityCtrl.h"
#include "MyMatrix.h"

class QKalmanFilter:public QBaseObject
{
// function part
public:
    enum KALMAN_MODEL    {
        SPP_STATIC = 0,
        SPP_KINEMATIC = 1,
        PPP_STATIC = 2,
        PPP_KINEMATIC = 3
    };
    enum KALMAN_SMOOTH_RANGE    {
        NO_SMOOTH = 0,
        SMOOTH = 1
    };

    enum KALMAN_FILLTER {
        KALMAN_STANDARD = 0
    };

    QKalmanFilter();
	~QKalmanFilter(void);
    void initVar();// Initialize some parameters
    //F: state transition matrix, Xk_1: previous filtering value, Pk_1: previous filtering error matrix, Qk_1: previous state transition noise matrix, Bk: observation matrix,
    //Rk: observation noise matrix, Lk: observation vector
	void KalmanforStatic(MatrixXd Bk,VectorXd Lk,MatrixXd F,MatrixXd Qw,MatrixXd Rk,VectorXd &Xk_1,MatrixXd &Pk_1);
    bool KalmanforStatic(QVector< SatlitData > &preEpoch, QVector< SatlitData > &currEpoch, double *m_ApproxRecPos, VectorXd &Xk_1, MatrixXd &Pk_1);
    // some get data simple function
    inline VectorXd getInitXk() { return m_init_Xk; }
    inline VectorXd getXk() { return m_Xk_1; }
    inline MatrixXd getQk() {return m_Pk_1;}

    // set some configure
    void setModel(KALMAN_MODEL model_type);
    inline KALMAN_MODEL getModel() {return m_KALMAN_MODEL;}
    inline void setSmoothRange(KALMAN_SMOOTH_RANGE smooth_range) {m_KALMAN_SMOOTH_RANGE = smooth_range;}
    inline KALMAN_SMOOTH_RANGE getSmoothRange() {return m_KALMAN_SMOOTH_RANGE;}
    inline void setFilterMode(KALMAN_FILLTER filter_mode) {m_KALMAN_FILLTER = filter_mode;}
    inline KALMAN_FILLTER getFilterMode() {return m_KALMAN_FILLTER;}
private:
    void printMatrix(MatrixXd mat);// print matrix Debug
    void initKalman(QVector< SatlitData > &currEpoch,MatrixXd &B,VectorXd &L);// kalman initialization
	void changeKalmanPara(QVector< SatlitData > &epochSatlitData,QVector< int >oldPrnFlag );
    void Obtaining_equation( QVector< SatlitData > &currEpoch, double *m_ApproxRecPos, MatrixXd &mat_B, VectorXd &Vct_L,
                             MatrixXd &mat_P);// get observation equation
    void ls_solver(QVector< SatlitData > &currEpoch, double *m_ApproxRecPos);// use least square method solver B*X = L
    // the residual error after Kalman filtering is used as the gross error detection, and the circular filtering with gross error is kicked out.
    bool isSatelliteChange(QVector< SatlitData > &preEpoch,QVector< SatlitData > &currEpoch, QVector< int > &oldPrnFlag);
    void updateRk(QVector< SatlitData > &currEpoch, int B_len);// update Rk(Observation Covariance)
    void filter(QVector< SatlitData > &preEpoch, QVector< SatlitData > &currEpoch, VectorXd &X, MatrixXd &P);
// variable section
public:

private:
    bool isInitPara;// determines whether the first epoch is initialized
	MatrixXd m_Fk_1,m_Pk_1,m_Qwk_1,m_Rk_1;
    VectorXd m_Xk_1, m_init_Xk;// are dX,dY,dZ,dT(zenith tropospheric residual),dVt(receiver clock difference), N1,N2... Nm(ambiguity)
    bool m_VarChang;// marks whether the matrix changes in the next filtering period
    MyMatrix m_matrix;// print matrix to file
    double m_SPP_Pos[3];
    QualityCtrl m_qualityCtrl;
    KALMAN_MODEL m_KALMAN_MODEL;
    KALMAN_SMOOTH_RANGE m_KALMAN_SMOOTH_RANGE;
    KALMAN_FILLTER m_KALMAN_FILLTER;
    int m_const_param;// Invariant parameters in filtering
    int m_sys_num;
    QString m_sys_str;
    double m_LP_whight;// Carrier and Pseudo Range Weight Ratio
};

#endif
