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

#ifndef SRIFALGORITHM_H_
#define SRIFALGORITHM_H_

#include "QGlobalDef.h"
#include "QualityCtrl.h"
#include "MyMatrix.h"
#include <Eigen/Dense>
#include <iostream>
using namespace std;
using namespace Eigen;


class SRIFAlgorithm:public QBaseObject
{
public:
    enum SRIF_MODEL    {
        SPP_STATIC = 0,
        SPP_KINEMATIC = 1,
        PPP_STATIC = 2,
        PPP_KINEMATIC = 3
    };
    enum SRIF_SMOOTH_RANGE    {
        NO_SMOOTH = 0,
        SMOOTH = 1
    };

    SRIFAlgorithm();
	virtual ~SRIFAlgorithm();
// Process GNSS
    // use SRIF Algorithm to GNSS
    bool SRIFforStatic(QVector< SatlitData > &preEpoch, QVector< SatlitData > &currEpoch, double *m_ApproxRecPos, VectorXd &X, MatrixXd &P);

	// init prior matrix and transition matrix
	void InitSRIF(MatrixXd &Rp, MatrixXd &Zp, MatrixXd &Phi, MatrixXd &G, MatrixXd &Rwk_1);
	// only input observation to SRIF Filter
	VectorXd SRIFilter(MatrixXd &A, MatrixXd &L);
    // get init X and m_Xk
    inline VectorXd getInitXk() { return m_init_Xk; }
    inline VectorXd getXk() { return m_Xk; }
    inline MatrixXd getQk() {MatrixXd Qk; getQ(Qk);}

    // set some configure
    void setModel(SRIF_MODEL model_type);
    inline SRIF_MODEL getModel() {return m_SRIF_MODEL;}
    inline void setSmoothRange(SRIF_SMOOTH_RANGE smooth_range) {m_SRIF_SMOOTH_RANGE = smooth_range;}
    inline SRIF_SMOOTH_RANGE getSmoothRange() {return m_SRIF_SMOOTH_RANGE;}

// SRIF Algorithm
	//set transition matrix
	void setPhi(MatrixXd &newPhi) { this->m_Phi = newPhi; this->m_Phi_Inv = newPhi.inverse();}
	//set inverse of transition matrix
	void setPhi_Inv(MatrixXd &newPhiInv) { this->m_Phi_Inv = newPhiInv; }
    void setRwk_1(MatrixXd &newRwk_1) { this->m_Rwk = newRwk_1; }
	void setG(MatrixXd &newG) { this->m_G = newG; }
	void getQ(MatrixXd &Q){ Q =  (this->m_Rp.transpose()*this->m_Rp).inverse();}
private:
	// initVar
	void initVar();
	// The most primitive use SRIF Factorization Matrix solve Least squre
	void SRIFMeasureUpdate(MatrixXd &Rp, MatrixXd &Zp, MatrixXd &A, MatrixXd &L);
	// use SRIF Factorization Matrix update Time
    void SRIFTimeUpdate(MatrixXd &Rp, MatrixXd &Zp, MatrixXd &Phi_Inv, MatrixXd &G,
            MatrixXd *Rwk_1 = NULL, MatrixXd *Rwk = NULL,  MatrixXd *Rwx = NULL, MatrixXd *Zw = NULL);
	// QR Factorization (Eigen)
	void QRDecompose(MatrixXd &eigenMat, MatrixXd &R);
	// Gauss factorization back generation
	void gaussBackGen(MatrixXd &upTri, MatrixXd &L, VectorXd &Y);

    // use to GNSS
    void initSRIFPara(QVector< SatlitData > &currEpoch,MatrixXd &B,VectorXd &L);
    void changeSRIFPara( QVector< SatlitData > &epochSatlitData,QVector< int >oldPrnFlag, int preEpochLen);
    void preWhiteMatrix(MatrixXd &matB, MatrixXd &matL, MatrixXd &whiteMat, MatrixXd *matP = NULL);
    // for Kinematic
    void ls_solver(QVector< SatlitData > &currEpoch, double *m_ApproxRecPos);
    void Obtaining_equation( QVector< SatlitData > &currEpoch, double *m_ApproxRecPos, MatrixXd &mat_B, VectorXd &Vct_L,
                                 MatrixXd &mat_P);
    // The residual after SRIF filtering is used as gross error detection, and there exists Gross Error Cyclic filtering kickout.
    bool isSatelliteChange(QVector< SatlitData > &preEpoch,QVector< SatlitData > &currEpoch, QVector< int > &oldPrnFlag);
    void updatePk(QVector< SatlitData > &currEpoch, int B_len);// update Rk(Observation Covariance)
    void filter(QVector< SatlitData > &preEpoch, QVector< SatlitData > &currEpoch, VectorXd &X, MatrixXd &P);

// Variable
private:
	MatrixXd m_Rp, m_Zp, m_Phi, m_Phi_Inv,
             m_G, m_Q, m_Rwk;
    VectorXd m_Xk, m_Yk, m_init_Xk;
	bool m_initSRIF;
    // use to GNSS
    bool m_isInitPara, m_VarChang, m_isInitWhite, m_isKinematic;
    MatrixXd m_Pk;
    MyMatrix m_matrix;
    // for kinematic
    double m_SPP_Pos[3];
    QualityCtrl m_qualityCtrl;
    SRIF_MODEL m_SRIF_MODEL;
    SRIF_SMOOTH_RANGE m_SRIF_SMOOTH_RANGE;
    int m_const_param;// Invariant parameters in filtering
    int m_sys_num;
    QString m_sys_str;
    double m_LP_whight;// Carrier and Pseudo Range Weight Ratio
};

#endif /* SRIFALGORITHM_H_ */
