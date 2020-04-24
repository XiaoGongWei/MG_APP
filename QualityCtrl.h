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
#ifndef QUALITYCTRL_H
#define QUALITYCTRL_H
#include "QGlobalDef.h"


using namespace Eigen;

class QualityCtrl
{
public:
    QualityCtrl();
    // use clk detect gross error
    bool VtPVCtrl_CLK(QVector < SatlitData > &epochSatlitData, double *predict_pos, VectorXd &del_flag);
    bool VtPVCtrl_CLKA(QVector < SatlitData > &epochSatlitData, double *predict_pos);
    // mat_B * X = mat_L; mat_P; del_flag store delete erro flag
    bool VtPVCtrl_Filter_LC(MatrixXd mat_B, VectorXd vec_L, VectorXd vec_X, VectorXd &del_flag, int sat_len, double *LP_threshold = NULL);
    bool VtPVCtrl_Filter_C(MatrixXd mat_B, VectorXd vec_L, VectorXd vec_X, VectorXd &del_flag, int sat_len);
    bool VtPVCtrl_C(MatrixXd mat_B, VectorXd vec_L, MatrixXd mat_P, VectorXd &del_flag, int sat_len);
    bool VtPVCtrlA_C(MatrixXd mat_B, VectorXd vec_L, MatrixXd mat_P, VectorXd &del_flag, int sat_len);
    bool solver_LS(MatrixXd mat_B, VectorXd vec_L, MatrixXd mat_P, VectorXd del_flag, VectorXd &vec_X);
    bool deleteMat(MatrixXd &mat_B, VectorXd del_cols, VectorXd del_rows);// delete Matrix Rows
    bool addZeroMat(MatrixXd &mat_B, int add_row_index, int add_col_index);
private:
    void Obtaining_equation(QVector< SatlitData > &currEpoch, double *ApproxRecPos, MatrixXd &mat_B, VectorXd &Vct_L, MatrixXd &mat_P, bool isSmoothRange = false);
    void sort_vec(const VectorXd& vec, VectorXd& sorted_vec,  VectorXi& ind);
};

#endif // QUALITYCTRL_H
