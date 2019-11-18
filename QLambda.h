/*************************************************************************
**
**  MG-APP----Multi-GNSS-Automatic Precise Positioning Software
**
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

#ifndef QLAMBDA_H
#define QLAMBDA_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdarg.h>

#include <Eigen/Dense>
using namespace Eigen;

/*------------------------------------------------------------------------------
* lambda.c : integer ambiguity resolution
*
*          Copyright (C) 2007-2008 by T.TAKASU, All rights reserved.
*
* reference :
*     [1] P.J.G.Teunissen, The least-square ambiguity decorrelation adjustment:
*         a method for fast GPS ambiguity estimation, J.Geodesy, Vol.70, 65-82,
*         1995
*     [2] X.-W.Chang, X.Yang, T.Zhou, MLAMBDA: A modified LAMBDA method for
*         integer least-squares estimation, J.Geodesy, Vol.79, 552-565, 2005
*
* version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
* history : 2007/01/13 1.0 new
*-----------------------------------------------------------------------------*/


/* constants/macros ----------------------------------------------------------*/

#define LOOPMAX     10000           /* maximum count of search loop */

#define SGN(x)      ((x)<=0.0?-1.0:1.0)
#define ROUND_RTKLIB(x)    (floor((x)+0.5))
#define SWAP(x,y)   do {double tmp_; tmp_=x; x=y; y=tmp_;} while (0)


class QLambda
{
public:
	QLambda(void);
	~QLambda(void);
    void QLamdaSearch(VectorXd ambfloat, MatrixXd Q_mat, MatrixXd &amb_fix, double *ratio, int out_fixamb_num = 4);

private:
    int lambda(int n, int m, const double *a, const double *Q, double *F,
                      double *s);
    int LD(int n, const double *Q, double *L, double *D);
    void gauss(int n, double *L, double *Z, int i, int j);
    void perm(int n, double *L, double *D, int j, double del, double *Z);
    void reduction(int n, double *L, double *D, double *Z);
    int search(int n, int m, const double *L, const double *D,
                      const double *zs, double *zn, double *s);
    void fatalerr(const char *format, ...);
    double *mat(int n, int m);
    double *zeros(int n, int m);
    double *eye(int n);
    int *imat(int n, int m);
    void matcpy(double *A, const double *B, int n, int m);
    int matinv(double *A, int n);
    int ludcmp(double *A, int n, int *indx, double *d);
    void lubksb(const double *A, int n, const int *indx, double *b);
    int solve(const char *tr, const double *A, const double *Y, int n,
                     int m, double *X);
    void matmul(const char *tr, int n, int k, int m, double alpha,
                       const double *A, const double *B, double beta, double *C);
};

#endif

