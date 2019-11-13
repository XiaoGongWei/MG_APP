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

#ifndef MYMATRIX_H_
#define MYMATRIX_H_

#include <iostream>
#include <fstream>
#include <iomanip>
#include <Eigen/Dense>
#include <QString>
using namespace Eigen;
using namespace std;

class MyMatrix {
public:
	MyMatrix();
	virtual ~MyMatrix();
	MatrixXd readCSV(const char *filename);
	bool writeCSV(const char *filename, const MatrixXd &mat);
	void printMatrix(const MatrixXd &mat);
    void keepMatPricision(MatrixXd &Qmat, int keepNum = 12);// Keep keepNum decimal places
    void keepMatPricision(VectorXd &Vct_v, int keepNum = 12);// Keep keepNum decimal places
};


#endif /* MYMATRIX_H_ */
