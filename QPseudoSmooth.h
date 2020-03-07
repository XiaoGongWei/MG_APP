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

#ifndef QPSEUDOSMOOTH_H
#define QPSEUDOSMOOTH_H
#include "QGlobalDef.h"

// add by xiaogongwei 2018.11.20
class QPseudoSmooth
{
public:
    enum SmoothMooth
    {
        Hatch = 0
    };
    QPseudoSmooth();
    bool SmoothPesudoRange(QVector < SatlitData > &prevEpochSatlitData, QVector < SatlitData > &epochSatlitData);
    bool isSatChanged(const QVector<SatlitData> &preEpoch, const QVector<SatlitData> &currEpoch, QVector< int > &changePrnFlag);
private:
    double m_wa, m_wb;// soomth paramter
    SmoothMooth m_method;
};

#endif // QPSEUDOSMOOTH_H
