/*************************************************************************
**
**  MG-APPS----Multi-GNSS-Automatic Precise Positioning Software
**
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


#ifndef MYCOMPRESS_H
#define MYCOMPRESS_H

#include <QByteArray>
#include <QString>
#include <QDir>
#include <QCoreApplication>
#include <QProcess>

#if defined(__linux__)
// Linux
static bool MYCOMPRESS_H_isLiux = true;
static QString COMPRESSPATHSEG= "/";
#elif defined(_WIN32)
// Windows
static bool MYCOMPRESS_H_isLiux = false;
static QString COMPRESSPATHSEG = "/";
#endif

class MyCompress
{
public:
    MyCompress();
    bool UnCompress(QString unCompressFile, QString unCompress_floder);

private:
    bool isDirExist(QString fullPath);
    QString m_App_floder;



};

#endif // MYCOMPRESS_H
