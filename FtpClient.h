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


#ifndef FTPCLIENT_H
#define FTPCLIENT_H

#include <QObject>
#include <QFile>
#include <QUrl>
#include <QByteArray>
#include <QDir>
#include <QDebug>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QEventLoop>

/*
Example:
FtpCLient clicent;
clicent.FtpSetUserInfor("david","xiaogongwei");//if you have account or anonymous
clicent.FtpSetHostPort("127.0.0.1");
clicent.FtpGet("/Documents/notify_minutes.txt", "./xiao_ftp.txt");
clicent.FtpPut("./X_epoch_2879.csv", "/Documents/xiao_ftp.csv");
*/

class FtpCLient:public QObject
{
    Q_OBJECT
public:
    FtpCLient();
    void FtpSetUserInfor(QString user, QString pwd);
    void FtpSetHostPort(QString host_str, int port = 21);
    bool FtpGet(QString net_file_path, QString local_file_path);
    bool FtpPut(QString local_file_path, QString net_file_path);
    bool downloadHTTPFile(QString net_file_path, QString local_file_path);
    bool pushData2Http(QString http_url, char *json_format);

private:
    bool isExistFile(QString file_path);// file_path: filename, such as c:\\user\\xiao.pdf

signals:


protected slots:
    void replay_finished();
    void ftp_error(QNetworkReply::NetworkError net_error);
    // Upload progress
    void ftp_uploadProgress(qint64 bytesSent, qint64 bytesTotal);
    // Download progress
    void ftp_downloadProgress(qint64 bytesReceived, qint64 bytesTotal);
    void http_finished();

private:
    QFile *m_pFile;
    QNetworkReply *m_pReply;
    QNetworkAccessManager *m_pManager;
    QUrl *m_pUrl;
    bool m_isDownload;
    bool m_isPush;

};


#endif // FTPCLIENT_H
