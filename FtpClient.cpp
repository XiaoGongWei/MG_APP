#include "FtpClient.h"


bool FtpClient::downloadHTTPFile(QString net_file_path, QString local_file_path)
{
    QEventLoop loop;
    QUrl tUrl = QUrl(net_file_path);
    if(isExistFile(local_file_path)) return false;
    m_pFile = new QFile(local_file_path);
    m_pFile->open(QIODevice::WriteOnly);
    QNetworkRequest request;
    request.setUrl(tUrl);
//    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/zip");

    m_pReply = m_pManager->get(request);
    connect(m_pReply, SIGNAL(finished()), this, SLOT(http_finished()));
    connect(m_pReply, SIGNAL(downloadProgress(qint64, qint64)), this, SLOT(ftp_downloadProgress(qint64, qint64)));
    connect(m_pReply, SIGNAL(error(QNetworkReply::NetworkError)), this, SLOT(ftp_error(QNetworkReply::NetworkError)));
    connect(m_pReply, SIGNAL(finished()),&loop, SLOT(quit()));
    loop.exec();
    return true;
}

/*
 * example:
 * FtpClient myftp;
 * char *json_data = "{\"datastreams\": [{\"id\": \"temp0\", \"datapoints\": [{\"at\": \"2019-06-22T10:55:18.889416\", \"value\": 0.12}]}]}";
 * QString put_url("http://api.heclouds.com/devices/531879860/datapoints");
 * myftp.pushData2Http(put_url, json_data);
*/
bool FtpClient::pushData2Http(QString http_url, char *json_format)
{
    QEventLoop loop;
    QUrl tUrl = QUrl(http_url);
    QNetworkRequest request;
    request.setUrl(tUrl);
    request.setHeader(QNetworkRequest::ContentTypeHeader, QVariant("application/json"));
    request.setRawHeader("api-key", "FOydn37wU3eESYzMuJ2=8XaeqzY=");
    QByteArray postArry(json_format);
    m_pReply = m_pManager->post(request, postArry);
    connect(m_pReply, SIGNAL(finished()), this, SLOT(http_finished()));
    connect(m_pReply, SIGNAL(finished()),&loop, SLOT(quit()));
    loop.exec();
    return true;
}

void FtpClient::http_finished()
{
    QNetworkReply *pReply = qobject_cast<QNetworkReply *>(sender());
//    QVariant status_code = pReply->attribute(QNetworkRequest::HttpStatusCodeAttribute);
//    qDebug() << "status_code: " << status_code << endl;
    switch (pReply->error())
    {
        case QNetworkReply::NoError:
           m_pFile->write(pReply->readAll());
           m_pFile->flush();
           qDebug()<< "Success!" << endl;
           break;
        case QNetworkReply::HostNotFoundError:
           qDebug()<< "Host Not Found!" << endl;
           break;
        case QNetworkReply::AuthenticationRequiredError:
           qDebug()<< "Login Failure!" << endl;
           break;
        default:
           qDebug()<< "Can not find this download File." << endl;
           break;
    }
    if(m_pFile->isOpen()) m_pFile->close();
    pReply->deleteLater();
}


FtpClient::FtpClient()
{
    m_pManager = new QNetworkAccessManager();
    m_pUrl = new QUrl();
    m_isDownload = true;
    m_isPush = true;
}

FtpClient::~FtpClient()
{

}

bool FtpClient::isExistFile(QString file_path)
{
    if(QFile::exists(file_path))
        return true;
    else
        return false;
}

void FtpClient::ftp_error(QNetworkReply::NetworkError net_error)
{
    m_isDownload = false;
    m_isPush = false;
    //Check for errors
    switch( net_error ){//Determine the status after the connection
     case QNetworkReply::NoError:
        qDebug()<< "Critical Error!" << endl;
        break;
     case QNetworkReply::HostNotFoundError:
        qDebug()<< "Host Not Found!" << endl;
        break;
     case QNetworkReply::AuthenticationRequiredError:
        qDebug()<< "Login Failure!" << endl;
        break;
     default:
        qDebug()<< "Can not find this download File." << endl;
        break;
     }
}

void FtpClient::replay_finished()
{
    QNetworkReply *pReply = qobject_cast<QNetworkReply *>(sender());
    switch (pReply->error())
    {
        case QNetworkReply::NoError :
            m_pFile->write(pReply->readAll());
            m_pFile->flush();
            break;
        default:
        break;
    }
    if(m_pFile->isOpen()) m_pFile->close();
    pReply->deleteLater();
}

//Set the FTP server username and password
void FtpClient::FtpSetUserInfor(QString user, QString pwd)
{
    m_pUrl->setUserName(user);
    m_pUrl->setPassword(pwd);
}
//Set address and port
void FtpClient::FtpSetHostPort(QString host_str, int port )
{
    m_pUrl->setHost(host_str);
    m_pUrl->setPort(port);
}
//download file
bool FtpClient::FtpGet(QString net_file_path, QString local_file_path)
{
    m_isDownload = true;
    QEventLoop loop;
    //If there is a deletion and the byte is 0
    if(isExistFile(local_file_path))
    {
        QFile file(local_file_path);
        if (file.exists() && file.size() == 0)
        {
            file.remove();
        }
    }

    m_pFile = new QFile(local_file_path);
    if(!m_pFile->open(QIODevice::WriteOnly))
    {
        qDebug()<< "can't open" << local_file_path;
        return false;
    }
    m_pUrl->setScheme("ftp");
    m_pUrl->setPath(net_file_path);
    m_pReply = m_pManager->get(QNetworkRequest(*m_pUrl));
    connect(m_pReply, SIGNAL(finished()), this, SLOT(replay_finished()));
    connect(m_pReply, SIGNAL(downloadProgress(qint64, qint64)), this, SLOT(ftp_downloadProgress(qint64, qint64)));
    connect(m_pReply, SIGNAL(error(QNetworkReply::NetworkError)), this, SLOT(ftp_error(QNetworkReply::NetworkError)));
    connect(m_pReply, SIGNAL(finished()),&loop, SLOT(quit()));
    loop.exec();
    if(m_isDownload)
        qDebug()<< net_file_path << "has download!";
    if(m_pFile->exists() && m_pFile->size() == 0)
    {
        m_isDownload = false;
        qDebug()<< net_file_path << "is zero bit, will be remove";
        if(m_pFile->isOpen())
            m_pFile->close();
        m_pFile->remove();
    }

    return m_isDownload;
}
//upload files
bool FtpClient::FtpPut(QString local_file_path, QString net_file_path)
{
    m_isPush = true;
    QEventLoop loop;
    QFile file(local_file_path);
    if(!file.open(QIODevice::ReadOnly))
    {
        qDebug()<< "can't open" << local_file_path;
        file.close();
        return false;
    }
    QByteArray data = file.readAll();
    m_pUrl->setScheme("ftp");
    m_pUrl->setPath(net_file_path);
    m_pReply = m_pManager->put(QNetworkRequest(*m_pUrl), data);
    connect(m_pReply, SIGNAL(uploadProgress(qint64, qint64)), this, SLOT(ftp_uploadProgress(qint64, qint64)));
    connect(m_pReply, SIGNAL(error(QNetworkReply::NetworkError)), this, SLOT(ftp_error(QNetworkReply::NetworkError)));
    connect(m_pReply, SIGNAL(finished()),&loop, SLOT(quit()));
    loop.exec();
    if(file.isOpen())
        file.close();
    if(m_isPush)
        qDebug()<< local_file_path << "has push!";
    else
        qDebug()<< local_file_path << "hasn't push!";
    return m_isPush;
}

// Upload progress
void FtpClient::ftp_uploadProgress(qint64 bytesSent, qint64 bytesTotal)
{
    qDebug() << "Sent/Total is :" << bytesSent
              << "/" << bytesTotal <<endl;

}
// Download progress
void FtpClient::ftp_downloadProgress(qint64 bytesReceived, qint64 bytesTotal)
{
    qDebug() << "Received/Total is :" << bytesReceived
              << "/" << bytesTotal <<endl;

}
