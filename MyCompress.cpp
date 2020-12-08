#include "MyCompress.h"

MyCompress::MyCompress()
{
    m_App_floder = QCoreApplication::applicationDirPath() + COMPRESSPATHSEG;
}

bool MyCompress::UnCompress(QString unCompressFile, QString unCompress_floder)
{
    if(!isDirExist(unCompress_floder))
        return false;
    bool is_Support = true;
    int point_index = unCompressFile.lastIndexOf(".");
     QString extent_name = unCompressFile.mid(point_index).trimmed();
//    is_Support = extent_name.compare(".Z", Qt::CaseInsensitive) == 0;
    if(is_Support)
    {
        if(MYCOMPRESS_H_isLiux)
        {
            QProcess myProcess;
            QString app_path = "uncompress";
            QStringList param;
            param << "-d" << "-f" << unCompressFile;
            myProcess.start(app_path, param);
            return myProcess.waitForFinished();
        }
        else
        {
            QProcess myProcess;
            QString app_path = m_App_floder.append("gzip.exe");
            QStringList param;
            param << "-d" << "-f" << unCompressFile;
            myProcess.start(app_path, param);
            return myProcess.waitForFinished();
        }
    }

    return false;
}

bool MyCompress::isDirExist(QString fullPath)
{
    QDir dir(fullPath);
    if(dir.exists())
    {
      return true;
    }
    else
    {
       bool ok = dir.mkpath(fullPath);//Create a multi-level directory
       return ok;
    }
    return false;
}
