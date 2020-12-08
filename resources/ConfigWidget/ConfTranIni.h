#ifndef CONFTRANINI_H
#define CONFTRANINI_H
#include <QJsonObject>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonParseError>
#include <QFile>
#include <QSettings>

class ConfTranIni
{
public:
    ConfTranIni(QString Inifilename);
    ~ConfTranIni();
    QString getFileName(){return m_Inifilename;}
    // read to files
    QString getValue(QString keypath);

    // write to files
    static bool writeJson2Ini(QString Inifilename, QJsonArray jsonArry);
    static bool writeJson2Ini(QString Inifilename, QString jsonText);
    static bool writeJson2File(QString jsonfilename, QJsonArray jsonArry);

private:
    QString m_Inifilename;
    QSettings *mp_QSettings;
};

#endif // CONFTRANINI_H
