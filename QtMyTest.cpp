#include <QSplashScreen>
#include <QPixmap>
#include <QDateTime>
#include <QCoreApplication>
#include <QDir>
#include <QDebug>

#include "QReadClk.h"
#include "QSPPModel.h"
#include "QualityCtrl.h"
#include "FtpClient.h"


void testRunFolder()
{
    // print run flooder
    qDebug() << "run floder." <<QCoreApplication::applicationDirPath();
    qDebug() << "run floder." <<QDir::currentPath();
}

void testReadClk()
{
    QStringList clkfiles;
    clkfiles.append("/home/david/Downloads/whp20042.clk");
    clkfiles.append("/home/david/Downloads/whp20043.clk");
    clkfiles.append("/home/david/Downloads/whp20044.clk");
    QReadClk myclk(clkfiles);
    myclk.setSatlitSys("G");
    myclk.getAllData();
}
void testSplash()
{
    // add QSplashScreen
    QString AppPath = QCoreApplication::applicationDirPath();
    QString splashLogoPath =  AppPath + PATHSEG + "images/splash_logo.jpg";
    QSplashScreen *splash = new QSplashScreen(QPixmap(splashLogoPath));
    splash->show();
    qApp->processEvents();
    splash->showMessage(QString("Loading..."));
    int sleep_num = 5;
    for(int i = 0;i < sleep_num;i++)
    {
        splash->showMessage(QString("MG-APP Success->%1/%2...").arg(i).arg(sleep_num));
#ifdef _WIN32
        #include <windows.h>
        Sleep(1000);
#else
        sleep(1000);
#endif
    }
    splash->finish(NULL);
}

void testSPP()
{
    QString spp_path = "/home/david/MySoft/TestData/SPP/CUTB";
    QSPPModel spp(spp_path, NULL, "Kalman", "G");
    spp.Run();
}

void testQVector()
{
    QVector< int > tempEpochSatlitData;
    tempEpochSatlitData.append(1); tempEpochSatlitData.append(2);
    tempEpochSatlitData.append(3); tempEpochSatlitData.append(4);
    reverse(tempEpochSatlitData.begin(), tempEpochSatlitData.end());
}

void testQualityCtrl()
{
    QualityCtrl m_QualityCtrl;

}

void testTransfer()
{
    double temp_num = 3.14159265358979;
    QString temp_str = QString::number(temp_num, 'f', 3);
}

void testFtpClient()
{
    FtpClient m_FtpClient;
    m_FtpClient.downloadHTTPFile("http://127.0.0.1/Online_MG_APP/resources/MatlabPlot.zip", "D:/xiao.zip");
//    m_FtpClient.downloadHTTPFile("https://cddis.nasa.gov/archive/gnss/data/daily/2018/002/18o/abmf0020.18o.Z", "D:/abmf0020.18o.Z");

    for(int i = 0;i < 0;i++)
    {
        m_FtpClient.pushData2Http("http://127.0.0.1/testJSON.php", "{\"firstName\": \"Gongwei\", \"lastName\": \"Xiao\"}");
#ifdef _WIN32
        Sleep(1000);
#else
        sleep(1000);
#endif
    }

    int a = 0;
}

void testMain()
{
//    testFtpClient();
//    testRunFolder();
    testSplash();
    int a = 0;
}



