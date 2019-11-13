//#define EIGEN_USE_MKL_ALL

#include <QApplication>
#include <QSplashScreen>
#include <QPixmap>
#include "mainwindow.h"
#include "QSPPModel.h"
#include <QDateTime>
#include <QStringList>
#include <QCoreApplication>
#include <QDir>


void testFunction()
{
    // print run flooder
//    qDebug() << "run floder." <<QCoreApplication::applicationDirPath();
//    qDebug() << "run floder." <<QDir::currentPath();

//    QStringList clkfiles;
//    clkfiles.append("/home/david/Downloads/whp20042.clk");
//    clkfiles.append("/home/david/Downloads/whp20043.clk");
//    clkfiles.append("/home/david/Downloads/whp20044.clk");
//    QReadClk myclk(clkfiles);
//    myclk.setSatlitSys("G");
//    myclk.getAllData();



    // add QSplashScreen
//    QSplashScreen *splash = new QSplashScreen(QPixmap("./resources/splash_logo.jpg"));
//    splash->show();
//    splash->showMessage(QString("Loading..."));
//    splash->showMessage(QString("Success->%1...").arg(1));

//    QString spp_path = "/home/david/MySoft/TestData/SPP/CUTB";
//    QSPPModel spp(spp_path, NULL, "Kalman", "G");
//    spp.Run();

}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    testFunction();// test function

    // Limit software usage time
    QDateTime current_time = QDateTime::currentDateTime();
    QString date_str = current_time.toString("yyyy MM dd hh mm ss");
    QStringList date_str_vct = date_str.split(" ");
    int my_year = date_str_vct.at(0).toInt(),
            my_moth = date_str_vct.at(1).toInt(),
            my_day = date_str_vct.at(2).toInt();
//    if(my_year >= 2019 && my_moth >= 10)
//        return 0;

    MainWindow wnd;
    wnd.show();


    return a.exec();
}



