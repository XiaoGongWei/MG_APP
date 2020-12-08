//#define EIGEN_USE_MKL_ALL
#include <QApplication>
#include <QStringList>
#include "mainwindow.h"
#include "QNewFunLib.h"
#include "QtMyTest.cpp"



int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

//    testMain();// test functions

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



