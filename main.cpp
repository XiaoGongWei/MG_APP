//#define EIGEN_USE_MKL_ALL

#include <QApplication>
#include "mainwindow.h"
#include "QtMyTest.cpp"




int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    testMain();// test some classes and functions

    MainWindow wnd;
    wnd.show();

    return a.exec();
}



