#include "ConfigWidget.h"
#include <QApplication>
#include "ConfTranIni.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    ConfigWidget w;
    w.show();



    return a.exec();
}
