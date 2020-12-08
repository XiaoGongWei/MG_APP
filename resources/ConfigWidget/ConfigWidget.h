#ifndef CONFIGWIDGET_H
#define CONFIGWIDGET_H

#include <QWidget>
#include <QJsonObject>
#include <QJsonArray>
#include <QMessageBox>
#include "ConfTranIni.h"

namespace Ui {
class ConfigWidget;
}

class ConfigWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ConfigWidget(QWidget *parent = 0);
    ~ConfigWidget();
    ConfTranIni myConfTranIni;
private:
    void initWidgt();
private slots:
    void clickOk();
    void clickCancel();

private:
    Ui::ConfigWidget *ui;


};

#endif // CONFIGWIDGET_H
