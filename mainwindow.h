/*************************************************************************
**
**  MG-APPS----Multi-GNSS-Automatic Precise Positioning Software
**  Copyright (C) 2016-2019 XiaoGongWei
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


#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QWidget>
#include <QVBoxLayout>
#include <qmath.h>
#include <QFileDialog>
#include <QMessageBox>
#include <QTextCursor>
#include <QIcon>
#include <QPoint>
#include <QString>
#include <QAction>
#include <QProcess>

#include "QPPPModel.h"
#include "QPPPBackSmooth.h"
#include "QSPPModel.h"
#include "QBatchProcess.h"
#include "QtPPPGUI/qtplot.h"



namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:
    void selectFilePath(); // select obsfile path
    void plotAllRes();// plot all figure
    void RunPPP();// Run Single Station PPP
    void RunSPP();// Run Single Station PPP
    void RunPPPBatch();// Run Batches Station PPP
    void AboutApp();

protected:
    void closeEvent(QCloseEvent *event);
    void paintEvent(QPaintEvent *);


private:
    void initWindow();
    void WaringBox(QString info = "defualt");
    void autoScrollTextEdit(QTextEdit *textEdit,QString &add_text);
    void plotSigleStation(PlotGUIData &station_data);
    void clearPlotGUIData(PlotGUIData &station_data);
    bool isDirExist(QString fullPath);
    QVector<QStringList> getConfObsType();

private:
    Ui::MainWindow *ui;
    QtPlot *mp_qtPlot;
    QString m_station_path, m_App_floder;// obsvertion path
    bool m_isRuned;
    bool m_isRunedBatch;
    int m_Display_Max_line;
    PlotGUIData m_single_data;
    QVector< PlotGUIData > m_mutiply_data;//store Big data in m_mutiply_data before Run
    QStringList m_mutiply_names;// store multiply stations
    // menu bar
    QMenu *m_otherMenu;
    ConfigWidget m_ConfigWidget;

};

#endif // MAINWINDOW_H
