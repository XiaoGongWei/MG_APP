#ifndef QTPLOT_H
#define QTPLOT_H
#include "qcustomplot.h"
#include <QWidget>
#include <QVector>
#include <QString>
#include <QPen>
#include <QColor>
#include <QTime>


class QtPlot: public QCustomPlot
{
public:
    explicit QtPlot(QWidget *parent = 0);
    ~QtPlot();
    // plotDataX, plotDataY is multiple line
    // plotNames is multiple line names
    void qtPlot2D(QVector< QVector< double > >plotDataX, QVector< QVector< double > >plotDataY, QVector< QString >plotNames, QString xAxisName = "X_axis", QString yAxisName = "Y_axis");


private:
    double findMax(QVector< double > findData);
    double findMin(QVector< double > findData);



};

#endif // QTPLOT_H
