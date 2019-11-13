#include "qtplot.h"

QtPlot::QtPlot(QWidget *parent) : QCustomPlot(parent)
{
    setGeometry(300,300, 500, 300);

}

QtPlot::~QtPlot()
{

}

void QtPlot::qtPlot2D(QVector< QVector< double > >plotDataX, QVector< QVector< double > >plotDataY,
                      QVector< QString >plotNames, QString xAxisName, QString yAxisName)
{

    int xLength = 0, yLength = 0, RGB[3] = {0};
    double xAxisRange[2] = {0}, yAxisRange[2] = {0};
    QPen pen;
    xLength = plotDataX.length();
    yLength = plotDataY.length();
    if(xLength != yLength) return ;
    // setInteraction for frendly
    setInteraction(QCP::iRangeDrag);
    setInteraction(QCP::iRangeZoom);
    setInteraction(QCP::iSelectLegend);
    setInteraction(QCP::iSelectOther);

//    setSelectionRectMode(QCP::srmZoom);
    setInteraction(QCP::iSelectPlottables);
    setInteraction(QCP::iMultiSelect, true);
    setMultiSelectModifier(Qt::ControlModifier);
    legend->setVisible(true);
    // set rand seed
    qsrand(QTime(0,0,0).secsTo(QTime::currentTime()));
    // plot every curve
    QVector< double > xData, yData;
    QVector< double > xAxis_min, xAxis_max, yAxis_min, yAxis_max;
    for(int i = 0; i < xLength; i++)
    {
        // addGraph
        addGraph();
        // set plot Style
        pen.setStyle(Qt::SolidLine);
        pen.setWidth(3);
        switch(i)
        {
        case 0:
            RGB[0] = 255; RGB[1] = 0; RGB[2] = 0;
            break;
        case 1:
            RGB[0] = 0; RGB[1] = 255; RGB[2] = 0;
            break;
        case 2:
            RGB[0] = 0; RGB[1] = 0; RGB[2] = 255;
            break;
        default :
            RGB[0] = qrand() % 256; RGB[1] = qrand() % 256; RGB[2] = qrand() % 256;
        }
        pen.setColor(QColor(RGB[0], RGB[1], RGB[2]));
        graph(i)->setPen(pen);
        graph(i)->setName(plotNames.at(i));
        graph(i)->setAntialiased(true);
        // set plot data
        xData = plotDataX.at(i);
        yData = plotDataY.at(i);
        if(xData.length() != yData.length()) break;
        graph(i)->setData(xData, yData);
        // find max and min
        xAxis_min.append(findMin(xData));
        xAxis_max.append(findMax(xData));
        yAxis_min.append(findMin(yData));
        yAxis_max.append(findMax(yData));
    }
    xAxisRange[0] = findMin(xAxis_min);
    xAxisRange[1] = findMax(xAxis_max);
    yAxisRange[0] = findMin(yAxis_min);
    yAxisRange[1] = findMax(yAxis_max);

    xAxis->setRange(xAxisRange[0], xAxisRange[1]);
    yAxis->setRange(yAxisRange[0], yAxisRange[1]);
    xAxis->setLabel(xAxisName);
    yAxis->setLabel(yAxisName);
    axisRect()->setupFullAxesBox(true);
//    axisRect()->setBackground(QPixmap);


}


double QtPlot::findMax(QVector< double > findData)
{
    if(findData.length() == 0) return 0;
    double max_data = findData.at(0), max_flag = 0;
    for(int i = 0; i < findData.length();i++)
        if(max_data < findData.at(i))
        {
            max_data = findData.at(i);
            max_flag = i;
        }
    return max_data;
}
double QtPlot::findMin(QVector< double > findData)
{
    if(findData.length() == 0) return 0;
    double min_data = findData.at(0), min_flag = 0;
    for(int i = 0; i < findData.length();i++)
        if(min_data > findData.at(i))
        {
            min_data = findData.at(i);
            min_flag = i;
        }
    return min_data;
}

