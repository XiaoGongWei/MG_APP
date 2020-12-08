# this is GUI
QT += gui
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport
RC_ICONS = widget.ico
# this is console APP
# QT -= gui
#CONFIG += c++11
#CONFIG += console
#CONFIG -= app_bundle

QT += core
QT += network

TARGET = MG_APP
TEMPLATE = app
INCLUDEPATH += \
./resources/myeigen \
./resources/ConfigWidget


## openMP
#QMAKE_CXXFLAGS += -fopenmp
#LIBS += -fopenmp
## intel MKL
#unix:INCLUDEPATH += /opt/intel/mkl/include
#unix:LIBS += -L/opt/intel/mkl/lib/intel64 \
#    -lmkl_intel_lp64 -lmkl_intel_thread -lmkl_core \
#    -L/opt/intel/lib/intel64 \
#    -liomp5 -lpthread -ldl -lm


SOURCES += \
    QWrite2File.cpp \
    QWindUp.cpp \
    QTropDelay.cpp \
    QTideEffect.cpp \
    QReadSP3.cpp \
    QReadOFile.cpp \
    QReadGPSN.cpp \
    QReadClk.cpp \
    QReadAnt.cpp \
    QPPPModel.cpp \
    QNewFunLib.cpp \
    QKalmanFilter.cpp \
    QCmpGPST.cpp \
    QBaseObject.cpp \
    main.cpp \
    MyMatrix.cpp \
    SRIFAlgorithm.cpp \
    FtpClient.cpp \
    MyCompress.cpp \
    QBatchProcess.cpp \
    QtPPPGUI/qcustomplot.cpp \
    mainwindow.cpp \
    QtPPPGUI/qtplot.cpp \
    QualityCtrl.cpp \
    QPseudoSmooth.cpp \
    QSPPModel.cpp \
    QPPPBackSmooth.cpp \
    QtMyTest.cpp \
    QRTWrite2File.cpp \
    resources/ConfigWidget/ConfigWidget.cpp \
    resources/ConfigWidget/ConfTranIni.cpp

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

HEADERS += \
    QWrite2File.h \
    QWindUp.h \
    QTropDelay.h \
    QTideEffect.h \
    QReadSP3.h \
    QReadOFile.h \
    QReadGPSN.h \
    QReadClk.h \
    QReadAnt.h \
    QPPPModel.h \
    QNewFunLib.h \
    QKalmanFilter.h \
    QGlobalDef.h \
    QCmpGPST.h \
    QBaseObject.h \
    MyMatrix.h \
    SRIFAlgorithm.h \
    FtpClient.h \
    MyCompress.h \
    QBatchProcess.h \
    QtPPPGUI/qcustomplot.h \
    mainwindow.h \
    QtPPPGUI/qtplot.h \
    QualityCtrl.h \
    QPseudoSmooth.h \
    QSPPModel.h \
    QPPPBackSmooth.h \
    QRTWrite2File.h \
    resources/ConfigWidget/ConfigWidget.h \
    resources/ConfigWidget/ConfTranIni.h


FORMS += \
    mainwindow.ui \
    resources/ConfigWidget/ConfigWidget.ui

DISTFILES += \
    Licences/README.md \
    Licences/THANKS \
    Licences/COPYING \
    QtPPPGUI/README.md \
    README.md
