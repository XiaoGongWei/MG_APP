#include "QBatchProcess.h"

QBatchProcess::QBatchProcess(QString files_path, QTextEdit *pQTextEdit, QString Method, QString Satsystem, QString TropDelay,
                             double CutAngle, bool isKinematic, QString Smooth_Str, bool isBackBatch)
{
    // GNSS configure
    mp_QTextEdit = pQTextEdit;
    m_Method = Method;
    m_Satsystem = Satsystem;
    m_TropDelay = TropDelay;
    m_CutAngle = CutAngle;

    // store data
    M_ObsFiles_Path = files_path;
    m_mkdir_name = "allStations";
    m_isRuned = false;
    m_isKinematic = isKinematic;
    m_Smooth_Str = Smooth_Str;
    m_isBackBatch = isBackBatch;
}
QBatchProcess::~QBatchProcess()
{

}
//isDisplayEveryEpoch represent is disply every epoch information?(ENU or XYZ)
bool QBatchProcess::Run(bool isDisplayEveryEpoch)
{
    if(m_isRuned) return false;
    // clear data
    m_AllStationsData.clear();
    // distibute multiply .O files to destin_floder
    QString mkdir = m_mkdir_name + PATHSEG;
    QString destin_floder = M_ObsFiles_Path + PATHSEG + mkdir;
    if(!distribute(M_ObsFiles_Path, destin_floder))
    {
        QString erro_info = "QBatchProcess::Run: distribute files Bad!";
        ErroTrace(erro_info);
        return false;
    }
// ppp, Multiple  sation PPP
    // get and print floders path
    QString multiple_station_floder = destin_floder;
    QString disPlayQTextEdit = "Run multiply floder: " + multiple_station_floder;
    autoScrollTextEdit(mp_QTextEdit, disPlayQTextEdit);// display for QTextEdit

    // run batch PPP
    QDir stations_floder(multiple_station_floder);
    stations_floder.setFilter(QDir::Dirs);
    // get floder path
    QFileInfoList list_info = stations_floder.entryInfoList();
    QStringList floder_list;
    for(int i = 0; i < list_info.length(); i++)
    {
        QFileInfo file_info = list_info.at(i);
        if(file_info.fileName() == "." || file_info.fileName() == "..") continue;
        if(file_info.isDir())
            floder_list.append(file_info.absoluteFilePath());
    }
    // get and save floders name
    QStringList AllStations = stations_floder.entryList();
    disPlayQTextEdit.clear();
    for(int i = 0; i < AllStations.length(); i++ )
    {
        if(AllStations.at(i) == "." || AllStations.at(i) == "..") continue;
        m_AllStations.append(AllStations.at(i));
        disPlayQTextEdit += AllStations.at(i) + ", ";
    }
    // display floders name
    disPlayQTextEdit = ENDLINE + "Stations Names: " + ENDLINE + disPlayQTextEdit + ENDLINE;
    autoScrollTextEdit(mp_QTextEdit, disPlayQTextEdit);// display for QTextEdit
    // juge is equal
    if(m_AllStations.length() != floder_list.length())
    {
        QString erro_info = "QBatchProcess::Run: m_AllStations.length() != floder_list.length()";
        ErroTrace(erro_info);
        return false;
    }
    // run all stations
    double allTime = 0;
    int allStations_len = floder_list.length();
    for(int i = 0;i < allStations_len; i++)
    {
        QString ppp_path = floder_list.at(i);
        disPlayQTextEdit = "****** Will use PPP process station: " + m_AllStations.at(i) +
                + " ( " + QString::number(i+1) + "/" + QString::number(allStations_len) +  " ) ******" +ENDLINE;
        autoScrollTextEdit(mp_QTextEdit, disPlayQTextEdit);// display for QTextEdit
        // run single station
        QTime myTime;
        myTime.start();//start the timer

        PlotGUIData single_data;// get single station data
        if(m_isBackBatch)
        {
            QPPPBackSmooth myPPP(ppp_path, mp_QTextEdit, m_Method, m_Satsystem, m_TropDelay, m_CutAngle, m_isKinematic, m_Smooth_Str);
            myPPP.Run(isDisplayEveryEpoch);
            myPPP.getRunResult(single_data);
            m_AllStationsData.append(single_data);

        }
        else
        {
            QPPPModel myPPP(ppp_path, mp_QTextEdit, m_Method, m_Satsystem, m_TropDelay, m_CutAngle, m_isKinematic, m_Smooth_Str);
            myPPP.Run(isDisplayEveryEpoch);
            myPPP.getRunResult(single_data);
            m_AllStationsData.append(single_data);
        }
        float m_diffTime = myTime.elapsed() / 1000.0;

        // display time
        disPlayQTextEdit = "Batch Process The Elapse Time: " + QString::number(m_diffTime) + "s" + ENDLINE;
        autoScrollTextEdit(mp_QTextEdit, disPlayQTextEdit);// display for QTextEdit
        allTime += m_diffTime;
    }
    allTime = allTime / 60; // transfer senconds to minutes
    disPlayQTextEdit = "Good luck to you, all file success processed! use time:  " + QString::number(allTime)
            + " minutes" + ENDLINE;
    autoScrollTextEdit(mp_QTextEdit, disPlayQTextEdit);// display for QTextEdit
    m_isRuned = true;
	return true;
}

QStringList QBatchProcess::getStationNames()
{
    QStringList temp;
    if(m_isRuned)
        return m_AllStations;
    else
        return temp;
}

void QBatchProcess::getStoreAllData(QVector< PlotGUIData > &all_SationData)
{// use pointer store for External data
     all_SationData = m_AllStationsData;
}

// The edit box automatically scrolls, adding one row or more lines at a time.
void QBatchProcess::autoScrollTextEdit(QTextEdit *textEdit,QString &add_text)
{
    if(textEdit == NULL) return ;
    int m_Display_Max_line = 99999;
    //Add line character and refresh edit box.
    QString insertText = add_text + ENDLINE;
    textEdit->insertPlainText(insertText);
    //Keep the editor in the last line of the cursor.
    QTextCursor cursor=textEdit->textCursor();
    cursor.movePosition(QTextCursor::End);
    textEdit->setTextCursor(cursor);
    textEdit->repaint();
    //If you exceed a certain number of lines, empty it.
    if(textEdit->document()->lineCount() > m_Display_Max_line)
    {
        textEdit->clear();
    }
}

bool QBatchProcess::isDirExist(QString fullPath)
{
    QDir dir(fullPath);
    if(dir.exists())
    {
      return true;
    }
    else
    {
       bool ok = dir.mkpath(fullPath);//Create a multi-level directory
       return ok;
    }
    return false;
}

bool QBatchProcess::isFileExist(QString fullFileName)
{
    QFileInfo fileinfo(fullFileName);
    if(fileinfo.isFile())
        return true;
    else
        return false;
}

bool QBatchProcess::distribute(QString ofile_path, QString destin_floder)
{
    if(!isDirExist(destin_floder))
        return false;
    QDir path_dir(ofile_path);
    QStringList m_fliterList;
    m_fliterList.clear();
    m_fliterList.append("*.*o");
    path_dir.setFilter(QDir::Files | QDir::NoSymLinks);
    QStringList ObsFileNameList = path_dir.entryList(m_fliterList);

    for(int i = 0; i < ObsFileNameList.length();i++)
    {
        QString obs_file_name = ObsFileNameList.at(i),
                obs_file_path = ofile_path + PATHSEG + obs_file_name,
                floder_path = destin_floder + ObsFileNameList.at(i) + PATHSEG,
                destin_file_name = floder_path + obs_file_name;
        if(!isDirExist(floder_path))
        {
            QString erro_info = "QBatchProcess::distribute: make dir error. " + PATHSEG + floder_path;
            ErroTrace(erro_info);
            return false;
        }
        if(!isFileExist(destin_file_name))
        {
            if(!QFile::copy(obs_file_path, destin_file_name))
            {
                QString erro_info = "QBatchProcess::distribute: move file error." + obs_file_path;
                ErroTrace(erro_info);
                return false;
            }
        }

    }
    return true;
}

