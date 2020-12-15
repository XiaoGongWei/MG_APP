#include "QNewFunLib.h"


QNewFunLib::QNewFunLib(void)
{
}


QNewFunLib::~QNewFunLib(void)
{
}

//Calculate the projection point of SatPos on the 1 and 2 receiver lines
void QNewFunLib::computeCrossPoint(Vector3d Recv1Pos,Vector3d Recv2Pos,Vector3d SatPos,Vector3d *crossPoint,Vector3d *talpha)
{
	Vector3d crossPos,alpha,Recv1_Sat;
	double t = 0,alphaNorm = 0;
	crossPos.setZero();
	alpha.setZero();
	Recv1_Sat.setZero();
	alpha = Recv2Pos - Recv1Pos;
	Recv1_Sat = SatPos - Recv1Pos;
	alphaNorm = alpha.norm();
	t = alpha.dot(Recv1_Sat)/(alphaNorm*alphaNorm);
	crossPos = t*alpha + Recv1Pos;
	alpha = alpha/alpha.norm();
	*crossPoint = crossPos;
	if (talpha) *talpha = alpha;
}

bool QNewFunLib::deleteDirectory(const QString &path)
{
    if (path.isEmpty())
        return false;

    QDir dir(path);
    if(!dir.exists())
        return true;

    dir.setFilter(QDir::AllEntries | QDir::NoDotAndDotDot);
    QFileInfoList fileList = dir.entryInfoList();
    for (int i = 0; i < fileList.length(); i++)
    {
        QFileInfo fi = fileList.at(i);
        if (fi.isFile())
            fi.dir().remove(fi.fileName());
        else
            deleteDirectory(fi.absoluteFilePath());
    }
    return dir.rmpath(dir.absolutePath());
}

