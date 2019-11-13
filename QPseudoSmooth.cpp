#include "QPseudoSmooth.h"



QPseudoSmooth::QPseudoSmooth()
{
    m_wa = 1;
    m_wb = 0;
    m_method = Hatch;

}


bool QPseudoSmooth::SmoothPesudoRange(QVector < SatlitData > &prevEpochSatlitData, QVector < SatlitData > &epochSatlitData)
{
    int epochLen = epochSatlitData.length(), preEpochLen = prevEpochSatlitData.length();
    // if(0 == preEpochLen)
    if(0 == preEpochLen)
    {// init first epoch PP3 or Last epoch abnormal
        for(int i = 0; i < epochLen;i++)
        {
            epochSatlitData[i].PP3_Smooth = epochSatlitData[i].PP3;
            epochSatlitData[i].PP3_Smooth_NUM = 1;
            epochSatlitData[i].PP3_Smooth_Q = 1 / epochSatlitData[i].SatWight;
        }
        return true;
    }
    // judge satlite is changed
    QVector< int > changePrnFlag;
    bool isChangeSat = false;
    isChangeSat = isSatChanged(prevEpochSatlitData, epochSatlitData, changePrnFlag);
    if(changePrnFlag.length() != epochLen) return false;
    // smooth Pesudorange
    for(int i = 0;i < epochLen; i++)
    {
        int pre_sat_index = changePrnFlag.at(i);
        if(pre_sat_index >= preEpochLen)
            ErroTrace("QPseudoSmooth::SmoothPesudoRange Array crossing.");

        if( -1 != pre_sat_index)
        {
            SatlitData pre_sat_data = prevEpochSatlitData.at(pre_sat_index),
                    curr_sat_data = epochSatlitData.at(i);
            epochSatlitData[i].PP3_Smooth_NUM = pre_sat_data.PP3_Smooth_NUM + 1;
            // select method
            switch(m_method)
            {
            case Hatch:
                m_wa = 1.0 / epochSatlitData[i].PP3_Smooth_NUM;
                m_wb = 1.0 - m_wa;
                break;
            default:
                m_wa = 1;
                m_wb = 0;
                break;
            }
            // smooth data
            epochSatlitData[i].PP3_Smooth = m_wa * curr_sat_data.PP3
                    + m_wb*( pre_sat_data.PP3_Smooth + curr_sat_data.LL3 - pre_sat_data.LL3);
            epochSatlitData[i].PP3_Smooth_Q = m_wa*m_wa*(1/curr_sat_data.SatWight) + m_wb*m_wb*pre_sat_data.PP3_Smooth_Q
                    +m_wb*m_wb*(1e-4/curr_sat_data.SatWight) + m_wb*m_wb*(1e-4/pre_sat_data.SatWight);
        }
        else
        {
            epochSatlitData[i].PP3_Smooth = epochSatlitData[i].PP3;
            epochSatlitData[i].PP3_Smooth_NUM = 1;
            epochSatlitData[i].PP3_Smooth_Q = 1 / epochSatlitData[i].SatWight;
        }

    }

    return true;
}


bool QPseudoSmooth::isSatChanged(const QVector< SatlitData > &preEpoch,const QVector< SatlitData > &currEpoch, QVector< int > &changePrnFlag)
{
    int preEpochLen = preEpoch.length();
    int epochLenLB = currEpoch.length();
    //Determine whether the number of satellites has changed (comparison of two epochs before and after)
    QVector< int > oldPrnFlag;//Compared with the location of the same satellite in the previous epoch, it is not found with -1
    int oldSatLen = 0;
    bool isNewSatlite = false;
    for (int i = 0;i < epochLenLB;i++)
    {//Whether the satellite inspections before and after the cycle are completely equal
        SatlitData epochSatlit = currEpoch.at(i);
        bool Isfind = false;//Whether the tag finds the last epoch
        for (int j = 0;j < preEpochLen;j++)
        {
            SatlitData preEpochSatlit = preEpoch.at(j);
            if (epochSatlit.PRN == preEpochSatlit.PRN&&epochSatlit.SatType == preEpochSatlit.SatType)
            {
                oldPrnFlag.append(j);
                Isfind = true;
                oldSatLen++;
                break;
            }
        }
        if (!Isfind)
        {
            oldPrnFlag.append(-1);
            isNewSatlite = true;
        }
    }
    changePrnFlag = oldPrnFlag;
    return isNewSatlite;
}
