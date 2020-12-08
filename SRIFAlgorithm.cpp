//============================================================================
// Name        : SRIFAlgorithm.cpp
// Author      : xiaogongwei
// Version     :
// Copyright   : Copyright attributed to David
// Description : Hello World in C++, Ansi-style
//============================================================================

#include "SRIFAlgorithm.h"


SRIFAlgorithm::SRIFAlgorithm() {
	// TODO Auto-generated constructor stub
	initVar();
}

SRIFAlgorithm::~SRIFAlgorithm() {
	// TODO Auto-generated destructor stub
}

void SRIFAlgorithm::initVar()
{
    this->m_initSRIF = false;
    this->m_isInitPara = false;
    this->m_isInitWhite = false;
    m_SPP_Pos[0] = 0; m_SPP_Pos[1] = 0; m_SPP_Pos[2] = 0;
    m_Xk.resize(32);// XiaoGongWei Update:2018.12.02
    m_init_Xk.resize(32);// XiaoGongWei Update:2018.12.02
    m_Xk.setZero();// XiaoGongWei Update:2018.12.02
    m_init_Xk.setZero();// XiaoGongWei Update:2018.12.02
    m_const_param = 4;// [dx,dy,dz,mf,clki]
    m_sys_num = 1;
    m_sys_str = "G";
    m_LP_whight = 1e6;
    m_xyz_dynamic_Qw = 1e6; m_zwd_Qw = 3e-8; m_clk_Qw = 1e6; m_amb_Qw = 1e-16; m_ion_Qw = 0.1;
    m_xyz_dynamic_Pk = 1e6; m_zwd_Pk = 10; m_clk_Pk = 1e6; m_amb_Pk = 1e6; m_ion_Pk = 10;
}

void SRIFAlgorithm::setFilterParams(QVector<QStringList> Qw_Pk_LPacc)
{
    if(Qw_Pk_LPacc.length() >= 2){
        QStringList Qw_StrList =  Qw_Pk_LPacc.at(0),Pk_StrList = Qw_Pk_LPacc.at(1);
        if(Qw_StrList.length() < 5) return ; if(Pk_StrList.length() < 5) return ;
        // set Qw
        m_xyz_dynamic_Qw = Qw_StrList.at(0).toDouble(); m_zwd_Qw = Qw_StrList.at(1).toDouble();
        m_clk_Qw = Qw_StrList.at(2).toDouble(); m_amb_Qw = Qw_StrList.at(3).toDouble();
        m_ion_Qw = Qw_StrList.at(4).toDouble();
        // set Pk
        m_xyz_dynamic_Pk = Pk_StrList.at(0).toDouble(); m_zwd_Pk = Pk_StrList.at(1).toDouble();
        m_clk_Pk= Pk_StrList.at(2).toDouble(); m_amb_Pk = Pk_StrList.at(3).toDouble();
        m_ion_Pk = Pk_StrList.at(4).toDouble();
    }
    if(Qw_Pk_LPacc.length() >= 3){
        QStringList LP_StrList = Qw_Pk_LPacc.at(2);
        double LP_ratio = 1e3;
        if(LP_StrList.length() == 2 && LP_StrList.at(0).toDouble() != 0 )
            LP_ratio = LP_StrList.at(1).toDouble() / LP_StrList.at(0).toDouble();
        m_LP_whight = LP_ratio * LP_ratio;// set m_LP_whight
    }
    // some time Qw as denominator,such as 1/Qw; so Qw must not zero
    double myEps = 1e-16;
    if(0 == m_xyz_dynamic_Qw) m_xyz_dynamic_Qw = myEps;
    if(0 == m_zwd_Qw) m_zwd_Qw = myEps; if(0 == m_clk_Qw) m_clk_Qw = myEps;
    if(0 == m_amb_Qw) m_amb_Qw = myEps; if(0 == m_ion_Qw) m_ion_Qw = myEps;

}

//
void SRIFAlgorithm::setModel(SRIF_MODEL model_type)
{
    m_SRIF_MODEL = model_type;
    m_sys_num = getSystemnum();
    m_sys_str = getSatlitSys();
    switch (model_type)
    {
    case SRIF_MODEL::SPP_STATIC:
    case SRIF_MODEL::SPP_KINEMATIC:
        m_const_param = 3 + m_sys_num;//[dx,dy,dz,mf]
        break;
    case SRIF_MODEL::PPP_KINEMATIC:
    case SRIF_MODEL::PPP_STATIC:
        m_const_param = 4 + m_sys_num;//[dx,dy,dz,mf,clki]
        break;
    default:
        m_const_param = 4+1;
        break;
    }
}
//Initialize SRIF
void SRIFAlgorithm::initSRIFPara(QVector< SatlitData > &currEpoch,MatrixXd &B,VectorXd &L)
{
    int epochLenLB = currEpoch.length();

    //Fk_1 initialization
    switch (m_SRIF_MODEL) {
    case SRIF_MODEL::SPP_STATIC:
    case SRIF_MODEL::SPP_KINEMATIC:
        m_Phi.resize(m_const_param, m_const_param);
        m_Phi.setIdentity(m_const_param, m_const_param);
        break;
    case SRIF_MODEL::PPP_KINEMATIC:
    case SRIF_MODEL::PPP_STATIC:
        m_Phi.resize(m_const_param+epochLenLB,m_const_param+epochLenLB);
        m_Phi.setIdentity(m_const_param+epochLenLB,m_const_param+epochLenLB);
        break;
    default:
        break;
    }

    //Initial state covariance matrix m_Q initialization(not used)
    switch (m_SRIF_MODEL) {
    case SRIF_MODEL::SPP_STATIC:
    case SRIF_MODEL::SPP_KINEMATIC:
        m_Q.resize(m_const_param,m_const_param);
        m_Q.setZero();
        for(int i = 3; i < m_const_param;i++) m_Q(i,i) = m_clk_Pk;// for clock
        break;
    case SRIF_MODEL::PPP_KINEMATIC:
    case SRIF_MODEL::PPP_STATIC:
        m_Q.resize(m_const_param+epochLenLB,m_const_param+epochLenLB);
        m_Q.setZero();
        m_Q(0,0) = m_xyz_dynamic_Pk;m_Q(1,1) = m_xyz_dynamic_Pk;m_Q(2,2) = m_xyz_dynamic_Pk;
        m_Q(3,3) = m_zwd_Pk;
        for(int i = 4; i < m_const_param;i++) m_Q(i,i) = m_clk_Pk; // for clock
        for (int i = 0;i < epochLenLB;i++)	m_Q(m_const_param+i,m_const_param+i) = m_amb_Pk;// for Ambiguity
        break;
    default:
        ErroTrace("QKalmanFilter::initKalman Bad.");
        break;
        }

    //Chole decomposition of m_Rwk covariance inverse
    switch (m_SRIF_MODEL) {
        case SRIF_MODEL::SPP_STATIC:
        case SRIF_MODEL::SPP_KINEMATIC:
            m_Rwk.resize(m_const_param,m_const_param);
            m_Rwk.setZero();
            m_Rwk(0,0) = 1e10; m_Rwk(1,1) = 1e10; m_Rwk(2,2) = 1e10;
            for(int i = 3; i < m_const_param;i++) m_Rwk(i,i) = sqrt(1/m_clk_Qw);// for clock
            break;
        case SRIF_MODEL::PPP_KINEMATIC:
        case SRIF_MODEL::PPP_STATIC:
            m_Rwk.resize(m_const_param+epochLenLB,m_const_param+epochLenLB);
            m_Rwk.setZero();
            m_Rwk(0,0) = 1e10; m_Rwk(1,1) = 1e10; m_Rwk(2,2) = 1e10;
            m_Rwk(3,3) = sqrt(1/m_zwd_Qw);//Zenith tropospheric residual variance
            for(int i = 4; i < m_const_param;i++) m_Rwk(i,i) = sqrt(1/m_clk_Qw); // for clock
            for(int i = m_const_param;i < m_const_param+epochLenLB;i++)// for Ambiguity
                m_Rwk(i,i) = sqrt(1/m_amb_Qw);
            break;
        default:
            ErroTrace("QKalmanFilter::initKalman Bad.");
            break;
        }

    // juge is kinematic
    if(m_SRIF_MODEL == SRIF_MODEL::SPP_KINEMATIC || m_SRIF_MODEL == SRIF_MODEL::PPP_KINEMATIC)
    {
        m_Rwk(0,0) = sqrt(1/m_xyz_dynamic_Qw);
        m_Rwk(1,1) = sqrt(1/m_xyz_dynamic_Qw);
        m_Rwk(2,2) = sqrt(1/m_xyz_dynamic_Qw);
    }

    // init m_G of m_Rwk
    switch (m_SRIF_MODEL) {
        case SRIF_MODEL::SPP_STATIC:
        case SRIF_MODEL::SPP_KINEMATIC:
            m_G.resize(m_const_param,m_const_param);
            m_G.setIdentity();
//            for(int i = 3; i < m_const_param;i++) m_G(i,i) = 1;// for clock
            break;
        case SRIF_MODEL::PPP_KINEMATIC:
        case SRIF_MODEL::PPP_STATIC:
            m_G.resize(m_const_param+epochLenLB,m_const_param+epochLenLB);
            m_G.setIdentity();
//            m_G(3,3) = 1;//Zenith troposphere
//            for(int i = 4; i < m_const_param;i++) m_G(i,i) = 1; // for clock
            break;
        default:
            ErroTrace("QKalmanFilter::initKalman Bad.");
            break;
        }
//    if(m_SRIF_MODEL == SRIF_MODEL::SPP_KINEMATIC || m_SRIF_MODEL == SRIF_MODEL::PPP_KINEMATIC)
//    {
//        m_G(0,0) = 1;
//        m_G(1,1) = 1;
//        m_G(2,2) = 1;
//    }

    //Xk initialization, least squares initialization
    switch (m_SRIF_MODEL) {
        case SRIF_MODEL::SPP_STATIC:
        case SRIF_MODEL::SPP_KINEMATIC:
            m_Xk.resize(m_const_param);
            m_Xk.setZero();
            break;
        case SRIF_MODEL::PPP_KINEMATIC:
        case SRIF_MODEL::PPP_STATIC:
            m_Xk.resize(epochLenLB+m_const_param);
            m_Xk.setZero();
            break;
        default:
            ErroTrace("QKalmanFilter::initKalman Bad.");
            break;
        }
    // init SRIF
    m_Rp = B.transpose()*B;
    m_Zp = B.transpose()*L;

    m_Xk = m_Rp.inverse()*m_Zp;
    m_init_Xk = m_Xk;

    this->m_isInitPara = true;//Not initialized after
}

//Initialize SRIF for nocombination
void SRIFAlgorithm::initSRIFPara_NoCombination(QVector< SatlitData > &currEpoch,MatrixXd &B,VectorXd &L)
{
    int epochLenLB = currEpoch.length();

    //Fk_1 initialization
    switch (m_SRIF_MODEL) {
    case SRIF_MODEL::SPP_STATIC:
    case SRIF_MODEL::SPP_KINEMATIC:
        m_Phi.resize(m_const_param+epochLenLB, m_const_param+epochLenLB);
        m_Phi.setIdentity();
        break;
    case SRIF_MODEL::PPP_KINEMATIC:
    case SRIF_MODEL::PPP_STATIC:
        m_Phi.resize(m_const_param+3*epochLenLB,m_const_param+3*epochLenLB);
        m_Phi.setIdentity();
        break;
    default:
        break;
    }

    //Initial state covariance matrix m_Q initialization(not used)
    switch (m_SRIF_MODEL) {
    case SRIF_MODEL::SPP_STATIC:
    case SRIF_MODEL::SPP_KINEMATIC:
        m_Q.resize(m_const_param+epochLenLB,m_const_param+epochLenLB);
        m_Q.setZero();
        for(int i = 3; i < m_const_param;i++) m_Q(i,i) = m_clk_Pk;// for clock
        for(int i = m_const_param; i < m_const_param+epochLenLB;i++) m_Q(i,i) = m_ion_Pk;// for ION
        break;
    case SRIF_MODEL::PPP_KINEMATIC:
    case SRIF_MODEL::PPP_STATIC:
        m_Q.resize(m_const_param+3*epochLenLB,m_const_param+3*epochLenLB);
        m_Q.setZero();
        m_Q(0,0) = m_xyz_dynamic_Pk;m_Q(1,1) = m_xyz_dynamic_Pk;m_Q(2,2) = m_xyz_dynamic_Pk;
        m_Q(3,3) = m_zwd_Pk;
        for(int i = 4; i < m_const_param;i++) m_Q(i,i) = m_clk_Pk; // for clock
        for (int i = m_const_param;i < m_const_param+epochLenLB;i++)	m_Q(i,i) = m_ion_Pk;// for ION
        for (int i = m_const_param+epochLenLB;i < m_const_param+3*epochLenLB;i++)	m_Q(i,i) = m_amb_Pk;// for AMB
        break;
    default:
        ErroTrace("QKalmanFilter::initKalman Bad.");
        break;
        }

    //Chole decomposition of m_Rwk covariance inverse
    switch (m_SRIF_MODEL) {
        case SRIF_MODEL::SPP_STATIC:
        case SRIF_MODEL::SPP_KINEMATIC:
            m_Rwk.resize(m_const_param+epochLenLB,m_const_param+epochLenLB);
            m_Rwk.setZero();
            m_Rwk(0,0) = 1e10; m_Rwk(1,1) = 1e10; m_Rwk(2,2) = 1e10;
            for(int i = 3; i < m_const_param;i++) m_Rwk(i,i) = sqrt(1/m_clk_Qw);// for clock
            for(int i = m_const_param; i < m_const_param+epochLenLB;i++) m_Rwk(i,i) = sqrt(1/m_ion_Qw);// for ION
            break;
        case SRIF_MODEL::PPP_KINEMATIC:
        case SRIF_MODEL::PPP_STATIC:
            m_Rwk.resize(m_const_param+3*epochLenLB,m_const_param+3*epochLenLB);
            m_Rwk.setZero();
            m_Rwk(0,0) = 1e10; m_Rwk(1,1) = 1e10; m_Rwk(2,2) = 1e10;
            m_Rwk(3,3) = sqrt(1/m_zwd_Qw);//Zenith tropospheric residual variance
            for(int i = 4; i < m_const_param;i++) m_Rwk(i,i) =  sqrt(1/m_clk_Qw); // for clock
            for(int i = m_const_param;i < m_const_param+epochLenLB;i++) m_Rwk(i,i) =  sqrt(1/m_ion_Qw);// for ION
            for(int i = m_const_param+epochLenLB;i < m_const_param+3*epochLenLB;i++) m_Rwk(i,i) =  sqrt(1/m_amb_Qw);// for AMB
            break;
        default:
            ErroTrace("QKalmanFilter::initKalman Bad.");
            break;
        }

    // juge is kinematic
    if(m_SRIF_MODEL == SRIF_MODEL::SPP_KINEMATIC || m_SRIF_MODEL == SRIF_MODEL::PPP_KINEMATIC)
    {
        m_Rwk(0,0) = 1/m_xyz_dynamic_Qw;
        m_Rwk(1,1) = 1/m_xyz_dynamic_Qw;
        m_Rwk(2,2) = 1/m_xyz_dynamic_Qw;
    }

    // init m_G of m_Rwk
    switch (m_SRIF_MODEL) {
        case SRIF_MODEL::SPP_STATIC:
        case SRIF_MODEL::SPP_KINEMATIC:
            m_G.resize(m_const_param+epochLenLB,m_const_param+epochLenLB);
            m_G.setIdentity();
//            for(int i = 3; i < m_const_param;i++) m_G(i,i) = 1;// for clock
            break;
        case SRIF_MODEL::PPP_KINEMATIC:
        case SRIF_MODEL::PPP_STATIC:
            m_G.resize(m_const_param+3*epochLenLB,m_const_param+3*epochLenLB);
            m_G.setIdentity();
//            m_G(3,3) = 1;//Zenith troposphere
//            for(int i = 4; i < m_const_param;i++) m_G(i,i) = 1; // for clock
            break;
        default:
            ErroTrace("QKalmanFilter::initKalman Bad.");
            break;
        }
//    if(m_SRIF_MODEL == SRIF_MODEL::SPP_KINEMATIC || m_SRIF_MODEL == SRIF_MODEL::PPP_KINEMATIC)
//    {
//        m_G(0,0) = 1;
//        m_G(1,1) = 1;
//        m_G(2,2) = 1;
//    }

    //Xk initialization, least squares initialization
    switch (m_SRIF_MODEL) {
        case SRIF_MODEL::SPP_STATIC:
        case SRIF_MODEL::SPP_KINEMATIC:
            m_Xk.resize(m_const_param+epochLenLB);
            m_Xk.setZero();
            break;
        case SRIF_MODEL::PPP_KINEMATIC:
        case SRIF_MODEL::PPP_STATIC:
            m_Xk.resize(m_const_param+3*epochLenLB);
            m_Xk.setZero();
            break;
        default:
            ErroTrace("QKalmanFilter::initKalman Bad.");
            break;
        }
    // init SRIF
    m_Rp = B.transpose()*B;
    m_Zp = B.transpose()*L;

    m_Xk = m_Rp.inverse()*m_Zp;
    m_init_Xk = m_Xk;

    this->m_isInitPara = true;//Not initialized after
}

//Change the size of the SRIF parameter (only PPP can change paramater)
void SRIFAlgorithm::changeSRIFPara( QVector< SatlitData > &epochSatlitData,QVector< int >oldPrnFlag, int preEpochLen)
{
    int epochLenLB = epochSatlitData.length();
    m_Phi.resize(m_const_param+epochLenLB,m_const_param+epochLenLB);
    m_Phi.setZero();
    m_Phi.setIdentity(m_const_param+epochLenLB,m_const_param+epochLenLB);
    m_Phi_Inv = m_Phi.inverse();
    //Xk_1 change
    VectorXd tempXk_1 = m_Xk;
    m_Xk.resize(epochLenLB+m_const_param);
    m_Xk.setZero();
    //Xk.resize(epochLenLB+5);
    for (int i = 0;i < m_const_param;i++)
        m_Xk(i) = tempXk_1(i);
    for (int i = 0;i<epochLenLB;i++)
    {
        if (oldPrnFlag.at(i)!=-1)//Save the old satellite ambiguity
            m_Xk(m_const_param+i) = tempXk_1(oldPrnFlag.at(i)+m_const_param);
        else
        {//New satellite ambiguity calculation
            SatlitData oneStalit = epochSatlitData.at(i);
            m_Xk(m_const_param+i) = (oneStalit.PP3 - oneStalit.LL3)/M_GetLamta3(oneStalit.Frq[0],oneStalit.Frq[1]);
        }
    }
    //m_Rwk system noise initialization
    m_Rwk.resize(m_const_param+epochLenLB,m_const_param+epochLenLB);
    m_Rwk.setZero();
    m_Rwk(0,0) = 1e10; m_Rwk(1,1) = 1e10; m_Rwk(2,2) = 1e10;
    m_Rwk(3,3) = sqrt(1/m_zwd_Qw);//Zenith tropospheric residual variance 3e-8
    for(int i = 4; i < m_const_param;i++) m_Rwk(i,i) = sqrt(1/m_clk_Qw); // for clock
    for(int i = m_const_param;i < m_const_param+epochLenLB;i++)// for Ambiguity
        m_Rwk(i,i) = sqrt(1/m_amb_Qw);
    // juge is kinematic
    if(m_SRIF_MODEL == SRIF_MODEL::PPP_KINEMATIC)
    {
        m_Rwk(0,0) = sqrt(1/m_xyz_dynamic_Qw);
        m_Rwk(1,1) = sqrt(1/m_xyz_dynamic_Qw);
        m_Rwk(2,2) = sqrt(1/m_xyz_dynamic_Qw);
    }

    // init m_G of m_Rwk
    m_G.resize(m_const_param+epochLenLB,m_const_param+epochLenLB);
    m_G.setIdentity();

    //Reset Rk_1 observation noise matrix (reset on the outside, no need to repeat reset here)
    //The saved state covariance matrix Pk_1 is increased or decreased (here is more complicated, the main idea is to take out old satellite data, and initialize the new satellite data)
    MatrixXd tempPk_1 = m_Rp, tempZp_1 = m_Zp;
    m_Rp.resize(m_const_param+epochLenLB,m_const_param+epochLenLB);
    m_Rp.setZero();
    m_Zp.resize(m_const_param+epochLenLB,1);
    m_Zp.setZero();
    //If the number of satellites changes
    //if satlite number lost, should change tempZp_1 first!
    QVector< int > lost_satNum;//store lost satlite number
    for (int i = 0;i < preEpochLen;i++)
    {
        bool is_find = false;
        for(int j = 0;j < epochLenLB;j++)
        {
            int flag = oldPrnFlag.at(j);
            if(i == flag)
            {
                is_find = true;
                break;
            }
        }
        if(!is_find) lost_satNum.append(i);
    }
    // if have lost number change tempZp_1(previous epoch m_Xk sored in tempXk_1)
    for(int i = 0; i < lost_satNum.length();i++)
    {
        int lostNumber = lost_satNum.at(i) + m_const_param;// 5 is The first five parameters
        if(lostNumber + 1 > tempPk_1.cols()) break;
        for(int k = 0; k < tempZp_1.rows();k++)
        {
            tempZp_1(k, 0) = tempZp_1(k, 0) - tempPk_1(k, lostNumber)*tempXk_1(lostNumber,0);
        }
    }

    for (int i = 0;i < m_const_param;i++)
    {
        for (int j = 0;j < m_const_param;j++)
            m_Rp(i,j) = tempPk_1(i,j);
        m_Zp(i,0) = tempZp_1(i,0);
    }

    for (int n = 0; n < epochLenLB;n++)
    {
        int flag = oldPrnFlag.at(n);
        if ( flag != -1)//Description: The previous epoch contains this satellite data and needs to be taken from tempPk_1
        {
            flag += m_const_param;//The number of rows of this satellite in the original data tempPk_1
            m_Zp(n+m_const_param,0) = tempZp_1(flag,0);// save old Zp tp new Zp
            for (int i = 0;i < tempPk_1.cols();i++)
            {//Take out from tempPk_1 and skip the data with oldPrnFlag -1
                if (i < m_const_param)
                {
                    m_Rp(n+m_const_param,i) = tempPk_1(flag,i);
                    m_Rp(i,n+m_const_param) = tempPk_1(i,flag);
                }
                else
                {
                    int findCols = i - m_const_param,saveFlag = -1;
                    //Find if the data exists in the old linked list and where it will be saved
                    for (int m = 0;m < oldPrnFlag.length();m++)
                    {
                        if (findCols == oldPrnFlag.at(m))
                        {
                            saveFlag = m;
                            break;
                        }
                    }
                    if (saveFlag!=-1)
                    {
                        m_Rp(n+m_const_param,saveFlag+m_const_param) = tempPk_1(flag,i);
                        //Pk_1(saveFlag+5,n+5) = tempPk_1(i,flag);
                    }

                }//if (i < 5)
            }//for (int i = 0;i < tempPk_1.cols();i++)

        }
        else
        {
            //New satellite ambiguity calculation
            SatlitData oneStalit = epochSatlitData.at(n);
            double oneStalit_lamda = M_GetLamta3(oneStalit.Frq[0],oneStalit.Frq[1]);
            m_Rp(n+m_const_param,n+m_const_param) = oneStalit_lamda;
            m_Zp(n+m_const_param,0) = m_Xk(n+m_const_param,0) * oneStalit_lamda;

            m_Rwk(n+m_const_param, n+m_const_param) = sqrt(m_amb_Qw); // set new ambiguity noise matrix m_Rwk (this's important)

            for (int i = 0;i < m_const_param;i++)
            {
                m_Rp(n+m_const_param,i) = 0;
                m_Rp(i,n+m_const_param) = 0;
            }
        }
    }//Pk_1 saves the data
    MatrixXd error = m_Rp*m_Xk - m_Zp;
//    m_matrix.writeCSV("m_Rp.csv", m_Rp);
//    m_matrix.writeCSV("tempPk_1.csv", tempPk_1);

    m_VarChang = true;
}



//Change the size of the SRIF parameter (only PPP can change paramater)
void SRIFAlgorithm::changeSRIFPara_NoCombination( QVector< SatlitData > &epochSatlitData,QVector< int >oldPrnFlag, int preEpochLen)
{
    int epochLenLB = epochSatlitData.length();
    m_Phi.resize(m_const_param+3*epochLenLB,m_const_param+3*epochLenLB);
    m_Phi.setZero();
    m_Phi.setIdentity();
    m_Phi_Inv = m_Phi.inverse();
    //Xk_1 change
    VectorXd tempXk_1 = m_Xk;
    m_Xk.resize(3*epochLenLB+m_const_param);
    m_Xk.setZero();
    //Xk.resize(epochLenLB+5);
    for (int i = 0;i < m_const_param;i++)
        m_Xk(i) = tempXk_1(i);
    for (int i = 0;i<epochLenLB;i++)
    {
        if (oldPrnFlag.at(i)!=-1)//Save the old satellite ION and L1 and L2 ambiguity
        {
            m_Xk(m_const_param+i) = tempXk_1(oldPrnFlag.at(i)+m_const_param);// for ION
            m_Xk(m_const_param+epochLenLB+i) = tempXk_1(oldPrnFlag.at(i)+ preEpochLen + m_const_param);// for L1 ambiguity
            m_Xk(m_const_param+2*epochLenLB+i) = tempXk_1(oldPrnFlag.at(i)+ 2*preEpochLen + m_const_param);// for L2 ambiguity
        }

        else
        {//New satellite ambiguity calculation
            SatlitData oneStalit = epochSatlitData.at(i);
            double F1 = oneStalit.Frq[0], F2 = oneStalit.Frq[1];
            m_Xk(m_const_param+i) = (oneStalit.C1 - oneStalit.C2)/(1 - (F1*F1)/(F2*F2));
            m_Xk(m_const_param+epochLenLB+i) = (oneStalit.C1 - oneStalit.L1)/oneStalit.Frq[0];// new L1 ambiguity
            m_Xk(m_const_param+2*epochLenLB+i) = (oneStalit.C2 - oneStalit.L2)/oneStalit.Frq[1];// new L2 ambiguity
        }
    }
    //m_Rwk system noise initialization
    m_Rwk.resize(m_const_param+3*epochLenLB,m_const_param+3*epochLenLB);
    m_Rwk.setZero(); 
    m_Rwk(0,0) = 1e10; m_Rwk(1,1) = 1e10; m_Rwk(2,2) = 1e10;
    m_Rwk(3,3) = sqrt(1/m_zwd_Qw);//Zenith tropospheric residual variance
    for(int i = 4; i < m_const_param;i++) m_Rwk(i,i) =  sqrt(1/m_clk_Qw); // for clock
    for(int i = m_const_param;i < m_const_param+epochLenLB;i++) m_Rwk(i,i) =  sqrt(1/m_ion_Qw);// for ION
    for(int i = m_const_param+epochLenLB;i < m_const_param+3*epochLenLB;i++) m_Rwk(i,i) =  sqrt(1/m_amb_Qw);// for AMB

    // juge is kinematic
    if(m_SRIF_MODEL == SRIF_MODEL::PPP_KINEMATIC)
    {
        m_Rwk(0,0) = 1/m_xyz_dynamic_Qw;
        m_Rwk(1,1) = 1/m_xyz_dynamic_Qw;
        m_Rwk(2,2) = 1/m_xyz_dynamic_Qw;
    }

    // init m_G of m_Rwk
    m_G.resize(m_const_param+3*epochLenLB,m_const_param+3*epochLenLB);
    m_G.setIdentity();

    //Reset Rk_1 observation noise matrix (reset on the outside, no need to repeat reset here)
    //The saved state covariance matrix Pk_1 is increased or decreased (here is more complicated, the main idea is to take out old satellite data, and initialize the new satellite data)
    MatrixXd tempPk_1 = m_Rp, tempZp_1 = m_Zp;
    m_Rp.resize(m_const_param+3*epochLenLB,m_const_param+3*epochLenLB);
    m_Rp.setZero();
    m_Zp.resize(m_const_param+3*epochLenLB,1);
    m_Zp.setZero();
    //If the number of satellites changes
    //if satlite number lost, should change tempZp_1 first!
    QVector< int > lost_satNum;//store lost satlite number
    for (int i = 0;i < preEpochLen;i++)
    {
        bool is_find = false;
        for(int j = 0;j < epochLenLB;j++)
        {
            int flag = oldPrnFlag.at(j);
            if(i == flag)
            {
                is_find = true;
                break;
            }
        }
        if(!is_find) lost_satNum.append(i);
    }
    // if have lost number change tempZp_1(previous epoch m_Xk sored in tempXk_1)
    for(int i = 0; i < lost_satNum.length();i++)
    {
        int lostNumber = lost_satNum.at(i) + m_const_param;// 5 is The first five parameters
        if(lostNumber + 1 > tempPk_1.cols()) break;
        for(int k = 0; k < tempZp_1.rows();k++)
        {
            //
            tempZp_1(k, 0) = tempZp_1(k, 0) - tempPk_1(k, lostNumber)*tempXk_1(lostNumber,0)
                    - tempPk_1(k, lostNumber+preEpochLen)*tempXk_1(lostNumber+preEpochLen,0)
                    - tempPk_1(k, lostNumber+2*preEpochLen)*tempXk_1(lostNumber+2*preEpochLen,0);
        }
    }

    for (int i = 0;i < m_const_param;i++)
    {
        for (int j = 0;j < m_const_param;j++)
            m_Rp(i,j) = tempPk_1(i,j);
        m_Zp(i,0) = tempZp_1(i,0);
    }

    for (int n = 0; n < epochLenLB;n++)
    {
        int flag = oldPrnFlag.at(n);
        if ( flag != -1)//Description: The previous epoch contains this satellite data and needs to be taken from tempPk_1
        {
            flag += m_const_param;//The number of rows of this satellite in the original data tempPk_1
            m_Zp(n+m_const_param,0) = tempZp_1(flag,0);// save old Zp tp new Zp for ION
            m_Zp(n+m_const_param+epochLenLB,0) = tempZp_1(flag+preEpochLen,0);// save old Zp tp new Zp for L1 amb
            m_Zp(n+m_const_param+2*epochLenLB,0) = tempZp_1(flag+2*preEpochLen,0);// save old Zp tp new Zp for L2 amb
            for (int i = 0;i < tempPk_1.cols();i++)
            {//Take out from tempPk_1 and skip the data with oldPrnFlag -1
                if (i < m_const_param)
                {
                    // for ION
                    m_Rp(n+m_const_param,i) = tempPk_1(flag,i);
                    m_Rp(i,n+m_const_param) = tempPk_1(i,flag);
                    // for L1 AMB
                    m_Rp(n+m_const_param+epochLenLB,i) = tempPk_1(flag+preEpochLen,i);
                    m_Rp(i,n+m_const_param+epochLenLB) = tempPk_1(i,flag+preEpochLen);
                    // for L2 AMB
                    m_Rp(n+m_const_param+2*epochLenLB,i) = tempPk_1(flag+2*preEpochLen,i);
                    m_Rp(i,n+m_const_param+2*epochLenLB) = tempPk_1(i,flag+2*preEpochLen);
                }
                else
                {
                    int findCols = i - m_const_param,saveFlag = -1;
                    //Find if the data exists in the old linked list and where it will be saved
                    for (int m = 0;m < oldPrnFlag.length();m++)
                    {
                        if (findCols == oldPrnFlag.at(m))
                        {
                            saveFlag = m;
                            break;
                        }
                    }
                    if (saveFlag!=-1)
                    {
//                        qDebug() <<"(" << flag << "," << i << ") -> " <<" (" << n+m_const_param << "," << saveFlag+m_const_param << ")";
                        m_Rp(n+m_const_param,saveFlag+m_const_param) = tempPk_1(flag,i);// for ION
                        m_Rp(n+m_const_param,saveFlag+m_const_param+epochLenLB) = tempPk_1(flag,i+preEpochLen);
                        m_Rp(n+m_const_param,saveFlag+m_const_param+2*epochLenLB) = tempPk_1(flag,i+2*preEpochLen);

                        m_Rp(n+m_const_param+epochLenLB,saveFlag+m_const_param) = tempPk_1(flag+preEpochLen,i);// for L1 AMB
                        m_Rp(n+m_const_param+epochLenLB,saveFlag+m_const_param+epochLenLB) = tempPk_1(flag+preEpochLen,i+preEpochLen);// for L1 AMB
                        m_Rp(n+m_const_param+epochLenLB,saveFlag+m_const_param+2*epochLenLB) = tempPk_1(flag+preEpochLen,i+2*preEpochLen);// for L1 AMB

                        m_Rp(n+m_const_param+2*epochLenLB,saveFlag+m_const_param) = tempPk_1(flag+2*preEpochLen,i);// for L2 AMB
                        m_Rp(n+m_const_param+2*epochLenLB,saveFlag+m_const_param+epochLenLB) = tempPk_1(flag+2*preEpochLen,i+preEpochLen);// for L2 AMB
                        m_Rp(n+m_const_param+2*epochLenLB,saveFlag+m_const_param+2*epochLenLB) = tempPk_1(flag+2*preEpochLen,i+2*preEpochLen);// for L2 AMB
                    }

                }//if (i < 5)
            }//for (int i = 0;i < tempPk_1.cols();i++)

        }
        else
        {
            //New satellite ambiguity calculation
            SatlitData oneStalit = epochSatlitData.at(n);
            double oneStalit_lamda1 = M_C/oneStalit.Frq[0],oneStalit_lamda2 = M_C/oneStalit.Frq[1];
            // for ION
            m_Rp(n+m_const_param,n+m_const_param) = 1;
            m_Zp(n+m_const_param,0) = m_Xk(n+m_const_param,0) * 1;
            // for L1 amb
            m_Rp(n+m_const_param+epochLenLB,n+m_const_param+epochLenLB) = oneStalit_lamda1;
            m_Zp(n+m_const_param+epochLenLB,0) = m_Xk(n+m_const_param+epochLenLB,0) * oneStalit_lamda1;
            // for L2 amb
            m_Rp(n+m_const_param+2*epochLenLB,n+m_const_param+2*epochLenLB) = oneStalit_lamda2;
            m_Zp(n+m_const_param+2*epochLenLB,0) = m_Xk(n+m_const_param+2*epochLenLB,0) * oneStalit_lamda2;

            // set Rwk
            m_Rwk(n+m_const_param, n+m_const_param) = sqrt(m_ion_Qw); // set new ION noise matrix m_Rwk (this's important)
            m_Rwk(n+m_const_param+epochLenLB, n+m_const_param+epochLenLB) = sqrt(m_amb_Qw); // set new L1 ambiguity noise matrix m_Rwk (this's important)
            m_Rwk(n+m_const_param+2*epochLenLB, n+m_const_param+2*epochLenLB) = sqrt(m_amb_Qw); // set new L1 ambiguity noise matrix m_Rwk (this's important)

            for (int i = 0;i < m_const_param;i++)
            {
                m_Rp(n+m_const_param+epochLenLB,i) = 0;
                m_Rp(i,n+m_const_param+epochLenLB) = 0;
                m_Rp(n+m_const_param+2*epochLenLB,i) = 0;
                m_Rp(i,n+m_const_param+2*epochLenLB) = 0;
            }
        }
    }//Pk_1 saves the data
    MatrixXd error = m_Rp*m_Xk - m_Zp;
//    m_matrix.writeCSV("m_Rp.csv", m_Rp);
//    m_matrix.writeCSV("tempPk_1.csv", tempPk_1);

    m_VarChang = true;
}

// use least square method solver B*X = L
void SRIFAlgorithm::ls_solver(QVector< SatlitData > &currEpoch, double *m_ApproxRecPos)
{// this function will change m_ApproxRecPos data
    MatrixXd mat_B, mat_P, B1;
    Vector3d temp_Xk_1, diff_Xk;
    VectorXd ls_Xk, Vct_L, L1;// this ls_Xk contains [dX,dY,dZ,dTrop,dClock,N1,N2,..Nn]
    int loop_max = 20, epochLenLB = 0;// max loop iter time
    double ApproxRecPos[3] = {0};
    temp_Xk_1.setZero();
    epochLenLB = currEpoch.length();
    while(loop_max > 0)
    {
        Obtaining_equation(currEpoch, ApproxRecPos, mat_B, Vct_L, mat_P);
        B1 = mat_B.block(epochLenLB, 0, epochLenLB, m_const_param);
        L1 = Vct_L.tail(epochLenLB);
        ls_Xk = (B1.transpose()*B1).inverse()*B1.transpose()*L1;
//        ls_Xk = (mat_B.transpose()*mat_B).inverse()*mat_B.transpose()*Vct_L;
        // only compute [dX,dY,dZ]
        diff_Xk(0) = ls_Xk(0) - temp_Xk_1(0);
        diff_Xk(1) = ls_Xk(1) - temp_Xk_1(1);
        diff_Xk(2) = ls_Xk(2) - temp_Xk_1(2);
        // update m_ApproxRecPos and temp_Xk_1
        ApproxRecPos[0] = ApproxRecPos[0] + ls_Xk(0);
        ApproxRecPos[1] = ApproxRecPos[1] + ls_Xk(1);
        ApproxRecPos[2] = ApproxRecPos[2] + ls_Xk(2);
        temp_Xk_1(0) = ls_Xk(0);
        temp_Xk_1(1) = ls_Xk(1);
        temp_Xk_1(2) = ls_Xk(2);
        if(diff_Xk.cwiseAbs().maxCoeff() < 1)
            break;
        loop_max--;
    }
//    qDebug() << loop_max;
    m_ApproxRecPos[0] = ApproxRecPos[0];
    m_ApproxRecPos[1] = ApproxRecPos[1];
    m_ApproxRecPos[2] = ApproxRecPos[2];

}

// get matrix B and observer L
void SRIFAlgorithm::Obtaining_equation(QVector< SatlitData > &currEpoch, double *m_ApproxRecPos, MatrixXd &mat_B, VectorXd &Vct_L,
                             MatrixXd &mat_P)
{
    int epochLenLB = currEpoch.length(), const_num = 3;
    MatrixXd B, P;
    VectorXd L, sys_len;
    sys_len.resize(m_sys_str.length());
    sys_len.setZero();
    switch(m_SRIF_MODEL)
    {
    case SRIF_MODEL::SPP_STATIC:
    case SRIF_MODEL::SPP_KINEMATIC:
        B.resize(epochLenLB,m_const_param);
        P.resize(epochLenLB,epochLenLB);
        L.resize(epochLenLB);
        const_num = 3;// 3 is conntain [dx,dy,dz]
        break;
    case SRIF_MODEL::PPP_KINEMATIC:
    case SRIF_MODEL::PPP_STATIC:
        B.resize(2*epochLenLB,epochLenLB+m_const_param);
        P.resize(2*epochLenLB,2*epochLenLB);
        L.resize(2*epochLenLB);
        const_num = 4;// 4 is conntain [dx,dy,dz,mf]
        break;
    default:
        ErroTrace("QKalmanFilter::Obtaining_equation you should use setModel().");
        break;
    }
    // init matrix
    B.setZero();
    L.setZero();
    P.setIdentity();
    bool is_find_base_sat = false;
    for (int i = 0; i < epochLenLB;i++)
    {
        SatlitData oneSatlit = currEpoch.at(i);
        double li = 0,mi = 0,ni = 0,p0 = 0,dltaX = 0,dltaY = 0,dltaZ = 0;
        dltaX = oneSatlit.X - m_ApproxRecPos[0];
        dltaY = oneSatlit.Y - m_ApproxRecPos[1];
        dltaZ = oneSatlit.Z - m_ApproxRecPos[2];
        p0 = qSqrt(dltaX*dltaX+dltaY*dltaY+dltaZ*dltaZ);
        // compute li mi ni
        li = dltaX/p0;mi = dltaY/p0;ni = dltaZ/p0;
        //Correction of each
        double dlta = 0;
        dlta =  - oneSatlit.StaClock + oneSatlit.SatTrop - oneSatlit.Relativty -
            oneSatlit.Sagnac - oneSatlit.TideEffect - oneSatlit.AntHeight;
        // set B L P
        double LP_whight  = m_LP_whight;
        switch(m_SRIF_MODEL)
        {
        case SRIF_MODEL::SPP_STATIC:
        case SRIF_MODEL::SPP_KINEMATIC:
            //Computational B matrix
            //L3 carrier matrix
            B(i,0) = li;B(i,1) = mi;B(i,2) = ni;B(i,3) = -1;
            // debug by xiaogongwei 2019.04.03 for ISB
            for(int k = 1; k < m_sys_str.length();k++)
            {
                if(m_sys_str[k] == oneSatlit.SatType)
                {
                    B(i,3+k) = -1;
                    sys_len[k] = 1;//good no zeros cloumn in B,sys_lenmybe 0 1 1 0(debug by xiaogongwei 2019.04.09 for ISB)
                }

            }
            // debug by xiaogongwei 2019.04.10 is exist base system satlite clk
            if(m_sys_str[0] == oneSatlit.SatType)
                is_find_base_sat = true;
            //Pseudorange code L
            if(SRIF_SMOOTH_RANGE::SMOOTH == m_SRIF_SMOOTH_RANGE)
            {
                L(i) = p0 - oneSatlit.PP3_Smooth + dlta;
                // Pseudorange code L calculation weight matrix PP3
                P(i, i) = 1 / oneSatlit.PP3_Smooth_Q;// Pseudo-range right
            }
            else
            {
                L(i) = p0 - oneSatlit.PP3 + dlta;
                // Computing weight matrix P
                P(i, i) = oneSatlit.SatWight;// Pseudo-range right
            }
            break;
        case SRIF_MODEL::PPP_KINEMATIC:
        case SRIF_MODEL::PPP_STATIC:
            //Computational B matrix
            //L3 carrier matrix
            B(i,0) = li;B(i,1) = mi;B(i,2) = ni;B(i,3) = -oneSatlit.StaTropMap;B(i,4) = -1;
            for (int n = 0;n < epochLenLB;n++)//The diagonal part of the rear part initializes the wavelength of Lamta3, and the rest is 0.
                if (i == n)
                    B(i,m_const_param+n) = M_GetLamta3(oneSatlit.Frq[0],oneSatlit.Frq[1]);//LL3 wavelength
//                else
//                    B(i,m_const_param+n) = 0;
            //P3 pseudorange code matrix
            B(i+epochLenLB,0) = li;B(i+epochLenLB,1) = mi;B(i+epochLenLB,2) = ni;B(i+epochLenLB,3) = -oneSatlit.StaTropMap;B(i+epochLenLB,4) = -1;
//            for (int n = 0;n < epochLenLB;n++)//The latter part is all 0
//                B(i+epochLenLB,m_const_param+n) = 0;
            // debug by xiaogongwei 2019.04.03 for ISB
            for(int k = 1; k < m_sys_str.length();k++)
            {
                if(m_sys_str[k] == oneSatlit.SatType)
                {
                    B(i,4+k) = -1;
                    B(i+epochLenLB,4+k) = -1;
                    sys_len[k] = 1;//good no zeros cloumn in B,sys_lenmybe 0 1 1 0(debug by xiaogongwei 2019.04.09 for ISB)
                }
            }
            // debug by xiaogongwei 2019.04.10 is exist base system satlite clk
            if(m_sys_str[0] == oneSatlit.SatType)
                is_find_base_sat = true;
            //Carrier L  Pseudorange code L
            L(i) = p0 - oneSatlit.LL3 + dlta;
            L(i+epochLenLB) = p0 - oneSatlit.PP3 + dlta;
            // Computing weight matrix P
//            if(oneSatlit.UTCTime.epochNum <= 100) LP_whight = 1e6;// for convergence
            P(i, i) = oneSatlit.SatWight * LP_whight;// Carrier weight
            P(i + epochLenLB, i + epochLenLB) = oneSatlit.SatWight;// Pseudo-range right
            break;
        default:
            ErroTrace("SRIFAlgorithm::Obtaining_equation you should use setModel().");
            break;
        }//switch(m_SRIF_MODEL)

    }//B,L is calculated
    // save data to mat_B
    mat_B = B;
    Vct_L = L;
    mat_P = P;
    // debug by xiaogongwei 2019.04.04
    int no_zero = sys_len.size() - 1 - sys_len.sum();
    if(no_zero > 0 || !is_find_base_sat)
    {
        int new_hang = B.rows() + no_zero, new_lie = B.cols(), flag = 0;
        if(!is_find_base_sat) new_hang++; // debug by xiaogongwei 2019.04.10 is exist base system satlite clk
        mat_B.resize(new_hang,new_lie);
        mat_P.resize(new_hang,new_hang);
        Vct_L.resize(new_hang);

        mat_B.setZero();
        Vct_L.setZero();
        mat_P.setIdentity();
        // debug by xiaogongwei 2019.04.10 is exist base system satlite clk
        if(!is_find_base_sat)
        {
            for(int i = 0;i < B.rows();i++)
                B(i, const_num) = 0;
            mat_B(mat_B.rows() - 1, const_num) = 1;
        }
        mat_B.block(0,0,B.rows(),B.cols()) = B;
        mat_P.block(0,0,P.rows(),P.cols()) = P;
        Vct_L.head(L.rows()) = L;

        for(int i = 1; i < sys_len.size();i++)
        {
            if(0 == sys_len[i])
            {
                mat_B(B.rows()+flag, const_num+i) = 1;
                flag++;
            }
        }
    }//if(no_zero > 0)
}


// get matrix B and observer L for No Combination
void SRIFAlgorithm::Obtaining_equation_NoCombination(QVector< SatlitData > &currEpoch, double *m_ApproxRecPos, MatrixXd &mat_B, VectorXd &Vct_L,
                             MatrixXd &mat_P)
{
    int epochLenLB = currEpoch.length(), const_num = 3;
    MatrixXd B, P;
    VectorXd L, sys_len;
    sys_len.resize(m_sys_str.length());
    sys_len.setZero();
    switch(m_SRIF_MODEL)
    {
    case SRIF_MODEL::SPP_STATIC:
    case SRIF_MODEL::SPP_KINEMATIC:
        B.resize(2*epochLenLB,m_const_param+epochLenLB);
        P.resize(2*epochLenLB,2*epochLenLB);
        L.resize(2*epochLenLB);
        const_num = 3;// 3 is conntain [dx,dy,dz]
        break;
    case SRIF_MODEL::PPP_KINEMATIC:
    case SRIF_MODEL::PPP_STATIC:
        B.resize(4*epochLenLB,3*epochLenLB+m_const_param);
        P.resize(4*epochLenLB,4*epochLenLB);
        L.resize(4*epochLenLB);
        const_num = 4;// 4 is conntain [dx,dy,dz,mf]
        break;
    default:
        ErroTrace("QKalmanFilter::Obtaining_equation you should use setModel().");
        break;
    }
    // init matrix
    B.setZero();
    L.setZero();
    P.setIdentity();

    bool is_find_base_sat = false;
    for (int i = 0; i < epochLenLB;i++)
    {
        SatlitData oneSatlit = currEpoch.at(i);
        double li = 0,mi = 0,ni = 0,p0 = 0,dltaX = 0,dltaY = 0,dltaZ = 0;
        dltaX = oneSatlit.X - m_ApproxRecPos[0];
        dltaY = oneSatlit.Y - m_ApproxRecPos[1];
        dltaZ = oneSatlit.Z - m_ApproxRecPos[2];
        p0 = qSqrt(dltaX*dltaX+dltaY*dltaY+dltaZ*dltaZ);
        // compute li mi ni
        li = dltaX/p0;mi = dltaY/p0;ni = dltaZ/p0;
        //Correction of each
        double dlta = 0;
        dlta =  - oneSatlit.StaClock + oneSatlit.SatTrop - oneSatlit.Relativty -
            oneSatlit.Sagnac - oneSatlit.TideEffect - oneSatlit.AntHeight;
        // set B L P
        double LP_whight  = m_LP_whight;
        double F1 = oneSatlit.Frq[0], F2 = oneSatlit.Frq[1];
        double lamda1 = M_C/F1, lamda2 = M_C/F2;
        switch(m_SRIF_MODEL)
        {
        case SRIF_MODEL::SPP_STATIC:
        case SRIF_MODEL::SPP_KINEMATIC:
            //Computational B matrix
            //L3 carrier matrix
            B(2*i,0) = li;B(2*i,1) = mi;B(2*i,2) = ni;B(2*i,3) = -1;
            B(2*i+1,0) = li;B(2*i+1,1) = mi;B(2*i+1,2) = ni;B(2*i+1,3) = -1;
            B(2*i,i+m_const_param) = -1;// ION for P1
            B(2*i+1,i+m_const_param) = -(F1*F1)/(F2*F2);// ION for P2
            // debug by xiaogongwei 2019.04.03 for ISB
            for(int k = 1; k < m_sys_str.length();k++)
            {
                if(m_sys_str[k] == oneSatlit.SatType)
                {
                    B(2*i,3+k) = -1;
                    B(2*i+1,3+k) = -1;
                    sys_len[k] = 1;//good no zeros cloumn in B,sys_lenmybe 0 1 1 0(debug by xiaogongwei 2019.04.09 for ISB)
                }
            }
            // debug by xiaogongwei 2019.04.10 is exist base system satlite clk
            if(m_sys_str[0] == oneSatlit.SatType)
                is_find_base_sat = true;
            //Pseudorange code not use KALMAN_SMOOTH_RANGE::SMOOTH
            //Pseudorange code L
            if(SRIF_SMOOTH_RANGE::SMOOTH == m_SRIF_SMOOTH_RANGE)
            {
                L(2*i) = p0 - oneSatlit.CC1_Smooth + dlta;
                L(2*i+1) = p0 - oneSatlit.CC2_Smooth + dlta;
                P(2*i, 2*i) = 1 / oneSatlit.CC1_Smooth_Q;// Pseudo-range Wight
                P(2*i+1, 2*i+1) = 1 / oneSatlit.CC2_Smooth_Q;// Pseudo-range Wight
            }
            else
            {
                L(2*i) = p0 - oneSatlit.C1 + dlta;
                L(2*i+1) = p0 - oneSatlit.C2 + dlta;
                P(2*i, 2*i) = oneSatlit.SatWight;
                P(2*i+1, 2*i+1) = oneSatlit.SatWight;
            }
            break;
        case SRIF_MODEL::PPP_KINEMATIC:
        case SRIF_MODEL::PPP_STATIC:
            //Computational B matrix
            //L carrier matrix
            B(2*i,0) = li;B(2*i,1) = mi;B(2*i,2) = ni;B(2*i,3) = -oneSatlit.StaTropMap;B(2*i,4) = -1; // L1
            B(2*i+1,0) = li;B(2*i+1,1) = mi;B(2*i+1,2) = ni;B(2*i+1,3) = -oneSatlit.StaTropMap;B(2*i+1,4) = -1; // L2
            B(2*i,i+m_const_param) = +1;// ION for L1
            B(2*i+1,i+m_const_param) = +(F1*F1)/(F2*F2);// ION for L2
            B(2*i,i+m_const_param+epochLenLB) = lamda1;// N1 for L1
            B(2*i+1,i+m_const_param+2*epochLenLB) = lamda2;// N2 for L2

            //P pseudorange code matrix
            B(2*i+2*epochLenLB,0) = li;B(2*i+2*epochLenLB,1) = mi;B(2*i+2*epochLenLB,2) = ni;B(2*i+2*epochLenLB,3) = -oneSatlit.StaTropMap;B(2*i+2*epochLenLB,4) = -1;
            B(2*i+2*epochLenLB+1,0) = li;B(2*i+2*epochLenLB+1,1) = mi;B(2*i+2*epochLenLB+1,2) = ni;B(2*i+2*epochLenLB+1,3) = -oneSatlit.StaTropMap;B(2*i+2*epochLenLB+1,4) = -1;
            B(2*i+2*epochLenLB,i+m_const_param) = -1;// ION for P1
            B(2*i+2*epochLenLB+1,i+m_const_param) = -(F1*F1)/(F2*F2);// ION for P2

            // debug by xiaogongwei 2019.04.03 for ISB
            for(int k = 1; k < m_sys_str.length();k++)
            {
                if(m_sys_str[k] == oneSatlit.SatType)
                {
                    B(2*i,const_num+k) = -1;
                    B(2*i+1,const_num+k) = -1;
                    B(2*i+2*epochLenLB,const_num+k) = -1;
                    B(2*i+2*epochLenLB+1,const_num+k) = -1;
                    sys_len[k] = 1;//good no zeros cloumn in B,sys_lenmybe 0 1 1 0(debug by xiaogongwei 2019.04.09 for ISB)
                }
            }
            // debug by xiaogongwei 2019.04.10 is exist base system satlite clk
            if(m_sys_str[0] == oneSatlit.SatType)
                is_find_base_sat = true;
            //Carrier L  pseudorange code L
            L(2*i) = p0 - oneSatlit.LL1 + dlta;
            L(2*i+1) = p0 - oneSatlit.LL2 + dlta;
            L(2*i+2*epochLenLB) = p0 - oneSatlit.CC1 + dlta;
            L(2*i+2*epochLenLB+1) = p0 - oneSatlit.CC2 + dlta;
            // Computing weight matrix P
            P(2*i, 2*i) = oneSatlit.SatWight * LP_whight;// Carrier L1 weight
            P(2*i+1, 2*i+1) = oneSatlit.SatWight * LP_whight;// Carrier L2 weight
            P(2*i + epochLenLB, 2*i + epochLenLB) = oneSatlit.SatWight;// Pseudo-range C1 weight
            P(2*i + epochLenLB+1, 2*i + epochLenLB+1) = oneSatlit.SatWight;// Pseudo-range C2 weight
            break;
        default:
            ErroTrace("QKalmanFilter::Obtaining_equation you should use setModel().");
            break;
        }//switch(m_KALMAN_MODEL)

    }//B, L is calculated
    // save data to mat_B
    mat_B = B;
    Vct_L = L;
    mat_P = P;
//    m_matrix.writeCSV("./csv/mat_B.csv", mat_B);
//    m_matrix.writeCSV("./csv/mat_P.csv", mat_P);
    // debug by xiaogongwei 2019.04.04
    int no_zero = sys_len.size() - 1 - sys_len.sum();
    if(no_zero > 0 || !is_find_base_sat)
    {
        int new_hang = B.rows() + no_zero, new_lie = B.cols(), flag = 0;
        if(!is_find_base_sat) new_hang++; // debug by xiaogongwei 2019.04.10 is exist base system satlite clk
        mat_B.resize(new_hang,new_lie);
        mat_P.resize(new_hang,new_hang);
        Vct_L.resize(new_hang);

        mat_B.setZero();
        Vct_L.setZero();
        mat_P.setIdentity();
        // debug by xiaogongwei 2019.04.10 is exist base system satlite clk
        if(!is_find_base_sat)
        {
            for(int i = 0;i < B.rows();i++)
                B(i, const_num) = 0;
            mat_B(mat_B.rows() - 1, const_num) = 1;
        }
        mat_B.block(0,0,B.rows(),B.cols()) = B;
        mat_P.block(0,0,P.rows(),P.cols()) = P;
        Vct_L.head(L.rows()) = L;

        for(int i = 1; i < sys_len.size();i++)
        {
            if(0 == sys_len[i])
            {
                mat_B(B.rows()+flag, const_num+i) = 1;
                flag++;
            }
        }
    }//if(no_zero > 0)
//    m_matrix.writeCSV("./csv/mat_B1.csv", mat_B);
//    m_matrix.writeCSV("./csv/mat_P1.csv", mat_P);
}

bool SRIFAlgorithm::SRIFforStatic(QVector< SatlitData > &preEpoch,QVector< SatlitData > &currEpoch,
                                  double *m_ApproxRecPos,VectorXd &X,MatrixXd &P)
{
    // use spp get postion
    if (!m_isInitPara)
    {
        m_SPP_Pos[0] = m_ApproxRecPos[0];
        m_SPP_Pos[1] = m_ApproxRecPos[1];
        m_SPP_Pos[2] = m_ApproxRecPos[2];
    }
    //judge is Kinematic
    if(m_SRIF_MODEL == SRIF_MODEL::SPP_KINEMATIC || m_SRIF_MODEL == SRIF_MODEL::PPP_KINEMATIC)
    {
        // we solver five parameter[dx,dy,dz,dTrop,dClock],so epochLenLB > 4
        m_SPP_Pos[0] = m_ApproxRecPos[0];
        m_SPP_Pos[1] = m_ApproxRecPos[1];
        m_SPP_Pos[2] = m_ApproxRecPos[2];
        // must set zero of [dx,dy,dy] int Kinematic
        m_Xk(0) = 0; m_Xk(1) = 0; m_Xk(2) = 0;
    }

    //save filter sate for Quality Control
    MatrixXd temp_Rp = m_Rp, temp_Zp = m_Zp,  temp_Phi_Inv = m_Phi_Inv, temp_G = m_G,
            temp_Phi = m_Phi, temp_Rwk = m_Rwk, temp_Q = m_Q;
    VectorXd temp_Xk = m_Xk;
    double temp_SPP_POS[3] = {0};
    memcpy(temp_SPP_POS, m_SPP_Pos, 3*sizeof(double));

    // use spp clk as priori value for base clk
    SRIFAlgorithm::SRIF_MODEL srif_model = getModel();
    if(srif_model == SRIFAlgorithm::SRIF_MODEL::SPP_STATIC || srif_model == SRIFAlgorithm::SRIF_MODEL::SPP_KINEMATIC)
        m_Xk(3) =  m_ApproxRecPos[3];
    else
        m_Xk(4) =  m_ApproxRecPos[3];
    // filter
    filter(preEpoch, currEpoch, X, P);

    // Quality Control
    bool gross_LC = true;
    int minSatNum = 5, max_iter = 10;
    if(m_SRIF_MODEL == SRIF_MODEL::SPP_STATIC || m_SRIF_MODEL == SRIF_MODEL::PPP_STATIC)
        minSatNum = 1;
    else
        minSatNum = 5;
    while(gross_LC)
    {
        // get B, wightP ,L
        MatrixXd B, wightP;
        VectorXd L, delate_LC;
        if(getPPPModel() == PPP_MODEL::PPP_Combination)
            Obtaining_equation(currEpoch, m_SPP_Pos, B, L, wightP);
        else if(getPPPModel() == PPP_MODEL::PPP_NOCombination)
            Obtaining_equation_NoCombination(currEpoch, m_SPP_Pos, B, L, wightP);

        // detect gross error
        int sat_len = currEpoch.length();
        if(m_SRIF_MODEL == SRIF_MODEL::SPP_STATIC || m_SRIF_MODEL == SRIF_MODEL::SPP_KINEMATIC)
        {
            gross_LC = m_qualityCtrl.VtPVCtrl_Filter_C(B, L, m_Xk, delate_LC, sat_len);// QC pesoderange
        }
        else
        {
            if(getPPPModel() == PPP_MODEL::PPP_Combination)
                gross_LC = m_qualityCtrl.VtPVCtrl_Filter_LC(B, L, m_Xk, delate_LC, sat_len);// QC for carrire and pesoderange
            else if(getPPPModel() == PPP_MODEL::PPP_NOCombination)
                gross_LC = m_qualityCtrl.VtPVCtrl_Filter_LC_NoCombination(B, L, m_Xk, delate_LC, sat_len);// QC for carrire and pesoderange
        }
        max_iter--;
        if(gross_LC == false || max_iter <= 0) break;
        // delate gross Errors Satlites form end for start.
        QVector<int> del_flag;
        for(int i = sat_len - 1; i >= 0;i--)
        {
            if(0 != delate_LC[i])
                del_flag.append(i);
        }
        // delete gross Errors
        int del_len = del_flag.length();
        if(currEpoch.length() - del_len >= minSatNum)
        {
            for(int i = 0; i < del_len;i++)
                currEpoch.remove(del_flag[i]);

            // restore filter state
            m_Rp = temp_Rp; m_Zp = temp_Zp; m_Phi_Inv = temp_Phi_Inv; m_G = temp_G;
            m_Phi = temp_Phi; m_Rwk = temp_Rwk; m_Q = temp_Q;
            m_Xk = temp_Xk;
            memcpy(m_SPP_Pos, temp_SPP_POS, 3*sizeof(double));
            filter(preEpoch, currEpoch, X, P);
        }
        else
        {
            break;
        }
    }

    // Calculate the filtered residuals and save them in the satellite structure
    // get B, wightP ,L
    MatrixXd B, wightP;
    VectorXd L, Vk;
    int sat_len = currEpoch.length();
    if(getPPPModel() == PPP_MODEL::PPP_Combination)
        Obtaining_equation(currEpoch, m_SPP_Pos, B, L, wightP);
    else if(getPPPModel() == PPP_MODEL::PPP_NOCombination)
        Obtaining_equation_NoCombination(currEpoch, m_SPP_Pos, B, L, wightP);
    Vk = B*m_Xk - L;
    if(m_SRIF_MODEL == SRIF_MODEL::SPP_STATIC || m_SRIF_MODEL == SRIF_MODEL::SPP_KINEMATIC)
    {
        if(getPPPModel() == PPP_MODEL::PPP_Combination)
        {
            for(int i = 0; i < sat_len;i++)
            {
                currEpoch[i].VLL3 = 0;
                currEpoch[i].VPP3 = Vk[i];
            }
        }
        else if(getPPPModel() == PPP_MODEL::PPP_NOCombination)
        {
            for(int i = 0; i < sat_len;i++)
            {
                currEpoch[i].VL1 = 0; currEpoch[i].VL2 = 0;
                currEpoch[i].VC1 = Vk[2*i]; currEpoch[i].VC2 = Vk[2*i+1];
            }
        }
    }
    else
    {
        if(getPPPModel() == PPP_MODEL::PPP_Combination)
        {
            for(int i = 0; i < sat_len;i++)
            {
                currEpoch[i].VLL3 = Vk[i];
                currEpoch[i].VPP3 = Vk[i+sat_len];
            }
        }
        else if(getPPPModel() == PPP_MODEL::PPP_NOCombination)
        {
            for(int i = 0; i < sat_len;i++)
            {
                currEpoch[i].VL1 = Vk[2*i]; currEpoch[i].VL2 = Vk[2*i+1];
                currEpoch[i].VC1 = Vk[2*i+2*sat_len]; currEpoch[i].VC2 = Vk[2*i+2*sat_len+1];
            }
        }

    }

    //Save the results of this epoch (does not contain initialization data)
    X = m_Xk;
    P =  (m_Rp.transpose()*m_Rp).inverse();
    if(gross_LC)
    {
        // restore filter state
        m_Rp = temp_Rp; m_Zp = temp_Zp; m_Phi_Inv = temp_Phi_Inv; m_G = temp_G;
        m_Phi = temp_Phi; m_Rwk = temp_Rwk; m_Q = temp_Q;
        m_Xk = temp_Xk;
        memcpy(m_SPP_Pos, temp_SPP_POS, 3*sizeof(double));
        X.setZero();
        P.setIdentity();
        P = P * 1e10;
        // add bad flag 999 for filter bad
        for(int i = 0;i < currEpoch.length();i++)
            currEpoch[i].EpochFlag = 999;
    }
    else
    {
        // update m_ApproxRecPos use SRIF
        m_ApproxRecPos[0] = m_SPP_Pos[0] + m_Xk(0);
        m_ApproxRecPos[1] = m_SPP_Pos[1] + m_Xk(1);
        m_ApproxRecPos[2] = m_SPP_Pos[2] + m_Xk(2);
    }

    return (!gross_LC);
}


void SRIFAlgorithm::filter(QVector< SatlitData > &preEpoch,QVector< SatlitData > &currEpoch, VectorXd &X,MatrixXd &P)
{
    int preEpochLen = preEpoch.length();
    int epochLenLB = currEpoch.length();
    // get B, wightP ,L
    MatrixXd B, wightP;
    VectorXd L;
    if(getPPPModel() == PPP_MODEL::PPP_Combination)
        Obtaining_equation(currEpoch, m_SPP_Pos, B, L, wightP);
    else if(getPPPModel() == PPP_MODEL::PPP_NOCombination)
        Obtaining_equation_NoCombination(currEpoch, m_SPP_Pos, B, L, wightP);


    //First epoch initialization  Fillter init
    if (0 == preEpochLen)
    {
        if(getPPPModel() == PPP_MODEL::PPP_Combination)
            initSRIFPara(currEpoch,B,L);
        else if(getPPPModel() == PPP_MODEL::PPP_NOCombination)
            initSRIFPara_NoCombination(currEpoch,B,L);

        // if have P matrix use P.this is back smooth
        if(P.rows() > 1)
        {
            MatrixXd temp_Rp_inv = P.llt().matrixL();
            m_Xk = X;
            m_Rp = temp_Rp_inv.inverse();
            m_Zp = m_Rp*m_Xk;
        }
        InitSRIF(m_Rp, m_Zp, m_Phi, m_G, m_Rwk);
    }

    //Update Rk_1 (the number of satellites has not changed at this time)
    if(getPPPModel() == PPP_MODEL::PPP_Combination)
        updatePk(currEpoch, B.rows());
    else if(getPPPModel() == PPP_MODEL::PPP_NOCombination)
        updatePk_NoCombination(currEpoch, B.rows());


    //Determine whether the number of satellites has changed (comparison of two epochs before and after)
    QVector< int > oldPrnFlag;//Compared with the location of the same satellite in the previous epoch, it is not found with -1
    bool isNewSatlite = false;
    isNewSatlite = isSatelliteChange(preEpoch, currEpoch, oldPrnFlag);

    //Using SRIF filtering
    // use R white B and L
    MatrixXd matB, matL;
    MatrixXd R_Pk;
    R_Pk.resize(m_Pk.rows(), m_Pk.cols());
    R_Pk.setZero();
    // chol factorization
    for(int i = 0; i < m_Pk.rows();i++)
        R_Pk(i, i) = sqrt(m_Pk(i,i));
    matB = R_Pk*B;
    matL = R_Pk*L;

    //Increase or decrease n satellites
    if(SRIF_MODEL::PPP_KINEMATIC == m_SRIF_MODEL ||  SRIF_MODEL::PPP_STATIC == m_SRIF_MODEL)
    {
        //Increase or decrease n satellites
        if (((preEpochLen != epochLenLB) || isNewSatlite ) && preEpochLen != 0)
        {
            if(getPPPModel() == PPP_MODEL::PPP_Combination)
                changeSRIFPara(currEpoch,oldPrnFlag, preEpochLen);//Update all SRIF parameter data sizes
            else if(getPPPModel() == PPP_MODEL::PPP_NOCombination)
                changeSRIFPara_NoCombination(currEpoch,oldPrnFlag, preEpochLen);//Update all SRIF parameter data sizes
        }
    }
    //Version SRIF filtering
    m_Xk = SRIFilter(matB, matL);// update m_Xk
}

//Determine whether the number of satellites has changed (comparison of two epochs before and after) debug by xiaogongwei 2019.04.29
bool SRIFAlgorithm::isSatelliteChange(QVector< SatlitData > &preEpoch,QVector< SatlitData > &currEpoch, QVector< int > &oldPrnFlag)
{
    int preEpochLen = preEpoch.length();
    int epochLenLB = currEpoch.length();
    //Determine whether the number of satellites has changed (comparison of two epochs before and after)
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
                oldPrnFlag.append(j);//Compared with the location of the same satellite in the previous epoch, it is not found with -1
                Isfind = true;
                oldSatLen++;
                break;
            }
        }
        if (!Isfind)
        {
            oldPrnFlag.append(-1);//Compared with the location of the same satellite in the previous epoch, it is not found with -1
            isNewSatlite = true;
        }
    }
    return isNewSatlite;
}

// update Rk(Observation Covariance)
void SRIFAlgorithm::updatePk(QVector< SatlitData > &currEpoch, int B_len)
{
    int epochLenLB = currEpoch.length();

    //Update Rk_1 (the number of satellites has not changed at this time)
    if(m_SRIF_MODEL == SRIF_MODEL::SPP_STATIC || m_SRIF_MODEL == SRIF_MODEL::SPP_KINEMATIC)
    {
        m_Pk.resize(B_len,B_len);
        m_Pk.setIdentity();
        for (int i = 0;i < epochLenLB;i++)
        {
            SatlitData oneSatlit = currEpoch.at(i);
            if(SRIF_SMOOTH_RANGE::SMOOTH == m_SRIF_SMOOTH_RANGE)
                m_Pk(i, i) = 1 / oneSatlit.PP3_Smooth_Q;//Smoothing the right of pseudo-range equation
            else
                m_Pk(i, i) = oneSatlit.SatWight;//Smoothing the right of pseudo-range equation
        }
    }
    else
    {
        m_Pk.resize(B_len,B_len);
        m_Pk.setIdentity();
        for (int i = 0;i < epochLenLB;i++)
        {
            SatlitData oneSatlit = currEpoch.at(i);
            m_Pk(i,i) = m_LP_whight*oneSatlit.SatWight;//Carrier equation weight (small noise)(Debug by xiaogongwei 2018.12.04;)
            m_Pk(i+epochLenLB,i+epochLenLB) = oneSatlit.SatWight;//Smoothing the right of pseudo-range equation(noise)
        }
    }
}

// update Pk(Observation wight)
void SRIFAlgorithm::updatePk_NoCombination(QVector< SatlitData > &currEpoch, int B_len)
{
    int epochLenLB = currEpoch.length();

    //Update Rk_1 (the number of satellites has not changed at this time)
    if(m_SRIF_MODEL == SRIF_MODEL::SPP_STATIC || m_SRIF_MODEL == SRIF_MODEL::SPP_KINEMATIC)
    {
        m_Pk.resize(B_len,B_len);
        m_Pk.setIdentity();
        for (int i = 0;i < epochLenLB;i++)
        {
            SatlitData oneSatlit = currEpoch.at(i);

            if(SRIF_SMOOTH_RANGE::SMOOTH == m_SRIF_SMOOTH_RANGE)
            {
                m_Pk(2*i, 2*i) = 1/oneSatlit.CC1_Smooth_Q;//wight of C1 pseudorange equations Reciprocal (noise)
                m_Pk(2*i+1, 2*i+1) = 1/oneSatlit.CC2_Smooth_Q;//wight C2 of pseudorange equations Reciprocal (noise)
            }
            else
            {
                m_Pk(2*i, 2*i) = oneSatlit.SatWight;//wight of C1  pseudorange equations Reciprocal (noise)
                m_Pk(2*i+1, 2*i+1) = oneSatlit.SatWight;//wight of C2 pseudorange equations Reciprocal (noise)
            }
        }
    }
    else
    {
        m_Pk.resize(B_len,B_len);
        m_Pk.setIdentity();
        for (int i = 0;i < epochLenLB;i++)
        {
            SatlitData oneSatlit = currEpoch.at(i);
            m_Pk(2*i,2*i) = m_LP_whight * oneSatlit.SatWight;//wight of carrier equation Reciprocal (small noise)// 1/25000 =4e-4
            m_Pk(2*i+1,2*i+1) = m_LP_whight * oneSatlit.SatWight;//wight of carrier equation Reciprocal (small noise)// 1/25000 =4e-4
            m_Pk(2*i+epochLenLB,2*i+epochLenLB) = oneSatlit.SatWight;//wight of pseudorange equations Reciprocal (noise)
            m_Pk(2*i+epochLenLB+1,2*i+epochLenLB+1) = oneSatlit.SatWight;//wight of pseudorange equations Reciprocal (noise)
        }
    }
}

// use white Algorithm
void SRIFAlgorithm::preWhiteMatrix(MatrixXd &matB, MatrixXd &matL, MatrixXd &whiteMat, MatrixXd *matP)
{
    int matB_col = matB.cols();
    MatrixXd matH , matDa;
    if(NULL != matP)
    {
        matH = matP->llt().matrixL();
        matB = matH * matB;
        matL = matH * matL;
    }

    // get white matrix: matDa
    matDa = MatrixXd::Zero(matB_col, matB_col);
    for(int j = 0; j < matB_col; j++)
    {
        VectorXd col_temp_vct = matB.col(j);
        matDa(j, j) = 1 / col_temp_vct.norm();
    }

    //white matB
    matB = matB*matDa;
    //save matDa to whiteMat
    whiteMat = matDa;
}

// init prior matrix and transition matrix
void SRIFAlgorithm::InitSRIF(MatrixXd &Rp, MatrixXd &Zp, MatrixXd &Phi, MatrixXd &G, MatrixXd &Rwk_1)
{
	this->m_Rp = Rp;
	this->m_Zp = Zp;
	this->m_Phi = Phi;
	this->m_Phi_Inv = Phi.inverse();
	this->m_G = G;
    this->m_Rwk = Rwk_1;
	this->m_initSRIF = true;
}

/*
*	A  Measurement partials, an M by N matrix.
*	L  Observation Data vector, of length M
*	*/
VectorXd SRIFAlgorithm::SRIFilter(MatrixXd &A, MatrixXd &L)
{
	VectorXd Y;
	if(!m_initSRIF)
	{
		int Acols = A.cols();
		this->m_Rp = MatrixXd::Zero(Acols, Acols);
		this->m_Zp = MatrixXd::Zero(Acols, 1);
		this->m_Phi = MatrixXd::Zero(Acols, Acols);
		this->m_Phi_Inv = MatrixXd::Zero(Acols, Acols);
		this->m_G = MatrixXd::Zero(Acols, Acols);
        this->m_Rwk = MatrixXd::Zero(Acols, Acols);
		this->m_initSRIF = true;
		cout << "Waring: you should use SRIFAlgorithm::InitSRIF init Matrix!" << endl;
	}
	// use SRIF filter and update filter parameter (Rp, Zp)
    SRIFTimeUpdate(this->m_Rp, this->m_Zp, this->m_Phi_Inv, this->m_G, &(this->m_Rwk));
	SRIFMeasureUpdate(this->m_Rp, this->m_Zp, A, L);
	// solve Rp*X = Zp
	Y.resize(this->m_Zp.rows());
	gaussBackGen(this->m_Rp, this->m_Zp, Y);
	return Y;
}


/*
 * illustration: use SRIF Factorization Matrix solve Least squre
 * example:
 * | Rp Zp | QR -> | Rd Zd |
 * | A  L  |       | 0  ed |
 *[Rd Zd; 0 ed] stored in A L. A L as input,meanwhile as output
 *
 * Input:
 * 	Rp  a priori SRI matrix (upper triangular, dimension N*N)
 * 	Zp  a priori SRI data vector (length N)
 *	A  Measurement partials, an M by N matrix.
 *	L  Data vector, of length M
 * output:
 * 	Rp: updated matrix (upper triangular, dimension N*N)
 * 	Zp: updated vector (length N)
 * 	where Rp*x = Zp
 * reference: Bierman, G.J. "Factorization Methods for Discrete Sequential
 * 				Estimation," Academic Press, 1977.
 */
void SRIFAlgorithm::SRIFMeasureUpdate(MatrixXd &Rp, MatrixXd &Zp, MatrixXd &A, MatrixXd &L)
{
    int allM =  Rp.rows() + A.rows(), allN = A.cols() + L.cols();
    MatrixXd allMat, Rmat;
    if( Rp.cols() + 1 != allN)
    {
        cout << "ERROR: SRIFAlgorithm::SRIFMeasureUpdate!" << endl;
        exit(1);
    }
//	malloc allM*allN matrix store [Rp Zp;A L]
    allMat.resize(allM, allN);
    allMat.setZero();
//	storage Rp, Zp, AL in allMat
    allMat.block(0, 0, Rp.rows(), Rp.cols()) = Rp;
    allMat.block(0, Rp.cols(), Rp.rows(), 1) = Zp;
    allMat.block(Rp.rows(), 0, A.rows(), A.cols()) = A;
    allMat.block(Rp.rows(), allN - 1, L.rows(), L.cols()) = L;
//	QR factorization
//    m_mymatrix.writeCSV("allMat_MU.csv", allMat);
    QRDecompose(allMat, Rmat);
//    m_mymatrix.writeCSV("RMat_MU.csv", Rmat);
//	update Rp, Zp
    this->m_Zp = Rmat.block(0, Rp.cols(), Rp.rows(), 1);
    this->m_Rp = Rmat.block(0, 0, Rp.rows(), Rp.cols());

}

/*
 * illustration: use SRIF Factorization Matrix Update Time
 * example:
 * |
 * | Rwk_1            0         0 | QR ->   | Rwk Rwx Zw |
 * | -Rp*Phi_Inv*G  Rp*Phi_Inv  Zp|         | 0   Rp  Zp |
 *[Rd Zd; 0 ed] stored in AL. AL as input,meanwhile as output
 *
 * Input:
 * 		Rp: 	a priori square root information (SRI) matrix (an n * n upper triangular matrix)
 * 		Zp: 	a priori SRIF state vector, of length n*1 (state is X, Zp = Rp*X).
 * 		Phi:    transition matrix, an n * n matrix.
 *  	G :     The n by ns matrix associated with process noise.
 *  			The process noise covariance is G*Q*transpose(G) where inverse(Q)
 *  			is transpose(Rw)*Rw. G is destroyed on output.
 *  	Rwk_1:  a priori square root information matrix for the process noise, an ns by ns upper triangular matrix
 * 		Zw :    a priori 'state' associated with the process noise, a vector with ns elements.  Usually set to zero by
 *				the calling routine (for unbiased process noise).
 * 		Rw:     An ns by n matrix which is set to zero by this routine, but is used for output.
 * output:
 * 		Rp: 	updated matrix (upper triangular, dimension N*N)
 * 		Zp: 	updated vector (length N)
 * 		Rwk_1:  a posteriori square root information matrix for the process noise, an ns by ns upper triangular matrix
 * 		Rwx:
 * 		Zw :
 * 		[Rwk_1 Rwx Zw] use to SRIF smoothing data
 * 	where Rp*x = Zp
 * reference: Bierman, G.J. "Factorization Methods for Discrete Sequential
 * 				Estimation," Academic Press, 1977.
 */

void SRIFAlgorithm::SRIFTimeUpdate(MatrixXd &Rp, MatrixXd &Zp, MatrixXd &Phi_Inv, MatrixXd &G,
        MatrixXd *Rwk_1, MatrixXd *Rwk, MatrixXd *Rwx, MatrixXd *Zw)
{
	int allM = Rwk_1->rows() + Rp.rows(), allN = Rwk_1->cols() + Phi_Inv.cols() + 1;
	MatrixXd allMat, tempRes, tempPhiInv, Rmat;
	allMat.resize(allM, allN);
    allMat.setZero();
	// copy Rwk_1 to allMat top left corner
	allMat.block( 0, 0, Rwk_1->rows(), Rwk_1->cols()) = *Rwk_1;
	// compute tempRes = Rp*PhiInv
	tempRes = Rp*Phi_Inv;
	// copy tempRes to allMat bottom center corner
	allMat.block(Rwk_1->rows(), G.cols(), tempRes.rows(), tempRes.cols()) =
            tempRes;
	// compute -Rp*PhiInv*G, store in tempPhiInv = -Rwk_1*PhiInv*G
	tempPhiInv = -tempRes*G;
	// copy tempPhiInv to allMat bottom left corner
	allMat.block(Rwk_1->rows(), 0, tempPhiInv.rows(), tempPhiInv.cols()) = tempPhiInv;
	// copy Zp to allMat
	allMat.block(Rwk_1->rows(), allN - 1, Zp.rows(), 1) = Zp;
	// QR factorization
//    m_mymatrix.writeCSV("allMat_TU.csv", allMat);
    QRDecompose(allMat, Rmat);
//    m_mymatrix.writeCSV("RMat_TU.csv", Rmat);
	// copy data to Rp, Zp
	this->m_Zp = Rmat.block(Rwk_1->rows(), allN - 1, Zp.rows(), 1);
	this->m_Rp = Rmat.block(Rwk_1->rows(), Rwk_1->cols(), Rp.rows(), Rp.cols());
	//save as data, use to smoothing
	if(NULL != Rwx && NULL != Zw)
	{
		// copy allMat top left corner to Rwk_1
        *Rwk = allMat.block(0, 0, Rwk_1->rows(), Rwk_1->cols());
		// copy allMat top left corner to Rwk
		*Rwx = allMat.block(0, Rwk_1->cols(), Rwk_1->rows(), Phi_Inv.cols());
		// copy allMat top left corner to Zw
		*Zw = allMat.block(0, allN - 1, Zw->rows(), 1);
	}
}


// QR Factorization (Eigen function)
void SRIFAlgorithm::QRDecompose(MatrixXd &eigenMat, MatrixXd &R)
{
    int keepnum = -1;
    m_matrix.keepMatPricision(eigenMat,keepnum);

    HouseholderQR<MatrixXd> qr;
	qr.compute(eigenMat);
	R = qr.matrixQR().triangularView<Upper>();

    m_matrix.keepMatPricision(R,keepnum);
}

// Gauss factorization back generation
void SRIFAlgorithm::gaussBackGen(MatrixXd &upTri, MatrixXd &L, VectorXd &Y)
{
	int n = L.rows();
	if(Y.size() != L.rows())
	{
		cout << "Waring: SRIFAlgorithm::gaussBackGen." << endl;
		Y.resize(L.rows());
	}
	for(int k = n - 1;k >= 0;k--)
	{
		double sum = 0.0;
		for(int j = k + 1; j < n;j++)
			sum += upTri(k,j)*Y(j);
		Y(k) = (L(k) - sum) / upTri(k, k);
	}
}

