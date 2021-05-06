#include "QKalmanFilter.h"


QKalmanFilter::QKalmanFilter()
{
	initVar();
}


QKalmanFilter::~QKalmanFilter(void)
{
}

void QKalmanFilter::initVar()
{
	isInitPara = false;//The first epoch is only initialized once
	m_VarChang = false;
    m_KALMAN_MODEL = KALMAN_MODEL::PPP_STATIC;
    m_KALMAN_SMOOTH_RANGE = KALMAN_SMOOTH_RANGE::NO_SMOOTH;
    m_KALMAN_FILLTER = KALMAN_FILLTER::KALMAN_STANDARD;
    m_SPP_Pos[0] = 0;m_SPP_Pos[1] = 0; m_SPP_Pos[2] = 0;
    m_Xk_1.resize(32);// XiaoGongWei Update:2018.10.26
    m_init_Xk.resize(32);// XiaoGongWei Update:2018.10.26
    m_Xk_1.setZero();// XiaoGongWei Update:2018.10.26
    m_init_Xk.setZero();// XiaoGongWei Update:2018.10.26
    m_const_param = 4;// [dx,dy,dz,mf,clki]
    m_sys_num = 1;
    m_sys_str = "G";
    m_LP_whight = 1e6;
    m_xyz_dynamic_Qw = 1e6; m_zwd_Qw = 3e-8; m_clk_Qw = 1e6; m_amb_Qw = 1e-16; m_ion_Qw = 1e6;
    m_xyz_dynamic_Pk = 1e6; m_zwd_Pk = 10; m_clk_Pk = 1e6; m_amb_Pk = 1e6; m_ion_Pk = 10;
}

void QKalmanFilter::setFilterParams(QVector<QStringList> Qw_Pk_LPacc)
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
}

// set KALMAN_MODEL
void QKalmanFilter::setModel(KALMAN_MODEL model_type)
{
    m_KALMAN_MODEL = model_type;
    m_sys_num = getSystemnum();
    m_sys_str = getSatlitSys();
    switch (model_type)
    {
    case KALMAN_MODEL::SPP_STATIC:
    case KALMAN_MODEL::SPP_KINEMATIC:
        m_const_param = 3 + m_sys_num;//[dx,dy,dz,clki]
        break;
    case KALMAN_MODEL::PPP_KINEMATIC:
    case KALMAN_MODEL::PPP_STATIC:
        m_const_param = 4 + m_sys_num;//[dx,dy,dz,mf,clki]
        break;
    default:
        m_const_param = 4+1;
        break;
    }
}

//Print matrix for Debug
void QKalmanFilter::printMatrix(MatrixXd mat)
{
    qDebug()<<"Print Matrix......";
    for (int i = 0; i < mat.rows();i++)
    {
        for (int j = 0;j< mat.cols();j++)
        {
            cout <<mat(i,j)<<",";
        }
        cout << endl;
    }
    cout<<"___________________";
}

//Initialize Kalman
void QKalmanFilter::initKalman(QVector< SatlitData > &currEpoch,MatrixXd &B,VectorXd &L)
{
	int epochLenLB = currEpoch.length();

	//Fk_1 initialization
    switch (m_KALMAN_MODEL) {
    case KALMAN_MODEL::SPP_STATIC:
    case KALMAN_MODEL::SPP_KINEMATIC:
        m_Fk_1.resize(m_const_param, m_const_param);
        m_Fk_1.setIdentity(m_const_param, m_const_param);
        break;
    case KALMAN_MODEL::PPP_KINEMATIC:
    case KALMAN_MODEL::PPP_STATIC:
        m_Fk_1.resize(m_const_param+epochLenLB,m_const_param+epochLenLB);
        m_Fk_1.setIdentity(m_const_param+epochLenLB,m_const_param+epochLenLB);
        break;
    default:
        break;
    }
    //Xk_1 pesodurange init  Initialization, least squares initialization
    switch (m_KALMAN_MODEL) {
    case KALMAN_MODEL::SPP_STATIC:
    case KALMAN_MODEL::SPP_KINEMATIC:
        m_Xk_1.resize(m_const_param);
        m_Xk_1.setZero();
        break;
    case KALMAN_MODEL::PPP_KINEMATIC:
    case KALMAN_MODEL::PPP_STATIC:
        m_Xk_1.resize(epochLenLB+m_const_param);
        m_Xk_1.setZero();
        break;
    default:
        ErroTrace("QKalmanFilter::initKalman Bad.");
        break;
    }
    m_Xk_1 = (B.transpose()*B).inverse()*B.transpose()*L;
    m_init_Xk = m_Xk_1;
	//Initialization state covariance matrix Pk_1 initialization
    switch (m_KALMAN_MODEL) {
    case KALMAN_MODEL::SPP_STATIC:
    case KALMAN_MODEL::SPP_KINEMATIC:
        m_Pk_1.resize(m_const_param, m_const_param);
        m_Pk_1.setZero();
        m_Pk_1(0,0) = m_xyz_dynamic_Pk;m_Pk_1(1,1) = m_xyz_dynamic_Pk;m_Pk_1(2,2) = m_xyz_dynamic_Pk;
        for(int i = 3; i < m_const_param;i++) m_Pk_1(i,i) = m_clk_Pk;// for clock
        break;
    case KALMAN_MODEL::PPP_STATIC:
    case KALMAN_MODEL::PPP_KINEMATIC:
        m_Pk_1.resize(m_const_param+epochLenLB,m_const_param+epochLenLB);
        m_Pk_1.setZero();
        m_Pk_1(0,0) = m_xyz_dynamic_Pk;m_Pk_1(1,1) = m_xyz_dynamic_Pk;m_Pk_1(2,2) = m_xyz_dynamic_Pk;
        m_Pk_1(3,3) = m_zwd_Pk;
        for(int i = 4; i < m_const_param;i++) m_Pk_1(i,i) = m_clk_Pk; // for clock
        for (int i = 0;i < epochLenLB;i++)	m_Pk_1(m_const_param+i,m_const_param+i) = m_amb_Pk;// for Ambiguity
        break;
    default:
        ErroTrace("QKalmanFilter::initKalman Bad.");
        break;
    }

	//Qk_1 system noise initialization
    switch (m_KALMAN_MODEL) {
    case KALMAN_MODEL::SPP_STATIC:
    case KALMAN_MODEL::SPP_KINEMATIC:
        m_Qwk_1.resize(m_const_param, m_const_param);
        m_Qwk_1.setZero();
        for(int i = 3; i < m_const_param;i++) m_Qwk_1(i,i) = m_clk_Qw;// for clock
        break;
    case KALMAN_MODEL::PPP_KINEMATIC:
    case KALMAN_MODEL::PPP_STATIC:
        m_Qwk_1.resize(m_const_param+epochLenLB,m_const_param+epochLenLB);
        m_Qwk_1.setZero();
        m_Qwk_1(3,3) = m_zwd_Qw;//Zenith tropospheric residual variance
        for(int i = 4; i < m_const_param;i++) m_Qwk_1(i,i) = m_clk_Qw; // for clock
        break;
    default:
        ErroTrace("QKalmanFilter::initKalman Bad.");
        break;
    }
    if(m_KALMAN_MODEL == KALMAN_MODEL::SPP_KINEMATIC || m_KALMAN_MODEL == KALMAN_MODEL::PPP_KINEMATIC)
    {
        m_Qwk_1(0,0) = m_xyz_dynamic_Qw;
        m_Qwk_1(1,1) = m_xyz_dynamic_Qw;
        m_Qwk_1(2,2) = m_xyz_dynamic_Qw;
    }

	//Rk_1 initialization is in place to determine that there is no change in the number of satellites
	isInitPara = true;//No longer initialized after
}

//Initialize Kalman
void QKalmanFilter::initKalman_NoCombination(QVector< SatlitData > &currEpoch,MatrixXd &B,VectorXd &L)
{
    int epochLenLB = currEpoch.length();

    //Fk_1 initialization
    switch (m_KALMAN_MODEL) {
    case KALMAN_MODEL::SPP_STATIC:
    case KALMAN_MODEL::SPP_KINEMATIC:
        m_Fk_1.resize(m_const_param+epochLenLB, m_const_param+epochLenLB);
        m_Fk_1.setIdentity();
        break;
    case KALMAN_MODEL::PPP_KINEMATIC:
    case KALMAN_MODEL::PPP_STATIC:
        m_Fk_1.resize(m_const_param+3*epochLenLB,m_const_param+3*epochLenLB);
        m_Fk_1.setIdentity();
        break;
    default:
        break;
    }
    //Xk_1 pesodurange init  Initialization, least squares initialization
    switch (m_KALMAN_MODEL) {
    case KALMAN_MODEL::SPP_STATIC:
    case KALMAN_MODEL::SPP_KINEMATIC:
        m_Xk_1.resize(m_const_param+epochLenLB);
        m_Xk_1.setZero();
        break;
    case KALMAN_MODEL::PPP_KINEMATIC:
    case KALMAN_MODEL::PPP_STATIC:
        m_Xk_1.resize(3*epochLenLB+m_const_param);
        m_Xk_1.setZero();
        break;
    default:
        ErroTrace("QKalmanFilter::initKalman Bad.");
        break;
    }
    m_Xk_1 = (B.transpose()*B).inverse()*B.transpose()*L;
    m_init_Xk = m_Xk_1;
    //Initialization state covariance matrix Pk_1 initialization
    switch (m_KALMAN_MODEL) {
    case KALMAN_MODEL::SPP_STATIC:
    case KALMAN_MODEL::SPP_KINEMATIC:
        m_Pk_1.resize(m_const_param+epochLenLB, m_const_param+epochLenLB);
        m_Pk_1.setZero();
        m_Pk_1(0,0) = m_xyz_dynamic_Pk;m_Pk_1(1,1) = m_xyz_dynamic_Pk;m_Pk_1(2,2) = m_xyz_dynamic_Pk;
        for(int i = 3; i < m_const_param;i++) m_Pk_1(i,i) = m_clk_Pk;// for clock
        for(int i = m_const_param; i < epochLenLB+m_const_param;i++) m_Pk_1(i,i) = m_ion_Pk;// for ION
        break;
    case KALMAN_MODEL::PPP_STATIC:
    case KALMAN_MODEL::PPP_KINEMATIC:
        m_Pk_1.resize(m_const_param+3*epochLenLB,m_const_param+3*epochLenLB);
        m_Pk_1.setZero();
        m_Pk_1(0,0) = m_xyz_dynamic_Pk;m_Pk_1(1,1) = m_xyz_dynamic_Pk;m_Pk_1(2,2) = m_xyz_dynamic_Pk;
        m_Pk_1(3,3) = m_zwd_Pk;
        for(int i = 4; i < m_const_param;i++) m_Pk_1(i,i) = m_clk_Pk; // for clock
        for(int i = m_const_param; i < m_const_param+epochLenLB;i++) m_Pk_1(i,i) = m_ion_Pk; // for ION
        for (int i = m_const_param+epochLenLB;i < m_const_param+3*epochLenLB;i++)	m_Pk_1(i,i) = m_amb_Pk;// for Ambiguity
        break;
    default:
        ErroTrace("QKalmanFilter::initKalman Bad.");
        break;
    }

    //Qk_1 system noise initialization
    switch (m_KALMAN_MODEL) {
    case KALMAN_MODEL::SPP_STATIC:
    case KALMAN_MODEL::SPP_KINEMATIC:
        m_Qwk_1.resize(m_const_param+epochLenLB, m_const_param+epochLenLB);
        m_Qwk_1.setZero();
        for(int i = 3; i < m_const_param;i++) m_Qwk_1(i,i) = m_clk_Qw;// for clock
        for(int i = m_const_param; i < epochLenLB+m_const_param;i++) m_Pk_1(i,i) = m_ion_Qw;// for ION
        break;
    case KALMAN_MODEL::PPP_KINEMATIC:
    case KALMAN_MODEL::PPP_STATIC:
        m_Qwk_1.resize(m_const_param+3*epochLenLB,m_const_param+3*epochLenLB);
        m_Qwk_1.setZero();
        m_Qwk_1(3,3) = m_zwd_Qw;//Zenith tropospheric residual variance
        for(int i = 4; i < m_const_param;i++) m_Qwk_1(i,i) = m_clk_Qw; // for clock
        for(int i = m_const_param; i < m_const_param+epochLenLB;i++) m_Qwk_1(i,i) = m_ion_Qw; // for ION
        break;
    default:
        ErroTrace("QKalmanFilter::initKalman Bad.");
        break;
    }

    if(m_KALMAN_MODEL == KALMAN_MODEL::SPP_KINEMATIC || m_KALMAN_MODEL == KALMAN_MODEL::PPP_KINEMATIC)
    {
        m_Qwk_1(0,0) = m_xyz_dynamic_Qw;
        m_Qwk_1(1,1) = m_xyz_dynamic_Qw;
        m_Qwk_1(2,2) = m_xyz_dynamic_Qw;
    }

    //Rk_1 initialization is in place to determine that there is no change in the number of satellites
    isInitPara = true;//No longer initialized after
}

//Change the Kalman parameter size (only PPP can change paramater)
void QKalmanFilter::changeKalmanPara( QVector< SatlitData > &epochSatlitData,QVector< int >oldPrnFlag )
{
	int epochLenLB = epochSatlitData.length();

    m_Fk_1.resize(m_const_param+epochLenLB,m_const_param+epochLenLB);
    m_Fk_1.setZero();
    m_Fk_1.setIdentity(m_const_param+epochLenLB,m_const_param+epochLenLB);
	//Fk_1(4,4) = 0;//Static PPP has only a clock difference of 0
	//Xk_1 change
	VectorXd tempXk_1 = m_Xk_1;
    m_Xk_1.resize(epochLenLB+m_const_param);
	m_Xk_1.setZero();
	//Xk.resize(epochLenLB+5);
    for (int i = 0;i < m_const_param;i++)
		m_Xk_1(i) = tempXk_1(i);
    for (int i = 0;i<epochLenLB;i++)
    {
        if (oldPrnFlag.at(i)!=-1)//Save the old satellite ambiguity
            m_Xk_1(m_const_param+i) = tempXk_1(oldPrnFlag.at(i)+m_const_param);
        else
        {//New satellite ambiguity calculation
            SatlitData oneStalit = epochSatlitData.at(i);
            m_Xk_1(m_const_param+i) = (oneStalit.PP3 - oneStalit.LL3)/M_GetLamta3(oneStalit.Frq[0],oneStalit.Frq[1]);
        }
    }
	//Qk_1 system noise will not be updated, system noise is not measurable
    m_Qwk_1.resize(m_const_param+epochLenLB,m_const_param+epochLenLB);
	m_Qwk_1.setZero();
    if(m_KALMAN_MODEL == KALMAN_MODEL::PPP_KINEMATIC)
    {
        m_Qwk_1(0,0) = m_xyz_dynamic_Qw;
        m_Qwk_1(1,1) = m_xyz_dynamic_Qw;
        m_Qwk_1(2,2) = m_xyz_dynamic_Qw;
    }
    m_Qwk_1(3,3) = m_zwd_Qw;//Zenith tropospheric residual variance
    for(int i = 4; i < m_const_param;i++) m_Qwk_1(i,i) = m_clk_Qw; // for clock
	//Reset Rk_1 observation noise matrix (reset on the outside, no need to repeat reset here)
	//The saved state covariance matrix Pk_1 is increased or decreased (here is more complicated, the main idea is to take out old satellite data, and initialize the new satellite data)
	MatrixXd tempPk_1 = m_Pk_1;
    m_Pk_1.resize(m_const_param+epochLenLB, m_const_param+epochLenLB);
	m_Pk_1.setZero();
	//If the number of satellites changes
    for (int i = 0;i < m_const_param;i++)
        for (int j = 0;j < m_const_param;j++)
			m_Pk_1(i,j) = tempPk_1(i,j);

	for (int n = 0; n < epochLenLB;n++)
	{
		int flag = oldPrnFlag.at(n);
		if ( flag != -1)//Description: The previous epoch contains this satellite data and needs to be taken from tempPk_1
		{
            flag+=m_const_param;//The number of rows of this satellite in the original data tempPk_1
			for (int i = 0;i < tempPk_1.cols();i++)
			{//Take out from tempPk_1 and skip the data with oldPrnFlag -1
                if (i < m_const_param)
				{
                    m_Pk_1(n+m_const_param,i) = tempPk_1(flag,i);
                    m_Pk_1(i,n+m_const_param) = tempPk_1(i,flag);
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
                        m_Pk_1(n+m_const_param,saveFlag+m_const_param) = tempPk_1(flag,i);
						//Pk_1(saveFlag+5,n+5) = tempPk_1(i,flag);
					}

				}//if (i < 5)
			}//for (int i = 0;i < tempPk_1.cols();i++)

		}
		else
		{
            m_Pk_1(n+m_const_param,n+m_const_param) = m_amb_Pk;
//            for (int i = 0;i < m_const_param;i++)
//            {
//                m_Pk_1(n+m_const_param,i) = 1;
//                m_Pk_1(i,n+m_const_param) = 1;
//            }
		}
	}//Pk_1 saves the data

	m_VarChang = true;
}


//Change the Kalman parameter size (only PPP can change paramater)
void QKalmanFilter::changeKalmanPara_NoCombination( QVector< SatlitData > &epochSatlitData,QVector< int >oldPrnFlag, int preEpochLen)
{
    int epochLenLB = epochSatlitData.length();

    m_Fk_1.resize(m_const_param+3*epochLenLB,m_const_param+3*epochLenLB);
    m_Fk_1.setZero();
    m_Fk_1.setIdentity();
    //Fk_1(4,4) = 0;//Static PPP has only a clock difference of 0
    //Xk_1 change
    VectorXd tempXk_1 = m_Xk_1;
    m_Xk_1.resize(3*epochLenLB+m_const_param);
    m_Xk_1.setZero();
    for (int i = 0;i < m_const_param;i++)
        m_Xk_1(i) = tempXk_1(i);
    for (int i = 0;i<epochLenLB;i++)
    {
        if (oldPrnFlag.at(i)!=-1)//Save the old satellite ION and L1 L2 ambiguity
        {
            m_Xk_1(m_const_param+i) = tempXk_1(oldPrnFlag.at(i)+m_const_param);// for ION
            m_Xk_1(m_const_param+epochLenLB+i) = tempXk_1(oldPrnFlag.at(i)+preEpochLen+m_const_param);// for L1 ambiguity
            m_Xk_1(m_const_param+2*epochLenLB+i) = tempXk_1(oldPrnFlag.at(i)+2*preEpochLen+m_const_param);// for L1 ambiguity
        }
        else
        {//New satellite ambiguity calculation
            SatlitData oneStalit = epochSatlitData.at(i);
            m_Xk_1(m_const_param+epochLenLB+i) = (oneStalit.C1 - oneStalit.L1)/oneStalit.Frq[0];// new L1 ambiguity
            m_Xk_1(m_const_param+2*epochLenLB+i) = (oneStalit.C2 - oneStalit.L2)/oneStalit.Frq[1];// new L2 ambiguity
        }
    }
//    m_matrix.writeCSV("newXk.csv", m_Xk_1);
//    m_matrix.writeCSV("oldXk.csv", tempXk_1);
    //Qk_1 system noise will not be updated, system noise is not measurable
    m_Qwk_1.resize(m_const_param+3*epochLenLB,m_const_param+3*epochLenLB);
    m_Qwk_1.setZero();
    if(m_KALMAN_MODEL == KALMAN_MODEL::PPP_KINEMATIC)
    {
        m_Qwk_1(0,0) = m_xyz_dynamic_Qw;
        m_Qwk_1(1,1) = m_xyz_dynamic_Qw;
        m_Qwk_1(2,2) = m_xyz_dynamic_Qw;
    }
    m_Qwk_1(3,3) = m_zwd_Qw;//Zenith tropospheric residual variance
    for(int i = 4; i < m_const_param;i++) m_Qwk_1(i,i) = m_clk_Qw; // for clock
    for(int i = m_const_param; i < m_const_param+epochLenLB;i++) m_Qwk_1(i,i) = m_ion_Qw; // for ION
    //Reset Rk_1 observation noise matrix (reset on the outside, no need to repeat reset here)
    //The saved state covariance matrix Pk_1 is increased or decreased (here is more complicated, the main idea is to take out old satellite data, and initialize the new satellite data)
    MatrixXd tempPk_1 = m_Pk_1;
    m_Pk_1.resize(m_const_param+3*epochLenLB, m_const_param+3*epochLenLB);
    m_Pk_1.setZero();
    //If the number of satellites changes
    for (int i = 0;i < m_const_param;i++)
        for (int j = 0;j < m_const_param;j++)
            m_Pk_1(i,j) = tempPk_1(i,j);

    for (int n = 0; n < epochLenLB;n++)
    {
        int flag = oldPrnFlag.at(n);
        if ( flag != -1)//Description: The previous epoch contains this satellite data and needs to be taken from tempPk_1
        {
            flag+=m_const_param;//The number of rows of this satellite in the original data tempPk_1
            for (int i = 0;i < tempPk_1.cols();i++)
            {//Take out from tempPk_1 and skip the data with oldPrnFlag -1
                if (i < m_const_param)
                {
                    // for ION
                    m_Pk_1(n+m_const_param,i) = tempPk_1(flag,i);
                    m_Pk_1(i,n+m_const_param) = tempPk_1(i,flag);
                    // for L1 AMB
                    m_Pk_1(n+m_const_param+epochLenLB,i) = tempPk_1(flag+preEpochLen,i);
                    m_Pk_1(i,n+m_const_param+epochLenLB) = tempPk_1(i,flag+preEpochLen);
                    // for L2 AMB
                    m_Pk_1(n+m_const_param+2*epochLenLB,i) = tempPk_1(flag+2*preEpochLen,i);
                    m_Pk_1(i,n+m_const_param+2*epochLenLB) = tempPk_1(i,flag+2*preEpochLen);
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
                        m_Pk_1(n+m_const_param,saveFlag+m_const_param) = tempPk_1(flag,i);// for ION
                        m_Pk_1(n+m_const_param,saveFlag+m_const_param+epochLenLB) = tempPk_1(flag,i+preEpochLen);
                        m_Pk_1(n+m_const_param,saveFlag+m_const_param+2*epochLenLB) = tempPk_1(flag,i+2*preEpochLen);

                        m_Pk_1(n+m_const_param+epochLenLB,saveFlag+m_const_param) = tempPk_1(flag+preEpochLen,i);// for L1 AMB
                        m_Pk_1(n+m_const_param+epochLenLB,saveFlag+m_const_param+epochLenLB) = tempPk_1(flag+preEpochLen,i+preEpochLen);// for L1 AMB
                        m_Pk_1(n+m_const_param+epochLenLB,saveFlag+m_const_param+2*epochLenLB) = tempPk_1(flag+preEpochLen,i+2*preEpochLen);// for L1 AMB

                        m_Pk_1(n+m_const_param+2*epochLenLB,saveFlag+m_const_param) = tempPk_1(flag+2*preEpochLen,i);// for L2 AMB
                        m_Pk_1(n+m_const_param+2*epochLenLB,saveFlag+m_const_param+epochLenLB) = tempPk_1(flag+2*preEpochLen,i+preEpochLen);// for L2 AMB
                        m_Pk_1(n+m_const_param+2*epochLenLB,saveFlag+m_const_param+2*epochLenLB) = tempPk_1(flag+2*preEpochLen,i+2*preEpochLen);// for L2 AMB
                    }

                }//if (i < 5)
            }//for (int i = 0;i < tempPk_1.cols();i++)

        }
        else
        {
            m_Pk_1(n+m_const_param,n+m_const_param) = m_ion_Pk;// for ION
            m_Pk_1(n+m_const_param+epochLenLB,n+m_const_param+epochLenLB) = m_amb_Pk;// for L1 amb
            m_Pk_1(n+m_const_param+2*epochLenLB,n+m_const_param+2*epochLenLB) = m_amb_Pk;// for L2 amb

        }
    }//Pk_1 saves the data

//    m_matrix.writeCSV("newPk.csv", m_Pk_1);
//    m_matrix.writeCSV("oldPk.csv", tempPk_1);
    m_VarChang = true;
}


//First Kalman version Mr. Jikun Ou
// PBk is Weight matrix
void QKalmanFilter::KalmanforStaticOu(MatrixXd Bk,VectorXd Lk,MatrixXd F,MatrixXd Qwk,MatrixXd PBk,VectorXd &tXk_1,MatrixXd &tPk_1)
{

}

//Third version use to change Kalman
void QKalmanFilter::KalmanforStatic(MatrixXd Bk,VectorXd Lk,MatrixXd F,MatrixXd Qwk,
                                    MatrixXd Rk,VectorXd &tXk_1,MatrixXd &tPk_1)
{
    //Time update
    VectorXd Xkk_1 = F*tXk_1,Vk;
    MatrixXd Pkk_1 = F*tPk_1*F.transpose() + Qwk,I,tempKB,Kk;
    //Calculated gain matrix
    Kk = (Pkk_1*Bk.transpose())*((Bk*Pkk_1*Bk.transpose() + Rk).inverse());
    //Filter update
    Vk = Lk - Bk*Xkk_1;
    //Update X
    tXk_1 = Xkk_1 + Kk*Vk;
    //Filtered residual, normal download wave is very small
    VectorXd Vk_temp = Lk - Bk*tXk_1;

    tempKB = Kk*Bk;
    I.resize(tempKB.rows(),tempKB.cols());
    I.setIdentity();
    //Update P (Case I) (this update is extremely unstable)
    tPk_1 = (I - tempKB)*Pkk_1;

//    m_matrix.writeCSV("B.csv", Bk);
//    m_matrix.writeCSV("L.csv", Lk);

    //Update P(Case II)
//    MatrixXd Mk_1 = Pkk_1.inverse() + Bk.transpose()*Rk.inverse()*Bk;
//    tPk_1 =Mk_1.inverse();
//    MatrixXd newPk =  0.5*(tPk_1 + tPk_1.transpose());
//    tPk_1 = newPk;
    //printMatrix(tPk_1);
//    tPk_1 = 0.5*(tPk_1 + tPk_1.transpose());	//(In theory, it should be added but added or beating. Changed the original covariance data)
    //printMatrix(tPk_1);
}


////Third version  use to change Kalman
//void QKalmanFilter::KalmanforStatic(MatrixXd Bk,VectorXd Lk,MatrixXd F,MatrixXd Qwk,
//                                    MatrixXd Rk,VectorXd &tXk_1,MatrixXd &tPk_1)
//{
//    //
////    int keepnum = -1;
//    int keepnum = 9;
//    m_matrix.keepMatPricision(Bk,keepnum);
//    m_matrix.keepMatPricision(Lk,keepnum);
//    m_matrix.keepMatPricision(F,keepnum);
//    m_matrix.keepMatPricision(Qwk,keepnum);

//    m_matrix.keepMatPricision(Rk,keepnum);
//    m_matrix.keepMatPricision(tXk_1,keepnum);
//    m_matrix.keepMatPricision(tPk_1,keepnum);
//    //Time update
//    VectorXd Xkk_1 = F*tXk_1,Vk;
//    MatrixXd Pkk_1 = F*tPk_1*F.transpose() + Qwk,I,tempKB,Kk;

//    m_matrix.keepMatPricision(Xkk_1,keepnum);
//    m_matrix.keepMatPricision(Pkk_1,keepnum);

//    //Calculated gain matrix
//    Kk = (Pkk_1*Bk.transpose())*((Bk*Pkk_1*Bk.transpose() + Rk).inverse());
//    m_matrix.keepMatPricision(Kk,keepnum);
//    //Filter update
//    Vk = Lk - Bk*Xkk_1;
//    m_matrix.keepMatPricision(Vk,keepnum);
//    //Update X
//    tXk_1 = Xkk_1 + Kk*Vk;
//    m_matrix.keepMatPricision(tXk_1,keepnum);
//    //Filtered residual, normal download wave is very small
//    VectorXd Vk_temp = Lk - Bk*tXk_1;

//    tempKB = Kk*Bk;
//    I.resize(tempKB.rows(),tempKB.cols());
//    I.setIdentity();
//    m_matrix.keepMatPricision(tempKB,keepnum);
//    //Update P (Case I) (this update is extremely unstable)
//    tPk_1 = (I - tempKB)*Pkk_1;
//    //Update P(Case II)
////    MatrixXd Mk_1 = Pkk_1.inverse() + Bk.transpose()*Rk.inverse()*Bk;
////    tPk_1 =Mk_1.inverse();
//    //printMatrix(tPk_1);

//    m_matrix.keepMatPricision(tPk_1,keepnum);

////    tPk_1 = 0.5*(tPk_1 + tPk_1.transpose());	//(In theory, it should be added but added or beating. Changed the original covariance data)
//    //printMatrix(tPk_1);
//}


// get matrix B and observer L for Combination
void QKalmanFilter::Obtaining_equation(QVector< SatlitData > &currEpoch, double *m_ApproxRecPos, MatrixXd &mat_B, VectorXd &Vct_L,
                             MatrixXd &mat_P)
{
    int epochLenLB = currEpoch.length(), const_num = 3;
    MatrixXd B, P;
    VectorXd L, sys_len;
    sys_len.resize(m_sys_str.length());
    sys_len.setZero();
    switch(m_KALMAN_MODEL)
    {
    case KALMAN_MODEL::SPP_STATIC:
    case KALMAN_MODEL::SPP_KINEMATIC:
        B.resize(epochLenLB,m_const_param);
        P.resize(epochLenLB,epochLenLB);
        L.resize(epochLenLB);
        const_num = 3;// 3 is conntain [dx,dy,dz]
        break;
    case KALMAN_MODEL::PPP_KINEMATIC:
    case KALMAN_MODEL::PPP_STATIC:
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
        switch(m_KALMAN_MODEL)
        {
        case KALMAN_MODEL::SPP_STATIC:
        case KALMAN_MODEL::SPP_KINEMATIC:
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
            if(KALMAN_SMOOTH_RANGE::SMOOTH == m_KALMAN_SMOOTH_RANGE)
            {
                L(i) = p0 - oneSatlit.PP3_Smooth + dlta;
                // Computing weight matrix PP3
                P(i, i) = 1 / oneSatlit.PP3_Smooth_Q;// Pseudo-range right
            }
            else
            {
                L(i) = p0 - oneSatlit.PP3 + dlta;
                // Computing weight matrix P
                P(i, i) = oneSatlit.SatWight;// Pseudo-range right
            }
            break;
        case KALMAN_MODEL::PPP_KINEMATIC:
        case KALMAN_MODEL::PPP_STATIC:
            //Computational B matrix
            //L3 carrier matrix
            B(i,0) = li;B(i,1) = mi;B(i,2) = ni;B(i,3) = -oneSatlit.StaTropMap;B(i,4) = -1;
            for (int n = 0;n < epochLenLB;n++)//The diagonal part of the rear part initializes the wavelength of Lamta3, and the rest is 0.
                if (i == n)
                    B(i,m_const_param+n) = M_GetLamta3(oneSatlit.Frq[0],oneSatlit.Frq[1]);//LL3 wavelength
            //P3 pseudorange code matrix
            B(i+epochLenLB,0) = li;B(i+epochLenLB,1) = mi;B(i+epochLenLB,2) = ni;B(i+epochLenLB,3) = -oneSatlit.StaTropMap;B(i+epochLenLB,4) = -1;
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
            //Carrier L  pseudorange code L
            L(i) = p0 - oneSatlit.LL3 + dlta;
            L(i+epochLenLB) = p0 - oneSatlit.PP3 + dlta;
            // Computing weight matrix P
//            if(oneSatlit.UTCTime.epochNum <= 100) LP_whight = 1e6;// for convergence
            P(i, i) = oneSatlit.SatWight * LP_whight;// Carrier weight
            P(i + epochLenLB, i + epochLenLB) = oneSatlit.SatWight;// Pseudo-range right
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
// get matrix B and observer L for No Combination
void QKalmanFilter::Obtaining_equation_NoCombination(QVector< SatlitData > &currEpoch, double *m_ApproxRecPos, MatrixXd &mat_B, VectorXd &Vct_L,
                             MatrixXd &mat_P)
{
    int epochLenLB = currEpoch.length(), const_num = 3;
    MatrixXd B, P;
    VectorXd L, sys_len;
    sys_len.resize(m_sys_str.length());
    sys_len.setZero();
    switch(m_KALMAN_MODEL)
    {
    case KALMAN_MODEL::SPP_STATIC:
    case KALMAN_MODEL::SPP_KINEMATIC:
        B.resize(2*epochLenLB,m_const_param+epochLenLB);
        P.resize(2*epochLenLB,2*epochLenLB);
        L.resize(2*epochLenLB);
        const_num = 3;// 3 is conntain [dx,dy,dz]
        break;
    case KALMAN_MODEL::PPP_KINEMATIC:
    case KALMAN_MODEL::PPP_STATIC:
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
        switch(m_KALMAN_MODEL)
        {
        case KALMAN_MODEL::SPP_STATIC:
        case KALMAN_MODEL::SPP_KINEMATIC:
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
            if(KALMAN_SMOOTH_RANGE::SMOOTH == m_KALMAN_SMOOTH_RANGE)
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
        case KALMAN_MODEL::PPP_KINEMATIC:
        case KALMAN_MODEL::PPP_STATIC:
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

//Second version
bool QKalmanFilter::KalmanforStatic(QVector< SatlitData > &preEpoch,QVector< SatlitData > &currEpoch,double *m_ApproxRecPos,
                                    VectorXd &X,MatrixXd &P)
{
    if (!isInitPara)
    {
        m_SPP_Pos[0] = m_ApproxRecPos[0];
        m_SPP_Pos[1] = m_ApproxRecPos[1];
        m_SPP_Pos[2] = m_ApproxRecPos[2];
    }

    //judge is Kinematic
    if(m_KALMAN_MODEL == KALMAN_MODEL::PPP_KINEMATIC || m_KALMAN_MODEL == KALMAN_MODEL::SPP_KINEMATIC)
    {
        // we solver five parameter[dx,dy,dz,dTrop,dClock],so epochLenLB > 4
        m_SPP_Pos[0] = m_ApproxRecPos[0];
        m_SPP_Pos[1] = m_ApproxRecPos[1];
        m_SPP_Pos[2] = m_ApproxRecPos[2];
        // must set zero of [dx,dy,dy] int Kinematic
        m_Xk_1(0) = 0; m_Xk_1(1) = 0; m_Xk_1(2) = 0;
    }

    //save filter sate for Quality Control
    MatrixXd temp_Fk_1 = m_Fk_1, temp_Qwk_1 = m_Qwk_1,
            temp_Rk_1 = m_Rk_1, temp_Pk_1 = m_Pk_1;
    VectorXd temp_Xk_1 = m_Xk_1;
    double temp_SPP_POS[3] = {0};
    memcpy(temp_SPP_POS, m_SPP_Pos, 3*sizeof(double));

    // use spp clk as priori value for base clk
    QKalmanFilter::KALMAN_MODEL kalman_model = getModel();
    if(kalman_model == QKalmanFilter::KALMAN_MODEL::SPP_STATIC || kalman_model == QKalmanFilter::KALMAN_MODEL::SPP_KINEMATIC)
        if(m_ApproxRecPos[3] != 0) m_Xk_1(3) =  m_ApproxRecPos[3];// m_ApproxRecPos[3] (spp clk) must have value
    else
        if(m_ApproxRecPos[3] != 0) m_Xk_1(4) =  m_ApproxRecPos[3];// m_ApproxRecPos[3] (spp clk) must have value
    // filter
    filter(preEpoch, currEpoch, X, P);

    // Quality Control
    bool gross_LC = true;// false
    int minSatNum = 5, max_iter = 10;
    if(m_KALMAN_MODEL == KALMAN_MODEL::SPP_STATIC || m_KALMAN_MODEL == KALMAN_MODEL::PPP_STATIC)
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
        if(m_KALMAN_MODEL == KALMAN_MODEL::SPP_STATIC || m_KALMAN_MODEL == KALMAN_MODEL::SPP_KINEMATIC)
        {
            gross_LC = m_qualityCtrl.VtPVCtrl_Filter_C(B, L, m_Xk_1, delate_LC, sat_len);// QC pesoderange
        }
        else
        {
            if(getPPPModel() == PPP_MODEL::PPP_Combination)
            {
                if(getModel() == KALMAN_MODEL::PPP_STATIC)
                {
                    double LP_threshold[2] = {0.06, 10.0};
                    gross_LC = m_qualityCtrl.VtPVCtrl_Filter_LC(B, L, m_Xk_1, delate_LC, sat_len, LP_threshold);// QC for carrire and pesoderange
                }
                else
                {
                    double LP_threshold[2] = {0.1, 10.0};
                    gross_LC = m_qualityCtrl.VtPVCtrl_Filter_LC(B, L, m_Xk_1, delate_LC, sat_len, LP_threshold);// QC for carrire and pesoderange
                }
            }
            else if(getPPPModel() == PPP_MODEL::PPP_NOCombination)
            {
                if(getModel() == KALMAN_MODEL::PPP_STATIC)
                {
                    double L12P12_threshold[4] = {0.05, 0.05, 5.0, 5.0};
                    gross_LC = m_qualityCtrl.VtPVCtrl_Filter_LC_NoCombination(B, L, m_Xk_1, delate_LC, sat_len, L12P12_threshold);// QC for carrire and pesoderange
                }
                else
                {
                    double L12P12_threshold[4] = {0.05, 0.05, 5.0, 5.0};
                    gross_LC = m_qualityCtrl.VtPVCtrl_Filter_LC_NoCombination(B, L, m_Xk_1, delate_LC, sat_len, L12P12_threshold);// QC for carrire and pesoderange
                }
            }
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
            m_Fk_1 = temp_Fk_1; m_Qwk_1 = temp_Qwk_1; m_Rk_1 = temp_Rk_1;
            m_Pk_1 = temp_Pk_1; m_Xk_1 = temp_Xk_1;
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
    Vk = B*m_Xk_1 - L;

    if(m_KALMAN_MODEL == KALMAN_MODEL::SPP_STATIC || m_KALMAN_MODEL == KALMAN_MODEL::SPP_KINEMATIC)
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
    X = m_Xk_1;
    P = m_Pk_1;
    if(gross_LC)
    {
        // restore filter state
        m_Fk_1 = temp_Fk_1; m_Qwk_1 = temp_Qwk_1; m_Rk_1 = temp_Rk_1;
        m_Pk_1 = temp_Pk_1; m_Xk_1 = temp_Xk_1;
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
        // update m_ApproxRecPos use kalman
        m_ApproxRecPos[0] = m_SPP_Pos[0] + m_Xk_1(0);
        m_ApproxRecPos[1] = m_SPP_Pos[1] + m_Xk_1(1);
        m_ApproxRecPos[2] = m_SPP_Pos[2] + m_Xk_1(2);
    }
    return (!gross_LC);
}

void QKalmanFilter::filter(QVector< SatlitData > &preEpoch,QVector< SatlitData > &currEpoch, VectorXd &X,MatrixXd &P)
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


    //First epoch initialization  Filter init
    if (0 == preEpochLen)
    {
        if(getPPPModel() == PPP_MODEL::PPP_Combination)
            initKalman(currEpoch,B,L);
        else if(getPPPModel() == PPP_MODEL::PPP_NOCombination)
            initKalman_NoCombination(currEpoch,B,L);
    }

    // if have P matrix use P. This is for back smooth
    if(P.rows() > 1)
    {
        m_Xk_1 = X;
        m_Pk_1 = P;
    }

    //Update Rk_1（There is no change in the number of satellites）
    if(getPPPModel() == PPP_MODEL::PPP_Combination)
        updateRk(currEpoch, B.rows());
    else if(getPPPModel() == PPP_MODEL::PPP_NOCombination)
        updateRk_NoCombination(currEpoch, B.rows());

    //Determine whether the number of satellites has changed (comparison of two epochs before and after)
    QVector< int > oldPrnFlag;//Compared with the location of the same satellite in the previous epoch, it is not found with -1
    bool isNewSatlite = false;
    isNewSatlite = isSatelliteChange(preEpoch, currEpoch, oldPrnFlag);

    //Change filter parameters
    if(KALMAN_MODEL::PPP_KINEMATIC == m_KALMAN_MODEL ||  KALMAN_MODEL::PPP_STATIC == m_KALMAN_MODEL)
    {
        //Increase or decrease n satellites
        if (((preEpochLen != epochLenLB) || isNewSatlite ) && preEpochLen != 0)
        {
            if(getPPPModel() == PPP_MODEL::PPP_Combination)
                changeKalmanPara(currEpoch,oldPrnFlag);//Update all kalman parameter data sizes
            else if(getPPPModel() == PPP_MODEL::PPP_NOCombination)
                changeKalmanPara_NoCombination(currEpoch,oldPrnFlag, preEpochLen);//Update all kalman parameter data sizes
        }
    }

    //Version Kalman filter
    if(KALMAN_FILLTER::KALMAN_STANDARD ==  m_KALMAN_FILLTER)
        KalmanforStatic(B,L,m_Fk_1,m_Qwk_1,m_Rk_1,m_Xk_1,m_Pk_1);
    else if(KALMAN_FILLTER::KALMAN_MrOu ==  m_KALMAN_FILLTER)
        KalmanforStaticOu(B,L,m_Fk_1,m_Qwk_1,wightP,m_Xk_1,m_Pk_1);
}

//Determine whether the number of satellites has changed (comparison of two epochs before and after)   debug by xiaogongwei 2019.04.29
bool QKalmanFilter::isSatelliteChange(QVector< SatlitData > &preEpoch,QVector< SatlitData > &currEpoch, QVector< int > &oldPrnFlag)
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
void QKalmanFilter::updateRk(QVector< SatlitData > &currEpoch, int B_len)
{
    int epochLenLB = currEpoch.length();
    if(KALMAN_MODEL::SPP_STATIC == m_KALMAN_MODEL || KALMAN_MODEL::SPP_KINEMATIC == m_KALMAN_MODEL)
    {
        m_Rk_1.resize(B_len, B_len);// this m_Rk_1 is for ISB
        m_Rk_1.setIdentity();// this m_Rk_1 is for ISB
        for (int i = 0;i < epochLenLB;i++)
        {
            SatlitData oneSatlit = currEpoch.at(i);
            if(KALMAN_SMOOTH_RANGE::SMOOTH == m_KALMAN_SMOOTH_RANGE)
                m_Rk_1(i, i) = oneSatlit.PP3_Smooth_Q;//Covariance of pseudorange equations Reciprocal (noise)
            else
                m_Rk_1(i, i) = 1 / oneSatlit.SatWight;//Covariance of pseudorange equations Reciprocal (noise)
        }
    }
    else
    {
        m_Rk_1.resize(B_len, B_len);// this m_Rk_1 is for ISB
        m_Rk_1.setIdentity();// this m_Rk_1 is for ISB
        for (int i = 0;i < epochLenLB;i++)
        {
            SatlitData oneSatlit = currEpoch.at(i);
            double Q_LP_whight  = 1 / m_LP_whight;// Contrast in paper 2019.05.11 by xiaogongwei
//            if(oneSatlit.UTCTime.epochNum <= 100) LP_whight = 1e-6;// for convergence
            m_Rk_1(i,i) = Q_LP_whight / oneSatlit.SatWight;//Covariance of carrier equation Reciprocal (small noise)// 1/25000 =4e-4
            m_Rk_1(i+epochLenLB,i+epochLenLB) = 1 /oneSatlit.SatWight;//Covariance of pseudorange equations Reciprocal (noise)
        }
    }
}

// update Rk(Observation Covariance)
void QKalmanFilter::updateRk_NoCombination(QVector< SatlitData > &currEpoch, int B_len)
{
    int epochLenLB = currEpoch.length();
    if(KALMAN_MODEL::SPP_STATIC == m_KALMAN_MODEL || KALMAN_MODEL::SPP_KINEMATIC == m_KALMAN_MODEL)
    {
        m_Rk_1.resize(B_len, B_len);// this m_Rk_1 is for ISB
        m_Rk_1.setIdentity();// this m_Rk_1 is for ISB
        for (int i = 0;i < epochLenLB;i++)
        {
            SatlitData oneSatlit = currEpoch.at(i);
            if(KALMAN_SMOOTH_RANGE::SMOOTH == m_KALMAN_SMOOTH_RANGE)
            {
                m_Rk_1(2*i, 2*i) = oneSatlit.CC1_Smooth_Q;//Covariance of C1 pseudorange equations Reciprocal (noise)
                m_Rk_1(2*i+1, 2*i+1) = oneSatlit.CC2_Smooth_Q;//Covariance C2 of pseudorange equations Reciprocal (noise)
            }
            else
            {
                m_Rk_1(2*i, 2*i) = 1 / oneSatlit.SatWight;//Covariance of C1  pseudorange equations Reciprocal (noise)
                m_Rk_1(2*i+1, 2*i+1) = 1 / oneSatlit.SatWight;//Covariance of C2 pseudorange equations Reciprocal (noise)
            }
        }
    }
    else
    {
        m_Rk_1.resize(B_len, B_len);// this m_Rk_1 is for ISB
        m_Rk_1.setIdentity();// this m_Rk_1 is for ISB
        for (int i = 0;i < epochLenLB;i++)
        {
            SatlitData oneSatlit = currEpoch.at(i);
            double Q_LP_whight  = 1 / m_LP_whight;// Contrast in paper 2019.05.11 by xiaogongwei
            m_Rk_1(2*i,2*i) = Q_LP_whight / oneSatlit.SatWight;//Covariance of carrier equation Reciprocal (small noise)// 1/25000 =4e-4
            m_Rk_1(2*i+1,2*i+1) = Q_LP_whight / oneSatlit.SatWight;//Covariance of carrier equation Reciprocal (small noise)// 1/25000 =4e-4
            m_Rk_1(2*i+epochLenLB,2*i+epochLenLB) = 1 /oneSatlit.SatWight;//Covariance of pseudorange equations Reciprocal (noise)
            m_Rk_1(2*i+epochLenLB+1,2*i+epochLenLB+1) = 1 /oneSatlit.SatWight;//Covariance of pseudorange equations Reciprocal (noise)
        }
    }
}
