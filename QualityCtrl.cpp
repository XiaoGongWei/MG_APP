#include "QualityCtrl.h"

QualityCtrl::QualityCtrl()
{

}

// // Calculating the rate of change of satellite clock difference
void QualityCtrl::CmpSatClkRate(const QVector<SatlitData> &prevEpochSatlitData, QVector<SatlitData> &epochSatlitData)
{
    int preEpochLen = prevEpochSatlitData.length();
    int epochLen = epochSatlitData.length();
    if(epochLen == 0) return ;
    for (int i = 0;i < epochLen;i++)
    {
        SatlitData epochData = epochSatlitData.at(i);
        epochSatlitData[i].StaClockRate = 0;
        //Cycle slip detection
        for (int j = 0;j < preEpochLen;j++)//(jump == 0) not happen clock jump  && (jump == 0)
        {
            SatlitData preEpochData = prevEpochSatlitData.at(j);
            if (epochData.PRN == preEpochData.PRN&&epochData.SatType == preEpochData.SatType)
            {
                epochSatlitData[i].StaClockRate = epochData.StaClock - preEpochData.StaClock;
            }
        }
    }


}
/*=========================================================================
 *
 *  Copyright David Doria 2012 daviddoria@gmail.com
 *goodClass
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0.txt
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *=========================================================================*/

MatrixXd QualityCtrl::GenerateData()
{//Dimension × number of samples
  // Create 6 2-D points
  MatrixXd points(2, 6);
  // case I points
  VectorXd p = VectorXd::Zero(2);
  p[0] = 10; p[1] = 10;
  points.col(0) = p;
  p[0] = 10.1; p[1] = 10.1;
  points.col(1) = p;
  p[0] = 10.2; p[1] = 10.2;
  points.col(2) = p;

  p[0] = 5; p[1] = 5;
  points.col(3) = p;
  p[0] = 5.1; p[1] = 5.1;
  points.col(4) = p;
  p[0] = 5.2; p[1] = 5.2;
  points.col(5) = p;

  // case II points
  MatrixXd myP;
  myP.resize(1,5);
  double mpionts[5] = {0.04200, 0.0414999, 0.046100, -0.2167, 0.042100};
  for(int i = 0;i < 5;i++)
  {
      myP(0,i) = mpionts[i];
  }
  return myP;
}



/*
 * Purpose: Eliminating gross errors by using receiver clock errors
 * ------------------------------------------------------------
 * Authors: XiaoGongWei; Email: xiaogongwei10@163.com;
 * Date: 2018-11-17; github: https://github.com/xiaogongwei
 * -------------------------------------------------------------
 * reference: none (it just data processing experience).
 */
bool QualityCtrl::VtPVCtrl_CLK(QVector < SatlitData > &epochSatlitData, double *predict_pos, VectorXd &del_flag)
{
    bool ishasgross = false;
    if(epochSatlitData.length() < 2) return ishasgross;
    MatrixXd mat_B, mat_P;
    VectorXd Vct_L_CLK, Vct_L_CLK_sort;// Assuming that predict_pos is accurate, Vct_L_CLK is the receiver clock difference
    VectorXi Vct_L_CLK_ind;
    Obtaining_equation(epochSatlitData, predict_pos, mat_B, Vct_L_CLK, mat_P, false);
    Vct_L_CLK = -Vct_L_CLK;
    sort_vec(Vct_L_CLK, Vct_L_CLK_sort, Vct_L_CLK_ind);

    // compute referance value
    double ref_val = 0.0;
    int flag = 0;
    for(int i = 2;i < epochSatlitData.length() - 2;i++)
    {
        ref_val += Vct_L_CLK_sort[i];
        flag++;
    }
    ref_val = ref_val / flag;

    // find delete item
    double zgama = 1.0;
    del_flag.resize(epochSatlitData.length());
    del_flag.setZero();
    for(int i = 0;i < epochSatlitData.length();i++)
    {
        if(abs(Vct_L_CLK[i] - ref_val) > 3*zgama)
        {
            del_flag[i] = 1;
            ishasgross = true;
        }
    }

    int a = 0;
    return ishasgross;

}

bool QualityCtrl::VtPVCtrl_CLKA(QVector < SatlitData > &epochSatlitData, double *predict_pos)
{
    bool ishasgross = false;
    if(epochSatlitData.length() < 2) return ishasgross;
    MatrixXd mat_B, mat_P;
    VectorXd Vct_L_CLK;// Assuming that predict_pos is accurate, Vct_L_CLK is the receiver clock difference
    Vct_L_CLK.resize(epochSatlitData.length());
    for(int i = 0; i < epochSatlitData.length();i++)
    {
        SatlitData oneSatlit = epochSatlitData.at(i);
        double p = 0,dltaX = 0,dltaY = 0,dltaZ = 0;
        dltaX = oneSatlit.X - predict_pos[0];
        dltaY = oneSatlit.Y - predict_pos[1];
        dltaZ = oneSatlit.Z - predict_pos[2];
        p = qSqrt(dltaX*dltaX+dltaY*dltaY+dltaZ*dltaZ);
        //计算L矩阵
        double dlta = 0;//各项那个改正
        dlta =  - oneSatlit.StaClock + oneSatlit.SatTrop - oneSatlit.Relativty -
            oneSatlit.Sagnac - oneSatlit.TideEffect - oneSatlit.AntHeight;
        Vct_L_CLK[i] = p - oneSatlit.PP3 + dlta;
    }

    int a = 0;

    return ishasgross;

}



/*
 * Purpose: Eliminating gross errors by v = L - B*X
 * ------------------------------------------------------------
 * Example:
 * v = B*x_s - L; if vi < threshold
 * B(i,:) and L(i) should delete.
 * del_flag=[0 0 0 1 0 0]; 1 is in ith positions
 *---------------- ---------------------------------------------
 * Input:
 * 		mat_B: 	observesion  matrix (n*m);
 *      vec_L:  observesion  Vector (n*1);
 *      vec_X:  Parameters to be solved
 * -------------------------------------------------------------
 * output:
 * 		del_flag: the variable like [0 0 1 0 1]; i instructor you should
 *                delete  B(i,:) and L(i)
 * -------------------------------------------------------------
 * Authors: XiaoGongWei; Email: xiaogongwei10@163.com;
 * Date: 2018-11-17; github: https://github.com/xiaogongwei
 * -------------------------------------------------------------
 * reference: none (it just data processing experience).
 */
bool QualityCtrl::VtPVCtrl_Filter_LC(MatrixXd mat_B, VectorXd vec_L, VectorXd vec_X, VectorXd &del_flag , int sat_len, double *LP_threshold)
{//Detect only carrier gross errors
    VectorXd vec_V;
    bool is_Gross_Error = false;
    // init VectorXd; If you ignore calculation speed, you can also use vec_X.setZero() or vec_V.setZero();
    del_flag.resize(sat_len);
    del_flag.setZero();
    vec_V = mat_B * vec_X - vec_L;
    // judge is statify the threshold
    VectorXd vL3 = vec_V.head(sat_len), VC3 = vec_V.segment(sat_len, sat_len);
    VectorXd vec_L3_abs = vL3.cwiseAbs(), vec_C3_abs = VC3.cwiseAbs();
    double L_threshold = 0.10, P_threshold = 10.0;
    if(LP_threshold)
    {
        L_threshold = LP_threshold[0]; P_threshold = LP_threshold[1];
    }
    int delete_sat_num = 0;
    // find gross error
    for(int i = 0;i < sat_len; i++)
    {
        bool bad_error = (vec_L3_abs[i] > L_threshold || vec_C3_abs[i] > P_threshold);// (vec_V_abs[i] > 3*Zgama_L && vec_V_abs[i] > 0.008) || (vec_V_abs[i] > 0.15)
        if(bad_error)// Carrier residual less than (3*Zgama_L&&0.08) and 15 cm 2019.05.05
        {
            del_flag[i] = 1;// use 1 to Delete indication
            is_Gross_Error = true;
            delete_sat_num++;
        }
    }

    // if delete all satilie
    double keep_pro = (double)(sat_len - delete_sat_num) / sat_len;//keep probability
    if(keep_pro < 0.8)
    {// delete n max error
        del_flag.setZero();
        int max_flag1 = 0, max_flag2 = 0;
        for(int i = 0;i < 1;i++)
        {
            vec_L3_abs.maxCoeff(&max_flag1);
            vec_C3_abs.maxCoeff(&max_flag2);
            if((vec_L3_abs[max_flag1] > L_threshold))
            {
                del_flag[max_flag1] = 1;
                is_Gross_Error = true;
                vec_L3_abs[max_flag1] = -1;
            }
            if((vec_C3_abs[max_flag2] > P_threshold))
            {
                del_flag[max_flag2] = 1;
                is_Gross_Error = true;
                vec_C3_abs[max_flag2] = -1;
            }
        }
    }

    return is_Gross_Error;
}

bool QualityCtrl::VtPVCtrl_Filter_LC_NoCombination(MatrixXd mat_B, VectorXd vec_L, VectorXd vec_X, VectorXd &del_flag , int sat_len, double *L12P12_threshold)
{//Detect only carrier gross errors
    VectorXd vec_V;
    bool is_Gross_Error = false;
    // init VectorXd; If you ignore calculation speed, you can also use vec_X.setZero() or vec_V.setZero();
    del_flag.resize(sat_len);
    del_flag.setZero();
    vec_V = mat_B * vec_X - vec_L;
    // judge is statify the threshold
    VectorXd vL1 = vec_V.head(sat_len), vL2 = vec_V.segment(sat_len, sat_len),// L1 and L2
            vC1 = vec_V.segment(2*sat_len, sat_len), vC2 = vec_V.segment(3*sat_len, sat_len);// C1 and C2
    VectorXd vec_L1_abs = vL1.cwiseAbs(), vec_L2_abs = vL2.cwiseAbs(),// abs of L1 and L2
            vec_C1_abs = vC1.cwiseAbs(), vec_C2_abs = vC2.cwiseAbs();// abs of C1 and C2
    double L1_threshold = 0.05, L2_threshold = 0.05,
           C1_threshold = 10.0, C2_threshold = 10.0;
    if(L12P12_threshold)
    {
        L1_threshold = L12P12_threshold[0]; L2_threshold = L12P12_threshold[1];
        C1_threshold = L12P12_threshold[2]; C2_threshold = L12P12_threshold[3];
    }
    int delete_sat_num = 0;
    // find gross error
    for(int i = 0;i < sat_len; i++)
    {
        bool bad_error = (vec_L1_abs[i] > L1_threshold || vec_L2_abs[i] > L2_threshold ||
                          vec_C1_abs[i] > C1_threshold || vec_C2_abs[i] > C2_threshold);// (vec_V_abs[i] > 3*Zgama_L && vec_V_abs[i] > 0.008) || (vec_V_abs[i] > 0.15)
        if(bad_error)// Carrier residual less than (3*Zgama_L&&0.08) and 15 cm 2019.05.05
        {
            del_flag[i] = 1;// use 1 to Delete indication
            is_Gross_Error = true;
            delete_sat_num++;
        }
    }
    // if delete all satilie
    double keep_pro = (double)(sat_len - delete_sat_num) / sat_len;//keep probability
    if(keep_pro < 0.8)
    {// delete n max error
        del_flag.setZero();
        double LC_threshold[4] = {L1_threshold, L2_threshold, C1_threshold, C2_threshold};
        VectorXd *vec_abs[4] = {&vec_L1_abs, &vec_L2_abs, &vec_C1_abs, &vec_C2_abs};
        int max_flag= 0;
        for(int i = 0;i < 4;i++)
        {
            vec_abs[i]->maxCoeff(&max_flag);
            if((*vec_abs[i])[max_flag] > LC_threshold[i])
            {
                del_flag[max_flag] = 1;
                is_Gross_Error = true;
                (*vec_abs[i])[max_flag] = -999.0;
            }
        }
    }
    return is_Gross_Error;
}


// Purpose: Eliminating gross errors if mean(v>std(v)) > 0.04;
bool QualityCtrl::VtPVCtrl_Filter_newIGG(MatrixXd mat_B, VectorXd vec_L, VectorXd vec_X, VectorXd &del_flag , int sat_len)
{//Detect only carrier gross errors
    VectorXd vec_V;
    int vec_len = 2*sat_len;
    bool is_Gross_Error = false;
    // init VectorXd; If you ignore calculation speed, you can also use vec_X.setZero() or vec_V.setZero();
    del_flag.resize(vec_len);
    del_flag.setZero();
    vec_V = mat_B * vec_X - vec_L;

    // solver Zgama
    VectorXd v1 = vec_V.head(sat_len), v2 = vec_V.tail(sat_len);
    double v1_mean1 = v1.mean(), Zgama_square_L = 0.0, Zgama_L = 0.0;
    for(int i = 0;i < sat_len; i++)
    {
        Zgama_square_L +=( (v1[i] - v1_mean1)* (v1[i] - v1_mean1) ) / (v1.size()-1);
    }
    Zgama_L = sqrt(Zgama_square_L);

    // judge is statify the threshold
    VectorXd vec_V1_abs = v1.cwiseAbs(), vec_V2_abs = v2.cwiseAbs();
    // find vec_V1_abs > Zgama_L
    int allMaxNum = 0;
    double sum_L = 0.0, mean_check = 0.0;
    for(int i = 0;i < sat_len;i++)
    {
        if(vec_V1_abs[i] > Zgama_L)
        {
            sum_L+=vec_V1_abs[i];
            allMaxNum++;
        }
    }
    mean_check = sum_L/allMaxNum;
    if(mean_check < 0.04) return false;

    // find gross error
    for(int i = 0;i < sat_len; i++)
    {
        bool bad_error = (vec_V2_abs[i] > 10.0);
        if(bad_error)// Carrier residual less than (3*Zgama_L&&0.08) and 15 cm 2019.05.05
        {
            del_flag[i] = 1;// use 1 to Delete indication
            del_flag[i+sat_len] = 1;
            is_Gross_Error = true;
        }
    }

    // delete n max error
    int max_flag1 = 0;
    for(int i = 0;i < 1;i++)
    {
        vec_V1_abs.maxCoeff(&max_flag1);
        if((vec_V1_abs[max_flag1] > 0.02))
        {
            del_flag[max_flag1] = 1; del_flag[max_flag1+sat_len] = 1;
            vec_V1_abs[max_flag1] = -1;
            is_Gross_Error = true;
        }
    }

    return is_Gross_Error;
}

bool QualityCtrl::VtPVCtrl_Filter_C(MatrixXd mat_B, VectorXd vec_L, VectorXd vec_X, VectorXd &del_flag, int sat_len )
{
    VectorXd vec_V;
    int vec_len = vec_L.size();
    bool is_Gross_Error = false;
    // init VectorXd; If you ignore calculation speed, you can also use vec_X.setZero() or vec_V.setZero();
    del_flag.resize(vec_len);
    del_flag.setZero();
    vec_V = mat_B * vec_X - vec_L;

    // compute Zgama
    double vec_V_mean = vec_V.mean(), Zgama = 0.0, Zgama_square = 0;

    for(int i = 0;i < sat_len; i++)
    {
        Zgama_square +=( (vec_V[i] - vec_V_mean)* (vec_V[i] - vec_V_mean) ) / (vec_V.size()-1);
    }
    Zgama = sqrt(Zgama_square);

    // judge is statify the threshold
    VectorXd vec_V_abs = vec_V.cwiseAbs();
    for(int i = 0;i < sat_len; i++)
    {
        if(vec_V_abs[i] > 3*Zgama || vec_V_abs[i] > 10)// Hypothesis based on pseudorange residuals not greater than 6
        {
            del_flag[i] = 1;// use 1 to Delete indication
            is_Gross_Error = true;
        }
    }
    // if delete all satilie
    double keep_pro = (sat_len - del_flag.sum()) / sat_len;//keep probability
    if(keep_pro < 0.8)
    {// delete n max error
        int max_flag = 0;
        del_flag.setZero();
        for(int i = 0;i < 1;i++)
        {
            vec_V_abs.maxCoeff(&max_flag);
            del_flag[max_flag] = 1;
            vec_V_abs[max_flag] = -1;
        }
        is_Gross_Error = true;
    }
    return is_Gross_Error;
}


/*
 * Purpose: Eliminating gross errors by solving errors
 * ------------------------------------------------------------
 * Example:
 * B*X = L;( P )
 * x_s = inv(Bt*P*B)*Bt*P*L;  v = B*x_s - L; if vi < Multiple*Zgama
 * B(i,:) and L(i) should delete.
 * del_flag=[0 0 0 1 0 0]; 1 is in ith positions
 *---------------- ---------------------------------------------
 * Input:
 * 		mat_B: 	observesion  matrix (n*m);
 *      vec_L:  observesion  Vector (n*1);
 *      mat_P:  Weight matrix (n*n)
 *      Multiple: Multiple*Zgama. value defualt is 3.
 * -------------------------------------------------------------
 * output:
 * 		del_flag: the variable like [0 0 1 0 1]; i instructor you should
 *                delete  B(i,:) and L(i)
 * -------------------------------------------------------------
 * Authors: XiaoGongWei; Email: xiaogongwei10@163.com;
 * Date: 2018-11-17; github: https://github.com/xiaogongwei
 * -------------------------------------------------------------
 * reference: none (it just data processing experience).
 */
bool QualityCtrl::VtPVCtrl_C(MatrixXd mat_B, VectorXd vec_L, MatrixXd mat_P, VectorXd &del_flag, int sat_len)
{
    return false;// not use quality control
    VectorXd vec_X, vec_V;
    bool is_Gross_Error = false;
    // init del_flag
    del_flag.resize(sat_len);
    del_flag.setZero();
    // slove by least square
    MatrixXd mat_Q = (mat_B.transpose()*mat_P*mat_B).inverse();
    vec_X = mat_Q*mat_B.transpose()*mat_P*vec_L;
    vec_V = mat_B * vec_X - vec_L;
    vec_V = vec_V.head(sat_len);
    // compute Zgama
    double vec_V_mean = vec_V.mean(), Zgama = 0.0, Zgama_square = 0;

    for(int i = 0;i < sat_len; i++)
    {
        Zgama_square +=( (vec_V[i] - vec_V_mean)* (vec_V[i] - vec_V_mean) ) / (vec_V.size()-1);
    }
    Zgama = sqrt(Zgama_square);

    // judge is statify the threshold
    VectorXd vec_V_abs = vec_V.cwiseAbs();
    for(int i = 0;i < sat_len; i++)
    {
        if(vec_V_abs[i] > 3*Zgama || vec_V_abs[i] > 6)// Hypothesis based on pseudorange residuals not greater than 6
        {
            del_flag[i] = 1;// use 1 to Delete indication
            is_Gross_Error = true;
        }
    }
    // if delete all satilie
    double keep_pro = (sat_len - del_flag.sum()) / sat_len;//keep probability
    if(keep_pro < 0.8)
    {// delete n max error
        int max_flag = 0;
        del_flag.setZero();
        for(int i = 0;i < 1;i++)
        {
            vec_V_abs.maxCoeff(&max_flag);
            del_flag[max_flag] = 1;
            vec_V_abs[max_flag] = -1;
        }
        is_Gross_Error = true;
    }
    return is_Gross_Error;
}


/*
 * Purpose: use del_flag solver least square
 * ------------------------------------------------------------
 * Example:
 * B*X = L;( P ) del_flag=[0 0 0 1 0 0]; 1 is gross errors
 * delete B(i,:) and L(i).
 * x_s = inv(Bt*P*B)*Bt*P*L;
 *---------------- ---------------------------------------------
 * Input:
 * 		mat_B: 	observesion  matrix (n*n);
 *      vec_L:  observesion  Vector (n*1);
 *      mat_P:  Weight matrix (n*n)
 *      del_flag: the variable like [0 0 1 0 1]; i instructor you should
 *                delete  B(i,:) and L(i)
 * -------------------------------------------------------------
 * output:
 *      vec_X: return LS solve result. like example <x_s>
 * -------------------------------------------------------------
 * Authors: XiaoGongWei; Email: xiaogongwei10@163.com;
 * Date: 2018-11-17; github: https://github.com/xiaogongwei
 * -------------------------------------------------------------
 * reference: none (it just data processing experience).
 */
bool QualityCtrl::solver_LS(MatrixXd mat_B, VectorXd vec_L, MatrixXd mat_P, VectorXd del_flag, VectorXd &vec_X)
{
    int vct_len = vec_L.size(), del_sum = del_flag.sum(), param_len = mat_B.cols();
    vec_X.resize(param_len);
    vec_X.fill(0);
    // Security check
    if(del_flag.size() != vct_len)
        return false;
    if(del_sum == 0)
        return false;
    if(vct_len - del_sum < param_len)
        return false;
    // set gross errors in mat_P is zeros;
    VectorXd vec_Zero = VectorXd::Zero(vct_len);
    for(int i = 0;i < vct_len;i++)
    {
        if(del_flag[i] != 0)
        {
            mat_P.row(i) = vec_Zero;
            mat_P.col(i) = vec_Zero;
            mat_P(i,i) = 1e-10;
        }
    }
    // slove by least square
    MatrixXd mat_Q = (mat_B.transpose()*mat_P*mat_B).inverse();
    vec_X = mat_Q*mat_B.transpose()*mat_P*vec_L;
    return true;
}

/*
 * Purpose: delete Mattrix rows when del_flag equal 1.
 * ------------------------------------------------------------
 * Example:
 * mat_B = [1 2 0;1 3 0;0 0 0;4 5 0] del_cols=[0 0 1]; del_cols = [0 0 1 0];1 is delete flag
 * deleteMat(mat_B,del_cols, del_rows)
 * mat_B = [1 2;1 3; 4 5]
 *
 * QualityCtrl m_qc;
    MatrixXd mat;
    mat.resize(4,3);
    mat<< 1,2,0,
            1,3,0,
            0,0,0,
            4,5,0;
    VectorXd del_cols, del_rows;
    del_cols.resize(3); del_cols.setZero(); del_cols[2] = 1;
    del_rows.resize(4); del_rows.setZero(); del_rows[2] = 1;
    m_qc.deleteMat(mat,del_cols, del_rows);
    cout << mat;
 *---------------- ---------------------------------------------
 * Input:
 * 		mat_B: 	matrix (n*n);
 *      del_flag: the variable like [0 0 0 1 0]; i instructor you should
 *                delete mat_B(i,:)
 * -------------------------------------------------------------
 * output:
 *      mat_B: 	matrix (n-k*n);
 * -------------------------------------------------------------
 * Authors: XiaoGongWei; Email: xiaogongwei10@163.com;
 * Date: 2018-11-18; github: https://github.com/xiaogongwei
 * -------------------------------------------------------------
 * reference: none (it just data processing experience).
 */

bool QualityCtrl::deleteMat(MatrixXd &mat_B, VectorXd del_cols, VectorXd del_rows)
{
    int mat_B_rows = mat_B.rows(), mat_B_cols = mat_B.cols(),
            del_cols_num = del_cols.sum(), del_rows_num = del_rows.sum();
    // Security check
    if(mat_B_rows != del_rows.size() || mat_B_cols != del_cols.size())
        return false;
    if(del_rows_num >= mat_B_rows || del_cols_num >= mat_B_cols)
        return false;
    //
    MatrixXd mat_B_copy = MatrixXd::Zero(mat_B_rows - del_rows_num, mat_B_cols - del_cols_num);
    int copy_i = 0, copy_j = 0;
    for(int ni = 0;ni < mat_B.rows();ni++)
    {
        copy_j = 0;
        if(0 != del_rows[ni]) continue;
        //for epoch rows
        for(int nj = 0;nj < mat_B.cols();nj++)
        {
            if(0 == del_cols[nj])
            {
                mat_B_copy(copy_i, copy_j) = mat_B(ni, nj);
                copy_j++;
            }
        }
        copy_i++;

    }
    mat_B = mat_B_copy;
    return true;
}


/*
 * Purpose: add Mattrix zeros at
 * ------------------------------------------------------------
 * Example:
 * mat_B = [1 2;1 3; 4 5] add_row_index = 1; add_col_index = 2
 * addZeroMat(mat_B, add_row_index, add_col_index)
 * mat_B = [1 2 0;1 3 0; 4 5 0;0 0 0 ]
 *
 * QualityCtrl m_qc;
    MatrixXd mat;
    mat.resize(3,2);
    mat<< 1,2,
            1,3,
            4,5;
    m_qc.addZeroMat(mat,1, 1);
    cout << mat;
 *---------------- ---------------------------------------------
 * Input:
 * 		mat_B: 	matrix (n*n);
 *      add_row_index: int
 *      add_col_index: int
 * -------------------------------------------------------------
 * output:
 *      mat_B: 	matrix (n+1, n+1);
 * -------------------------------------------------------------
 * Authors: XiaoGongWei; Email: xiaogongwei10@163.com;
 * Date: 2018-11-18; github: https://github.com/xiaogongwei
 * -------------------------------------------------------------
 * reference: none (it just data processing experience).
 */

bool QualityCtrl::addZeroMat(MatrixXd &mat_B, int add_row_index, int add_col_index)
{
    int mat_B_rows = mat_B.rows(), mat_B_cols = mat_B.cols();

    //
    MatrixXd mat_B_copy = MatrixXd::Zero(mat_B_rows + 1, mat_B_cols+1);

    for(int i = 0; i < mat_B.rows();i++)
    {
        for(int j = 0; j < mat_B.cols();j++)
        {
            if(i < add_row_index+1 && j < add_col_index+1)
                mat_B_copy(i,j) = mat_B(i,j);
            else if(i < add_row_index+1 && j >= add_col_index+1)
                mat_B_copy(i,j+1) = mat_B(i,j);
            else if(i >= add_row_index+1 && j < add_col_index+1)
                mat_B_copy(i+1,j) = mat_B(i,j);
            else
                mat_B_copy(i+1,j+1) = mat_B(i,j);
        }
    }
    mat_B = mat_B_copy;
    return true;
}

/*
*Purpose:Sort vectors from large to small
*vec: vector to be sorted
*sorted_vec: sorted results
*ind: the position of each element in the sorting result in the original vector
*/
void QualityCtrl::sort_vec(const VectorXd& vec, VectorXd& sorted_vec,  VectorXi& ind)
{
  ind=VectorXi::LinSpaced(vec.size(),0,vec.size()-1);//[0 1 2 3 ... N-1]
  auto rule=[vec](int i, int j)->bool{
    return vec(i)>vec(j);
  };//Regular expression as predicate of sort
  std::sort(ind.data(),ind.data()+ind.size(),rule);
  //The data member function returns a pointer to the first element of vectorxd, similar to begin()
  sorted_vec.resize(vec.size());
  for(int i=0;i<vec.size();i++){
    sorted_vec(i)=vec(ind(i));
  }
}


// get matrix B and observer L
void QualityCtrl::Obtaining_equation(QVector< SatlitData > &currEpoch, double *ApproxRecPos, MatrixXd &mat_B, VectorXd &Vct_L, MatrixXd &mat_P, bool isSmoothRange)
{
    int epochLenLB = currEpoch.length();
    MatrixXd B(epochLenLB, 4), P(epochLenLB, epochLenLB);
    VectorXd L(epochLenLB);
    B.setZero();
    L.setZero();
    P.setIdentity();

    for (int i = 0; i < epochLenLB;i++)
    {
        SatlitData oneSatlit = currEpoch.at(i);
        double li = 0,mi = 0,ni = 0,p0 = 0,dltaX = 0,dltaY = 0,dltaZ = 0;
        dltaX = oneSatlit.X - ApproxRecPos[0];
        dltaY = oneSatlit.Y - ApproxRecPos[1];
        dltaZ = oneSatlit.Z - ApproxRecPos[2];
        p0 = qSqrt(dltaX*dltaX+dltaY*dltaY+dltaZ*dltaZ);
        li = dltaX/p0;mi = dltaY/p0;ni = dltaZ/p0;
        //计算B矩阵
        //P3伪距码矩阵
        B(i, 0) = li;B(i, 1) = mi;B(i, 2) = ni; B(i, 3) = -1;
        //计算L矩阵
        double dlta = 0;//各项那个改正
        dlta =  - oneSatlit.StaClock + oneSatlit.SatTrop - oneSatlit.Relativty -
            oneSatlit.Sagnac - oneSatlit.TideEffect - oneSatlit.AntHeight;
        //伪距码PP3
        if(isSmoothRange)
        {// add by xiaogongwei 2018.11.20
            L(i) = p0 - oneSatlit.PP3_Smooth + dlta;
            // 计算权阵P
            P(i, i) = 1 / oneSatlit.PP3_Smooth_Q;//smooth伪距权????
        }
        else
        {
            L(i) = p0 - oneSatlit.PP3 + dlta;
            // 计算权阵P
            P(i, i) = oneSatlit.SatWight;//伪距权
        }

    }//B,L计算完毕
    // save data to mat_B
    mat_B = B;
    Vct_L = L;
    mat_P = P;
}
