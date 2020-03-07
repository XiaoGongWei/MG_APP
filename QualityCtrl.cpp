#include "QualityCtrl.h"

QualityCtrl::QualityCtrl()
{

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
bool QualityCtrl::VtPVCtrl_Filter_LC(MatrixXd mat_B, VectorXd vec_L, VectorXd vec_X, VectorXd &del_flag , int sat_len)
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
    double v1_mean1_abs = vec_V1_abs.mean();
    int delete_sat_num = 0;
    // find gross error
    for(int i = 0;i < sat_len; i++)
    {
        bool bad_error = (vec_V1_abs[i] > 0.10 || vec_V2_abs[i] > 10.0 );// (vec_V_abs[i] > 3*Zgama_L && vec_V_abs[i] > 0.008) || (vec_V_abs[i] > 0.15)
        if(bad_error)// Carrier residual less than (3*Zgama_L&&0.08) and 15 cm 2019.05.05
        {
            del_flag[i] = 1;// use 1 to Delete indication
            del_flag[i+sat_len] = 1;
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
            vec_V1_abs.maxCoeff(&max_flag1);
            vec_V2_abs.maxCoeff(&max_flag2);
            if((vec_V1_abs[max_flag1] > 0.10))
            {
                del_flag[max_flag1] = 1; del_flag[max_flag1+sat_len] = 1;
                vec_V1_abs[max_flag1] = -1;
            }
            if((vec_V2_abs[max_flag2] > 10.0))
            {
                del_flag[max_flag2] = 1; del_flag[max_flag2+sat_len] = 1;
                vec_V2_abs[max_flag2] = -1;
            }
        }
        is_Gross_Error = true;
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
	for (int i = 0; i < epochLenLB; i++)
	{
		SatlitData oneSatlit = currEpoch.at(i);
		double li = 0, mi = 0, ni = 0, p0 = 0, dltaX = 0, dltaY = 0, dltaZ = 0;
		dltaX = oneSatlit.X - ApproxRecPos[0];
		dltaY = oneSatlit.Y - ApproxRecPos[1];
		dltaZ = oneSatlit.Z - ApproxRecPos[2];
		p0 = qSqrt(dltaX*dltaX + dltaY * dltaY + dltaZ * dltaZ);
		li = dltaX / p0; mi = dltaY / p0; ni = dltaZ / p0;
		B(i, 0) = li; B(i, 1) = mi; B(i, 2) = ni; B(i, 3) = -1;
		double dlta = 0;
		dlta = -oneSatlit.StaClock + oneSatlit.SatTrop - oneSatlit.Relativty -
			oneSatlit.Sagnac - oneSatlit.TideEffect - oneSatlit.AntHeight;
		if (isSmoothRange)
		{// add by xiaogongwei 2018.11.20
			L(i) = p0 - oneSatlit.PP3_Smooth + dlta;
			P(i, i) = 1 / oneSatlit.PP3_Smooth_Q;
		}
		else
		{
			L(i) = p0 - oneSatlit.PP3 + dlta;
			P(i, i) = oneSatlit.SatWight;
		}
	}
	// save data to mat_B
	mat_B = B;
	Vct_L = L;
	mat_P = P;
}
