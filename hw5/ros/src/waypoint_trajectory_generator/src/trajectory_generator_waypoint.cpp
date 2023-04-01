#include "trajectory_generator_waypoint.h"
#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;    
using namespace Eigen;

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint(){}
TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint(){}

//define factorial function, input i, output i!
int TrajectoryGeneratorWaypoint::Factorial(int x)
{
    int fac = 1;
    for(int i = x; i > 0; i--)
        fac = fac * i;
    return fac;
}
/*

    STEP 2: Learn the "Closed-form solution to minimum snap" in L5, then finish this PolyQPGeneration function

    variable declaration: input       const int d_order,                    // the order of derivative
                                      const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
                                      const Eigen::MatrixXd &Vel,           // boundary velocity
                                      const Eigen::MatrixXd &Acc,           // boundary acceleration
                                      const Eigen::VectorXd &Time)          // time allocation in each segment
                          output      MatrixXd PolyCoeff(m, 3 * p_num1d);   // position(x,y,z), so we need (3 * p_num1d) coefficients

*/

MatrixXd TrajectoryGeneratorWaypoint::getQ(const int segment, const int d_order, const int p_order, const Eigen::MatrixXd &time){
    int p_num1d = p_order+1;
    MatrixXd Q = MatrixXd::Zero(segment*p_num1d,segment*p_num1d);
    for(int k=0;k<segment;k++){
        MatrixXd Q_k = MatrixXd::Zero(p_num1d,p_num1d);
        for(int i=0;i<p_num1d;i++){
            for(int j=0;j<p_num1d;j++){
                if(i < d_order || j < d_order){
                    continue;
                }
                else{
                    Q_k(i,j) = 1.0 * Factorial(i)/Factorial(i-d_order) * Factorial(j)/Factorial(j-d_order) / (i+j-p_order) * pow(time(k),i+j-p_order);
                }
            }
        }
        Q.block(k*p_num1d,k*p_num1d,p_num1d,p_num1d) = Q_k;
    }
    return Q;
}

// MatrixXd getCoeff(const int d_order, const int p_order){
//     int p_num1d = p_order+1;
//     MatrixXd coeff = MatrixXd::Zero(d_order,p_num1d);
//     for(int i=0;i<d_order;i++){
//         for(int j=0;j<p_num1d;j++){
//             if(j<i) continue;
//             else{
//                 coeff(i,j) = Factorial(j) / Factorial(j-i);
//             }
//         }
//     }
//     return coeff;
// }

MatrixXd TrajectoryGeneratorWaypoint::getCoeff(const int d_order, const int p_order){
    int p_num1d = p_order+1;
    MatrixXd coeff = MatrixXd::Zero(d_order,p_num1d);
    for(int i=0;i<d_order;i++){
        for(int j=0;j<p_num1d;j++){
            if(j<i){
                continue;
            }
            else{
                coeff(i,j) = Factorial(j) / Factorial(j-i);
            }
        }
    }
    return coeff;
}

MatrixXd TrajectoryGeneratorWaypoint::getM(const int segment, const int d_order, const int p_order, const Eigen::MatrixXd &time){
    int p_num1d = p_order+1;
    MatrixXd M = MatrixXd::Zero(segment*p_num1d,segment*p_num1d);
    MatrixXd coeff = getCoeff(d_order,p_order);
    for(int k=0;k<segment;k++){
        MatrixXd M_k = MatrixXd::Zero(p_num1d,p_num1d);
        for(int i=0;i<d_order;i++){
            for(int j=0;j<p_num1d;j++){
                M_k(i+d_order,j) = coeff(i,j) * pow(time(k),j-i);
                if(i==j){
                    M_k(i,j) = coeff(i,j) * pow(time(k),j-i);
                }
            }
        }
        M.block(k*p_num1d,k*p_num1d,p_num1d,p_num1d) = M_k;
    }
    return M;

}

MatrixXd TrajectoryGeneratorWaypoint::getCt(const int segment, const int d_order,const int p_order){
    //d_order = 4,p_order = 7, segment = 5
    int ct_rows = d_order*2*segment; //40
    int ct_cols = d_order*2*segment - (segment-1)*d_order; //40-16 = 24
    cout << ct_rows << endl;
    cout << ct_cols << endl;
    MatrixXd Ct = MatrixXd::Zero(ct_rows,ct_cols);

    int start_col_df = 0; // 0
    int end_col_df = start_col_df + d_order*2+(segment-1) - 1 ; // 11
    int start_col_dp = end_col_df + 1; // 12
    int end_col_dp = ct_cols-1; // 23
    int overlap_jump = 2*d_order; // 8
    int start_col_last_df = start_col_df + d_order + segment-1; //0+4+4=8 
    int start_row_df = 0;
    int start_row_middle_point = start_row_df + d_order;
    int start_row_last_df = ct_rows - d_order; // 36   


    //第一段轨迹端点初d_order个状态变量
    for(int i=0;i<d_order;i++){
        Ct(i,i) = 1;
    }
    cout << 1 << endl;
    //中间每个控制点的位置状态
    for(int i=0;i<segment-1;i++){
        for(int j=0;j<d_order;j++){
            // cout << 2 << endl;
            printf("i:%d,j:%d\n",i,j);
            if(j==0){
                Ct(start_row_middle_point + i*overlap_jump + j,             d_order+i) = 1;
                Ct(start_row_middle_point + i*overlap_jump + j + d_order,   d_order+i) = 1;
            }
            else{
                cout << start_col_dp+i*(d_order-1)+j << endl;
                cout << start_col_dp+i*(d_order-1)+j << endl;
                cout << start_col_dp << endl;
                Ct(start_row_middle_point + i*overlap_jump + j,             start_col_dp + i*(d_order-1) + j-1) = 1;  //实际上j到这来是2，并且dp都是v,a,j，所以要j-1
                Ct(start_row_middle_point + i*overlap_jump + j + d_order,   start_col_dp + i*(d_order-1) + j-1) = 1;
            }
        }
    }
    cout << 1 << endl;
    //最后一段终末点的d_order个状态变量
    for(int i=0;i<d_order;i++){
        for(int j=0;j<d_order;j++){
            if(i==j){
                Ct(start_row_last_df+i,start_col_last_df+j) = 1;
            }
        }
    }
    return Ct;
}

Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
            const int d_order,                    // the order of derivative
            const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
            const Eigen::MatrixXd &Vel,           // boundary velocity
            const Eigen::MatrixXd &Acc,           // boundary acceleration
            const Eigen::VectorXd &Time)          // time allocation in each segment
{
    // enforce initial and final velocity and accleration, for higher order derivatives, just assume them be 0;
    int p_order   = 2 * d_order - 1;              // the order of polynomial
    int p_num1d   = p_order + 1;                  // the number of variables in each segment

    int segment = Time.size();                          // the number of segments
    MatrixXd PolyCoeff = MatrixXd::Zero(segment, 3 * p_num1d);           // position(x,y,z), so we need (3 * p_num1d) coefficients
    VectorXd Px(p_num1d * segment), Py(p_num1d * segment), Pz(p_num1d * segment);

    //get Q
    MatrixXd Q = getQ(segment,d_order,p_order,Time);
    cout << "Matrix Q is :\n" << endl << Q  << endl;


    //get M
    cout << segment << endl << d_order << endl << p_order << endl << Time << endl;
    MatrixXd M = getM(segment,d_order,p_order,Time);
    cout << "Matrix M is :\n" << endl << M << endl;


    //getCt
    MatrixXd Ct = getCt(segment,d_order,p_order); 
    cout << "Matrix Ct is :\n" << endl << Ct <<endl;

    MatrixXd C = Ct.transpose();
    cout << "C is : " << endl << C <<endl;


    MatrixXd M_inv = M.inverse();
    cout << "M_inv is : " << endl << M_inv <<endl;

    MatrixXd M_inv_t = M_inv.transpose();
    cout << "M_inv_t is : " << endl << M_inv_t <<endl;


    MatrixXd R = C * M_inv_t * Q * M_inv * Ct;
    cout << "R is : " << endl << R <<endl;


    int num_dF = 2 * d_order + segment - 1;
    int num_dP = (segment-1) * (d_order-1);

    MatrixXd R_pp = R.bottomRightCorner(num_dP,num_dP);
    cout << "R_pp is : " << endl << R_pp <<endl;

    MatrixXd R_fp = R.topRightCorner(num_dF,num_dP);
    cout << "R_fp is : " << endl << R_fp <<endl;

    //STEP3: COMPUTE df FOR x,y,z RESPECTIVELY
    for(int dim=0;dim<3;dim++){
        VectorXd wayPoints = Path.col(dim);
        cout << "waypoints: " << endl << wayPoints << endl;
        VectorXd d_F = VectorXd::Zero(num_dF);

        d_F(0) = wayPoints(0); //p0
        // add v0 a0 j0 ...
        for(int i=0;i<segment;i++){
            d_F(d_order+i) = wayPoints(i+1);
        } 
        d_F(d_order+segment-1) = wayPoints(segment);
        cout << "d_F is : " << endl << d_F <<endl;

        VectorXd d_P = -1.0 * R_pp.inverse() * R_fp.transpose() * d_F;
        cout << "d_P is : " << endl << d_P <<endl;


        VectorXd d_total(d_F.rows()+d_P.rows());
        d_total << d_F,d_P;
        VectorXd poly_coef_1d = M.inverse() * Ct * d_total;
        cout << "Dimension " << dim << " coefficients: " << endl << poly_coef_1d << endl;

        MatrixXd poly_coef_1d_t = poly_coef_1d.transpose();

        for(int k=0;k<segment;k++){
            PolyCoeff.block(k,dim*p_num1d,1,p_num1d) = poly_coef_1d_t.block(0,k*p_num1d,1,p_num1d);
        }
    }
    cout << "PolyCoeff : " << endl << PolyCoeff << endl;
    return PolyCoeff;
}
