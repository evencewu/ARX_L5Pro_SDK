#pragma once

#include <cstddef>
#include <math.h>
#include <arx_l5pro_api/Eigen/Dense>
#include <arx_l5pro_api/Eigen/Geometry> // 包含变换和位姿处理

namespace arx
{
    namespace solve
    {
        void l5_fk(double q[6], double send_end[6]);
        void fk(float q[6], float T[4][4]);
        void T_2_R(float T[4][4], float R[3][3]);
        void R_dot_R(float matrixA[3][3], float matrixB[3][3], float result[3][3]);
        void R_rot_xyz(float roll, float M_PItch, float yaw, float Rx[3][3], float Ry[3][3], float Rz[3][3]);
        void my_GetRPY(float R[3][3], float rpy[3]);
        void T_dot_T(float matrixA[4][4], float matrixB[4][4], float result[4][4]);
        void get_dh_T(float a, float alpha, float d, float q, float R[4][4]);

        void l5_ik_lm(double q0[6], double endstate_[6], double jointstate[6]);
        void ik_LM(float T_desir[4][4], float q0[6], float q_ik[6]);
        void my_tr2dlta(float A[4][4], float B[4][4], float dlt[6]);
        void my_get_jacobe(float q_[6], float jc[6][6]);
        void jcb_trans(float J[6][6], float Jt[6][6]);
        void J_dot_J(float matrixA[6][6], float matrixB[6][6], float result[6][6]);
        void J_dot_P(float R[6][6], float P[6], float relt[6]);
        void my_invT(float T[4][4], float Tinv[4][4]);
        void T_2_P(float T[4][4], float P[3]);
        void R_trans(float R[3][3], float Rt[3][3]);
        void R_dot_P(float R[3][3], float P[3], float relt[3]);

        Eigen::Isometry3d Xyzrpy2Isometry(double input[6]);
        std::vector<double> Isometry2Xyzrpy(Eigen::Isometry3d pose);
    }
}