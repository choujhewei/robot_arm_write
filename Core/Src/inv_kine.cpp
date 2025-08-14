/*
 * inv_kine.cpp
 *
 *  Created on: Jul 21, 2025
 *      Author: jeffr
 */
#include "main.h"
#include "inv_kine.h"
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stddef.h>

const float joint_min_deg[6] = {-180, -120, -135, -132, -90, -98};
const float joint_max_deg[6] = { 180,  120,  135,  67,  90,  98};

Node_t Node[MAX_NODES];

void Node_Init(void) {
	 Node[0] = (Node_t){{0,0,0}, {{1,0,0},{0,1,0},{0,0,1}}, {0,0,1}, {0,0,0}, 0.0f,
		 	 -3.1416f, 3.1416f, -1, 1, -1};

	 Node[1] = (Node_t){{0,0,0}, {{1,0,0},{0,1,0},{0,0,1}}, {0,0,1}, {0,0,0.07}, 0.0f,
	        DEG2RAD(joint_min_deg[0]), DEG2RAD(joint_max_deg[0]), 0, 2, -1};

	 Node[2] = (Node_t){{0,0,0}, {{1,0,0},{0,1,0},{0,0,1}}, {-1,0,0}, {0,0,0.04}, 0.0f,
	        DEG2RAD(joint_min_deg[1]), DEG2RAD(joint_max_deg[1]), 1, 3, -1};

	 Node[3] = (Node_t){{0,0,0}, {{1,0,0},{0,1,0},{0,0,1}}, {0,1,0}, {0,0,0.1054}, 0.0f,
	        DEG2RAD(joint_min_deg[2]), DEG2RAD(joint_max_deg[2]), 2, 4, -1};

	 Node[4] = (Node_t){{0,0,0}, {{1,0,0},{0,1,0},{0,0,1}}, {0,-1,0}, {0,0,0.1495}, 0.0f,
	        DEG2RAD(joint_min_deg[3]), DEG2RAD(joint_max_deg[3]), 3, 5, -1};

	 Node[5] = (Node_t){{0,0,0}, {{1,0,0},{0,1,0},{0,0,1}}, {0,0,1}, {0,-0.0183,0.0965}, 0.0f,
	        DEG2RAD(joint_min_deg[4]), DEG2RAD(joint_max_deg[4]), 4, 6, -1};

	 Node[6] = (Node_t){{0,0,0}, {{1,0,0},{0,1,0},{0,0,1}}, {-1,0,0}, {0,0,0.0536}, 0.0f,
	        DEG2RAD(joint_min_deg[5]), DEG2RAD(joint_max_deg[5]), 5, 7, -1};

	 Node[7] = (Node_t){{0,0,0}, {{1,0,0},{0,1,0},{0,0,1}}, {-1,0,0}, {0,0,0.049}, 0.0f,
		 0.0f, 0.0f, 6, -1, -1};
}


void Rodrigues(const float w[3], float dt, float Rout[3][3]) {
    float norm_w = sqrtf(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]);
    if (norm_w < 1e-6f) {
        memset(Rout, 0, 9*sizeof(float));
        Rout[0][0] = Rout[1][1] = Rout[2][2] = 1.0f;
        return;
    }
    float wn[3] = { w[0]/norm_w, w[1]/norm_w, w[2]/norm_w };
    float th = norm_w * dt;
    float W[3][3] = {
        {0,      -wn[2],  wn[1]},
        {wn[2],   0,     -wn[0]},
        {-wn[1],  wn[0],   0}
    };
    float W2[3][3] = {0};
    for(int i=0;i<3;i++) for(int j=0;j<3;j++)
        for(int k=0;k<3;k++) W2[i][j] += W[i][k]*W[k][j];
    float s = sinf(th), c = cosf(th);
    for(int i=0;i<3;i++) for(int j=0;j<3;j++)
        Rout[i][j] = (i==j?1.0f:0.0f) + W[i][j]*s + W2[i][j]*(1.0f-c);
}

void ForwardKinematics(int j) {
    if (j < 0 || j >= MAX_NODES) return;
    if (j > 0) {
        int mom = Node[j].mother;
        for(int i=0;i<3;i++)
            Node[j].p[i] = Node[mom].R[i][0]*Node[j].b[0]
                         + Node[mom].R[i][1]*Node[j].b[1]
                         + Node[mom].R[i][2]*Node[j].b[2]
                         + Node[mom].p[i];
        float Rloc[3][3];
        Rodrigues(Node[j].a, Node[j].q, Rloc);
        for(int i=0;i<3;i++) for(int k=0;k<3;k++)
            Node[j].R[i][k] = Node[mom].R[i][0]*Rloc[0][k]
                           + Node[mom].R[i][1]*Rloc[1][k]
                           + Node[mom].R[i][2]*Rloc[2][k];
    }
    if (Node[j].child  >= 0) ForwardKinematics(Node[j].child);
    if (Node[j].sister >= 0) ForwardKinematics(Node[j].sister);
}

void CalcJacobianMatrix(const int idx_array[], int size, float J[6][MAX_NODES]) {
    float target[3];
    for(int i=0;i<3;i++) target[i] = Node[idx_array[size-1]].p[i];
    for(int n=0;n<size;n++) {
        int j = idx_array[n];
        float a[3];
        for(int i=0;i<3;i++)
            a[i] = Node[j].R[i][0]*Node[j].a[0]
                 + Node[j].R[i][1]*Node[j].a[1]
                 + Node[j].R[i][2]*Node[j].a[2];
        float diff[3] = { target[0]-Node[j].p[0], target[1]-Node[j].p[1], target[2]-Node[j].p[2] };
        float cr[3] = { a[1]*diff[2] - a[2]*diff[1],
                        a[2]*diff[0] - a[0]*diff[2],
                        a[0]*diff[1] - a[1]*diff[0] };
        J[0][n] = cr[0]; J[1][n] = cr[1]; J[2][n] = cr[2];
        J[3][n] = a[0]; J[4][n] = a[1]; J[5][n] = a[2];
    }
}

void RPY2Rot(float roll, float pitch, float yaw, float Rout[3][3]) {
    float Cphi = cosf(roll),  Sphi = sinf(roll);
    float Cthe = cosf(pitch),Sthe = sinf(pitch);
    float Cpsi = cosf(yaw),   Spsi = sinf(yaw);
    Rout[0][0] = Cpsi*Cthe;
    Rout[0][1] = -Spsi*Cphi + Cpsi*Sthe*Sphi;
    Rout[0][2] =  Spsi*Sphi + Cpsi*Sthe*Cphi;
    Rout[1][0] = Spsi*Cthe;
    Rout[1][1] =  Cpsi*Cphi + Spsi*Sthe*Sphi;
    Rout[1][2] = -Cpsi*Sphi + Spsi*Sthe*Cphi;
    Rout[2][0] = -Sthe;
    Rout[2][1] =  Cthe*Sphi;
    Rout[2][2] =  Cthe*Cphi;
}

void Rot2RPY(const float R[3][3], float rpy[3]) {
    // Z-Y-X順序
    rpy[1] = -asinf(R[2][0]); // pitch
    if (fabsf(R[2][0]) < 0.999999f) {
        rpy[0] = atan2f(R[2][1], R[2][2]); // roll
        rpy[2] = atan2f(R[1][0], R[0][0]); // yaw
    } else {
        // singularity: pitch +-90度
        rpy[0] = atan2f(-R[0][1], R[1][1]);
        rpy[2] = 0;
    }
}

// 路徑搜尋
void FindRouteTo(int id, int idx_array[], int *sz) {
    if (id < 0) return;
    int mom = Node[id].mother;
    if (mom >= 0) FindRouteTo(mom, idx_array, sz);
    idx_array[(*sz)++] = id;
}

// --- SO(3) 誤差 ---
static void rot2omega(const float R[3][3], float w[3]) {
    float el[3] = { R[2][1]-R[1][2], R[0][2]-R[2][0], R[1][0]-R[0][1] };
    float norm_el = sqrtf(el[0]*el[0] + el[1]*el[1] + el[2]*el[2]);
    float trace_R = R[0][0] + R[1][1] + R[2][2];
    if (norm_el > 1e-8f) {
        float theta = atan2f(norm_el, trace_R-1.0f);
        for (int i=0; i<3; i++) w[i] = theta/norm_el * el[i];
    } else {
        w[0]=w[1]=w[2]=0.0f;
    }
}

static void CalcVWerr(const float Cref_p[3], const float Cref_R[3][3],
                      const float Cnow_p[3], const float Cnow_R[3][3], float err[6]) {
    for(int i=0; i<3; i++) err[i] = Cref_p[i] - Cnow_p[i];
    float RtR[3][3]={0};
    for (int i=0;i<3;i++) for (int j=0;j<3;j++)
        for (int k=0;k<3;k++) RtR[i][j] += Cnow_R[k][i]*Cref_R[k][j];
    float werr[3]; rot2omega(RtR, werr);
    for (int i=0;i<3;i++) { err[3+i]=0; for (int k=0;k<3;k++) err[3+i]+=Cnow_R[i][k]*werr[k]; }
}

// --- Joint Limit 懲罰法 ---
void CalcqPenalty(const int idx_array[], int n, float Penalty[]) {
    for (int i = 0; i < n; i++) {
        int j = idx_array[i];
        float over = (Node[j].q > Node[j].qmax) ? (Node[j].q - Node[j].qmax) : 0.0f;
        float under = (Node[j].q < Node[j].qmin) ? (Node[j].qmin - Node[j].q) : 0.0f;
        Penalty[i] = over*over + under*under;
    }
}
void CalcPenaltyMatrix(const int idx_array[], int n, float Jp[MAX_NODES][MAX_NODES]) {
    memset(Jp, 0, sizeof(float)*MAX_NODES*MAX_NODES);
    for (int i = 0; i < n; i++) {
        int j = idx_array[i];
        if (Node[j].q > Node[j].qmax)
            Jp[i][i]=1.0f;
        else if (Node[j].q < Node[j].qmin)
            Jp[i][i]=-1.0f;
        else
            Jp[i][i]=0.0f;
    }
}

// --- Gauss 消去法 ---
void GaussElimination(float A[MAX_NODES][MAX_NODES], float b[MAX_NODES], int n) {
    for (int i=0; i<n; i++) {
        float maxA = fabsf(A[i][i]);
        int maxRow = i;
        for (int k=i+1; k<n; k++) {
            if (fabsf(A[k][i]) > maxA) {
                maxA = fabsf(A[k][i]);
                maxRow = k;
            }
        }
        if (maxRow != i) {
            for (int j=0; j<n; j++) {
                float tmp = A[i][j]; A[i][j] = A[maxRow][j]; A[maxRow][j] = tmp;
            }
            float tmp = b[i]; b[i] = b[maxRow]; b[maxRow] = tmp;
        }
        for (int k=i+1; k<n; k++) {
            float c = A[k][i]/A[i][i];
            for (int j=i; j<n; j++)
                A[k][j] -= c*A[i][j];
            b[k] -= c*b[i];
        }
    }
    for (int i=n-1; i>=0; i--) {
        for (int j=i+1; j<n; j++)
            b[i] -= A[i][j]*b[j];
        b[i] /= A[i][i];
    }
}

float InverseKinematics_LMP(int endNode_ID, const float target_p[3], const float target_R[3][3], int idx_array[], int *idx_size) {
    // 參數
    const float wn_pos = 3.0f, wn_ang = 1.0f/(2.0f*M_PI), Wp = 10.0f;
    float We[6] = {wn_pos, wn_pos, wn_pos, wn_ang, wn_ang, wn_ang};
    *idx_size = 0;
    FindRouteTo(endNode_ID, idx_array, idx_size);
    int n = *idx_size;
    float I_mat[MAX_NODES][MAX_NODES] = {0};
    for (int i = 0; i < n; i++) I_mat[i][i] = 1.0f;
    ForwardKinematics(0);
    float err[6]; CalcVWerr(target_p, target_R, Node[endNode_ID].p, Node[endNode_ID].R, err);
    float Penalty[MAX_NODES] = {0}; CalcqPenalty(idx_array, n, Penalty);
    float Ek = 0; for (int i = 0; i < 6; i++) Ek += We[i]*err[i]*err[i];
    for (int i = 0; i < n; i++) Ek += Wp*Penalty[i]*Penalty[i];
    float dq[MAX_NODES], J[6][MAX_NODES], Jp[MAX_NODES][MAX_NODES], Jh[MAX_NODES][MAX_NODES], g_Err[MAX_NODES];
    for (int it = 0; it < 15; it++) {
        float en = 0; for (int i = 0; i < 6; i++) en += err[i]*err[i];
        en = sqrtf(en); if (en < 1e-6f) break;
        CalcJacobianMatrix(idx_array, n, J);
        CalcPenaltyMatrix(idx_array, n, Jp);
        memset(Jh, 0, sizeof(Jh));
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                for (int k = 0; k < 6; k++)
                    Jh[i][j] += J[k][i]*We[k]*J[k][j];
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                Jh[i][j] += Jp[i][i]*Wp*Jp[j][j]; // 懲罰項(對角)
        for (int i = 0; i < n; i++) Jh[i][i] += (Ek + 0.002f);
        for (int i = 0; i < n; i++) {
            g_Err[i] = 0;
            for (int k = 0; k < 6; k++)
                g_Err[i] += J[k][i]*We[k]*err[k];
            g_Err[i] += Jp[i][i]*Wp*Penalty[i];
        }
        float A[MAX_NODES][MAX_NODES]; float b[MAX_NODES];
        for (int i = 0; i < n; i++) { for (int j = 0; j < n; j++) A[i][j] = Jh[i][j]; b[i]=g_Err[i]; }
        GaussElimination(A, b, n);
        for (int i = 0; i < n; i++) dq[i] = b[i];
        for (int i = 0; i < n; i++) {
            int j = idx_array[i];
            Node[j].q += dq[i];
            if (Node[j].q > Node[j].qmax) Node[j].q = Node[j].qmax;
            if (Node[j].q < Node[j].qmin) Node[j].q = Node[j].qmin;
        }
        ForwardKinematics(0);
        CalcVWerr(target_p, target_R, Node[endNode_ID].p, Node[endNode_ID].R, err);
        CalcqPenalty(idx_array, n, Penalty);
        float Ek2 = 0; for (int i = 0; i < 6; i++) Ek2 += We[i]*err[i]*err[i];
        for (int i = 0; i < n; i++) Ek2 += Wp*Penalty[i]*Penalty[i];
        if (Ek2 < Ek) { Ek = Ek2; }
        else {
            for (int i = 0; i < n; i++) { int j = idx_array[i]; Node[j].q -= dq[i]; }
            ForwardKinematics(0);
            break;
        }
    }
    float norm = 0; for (int i = 0; i < 6; i++) norm += err[i]*err[i];
    return sqrtf(norm);
}
