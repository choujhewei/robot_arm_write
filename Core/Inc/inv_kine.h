/*
 * inv_kine.h
 *
 *  Created on: Jul 21, 2025
 *      Author: jeffr
 */

#ifndef INC_INV_KINE_H_
#define INC_INV_KINE_H_

#ifdef __cplusplus
extern "C" {
#endif

#define DEG2RAD(x) ((x) * 0.0174533f)
#define RAD2DEG(x) ((x) * 57.2958f)
#define MAX_NODES 13


typedef struct {
    float p[3];      // position in world frame
    float R[3][3];   // orientation matrix in world frame
    float a[3];      // joint axis vector (body frame)
    float b[3];      // link offset vector (body frame)
    float q;         // joint angle
    float qmin;      // min joint limit
    float qmax;      // max joint limit
    int   mother;    // parent index
    int   child;     // first child index
    int   sister;    // sibling index
} Node_t;

extern Node_t Node[MAX_NODES];

void Node_Init(void);

void Rodrigues(const float w[3], float dt, float Rout[3][3]);

void ForwardKinematics(int j);

void CalcJacobianMatrix(const int idx_array[], int size, float J[6][MAX_NODES]);

void RPY2Rot(float roll, float pitch, float yaw, float Rout[3][3]);

void Rot2RPY(const float R[3][3], float rpy[3]);

void FindRouteTo(int id, int idx_array[], int *sz);

float InverseKinematics_LMP(int endNode_ID, const float target_p[3], const float target_R[3][3], int idx_array[], int *idx_size);

#ifdef __cplusplus
}
#endif

#endif /* INC_INV_KINE_H_ */
