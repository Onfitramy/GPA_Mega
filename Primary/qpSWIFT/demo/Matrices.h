#ifndef __MATRICES_H__
#define __MATRICES_H__
#include "GlobalOptions.h"
#include "arm_math.h"

    /*
    QP *myQP;
	myQP = QP_SETUP_dense(3, 2, 0, P_m.pData, NULL, G_m.pData, q_v.pData, h, NULL, NULL, ROW_MAJOR_ORDERING);
	qp_int ExitCode = QP_SOLVE(myQP);
    */

    /* Cost Function P Matrix in CCS format */
    float P_d[9] = { 5.0, 1.0, 0.0, 1.0, 2.0, 1.0, 0.0, 1.0, 4.0 };
    arm_matrix_instance_f32 P_m = {3, 3, P_d};

    /* Cost Function c vector */
    float q_d[3] = {1.0, 2.0, 1.0};
    arm_matrix_instance_f32 q_v = {1, 3, q_d};

    /* Inequality Constraint G Matrix in CCS format */
    float G_d[6] = {-4.0, -4.0, 0.0, 0.0, 0.0, -1.0};
    arm_matrix_instance_f32 G_m = {2, 3, G_d};

    /* Inequality Constraint h vector */
    qp_real h[2] = {-1.0, -1.0};

#endif

/*! @file */