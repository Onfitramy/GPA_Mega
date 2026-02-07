#ifndef __MATRICES_H__
#define __MATRICES_H__
#include "GlobalOptions.h"
#include "arm_math.h"

    /* Cost Function P Matrix in CCS format */
    qp_real P_data[9] = { 5.0, 1.0, 0.0, 1.0, 2.0, 1.0, 0.0, 1.0, 4.0 };
    arm_matrix_instance_f64 P_mat = {3, 3, P_data};

    /* Cost Function c vector */
    qp_real c[3] = {1.0, 2.0, 1.0};

    /* Inequality Constraint G Matrix in CCS format */
    qp_real G_data[6] = {-4.0, -4.0, 0.0, 0.0, 0.0, -1.0};
    arm_matrix_instance_f64 G_mat = {2, 3, G_data};

    /* Inequality Constraint h vector */
    qp_real h[2] = {-1.0, -1.0};

#endif

/*! @file */