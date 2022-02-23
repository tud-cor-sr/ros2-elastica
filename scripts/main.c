/******************************************************************************
 *                       Code generated with sympy 1.9                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                       This file is part of 'project'                       *
 ******************************************************************************/
#include "main.h"
#include <math.h>

void main(double del_phi_j, double l_j, double theta_j, double *out_687268234297198886) {

   out_687268234297198886[0] = pow(l_j, 2)*(cos(theta_j) - 1)*cos(del_phi_j)/pow(theta_j, 2);
   out_687268234297198886[1] = 0;
   out_687268234297198886[2] = 0;
   out_687268234297198886[3] = pow(l_j, 2)*(cos(theta_j) - 1)*sin(del_phi_j)/pow(theta_j, 2);
   out_687268234297198886[4] = 0;
   out_687268234297198886[5] = 0;
   out_687268234297198886[6] = pow(l_j, 2)*(theta_j - sin(theta_j))/pow(theta_j, 2);
   out_687268234297198886[7] = 0;
   out_687268234297198886[8] = 1;
   out_687268234297198886[9] = -l_j*sin(del_phi_j);
   out_687268234297198886[10] = 0;
   out_687268234297198886[11] = -pow(theta_j, 2)*sin(del_phi_j)/pow(l_j, 2);
   out_687268234297198886[12] = l_j*cos(del_phi_j);
   out_687268234297198886[13] = 0;
   out_687268234297198886[14] = pow(theta_j, 2)*cos(del_phi_j)/pow(l_j, 2);
   out_687268234297198886[15] = 0;
   out_687268234297198886[16] = 1;
   out_687268234297198886[17] = 0;

}
