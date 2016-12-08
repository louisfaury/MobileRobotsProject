/*! 
 * \author Group 4
 * \file useful_gr4.h
 * \brief useful functions to use in the controller
 */

#ifndef _USEFUL_GR4_H_
#define _USEFUL_GR4_H_

#include "namespace_ctrl.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr4);

// PI
#define PI 3.14159265

// macros
#define RAD2DEG(X) (180*X/PI)
#define DEG2RAD(X) (PI*X/180)

#define MODULOPI(X) (X>PI) ? fmod(X,2*PI)-2*PI : ( (X<-PI) ? fmod(X,2*PI)+2*PI : fmod(X,2*PI) )

#define EPSILON  0.00000001  //dealing with float imprecision

double rnd();
double norm_dist(double dx, double dy);
double limit_range(double x, double min, double max);
double limit_angle(double x);
double first_order_filter(double last_val, double new_val, double tau, double delta_t, double threshold);
double sigmoid(double x);

struct covMatrix3D
{
    double xx = {0};
    double xy = {0};
    double xtheta = {0};
    double yy = {0};
    double ytheta = {0};
    double thetatheta = {0};
};

NAMESPACE_CLOSE();

#endif
