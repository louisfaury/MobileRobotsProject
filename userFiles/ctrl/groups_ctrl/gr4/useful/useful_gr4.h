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

#define MODULOPI(X) (fmod(X,2*PI)>PI+EPSILON) ? fmod(X,2*PI)-2*PI : ( (fmod(X,2*PI)<-PI-EPSILON) ? fmod(X,2*PI)+2*PI : fmod(X,2*PI) )
#define MIN(X, Y) (X>Y) ? Y : X

#define EPSILON  0.00000001  //dealing with float imprecision

double rnd();
double norm_dist(double dx, double dy);
double limit_range(double x, double min, double max);
double limit_angle(double x);
double first_order_filter(double last_val, double new_val, double tau, double delta_t, double threshold);
/*!
 * \brief sigmoid function for neural net
 * \param x : double value
 * \return 1 / ( 1 +exp(-x) )
 */
double sigmoid(double x);

/*!
 * \brief The covMatrix3D struct represents a 3d covariance matrix (symetric hence only n*(n+1)/2 = 6 coeffs)
 */
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
