/*! 
 * \author Group 4
 * \file odometry_gr4.h
 * \brief odometry of the robot
 */

#ifndef _ODOMETRY_GR4_H_
#define _ODOMETRY_GR4_H_ 
 
#include "namespace_ctrl.h"
#include "CtrlStruct_gr4.h"

NAMESPACE_INIT(ctrlGr4);

void update_odometry(CtrlStruct *cvs);

static constexpr double WHEEL_RADIUS = 0.03; //m
NAMESPACE_CLOSE();

#endif
