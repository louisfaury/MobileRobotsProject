/*! 
 * \author Group 4
 * \file calibration_gr4.h
 * \brief calibration of the robot
 */

#ifndef _CALIBRATION_GR4_H_
#define _CALIBRATION_GR4_H_ 
 
#include "namespace_ctrl.h"
#include "CtrlStruct_gr4.h"

NAMESPACE_INIT(ctrlGr4);

/// robot calibration
typedef struct RobotCalibration
{
	double t_flag; ///< time for flag

	int flag; ///< flag for calibration

    static const double BOX_WIDTH = 0.5;

} RobotCalibration;

void calibration(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
