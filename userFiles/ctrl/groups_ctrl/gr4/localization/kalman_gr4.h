/*! 
 * \author Group 4
 * \file kalman_gr4.h
 * \brief localization sensors fusion with Kalman
 */

#ifndef _KALMAN_GR4_H_
#define _KALMAN_GR4_H_

#include "CtrlStruct_gr4.h"
#include "init_pos_gr4.h"

NAMESPACE_INIT(ctrlGr4);

/// Kalman main structure
struct KalmanStruc
{
	int dummy_variable; ///< put your own variable, this is just an example without purpose
};

void kalman(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
