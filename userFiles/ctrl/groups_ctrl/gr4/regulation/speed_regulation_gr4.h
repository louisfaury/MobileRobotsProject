/*! 
 * \author Group 4
 * \file speed_regulation.h
 * \brief speed regulation
 */

#ifndef _SPEED_REGULATION_GR4_H_
#define _SPEED_REGULATION_GR4_H_ 

#include "namespace_ctrl.h"
#include "CtrlStruct_gr4.h"
#include "config_file_gr4.h"

NAMESPACE_INIT(ctrlGr4);

/// speed regulation
typedef struct SpeedRegulation
{
	double int_error_r; ///< integral term of the error for the right wheel
	double int_error_l; ///< integral term of the error for the left wheel

    double r_sp_ref;
    double l_sp_ref;

	double last_t; ///< last time the speed regulation was updated

    static constexpr double PI_PROP_GAIN = 24;
    static constexpr double PI_INT_CHAR_TIME = 0.2;
    static constexpr double PI_MAX_INT_ERR = 2;
    static constexpr double PI_MIN_INT_ERR = -2;
    static constexpr double PI_OUTPUT_EXTREMUM = 6*PI; //limiting speed to 6PI rad/s

} SpeedRegulation;

void speed_regulation(CtrlStruct *cvs, double r_sp_ref, double l_sp_ref);

NAMESPACE_CLOSE();

#endif
