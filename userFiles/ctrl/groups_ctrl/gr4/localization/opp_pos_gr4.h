/*! 
 * \author Group 4
 * \file init_pos_gr4.h
 * \brief opponents position
 */

#ifndef _OPP_POS_GR4_H_
#define _OPP_POS_GR4_H_ 
 
#include "namespace_ctrl.h"
#include "CtrlStruct_gr4.h"
#include "path_planning_gr4.h"

NAMESPACE_INIT(ctrlGr4);

/// robot position
typedef struct OpponentsPosition
{
	double x[2]; ///< x position of opponents [m]
	double y[2]; ///< y position of opponents [m]

	double last_t; ///< last time the filters were updated [s]
    double last_map_t; ///< last time MapHandler was updates [s]

	int nb_opp; ///< number of opponents

    static constexpr double  MAP_UPDATE_FREQUENCY = 0.5;

} OpponentsPosition;

// function prototype
int check_opp_front(CtrlStruct *cvs);
void opponents_tower(CtrlStruct *cvs);
int single_opp_tower(double last_rise, double last_fall, double rob_x, double rob_y, double rob_theta, double *new_x_opp, double *new_y_opp);

NAMESPACE_CLOSE();

#endif
