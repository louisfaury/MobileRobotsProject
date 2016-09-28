/*! 
 * \author Group 4
 * \file mcl_gr4.h
 * \brief localization using Monte Carlo technic
 */

#ifndef _MCL_GR4_H_
#define _MCL_GR4_H_

#include "CtrlStruct_gr4.h"
#include "init_pos_gr4.h"

NAMESPACE_INIT(ctrlGr4);

/// Markov main structure
struct MCLStruct
{
	int dummy_variable; ///< put your own variable, this is just an example without purpose
};

void mcl_update(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
