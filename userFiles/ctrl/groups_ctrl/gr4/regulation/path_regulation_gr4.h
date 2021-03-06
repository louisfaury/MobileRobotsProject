/*! 
 * \author Group 4
 * \file path_regulation_gr4.h
 * \brief regulation to follow a given path
 */

#ifndef _PATH_REGULATION_GR4_H_
#define _PATH_REGULATION_GR4_H_

#include "CtrlStruct_gr4.h"
#include "LinePathList_gr4.h"

NAMESPACE_INIT(ctrlGr4);

struct PathRegulation
{
    LinePathList* refPath;
    bool reached;
    double s;
    double last_t;
    double lost_t;///<last time the robot lost its trajectory

    static const double MIN_LOST_TIME = 2.;
};

PathRegulation* init_path_regulation();
void follow_path(CtrlStruct *cvs);
void free_path_regulation(PathRegulation*);
/*!
 * \brief reset_path_regulation : reset attributes to be able to re-start path regulation
 * \param cvs : ptr to main ctrl structure
 */
void reset_path_regulation(CtrlStruct* cvs);
/*!
 * \brief check_on_path : checks if we have not drifted from the path
 * \param cvs
 * \return true if we are close enough from the target path
 */
bool check_on_path(CtrlStruct* cvs);

NAMESPACE_CLOSE();

#endif
