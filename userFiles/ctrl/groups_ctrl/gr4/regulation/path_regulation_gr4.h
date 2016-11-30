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
};

PathRegulation* init_path_regulation();
void follow_path(CtrlStruct *cvs);
void free_path_regulation(PathRegulation*);
void reset_path_regulation(CtrlStruct* cvs);

NAMESPACE_CLOSE();

#endif
