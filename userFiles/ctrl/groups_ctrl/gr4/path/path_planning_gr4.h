/*!
 * \author Group 4
 * \file path_planning_gr4.h
 * \brief path-planning algorithm
 */

#ifndef _PATH_PLANNING_GR4_H_
#define _PATH_PLANNING_GR4_H_

#include "namespace_ctrl.h"
#include "CtrlStruct_gr4.h"
#include "SearchGraph_gr4.h"

NAMESPACE_INIT(ctrlGr4);

/// path-planning main structure
struct PathPlanning
{
    SearchGraph* searchGraph;

    static constexpr double MIN_DISTANCE_TO_OPPONENT = 0.6;
};

PathPlanning* init_path_planning();
void free_path_planning(PathPlanning *path);
bool pathPlanning(CtrlStruct* cvs);
void updateOpponents(CtrlStruct* cvs, Point opp, int id);
bool oppOnPath(PathPlanning *path, LinePathList *refPath);
void smoothPath(CtrlStruct* cvs);

NAMESPACE_CLOSE();

#endif
