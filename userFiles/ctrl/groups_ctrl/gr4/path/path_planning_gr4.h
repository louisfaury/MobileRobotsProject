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

/**
 * @brief free_path_planning : resets searchGraph
 * @param path : concerned PathPlanning
 */
void free_path_planning(PathPlanning *path);

/**
 * @brief pathPlanning : computes path from current robot position to current target
 * @param cvs : CtrlStruct instance representing the robot
 * @return true if path found, false otherwise
 */
bool pathPlanning(CtrlStruct* cvs);

/**
 * @brief updateOpponents : update opponents position on graph
 * @param cvs :  CtrlStruct instance representing the robot
 * @param opp :  current opponent position
 * @param id : opponent id (0 or 1)
 */
void updateOpponents(CtrlStruct* cvs, Point opp, int id);

/**
 * @brief oppOnPath : checks if an opponent is present on the already computed path leading to target
 * @param path : robot PathPlanning instance
 * @param refPath : previously computed path, currently followed by the robot
 * @return : true if an opponent is on the path, false otherwise
 */
bool oppOnPath(PathPlanning *path, LinePathList *refPath);

/**
 * @brief smoothPath : adds turns and target speed to the path computed by the A_star algorithm
 * @param cvs :  CtrlStruct instance representing the robot
 */
void smoothPath(CtrlStruct* cvs);

NAMESPACE_CLOSE();

#endif
