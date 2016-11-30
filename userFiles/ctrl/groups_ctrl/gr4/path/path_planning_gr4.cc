#include "path_planning_gr4.h"
#include "init_pos_gr4.h"
#include "opp_pos_gr4.h"
#include "useful_gr4.h"
#include "strategy_gr4.h"
#include "path_regulation_gr4.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr4);

/*! \brief initialize the path-planning algorithm (memory allocated)
 * 
 * \param[in,out] path path-planning main structure
 */
PathPlanning* init_path_planning()
{
	PathPlanning *path;

	// memory allocation
	path = (PathPlanning*) malloc(sizeof(PathPlanning));
    path->searchGraph = new SearchGraph();

	return path;
}

/*! \brief close the path-planning algorithm (memory released)
 * 
 * \param[in,out] path path-planning main structure
 */
void free_path_planning(PathPlanning *path)
{
    delete(path->searchGraph);

	free(path);
}

bool pathPlanning(CtrlStruct *cvs)
{
    bool res(false);

    PathPlanning* path = cvs->path;
    Strategy* strat = cvs->strat;
    PathRegulation* path_reg = cvs->path_reg;

    Point endLoc(strat->currentTarget->x(), strat->currentTarget->y());
    Point startLoc(cvs->rob_pos->x,cvs->rob_pos->y);

    int startId(0), endId(0);
    // find current cell given pos
    if ( path->searchGraph->findCell(startLoc,startId) )
        if ( path->searchGraph->findCell(endLoc,endId) )
        {
            res = path->searchGraph->computePath(path_reg->refPath, startId, endId);
            if (res && !path_reg->refPath->isEmpty())
                    smoothPath(cvs);
        }

    return res;
}

void smoothPath(CtrlStruct *cvs)
{
    PathRegulation* path_reg = cvs->path_reg;
    LinePathList* refPath = path_reg->refPath;
    RobotPosition* robPos = cvs->rob_pos;
    refPath->smooth(robPos->theta);
}

NAMESPACE_CLOSE();
