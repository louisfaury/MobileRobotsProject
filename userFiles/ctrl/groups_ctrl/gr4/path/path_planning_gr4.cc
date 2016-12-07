#include "path_planning_gr4.h"
#include "init_pos_gr4.h"
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
    PathPlanning* path = cvs->path;
    PathRegulation* path_reg = cvs->path_reg;
    LinePathList* refPath = path_reg->refPath;
    RobotPosition* robPos = cvs->rob_pos;
    int robotPosId;

    path->searchGraph->findCell(Point(robPos->x, robPos->y), robotPosId);
    refPath->smooth(robPos->theta, robotPosId);
}

void updateOpponents(CtrlStruct* cvs, Point opp, int index)
{
    RobotPosition *rob_pos = cvs->rob_pos;
    PathRegulation *path_reg = cvs->path_reg;
    PathPlanning *path = cvs->path;

    double distance;

    distance = sqrt((rob_pos->x-opp.x())*(rob_pos->x-opp.x())+(rob_pos->y-opp.y())*(rob_pos->y-opp.y()));
    //Updating opponents on Map
    path->searchGraph->updateOpponents(opp, index);
    //We now check if opponent is on our path and close enough to consider it as important
    if(distance < PathPlanning::MIN_DISTANCE_TO_OPPONENT)
    {
        if(oppOnPath(path, path_reg->refPath))
        {
            printf("Opponent on the way :(\n");
            reset_path_regulation(cvs);

            //TODO : what if not found (particularly if opponent is on target)
            pathPlanning(cvs);
        };
    }

}

bool oppOnPath(PathPlanning *path, LinePathList *refPath)
{
    bool res = false;
    std::vector<int> pathId = refPath->getPathId();
    std::vector<int>::iterator it = pathId.begin();
    for(; it!=pathId.end(); it++)
    {
        if(path->searchGraph->isOnOpponent(*it))
        {
            res = true;
            break;
        }

    }
    return res;
}


NAMESPACE_CLOSE();
