#include "path_regulation_gr4.h"
#include "path_planning_gr4.h"
#include "useful_gr4.h"
#include "speed_regulation_gr4.h"
#include "time.h"
#include "strategy_gr4.h"
#include "init_pos_gr4.h"

NAMESPACE_INIT(ctrlGr4);

PathRegulation *init_path_regulation()
{
    PathRegulation* path_regulation = (PathRegulation*)malloc(sizeof(PathRegulation));
    path_regulation->refPath = new LinePathList();
    path_regulation->reached = false;
    path_regulation->s = 0.;
    path_regulation->last_t = -1000.;
    path_regulation->lost_t = -1000.;

    return path_regulation;
}

/*! \brief follow a given path
 * 
 * \param[in,out] cvs controller main structure
 */
void follow_path(CtrlStruct *cvs)
{
    PathRegulation *path_reg = cvs->path_reg;
    LinePathList *refPath = path_reg->refPath;
    Strategy *strat = cvs->strat;
    double t = cvs->inputs->t;

    if(!refPath->isEmpty()){
        if(check_on_path(cvs))
        {// we're close to the target path
            double dt = t-path_reg->last_t;

            if (dt<0.1 && dt > EPSILON)
            {
                if ( !path_reg->reached )
                {
                    path_reg->reached = refPath->nextStep(path_reg->s, dt, cvs); // calls for next step on the path list
                }
                else
                    speed_regulation(cvs,0.,0.); // we reached final destination, sets speed to 0
            }
            path_reg->last_t=t;
        }
        else
        {// we have drifted from the target path
            reset_path_regulation(cvs);
            //If we were recently out of way, we might be against a wall
            if(fabs(t-path_reg->lost_t)<PathRegulation::MIN_LOST_TIME){
                strat->wait_t = t;
                if(strat->main_state == RETURN_TO_BASE_STATE)
                {
                    strat->main_state = BASE_WALL_STATE;
                }
                else{
                    strat->main_state = TARGET_WALL_STATE;
                }
            }else{
                if(!pathPlanning(cvs))
                {
                    reset_path_regulation(cvs); // reset the path reg and return in strategy to decision making competences
                    if(strat->main_state == RETURN_TO_BASE_STATE)
                    {
                        strat->main_state = BASE_PICKING_STATE;
                    }
                    else{
                        strat->main_state = TARGET_PICKING_STATE;
                    }
                }
            }
            path_reg->lost_t = t;

        }
    }
}

bool check_on_path(CtrlStruct* cvs)
{
    bool res(false);
    RobotPosition *rob_pos = cvs->rob_pos;
    LinePathList *path_ref = cvs->path_reg->refPath;
    Point pos = Point(rob_pos->x, rob_pos->y);
    Segment seg = path_ref->getCurrentSegment(cvs);

    if(seg.computeDistance(pos)<0.7*sqrt(2)*SearchGraph::CELL_SIZE)
    {
        res = true;
    }
    return res;
}

void free_path_regulation(PathRegulation* pr)
{
    delete(pr->refPath);
    free(pr);
}

void reset_path_regulation(CtrlStruct *cvs)
{
    // speed reg to 0, being cautious
    speed_regulation(cvs, 0., 0.);
    // clearing path
    PathRegulation* path_reg = cvs->path_reg;
    path_reg->refPath->clear();
    // re-init
    path_reg->s = 0.;
    path_reg->reached = false;

    //clearing map
    PathPlanning* path_pl = cvs->path;
    path_pl->searchGraph->clear();


}

NAMESPACE_CLOSE();


