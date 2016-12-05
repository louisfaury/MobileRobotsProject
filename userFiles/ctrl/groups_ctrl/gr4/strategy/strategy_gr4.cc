#include "strategy_gr4.h"
#include "path_planning_gr4.h"
#include "speed_regulation_gr4.h"
#include "path_regulation_gr4.h"
#include "init_pos_gr4.h"
#include "opp_pos_gr4.h"
#include "odometry_gr4.h"
#include "config_file.h"
#include <limits>
#include <math.h>

NAMESPACE_INIT(ctrlGr4);

/*!
 * \brief Target::Target : constructor for target struct
 * \param id : target id
 * \param v : value
 * \param p : loc
 */
Target::Target(int i, double v, Point p) : free(true), id(i), value(v), pos(p)
{
}

/*!
 * \brief Target::updateValue : update the currrent (this) target value
 * \param robPos : calling robot position
 * \param oppPos : calling robot's opponents position
 */
void Target::updateValue(Point robPos, Point oppPos)
{
    value = (free) ? pos.computeDistance(robPos) : 100.; // TODO : pimp with neural net wouhou
}

/*!
 * \brief Base::Base
 * \param i : id
 * \param p : loc
 */
Base::Base(int i, Point p) : id(i), loc(p)
{
}

/*! \brief intitialize the strategy structure
 * 
 * \return strategy structure initialized
 */
Strategy* init_strategy()
{
    Strategy *strat;

    strat = (Strategy*) malloc(sizeof(Strategy));

    strat->targets[0] = new Target(1, 1, Point(TARGET_A_X,TARGET_A_Y));
    strat->targets[1] = new Target(2, 2, Point(TARGET_B_X, TARGET_B_Y));
    strat->targets[2] = new Target(3, 1, Point(TARGET_C_X, TARGET_C_Y));
    strat->targets[3] = new Target(4, 2, Point(TARGET_D_X, TARGET_D_Y));
    strat->targets[4] = new Target(5, 1, Point(TARGET_E_X, TARGET_E_Y));
    strat->targets[5] = new Target(6, 3, Point(TARGET_F_X, TARGET_F_Y));
    strat->targets[6] = new Target(7, 1, Point(TARGET_G_X, TARGET_G_Y));
    strat->targets[7] = new Target(8, 2, Point(TARGET_H_X, TARGET_H_Y));

    strat->bases[0] = new Base(1, Point(BLUE_T1,BLUE_T2));
    strat->bases[1] = new Base(2, Point(-BLUE_T1, -BLUE_T2));

    strat->currentTarget =  new Point(0.,0.);

    strat->last_t = 0.;
    strat->opp_ctr = 0;

    return strat;
}

/*! \brief release the strategy structure (memory released)
 * 
 * \param[out] strat strategy structure to release
 */
void free_strategy(Strategy *strat)
{
    delete(strat->currentTarget);
    for(int i = 0; i < Strategy::TARGET_NUMBER; i++)
        delete(strat->targets[i]);
    for (int i = 0; i < Strategy::BASE_NUMBER; i++)
        delete(strat->bases[i]);
	free(strat);
}

/*! \brief strategy during the game
 * 
 * \param[in,out] cvs controller main structure
 */
void main_strategy(CtrlStruct *cvs)
{
    // variables declaration
    Strategy *strat;
    CtrlIn *inputs;
    PathRegulation* pathReg = cvs->path_reg;

    // variables initialization
    strat  = cvs->strat;
    inputs = cvs->inputs;
    double t = inputs->t;

	switch (strat->main_state)
	{
        case TARGET_HARVESTING_STATE:
            if (!pathReg->reached)
            {
                follow_path(cvs);
                if ( !checkTargetStatus(cvs) )
                {
                    strat->main_state = TARGET_PICKING_STATE;
                }
                // TODO : check if an opponent spend some time near a target
            }
            else
            {// path regulation has reached goal
                reset_path_regulation(cvs);
                strat->last_t = t;
                strat->main_state = WAIT_STATE;
            }
            break;

        case TARGET_PICKING_STATE:
            if ( updateBestTarget(cvs) )
            {
                if ( pathPlanning(cvs) )
                {
                    strat->main_state = TARGET_HARVESTING_STATE;
                }
            }
            else
            {// no more targets !
                strat->main_state = RETURN_TO_BASE_STATE;
            }

			break;

        case RETURN_TO_BASE_STATE:
            if (!pathReg->reached)
                    follow_path(cvs);
            else
            {// path regulation has reached goal
                reset_path_regulation(cvs);

                if ( reachCheck(cvs) )
                {
                    cvs->outputs->flag_release = true;
                    strat->main_state = TARGET_PICKING_STATE;
                }
                else
                    strat->main_state = BASE_PICKING_STATE;
            }
            break;

        case BASE_PICKING_STATE:
            findClosestBase(cvs);
            if ( pathPlanning(cvs) )
            {
                strat->main_state = RETURN_TO_BASE_STATE;
            }
            break;

        case WAIT_STATE:
        cvs->outputs->flag_release = false;
            //waits for the target to be picked
            speed_regulation(cvs,0.,0.);
            if (t-strat->last_t>1.5)
            {
                if ( reachCheck(cvs) )
                { //a new target was picked
                    strat->targets[strat->currentTargetId]->free = false;
                    if (inputs->nb_targets>=2)
                        strat->main_state = BASE_PICKING_STATE;
                    else
                        strat->main_state = TARGET_PICKING_STATE;
                }
                else
                    strat->main_state = TARGET_PICKING_STATE;
            }
            break;
		default:
			printf("Error: unknown strategy main state: %d !\n", strat->main_state);
			exit(EXIT_FAILURE);
	}
}


bool updateBestTarget(CtrlStruct *cvs)
{
    bool res(true);

    Strategy* strat = cvs->strat;
    double minValue(std::numeric_limits<double>::max()), currentValue(0.);
    Target* currentTarget;
    for (int i=0; i<strat->TARGET_NUMBER; i++)
    {
        currentTarget = strat->targets[i];
        currentTarget->updateValue( Point(cvs->rob_pos->x,cvs->rob_pos->y), Point(cvs->opp_pos->x[1],cvs->opp_pos->y[1]) );
        currentValue = currentTarget->value;
        if (currentValue<minValue)
        {
            minValue = currentValue;
            *strat->currentTarget = currentTarget->pos;
            strat->currentTargetId = i;
        }
    }

    if ( minValue == std::numeric_limits<double>::max())
        res = false; // no more targe to be found

    return res;
}

void findClosestBase(CtrlStruct* cvs)
{
    Strategy* strat = cvs->strat;

    double d1, d2;

    Point currentPos(cvs->rob_pos->x,cvs->rob_pos->y);
    d1 = currentPos.computeDistance((strat->bases[0])->loc);
    d2 = currentPos.computeDistance(strat->bases[1]->loc);

    *strat->currentTarget = (d1 < d2) ? strat->bases[0]->loc : strat->bases[1]->loc;
}

bool reachCheck(CtrlStruct *cvs)
{//returns true if we are indeed close to reach point
    Strategy* strat = cvs->strat;
    Point robPos = Point(cvs->rob_pos->x,cvs->rob_pos->y);

    double dist( strat->currentTarget->computeDistance(robPos) );

    return ( dist < sqrt(2)*SearchGraph::CELL_SIZE );
}

bool checkTargetStatus(CtrlStruct* cvs)
{
    bool res = true;
    Strategy* strat = cvs->strat;
    Point oppPos = Point( cvs->opp_pos->x[0], cvs->opp_pos->y[1] );

    Target* cTarget;
    for (int i=0; i<Strategy::TARGET_NUMBER; i++)
    {
        cTarget = strat->targets[i];
        if ( oppPos.computeDistance(cTarget->pos) < sqrt(2)*SearchGraph::CELL_SIZE )
        {
            strat->opp_ctr++;
            if (strat->opp_ctr > 20)
            {
                cTarget->free = false;
                strat->opp_ctr *= 0;
                if (i==strat->currentTargetId)
                    res = false;
            }

        }
    }

    return res;
}

NAMESPACE_CLOSE();
