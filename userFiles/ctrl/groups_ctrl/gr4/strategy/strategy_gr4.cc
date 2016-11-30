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


    strat->currentTarget =  new Point(0.,0.);

    strat->last_t = 0.;

    return strat;
}

/*! \brief release the strategy structure (memory released)
 * 
 * \param[out] strat strategy structure to release
 */
void free_strategy(Strategy *strat)
{
    delete(strat->currentTarget);
    for(int i = 0; i< Strategy::TARGET_NUMBER; i++)
    {
        delete(strat->targets[i]);
    }
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
                follow_path(cvs);
            else
            {// path regulation has reached goal
                reset_path_regulation(cvs);
                strat->targets[strat->currentTargetId]->free = false;
                strat->last_t = t;
                strat->main_state = WAIT_STATE;
            }
            break;

        case TARGET_PICKING_STATE:
            updateBestTarget(cvs);
            if ( pathPlanning(cvs) )
                // path planning succeeded
                strat->main_state = TARGET_HARVESTING_STATE;
            // TODO : else case
			break;

        case RETURN_TO_BASE_STATE:
            if (!pathReg->reached)
                    follow_path(cvs);
            else
            {// path regulation has reached goal
                reset_path_regulation(cvs);
                strat->last_t = t;
                cvs->outputs->flag_release = true;
                strat->main_state = TARGET_PICKING_STATE;
            }
            break;

        case BASE_PICKING_STATE:
            *strat->currentTarget = Point(-0.75,-1.250);
            if ( pathPlanning(cvs) )
            {
                printf("return to base\n");
                strat->main_state = RETURN_TO_BASE_STATE;
            }
            break;

        case WAIT_STATE:
        cvs->outputs->flag_release = false;
            //waits for the target to be picked
            speed_regulation(cvs,0.,0.);
            if (t-strat->last_t>1.5)
            {
                if (inputs->nb_targets>=2)
                    strat->main_state = BASE_PICKING_STATE;
                else
                    strat->main_state = TARGET_PICKING_STATE;
            }
            break;
		default:
			printf("Error: unknown strategy main state: %d !\n", strat->main_state);
			exit(EXIT_FAILURE);
	}
}


void updateBestTarget(CtrlStruct *cvs)
{
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
}

NAMESPACE_CLOSE();

