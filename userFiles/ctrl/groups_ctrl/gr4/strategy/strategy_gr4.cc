#include "strategy_gr4.h"
#include "path_planning_gr4.h"
#include "speed_regulation_gr4.h"
#include "path_regulation_gr4.h"
#include "init_pos_gr4.h"
#include "opp_pos_gr4.h"
#include "odometry_gr4.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr4);

/*! \brief intitialize the strategy structure
 * 
 * \return strategy structure initialized
 */
Strategy* init_strategy()
{
    Strategy *strat;

    strat = (Strategy*) malloc(sizeof(Strategy));
    strat->target = new Point();
    return strat;
}

/*! \brief release the strategy structure (memory released)
 * 
 * \param[out] strat strategy structure to release
 */
void free_strategy(Strategy *strat)
{
    delete(strat->target);
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
    Point *target;
    PathPlanning* pathPlanner = cvs->path;
    // variables initialization
    strat  = cvs->strat;
    inputs = cvs->inputs;
    target = strat->target;

    //TODO : refine
    target->setCoord(0.,0.);

	switch (strat->main_state)
	{
		case GAME_STATE_A:
            static bool init = false;
            static bool found = false;
            if (!init)
            {
                found = pathPlanning(cvs);
                init = true;
            }
            if (found)
                follow_path(cvs);
			break;

		case GAME_STATE_B:
			speed_regulation(cvs, 0.0, 0.0);
			break;

		case GAME_STATE_C:
			speed_regulation(cvs, 0.0, 0.0);
			break;

		case GAME_STATE_D:
			speed_regulation(cvs, 0.0, 0.0);
			break;

		case GAME_STATE_E:
			speed_regulation(cvs, 0.0, 0.0);
			break;

		default:
			printf("Error: unknown strategy main state: %d !\n", strat->main_state);
			exit(EXIT_FAILURE);
	}
}

NAMESPACE_CLOSE();
