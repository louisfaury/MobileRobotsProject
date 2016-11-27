#include "strategy_gr4.h"
#include "path_planning_gr4.h"
#include "speed_regulation_gr4.h"
#include "path_regulation_gr4.h"
#include "init_pos_gr4.h"
#include "opp_pos_gr4.h"
#include "odometry_gr4.h"
#include "config_file.h"
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

    strat->targets[0] = (new Point(TARGET_A_X, TARGET_A_Y));
    strat->targets[1] = (new Point(TARGET_B_X, TARGET_B_Y));
    strat->targets[2] = (new Point(TARGET_C_X, TARGET_C_Y));
    strat->targets[3] = (new Point(TARGET_D_X, TARGET_D_Y));
    strat->targets[4] = (new Point(TARGET_E_X, TARGET_E_Y));
    strat->targets[5] = (new Point(TARGET_F_X, TARGET_F_Y));
    strat->targets[6] = (new Point(TARGET_G_X, TARGET_G_Y));
    strat->targets[7] = (new Point(TARGET_H_X, TARGET_H_Y));

    for(int i = 0; i< Strategy::TARGET_NUMBER;i++)
    {
        strat->found[i]=false;
    }

    strat->currentTarget = strat->targets[0];
    strat->found[0]= true;

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

void next_target(Strategy *strat)
{
    for(int i = 0; i< Strategy::TARGET_NUMBER; i++)
    {
        if(!strat->found[i])
        {
            strat->currentTarget = strat->targets[i];
            strat->found[i]=true;
            break;
        }
    }
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
    PathRegulation* pathReg = cvs->path_reg;

    static bool init = false;
    static bool found = false;

    // variables initialization
    strat  = cvs->strat;
    inputs = cvs->inputs;
    target = strat->currentTarget;

    if(pathReg->reached)
    {
        next_target(strat);
        reset(cvs);
        init=false;
    }

	switch (strat->main_state)
	{
		case GAME_STATE_A:

            if (!init)
            {
                found = pathPlanning(cvs);
                init = true;
            }
            if (found)
                follow_path(cvs);
            else
                speed_regulation(cvs, 0., 0.); // TODO : not pretty, need to be fixed once and far all
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
