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
Target::Target(int i, double v, Point p) : free(true), id(i), value(v), pos(p), reachable(true)
{
}

/*!
 * \brief Target::updateValue : update the currrent (this) target value
 * \param robPos : calling robot position
 * \param oppPos : calling robot's opponents position
 */
void Target::updateScore(Point robPos, Point oppPos, int targetNo)
{
    // trained neural net
    double x1 = pos.computeDistance(oppPos);// distToOpp
    double x2 = pos.computeDistance(robPos);// distToRob
    double x3 = distanceToClosest;//dist to closest target
    double x4 = value; //target value
    double x5 = targetNo; //number of targets  carried by the robot
    double bias = 0.5;

    double omega11 = -0.9710;
    double omega21 = -0.3218;
    double omega12 = -3.1130;
    double omega22 = -3.1061;
    double omega32 = 1.6109;
    double omega42 = 5.2300;

    double z1 = sigmoid(omega11*x1 + omega21*x2);


    //neural net structure
    score = (free) ? ( omega12*(z1-bias) + omega22*x2 + omega32*x3*(1-x5) + omega42*x4 ) : -std::numeric_limits<double>::max();
}

/*!
 * \brief Base::Base
 * \param i : id
 * \param p : loc
 */
Base::Base(Point p) : loc(p)
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

    strat->targets[0] = new Target(0, 1, Point(TARGET_A_X,TARGET_A_Y));
    strat->targets[1] = new Target(1, 2, Point(TARGET_B_X, TARGET_B_Y));
    strat->targets[2] = new Target(2, 1, Point(TARGET_C_X, TARGET_C_Y));
    strat->targets[3] = new Target(3, 2, Point(TARGET_D_X, TARGET_D_Y));
    strat->targets[4] = new Target(4, 1, Point(TARGET_E_X, TARGET_E_Y));
    strat->targets[5] = new Target(5, 3, Point(TARGET_F_X, TARGET_F_Y));
    strat->targets[6] = new Target(6, 1, Point(TARGET_G_X, TARGET_G_Y));
    strat->targets[7] = new Target(7, 2, Point(TARGET_H_X, TARGET_H_Y));

    strat->targets[0]->distanceToClosest = (strat->targets[0]->pos).computeDistance(strat->targets[7]->pos);
    strat->targets[1]->distanceToClosest = (strat->targets[1]->pos).computeDistance(strat->targets[0]->pos);
    strat->targets[2]->distanceToClosest = (strat->targets[2]->pos).computeDistance(strat->targets[3]->pos);
    strat->targets[3]->distanceToClosest = (strat->targets[3]->pos).computeDistance(strat->targets[2]->pos);
    strat->targets[4]->distanceToClosest = (strat->targets[4]->pos).computeDistance(strat->targets[5]->pos);
    strat->targets[5]->distanceToClosest = (strat->targets[5]->pos).computeDistance(strat->targets[4]->pos);
    strat->targets[6]->distanceToClosest = (strat->targets[6]->pos).computeDistance(strat->targets[5]->pos);
    strat->targets[7]->distanceToClosest = (strat->targets[7]->pos).computeDistance(strat->targets[0]->pos);

    strat->base = new Base(Point(0.0,0.0));

    strat->currentTarget =  new Point(0.0,0.0);

    strat->last_t = 0.;
    strat->opp_ctr = 0;
    strat->triggerEndGame = false;

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
    delete(strat->base);
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

    strat->base->loc = ( cvs->team_id == TEAM_A ) ? Point(-BLUE_T1,-BLUE_T2) : Point(-YELLOW_T1,-YELLOW_T2);


    //FSM
    switch (strat->main_state)
    {

    //Going to the selected target
    case TARGET_HARVESTING_STATE:
        //Has it been reached ?
        if (!pathReg->reached)
        {
            //path control loop
            follow_path(cvs);
            if ( !checkTargetStatus(cvs) )
            {
                //If target is not free anymore we pick another target
                strat->main_state = TARGET_PICKING_STATE;
            }
        }
        else
        {// path regulation has reached goal
            reset_path_regulation(cvs);
            strat->last_t = t;
            strat->main_state = WAIT_STATE;
        }
        break;

    //choosing best target
    case TARGET_PICKING_STATE:
        reset_path_regulation(cvs);
        //check if their is an available target
        if(check_free_targets(strat))
        {
            //check if a target is reachable by the robot (a* can find a path)
            if(check_reachable_targets(strat)){
                updateBestTarget(cvs);
                if(pathPlanning(cvs)){
                    //path computed by a* algorithm
                    //going to harvesting mode
                    strat->main_state = TARGET_HARVESTING_STATE;
                }
                else
                {
                    //if not reachable by a*, we set its reachable attribute to false
                    reset_path_regulation(cvs);
                    strat->targets[strat->currentTargetId]->reachable = false;
                 }
            }
            else
            {
                //No reachable target, robot is stuck, we wait a bit
                strat->wait_t = t;
                strat->main_state = STUCK_STATE_TARGET;
            }
        }else{
            //No more target, end of the game, return to base
            strat->triggerEndGame = true;
            strat->main_state = BASE_PICKING_STATE;
        }

        break;

    //going to base
    case RETURN_TO_BASE_STATE:
        if (!pathReg->reached)
            follow_path(cvs);
        else
        {// path regulation has reached goal

            reset_path_regulation(cvs);
            if ( reachCheck(cvs) )
            {
                cvs->outputs->flag_release = true;
                if ( strat->triggerEndGame)
                    cvs->main_state = STOP_END_STATE;
                else
                    strat->main_state = TARGET_PICKING_STATE;
            }
            else
                strat->main_state = BASE_PICKING_STATE;
        }
        break;


    //currently blocked, we wait
    case STUCK_STATE_TARGET:
        // speed reg to 0, being cautious
        speed_regulation(cvs, 0., 0.);
        reset_path_regulation(cvs);
        if ( t-strat->wait_t >Strategy::STUCK_TIME )
        {
            //we have waited lon enough, all targets are set to reachable
            //going back to picking mode
            reset_reachable_states(strat);
            strat->main_state = TARGET_PICKING_STATE;
        }
        break;

    //currently blocked, we wait a bit
    case STUCK_STATE_BASE:
        // speed reg to 0, being cautious
        speed_regulation(cvs, 0., 0.);
        reset_path_regulation(cvs);
        if ( t-strat->wait_t >Strategy::STUCK_TIME )
        {
            strat->main_state = BASE_PICKING_STATE;
            reset_reachable_states(strat);
        }
        break;

     //is the base reachable ?
    case BASE_PICKING_STATE:
        *strat->currentTarget = strat->base->loc;
        reset_path_regulation(cvs);
        if ( pathPlanning(cvs) )
        {
            strat->main_state = RETURN_TO_BASE_STATE;
        }
        else
        {
            reset_path_regulation(cvs);
            strat->wait_t = t;
            strat->main_state = STUCK_STATE_BASE;
        }
        break;

    //we are on a target, we wait 1.5s to pick it
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
            {
                //We make sure to go elsewhere to make sure we do not get stuck here
                strat->targets[strat->currentTargetId]->reachable = false;
                strat->main_state = TARGET_PICKING_STATE;
            }
        }
        break;
    default:
        printf("Error: unknown strategy main state: %d !\n", strat->main_state);
        exit(EXIT_FAILURE);
    }
}



bool check_reachable_targets(Strategy* strat)
{
    bool res(false);
    Target* currentTarget;
    for (int i=0; i<strat->TARGET_NUMBER; i++)
    {
        currentTarget = strat->targets[i];
        if(currentTarget->reachable && currentTarget->free)
        {
            res = true;
            break;
        }
    }
    return res;
}

void reset_reachable_states(Strategy *strat)
{
    Target* currentTarget;
    for (int i=0; i<strat->TARGET_NUMBER; i++)
    {
        currentTarget = strat->targets[i];
        currentTarget->reachable = true;
    }
}

bool check_free_targets(Strategy* strat)
{
    bool res(false);
    Target* currentTarget;
    for (int i=0; i<strat->TARGET_NUMBER; i++)
    {
        currentTarget = strat->targets[i];
        if(currentTarget->free)
        {
            res = true;
            break;
        }
    }
    return res;

}

bool updateBestTarget(CtrlStruct *cvs)
{
    bool res(false);
    Strategy* strat = cvs->strat;
    double score(-std::numeric_limits<double>::max()), currentScore(0.);
    Target* currentTarget;
    for (int i=0; i<strat->TARGET_NUMBER; i++)
    {
        currentTarget = strat->targets[i];
        currentTarget->updateScore( Point(cvs->rob_pos->x,cvs->rob_pos->y), Point(cvs->opp_pos->x[1],cvs->opp_pos->y[1]), cvs->inputs->nb_targets);
        currentScore = currentTarget->score;

        if (currentScore>score)
        {
            //There is at least one free target, even if not reachable
            res = true;
            if(currentTarget->reachable)
            {
                score = currentScore;
                *strat->currentTarget = currentTarget->pos;
                strat->currentTargetId = i;
            }
        }
    }

    return res;
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
    Point oppPos = Point( cvs->opp_pos->x[0], cvs->opp_pos->y[0] );

    Target* cTarget;
    for (int i=0; i<Strategy::TARGET_NUMBER; i++)
    {
        cTarget = strat->targets[i];
        if ( oppPos.computeDistance(cTarget->pos) < 0.5*sqrt(2)*SearchGraph::CELL_SIZE )
        {
            strat->opp_ctr++;
            if (strat->opp_ctr > 10)
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
