 /*!
 * \author Group 4
 * \file strategy_gr4.h
 * \brief strategy during the game
 */

#ifndef _STRATEGY_GR4_H_
#define _STRATEGY_GR4_H_

#include "CtrlStruct_gr4.h"
#include "geometric_objects_gr4.h"
#include "array"

NAMESPACE_INIT(ctrlGr4);

//! some forward declarations
struct Target;
struct Base;

/// strategy main structure
typedef struct Strategy
{
    static constexpr int TARGET_NUMBER  = 8;
    static constexpr double STUCK_TIME = 1.;

    int main_state; ///< main state of the strategy
    Target* targets[TARGET_NUMBER]; ///< list of targets (hardcoded)
    Base* base; ///< coordinates of the robot base (hardcoded)
    int currentTargetId; ///<position of the current target in the target array
    Point* currentTarget; ///< coordinates of the current target
    double last_t, wait_t; ///<last_t : last time we entered in the strategy module, wait_t : last time we entered in a stuck state
    int opp_ctr; ///< number of opponents
    bool triggerEndGame; ///< boolean flag set to true when every target has been picked by the robot or an opponent
} Strategy;

/// 'main_state' states
enum {TARGET_HARVESTING_STATE, TARGET_PICKING_STATE, RETURN_TO_BASE_STATE, BASE_PICKING_STATE, WAIT_STATE, STUCK_STATE_TARGET, STUCK_STATE_BASE
     };

/**
 * @brief init_strategy : module intialisation, initialise targets list and base coordinate depending on the robot team
 * @return : pointer to the created strategy mmodule
 */
Strategy* init_strategy();
void free_strategy(Strategy *strat);

/**
 * @brief main_strategy Strategy loop corresponding to the FSM
 * @param cvs
 */
void main_strategy(CtrlStruct *cvs);

/**
 * @brief updateBestTarget : retrieves the currently reachable target with the highest score
 * @param cvs
 * @return : true if a target with score >-infinite  has been found
 */
bool updateBestTarget(CtrlStruct* cvs);

/**
 * @brief reachCheck : check if current target has been reached
 * @param cvs
 * @return
 */
bool reachCheck(CtrlStruct* cvs);

/**
 * @brief checkTargetStatus : updates targets status (free or already picked by the robot or an opponent) and checks if current target is still available
 * @param cvs
 * @return : true if current target is still free, false if we believe another robot has picked it
 */
bool checkTargetStatus(CtrlStruct* cvs);

/**
 * @brief reset_reachable_states : sets all targets reachable attribute to true
 * @param strat
 */
void reset_reachable_states(Strategy *strat);

/**
 * @brief check_reachable_targets : checks if at least one target is reachable
 * @param strat
 * @return : true/false
 */
bool check_reachable_targets(Strategy* strat);

/**
 * @brief check_free_targets : checks if at leats one target is free
 * @param strat
 * @return true/false
 */
bool check_free_targets(Strategy* strat);

/*! Target structure */
struct Target
{
    Target(int i, double v, Point p);
    /**
     * @brief updateScore : computes concerned target score according to the neural network structure
     * @param robPos : current robot position
     * @param oppPos : current opponent position
     * @param targetNo : number of targets currently carried by the robot
     */
    void updateScore(Point robPos, Point oppPos, int targetNo);

    bool free; ///<has the target been picked
    int id; ///< target id
    int value; ///< number of points
    int closestTargetId; ///< other target that is the closes to current target
    double score; ///< target computed score
    double distanceToClosest; ///< distance to the closest other target
    Point pos; ///< target coordinates
    bool reachable; ///< is it accessible to the robot ?
};

struct Base
{
    Base(Point p);
    Point loc;///< base coordinates
};


NAMESPACE_CLOSE();

#endif
