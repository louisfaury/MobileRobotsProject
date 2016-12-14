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
    Target* targets[TARGET_NUMBER];
    Base* base;
    int currentTargetId; // TODO : find something better
    Point* currentTarget;
    double last_t, wait_t;
    int opp_ctr;
    bool triggerEndGame;
} Strategy;

/// 'main_state' states
enum {TARGET_HARVESTING_STATE, TARGET_PICKING_STATE, RETURN_TO_BASE_STATE, BASE_PICKING_STATE, WAIT_STATE, STUCK_STATE_TARGET, STUCK_STATE_BASE
     };

Strategy* init_strategy();
void free_strategy(Strategy *strat);
void main_strategy(CtrlStruct *cvs);
bool updateBestTarget(CtrlStruct* cvs);
void findClosestBase(CtrlStruct* cvs);
bool reachCheck(CtrlStruct* cvs);
bool checkTargetStatus(CtrlStruct* cvs);
void reset_reachable_states(Strategy *strat);
bool check_reachable_targets(Strategy* strat);
bool check_free_targets(Strategy* strat);

/*! Target structure */
struct Target
{
    Target(int i, double v, Point p);
    void updateScore(Point robPos, Point oppPos, int teamId, int targetNo);
    bool free;
    int id;
    int value;
    int closestTargetId;
    double score;
    double distanceToClosest;
    Point pos;
    bool reachable;
};

struct Base
{
    Base(Point p);
    Point loc;
};


NAMESPACE_CLOSE();

#endif
