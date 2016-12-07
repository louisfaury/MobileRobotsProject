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
    static constexpr int BASE_NUMBER = 2;
    int main_state; ///< main state of the strategy
    Target* targets[TARGET_NUMBER];
    Base* bases[BASE_NUMBER];
    int currentTargetId; // TODO : find something better
    Point* currentTarget;
    double last_t;
    int opp_ctr;
} Strategy;

/// 'main_state' states
enum {TARGET_HARVESTING_STATE, TARGET_PICKING_STATE, RETURN_TO_BASE_STATE, BASE_PICKING_STATE, WAIT_STATE
     };

Strategy* init_strategy();
void free_strategy(Strategy *strat);
void main_strategy(CtrlStruct *cvs);
bool updateBestTarget(CtrlStruct* cvs);
void findClosestBase(CtrlStruct* cvs);
bool reachCheck(CtrlStruct* cvs);
bool checkTargetStatus(CtrlStruct* cvs);

/*! Target structure */
struct Target
{
    Target(int i, double v, Point p);
    void updateValue(Point robPos, Point oppPos);
    bool free;
    int id;
    int value;
    double distanceToClosest;
    Point pos;
};

struct Base
{
    Base(int i, Point p);

    int id;
    Point loc;
};


NAMESPACE_CLOSE();

#endif
