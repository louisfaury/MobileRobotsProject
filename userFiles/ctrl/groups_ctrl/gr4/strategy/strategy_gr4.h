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

/// strategy main structure
typedef struct Strategy
{
    static constexpr int TARGET_NUMBER  = 8;
    int main_state; ///< main state of the strategy
    Point* currentTarget;
    Point* targets[TARGET_NUMBER];
    bool found[TARGET_NUMBER];

} Strategy;

/// 'main_state' states (adapt with your own states)
enum {GAME_STATE_A, GAME_STATE_B, GAME_STATE_C, GAME_STATE_D, GAME_STATE_E};

Strategy* init_strategy();
void free_strategy(Strategy *strat);
void main_strategy(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
