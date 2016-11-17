/*!
 * @file CurvePath.h
 * @author Louis Faury
 * @date 15/11
 */

#ifndef CURVEPATH_H
#define CURVEPATH_H

#include "namespace_ctrl.h"
#include "CtrlStruct_gr4.h"
#include "geometric_objects_gr4.h"
#include "useful_gr4.h"
#include "Path_gr4.h"

NAMESPACE_INIT(ctrlGr4);

class CurvePath : public Path
{
public:
    CurvePath();
    CurvePath(double angle, bool sign);

    /*!
     * @function nextStep(double,double,CtrlStruct) : bool
     * @brief compute the speed regulation rules for the robot on the next dt time step
     * @returns bool if path is completed
     */
    virtual bool nextStep(double& alpha, double dt, CtrlStruct* cvs);

    static constexpr double ANG_SPEED = PI/2;

protected:
    bool m_sign; // true if in trigonometric direction
};

NAMESPACE_CLOSE();

#endif // CURVEPATH_H
