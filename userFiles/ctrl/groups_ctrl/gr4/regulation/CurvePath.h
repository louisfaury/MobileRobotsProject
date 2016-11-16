#ifndef CURVEPATH_H
#define CURVEPATH_H

#include "namespace_ctrl.h"
#include "CtrlStruct_gr4.h"
#include "geometric_objects_gr4.h"
#include "useful_gr4.h"

NAMESPACE_INIT(ctrlGr4);

class CurvePath
{
public:
    CurvePath();
    CurvePath(double angle, bool sign);

    bool nextStep(double alpha, double dt, CtrlStruct* cvs);

    static constexpr double ANG_SPEED = PI/2;

protected:
    double m_angle;
    bool m_sign; // true if in trigonometric direction
};

NAMESPACE_CLOSE();

#endif // CURVEPATH_H
