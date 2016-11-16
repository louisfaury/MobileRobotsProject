#ifndef LINEPATH_H
#define LINEPATH_H

#include "namespace_ctrl.h"
#include "CtrlStruct_gr4.h"
#include "geometric_objects_gr4.h"

NAMESPACE_INIT(ctrlGr4);

class LinePath
{
public:
    LinePath();
    LinePath(Point start, double length, double angle);

    Point   start(){ return m_start; }
    double  length(){ return m_length; }
    double  angle(){ return m_angle; }
    bool    nextStep(double& s, double dt, CtrlStruct* cvs);

    static constexpr double MAX_ACC   = 1.0;
    static constexpr double MAX_DESAC = 1.0;
    static constexpr double MAX_SPEED = 2.0;

protected:
    Point m_start;
    double m_length;
    double m_angle;
};


NAMESPACE_CLOSE();

#endif // LINEPATH_H
