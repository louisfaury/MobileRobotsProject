/*!
 * @file LinePath.h
 * @author Louis Faury
 * @date 15/11
 */

#ifndef LINEPATH_H
#define LINEPATH_H

#include "namespace_ctrl.h"
#include "CtrlStruct_gr4.h"
#include "geometric_objects_gr4.h"
#include "Path_gr4.h"

NAMESPACE_INIT(ctrlGr4);

class LinePath : public Path
{
public:
    LinePath();
    LinePath(Point start, double length, double angle);

    Point   start(){ return m_start; }
    double  angle(){ return m_angle; }
    /*!
     * @function nextStep(double,double,CtrlStruct) : bool
     * @brief compute the speed regulation rules for the robot on the next dt time step
     * @returns bool if path is completed
     */
    virtual bool    nextStep(double& s, double dt, CtrlStruct* cvs);

    static constexpr double MAX_ACC   = 1.0;
    static constexpr double MAX_DESAC = 1.0;
    static constexpr double MAX_SPEED = 2.0;

protected:
    Point m_start;
    double m_angle;
};


NAMESPACE_CLOSE();

#endif // LINEPATH_H
