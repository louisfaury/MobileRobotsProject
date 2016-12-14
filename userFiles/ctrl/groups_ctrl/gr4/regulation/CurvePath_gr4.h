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
    CurvePath(double angle, int sign, int startId, int endId);

    /*!
     * @function nextStep(double,double,CtrlStruct) : bool
     * @brief compute the speed regulation rules for the robot on the next dt time step
     * @returns bool if path is completed
     */
    virtual bool nextStep(double& alpha, double dt, CtrlStruct* cvs);

    /*! setters !*/
    virtual void setEndSpeed(double){} //useless here, needed only for inheritance (father is virtual pure)
    virtual double smoothFromEnd(double /*endSpeed*/){ return 0; }

    /*! verbose functions for tests !*/
    virtual void describe(){ printf("%f,%s,%d\n",m_length, m_id.c_str(), m_sign); }

    /*! kinematics constraints !*/
    static const double MAX_ANGULAR_ACC = 4*PI;
    static const double WHEEL_MAX_SPEED = PI;

protected:
    int m_sign; // +1 if in trigonometric direction, -1 otherwise
};

NAMESPACE_CLOSE();

#endif // CURVEPATH_H
