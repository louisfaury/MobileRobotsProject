/*!
 * @file LinePath.cpp
 * @author Louis Faury
 * @date 15/11
 */

#include "LinePath_gr4.h"
#include "speed_regulation_gr4.h"

NAMESPACE_INIT(ctrlGr4);

LinePath::LinePath()
{
}


LinePath::LinePath(Point start, double length, double angle) : Path(length), m_start(start), m_angle(angle)
{
}

bool LinePath::nextStep(double &s, double dt, CtrlStruct *cvs)
{
    bool end = false;

    double cRightSpeed = cvs->inputs->r_wheel_speed;
    double cLeftSpeed = cvs->inputs->l_wheel_speed;
    double linearSpeed = 0.5 * (cRightSpeed + cLeftSpeed); //curent speed of the robot

    double tSpeed;
    double acc;

    // speed profile, obtain to reach curve that arrive at objective with 0 speed at max desacc.
    double speedTarget = MAX_DESAC * sqrt((2/MAX_DESAC)*-m_length-s);
    acc = (speedTarget - linearSpeed)/dt;
    // complying with kinematics constraints
    acc = std::min( acc, MAX_ACC);
    acc = std::max( acc, -MAX_DESAC);
    acc = std::min( acc, (MAX_SPEED-linearSpeed)/dt);

    // update
    tSpeed = linearSpeed + acc*dt;
    s += tSpeed*dt;

    // applying to speed regulation
    speed_regulation(cvs, tSpeed, tSpeed);

    if ( fabs(s-m_length)<EPSILON )
    {
        s = m_length;
        end = true;
    }
    return true;
}

NAMESPACE_CLOSE();
