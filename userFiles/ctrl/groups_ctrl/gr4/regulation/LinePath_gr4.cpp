/*!
 * @file LinePath.cpp
 * @author Louis Faury
 * @date 15/11
 */

#include "LinePath_gr4.h"
#include <math.h>
#include "speed_regulation_gr4.h"
#include <time.h>

NAMESPACE_INIT(ctrlGr4);

LinePath::LinePath()
{
}


LinePath::LinePath(Point start, double length, double angle) : Path(length), m_start(start), m_angle(angle), m_endSpeed(0.)
{
    m_id = "Line";
}

bool LinePath::nextStep(double &s, double dt, CtrlStruct *cvs)
{
    bool end = false;

    double cRightSpeed = cvs->sp_reg->r_sp_ref;
    double cLeftSpeed = cvs->sp_reg->l_sp_ref;

    double linearSpeed = 0.5 * RobotGeometry::WHEEL_RADIUS * (cRightSpeed + cLeftSpeed); //curent speed of the robot

    double tSpeed;
    double acc;

    // speed profile, obtain to reach curve that arrive at objective with endSpeed at max desacc (obtained via derivative computations)
    acc = 1/(2*dt)*(-MAX_DESAC*dt -2*linearSpeed + sqrt( std::max(0.,MAX_DESAC*MAX_DESAC*dt*dt + 4*m_endSpeed*m_endSpeed + 8*MAX_DESAC*(m_length-s)
                                                                        -4*linearSpeed*MAX_DESAC*dt)) );

    // complying with kinematics constraints
    acc = std::min( acc, MAX_ACC);
    acc = std::max( acc, -MAX_DESAC);
    acc = std::min( acc, (MAX_SPEED-linearSpeed)/dt);
    acc = std::max( acc, -linearSpeed/dt);

    // update
    tSpeed = linearSpeed + acc*dt;
    s += linearSpeed*dt+0.5*acc*dt*dt;

    // applying to speed regulation
    speed_regulation(cvs, tSpeed/(RobotGeometry::WHEEL_RADIUS), tSpeed/(RobotGeometry::WHEEL_RADIUS));

    if ( s > m_length -EPSILON )
    {// we've reached the end of the path
        s = m_length;
        end = true;
    }

    return end;
}

double LinePath::smoothFromEnd(double endSpeed)
{
    double startSpeed;
    startSpeed = sqrt( endSpeed*endSpeed + 2*MAX_DESAC*m_length);
    startSpeed = std::min( MAX_SPEED,startSpeed);
    return startSpeed;
}

NAMESPACE_CLOSE();



