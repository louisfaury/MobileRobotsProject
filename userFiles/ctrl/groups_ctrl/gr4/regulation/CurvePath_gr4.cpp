/*!
 * @file CurvePath.cpp
 * @author Louis Faury
 * @date 15/11
 */

#include "CurvePath_gr4.h"
#include "speed_regulation_gr4.h"
#include "config_file_gr4.h"

NAMESPACE_INIT(ctrlGr4);

CurvePath::CurvePath() : m_sign(1)
{
}

bool CurvePath::nextStep(double& alpha, double dt, CtrlStruct *cvs)
{

    bool end = false;

    double cRightSpeed = cvs->sp_reg->r_sp_ref;
    double cLeftSpeed = cvs->sp_reg->l_sp_ref;

    double maxDeltaAcc = MAX_ANGULAR_ACC*RobotGeometry::WHEEL_BASE;
    double deltaSpeed = m_sign * RobotGeometry::WHEEL_RADIUS * (cRightSpeed - cLeftSpeed); //curent deltaSpeed of the robot

    double deltaAcc;

    // speed profile, obtain to reach curve that arrive at objective with 0 angular speed at max angular desacc.
    deltaAcc = 1/(2*dt)*(-maxDeltaAcc*dt -2*deltaSpeed + sqrt( std::max(0.,maxDeltaAcc*maxDeltaAcc*dt*dt + 8*maxDeltaAcc*RobotGeometry::WHEEL_BASE*(m_length-alpha)
                                                                        -4*deltaSpeed*maxDeltaAcc*dt)) );

    // complying with kinematics constraints
    deltaAcc = std::min( deltaAcc, maxDeltaAcc);
    deltaAcc = std::max( deltaAcc, -maxDeltaAcc);
    deltaAcc = std::min( deltaAcc, (2*WHEEL_MAX_SPEED-deltaSpeed)/dt);
    deltaAcc = std::max( deltaAcc, -deltaSpeed/dt);

    //printf("deltaAcc:%f, s : %f\n",deltaAcc, RAD2DEG(alpha));

    // update
    alpha += (deltaSpeed*dt+deltaAcc*dt*dt)/(RobotGeometry::WHEEL_BASE);

    double nLeftSpeed = cLeftSpeed - m_sign * 0.5 * deltaAcc *dt / (RobotGeometry::WHEEL_RADIUS);
    double nRightSpeed = cRightSpeed + m_sign* 0.5 *deltaAcc * dt / (RobotGeometry::WHEEL_RADIUS);

    // applying to speed regulation
    speed_regulation(cvs, nRightSpeed, nLeftSpeed);

    if ( alpha > m_length -EPSILON )
    {
        alpha = m_length;
        end = true;
    }
    return end;
}

CurvePath::CurvePath(double angle, int sign) : Path(angle), m_sign(sign)
{
    m_id = "Curve";
}

NAMESPACE_CLOSE();
