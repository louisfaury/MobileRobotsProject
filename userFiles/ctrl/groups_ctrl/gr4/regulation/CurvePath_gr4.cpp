/*!
 * @file CurvePath.cpp
 * @author Louis Faury
 * @date 15/11
 */

#include "CurvePath_gr4.h"
#include "speed_regulation_gr4.h"
#include "config_file_gr4.h"

NAMESPACE_INIT(ctrlGr4);

CurvePath::CurvePath()
{
}

bool CurvePath::nextStep(double& alpha, double dt, CtrlStruct *cvs)
{
    bool end = false;

    double nAngle;
    double correctCoeff(1.);
    double angSpeed, deltaWheelSpeed;

    // here m_length = angle !
    if ( m_length > EPSILON )
    {
        nAngle = alpha + ANG_SPEED*dt; // next angle with ANG_SPEED

        if ( nAngle > m_length + EPSILON )
        {
            correctCoeff = 1. - (nAngle-m_length)/m_length; // correcting if going too far on next time step
            end = true;
        }

        angSpeed = ANG_SPEED * correctCoeff;
        deltaWheelSpeed = ANG_SPEED * RobotGeometry::WHEEL_BASE; // computing wheels speed

        speed_regulation(cvs, 0.5*m_sign*deltaWheelSpeed, -0.5*m_sign*deltaWheelSpeed);
    }
    else
        end = true;
    return end;
}

CurvePath::CurvePath(double angle, bool sign) : Path(angle), m_sign(sign)
{
}

NAMESPACE_CLOSE();
