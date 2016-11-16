#include "CurvePath.h"
#include "speed_regulation_gr4.h"
#include "config_file_gr4.h"

NAMESPACE_INIT(ctrlGr4);

CurvePath::CurvePath()
{
}

bool CurvePath::nextStep(double alpha, double dt, CtrlStruct *cvs)
{
    bool end = false;

    double nAngle;
    double correctCoeff(1.);
    double angSpeed, deltaWheelSpeed;

    if ( m_angle > EPSILON )
    {
        nAngle = alpha + ANG_SPEED*dt;

        if ( nAngle > m_angle + EPSILON )
        {
            correctCoeff = 1. - (nAngle-m_angle)/m_angle;
            end = true;
        }

        angSpeed = ANG_SPEED * correctCoeff;
        deltaWheelSpeed = ANG_SPEED * RobotGeometry::WHEEL_BASE;

        speed_regulation(cvs, 0.5*m_sign*deltaWheelSpeed, -0.5*m_sign*deltaWheelSpeed);
    }
    else
        end = true;
    return end;
}

CurvePath::CurvePath(double angle, bool sign) : m_angle(angle), m_sign(sign)
{
}

NAMESPACE_CLOSE();
