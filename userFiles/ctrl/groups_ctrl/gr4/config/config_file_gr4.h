/*!
 *  @file : config_file_gr4.h
 *  @brief : register macros and robots geometry variables
 */

#ifndef CONFIG_FILE_GR4_H
#define CONFIG_FILE_GR4_H

#include "namespace_ctrl.h"
#include "useful_gr4.h"

NAMESPACE_INIT(ctrlGr4);

//geometry
namespace RobotGeometry
{
    static const double WHEEL_BASE     = 0.225;     //m
    static const double WHEEL_RADIUS   = 0.03;      //m
    static const double BEACON_RADIUS  = 0.04;      //m
    static const double TOWER_X        = 0.083;     //m
    static const double TOWER_Y        = 0.;        //m
    static const double TOWER_THETA    = 0.;        //rad
    static const double BACK_TO_CENTER = 0.06;      //m
    static const double ENC_RES        = 0.0001;    //encoder resolution, rad
    static const double KS             = 0.07;      //7% linear slip
    static const double KTHETA         = 0.08;      //8% angular slip
    static const double OBS_VAR_X      = 0.00002;   // cov. for direct input triangulation
    static const double OBS_VAR_Y      = 0.00002;   // cov. for direct input triangulation
    static const double OBS_VAR_THETA  = DEG2RAD(0.2)*DEG2RAD(0.2); // (0.2°)²  cov for direct input triangulation
    static const double CHI2_3D_01     = 9.348;     // P(chi2_3D >= 9.348) = 0.025 : decision threshold for Kalman (very low but needed for convergence !) -> removes only unlikely obs.
    /*
     * \brief moveToRef : Moves the point p(x,y) expressed in pRef frame to the frame (pRef,theraRef) where pRef = (xRef, yRef)
     * \param xRef
     * \param yRef
     * \param thetaRef
     * \param x
     * \param y
     * \param theta
     */
    static void moveToRef(double xRef, double yRef, double thetaRef, double& x, double& y)
    {
        double xTmp(x), yTmp(y);
        x = xRef + cos(thetaRef)*xTmp - sin(thetaRef)*yTmp;
        y = yRef + cos(thetaRef)*yTmp + sin(thetaRef)*xTmp;
    }
}

namespace ConstraintConstant
{
    static const double MIN_OPP_DIST    = 0.4;
    static const double ANG_FRONT_WIDTH = DEG2RAD(20);
    static const double POS_UPDATE_THRESHOLD = 0.5; //no jumps > 50cm
    static const double ANG_UPDATE_THRESHOLD = DEG2RAD(20); //no jums >20deg
}
NAMESPACE_CLOSE();


#endif // CONFIG_FILE_GR4_H

