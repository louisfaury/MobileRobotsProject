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
    static constexpr double WHEEL_BASE     = 0.225;     //m
    static constexpr double WHEEL_RADIUS   = 0.03;      //m
    static constexpr double BEACON_RADIUS  = 0.04;      //m
    static constexpr double TOWER_X        = 0.083;     //m
    static constexpr double TOWER_Y        = 0.;        //m
    static constexpr double TOWER_THETA    = 0.;        //rad
    static constexpr double BACK_TO_CENTER = 0.06;      //m
    static constexpr double ENC_RES        = 0.00001;   //encoder resolution, rad
    static constexpr double KS             = 0.06;      //6% linear slip
    static constexpr double KTHETA         = 0.05;      //5% angular slip
    static constexpr double OBS_VAR_X      = 0.0002;    // (1cm)² cov. for direct input triangulation
    static constexpr double OBS_VAR_Y      = 0.0002;    // (1cm)² cov. for direct input triangulation
    static constexpr double OBS_VAR_THETA  = DEG2RAD(2)*DEG2RAD(2); // (1°)²  cov for direct input triangulation

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
        xRef-= RobotGeometry::TOWER_X*cos(thetaRef);
        yRef-= RobotGeometry::TOWER_X*sin(thetaRef);
    }
}

namespace ConstraintConstant
{
    static constexpr double MIN_OPP_DIST    = 0.4;
    static constexpr double ANG_FRONT_WIDTH = DEG2RAD(20);
    static constexpr double POS_UPDATE_THRESHOLD = 0.5; //no jumps > 50cm
    static constexpr double ANG_UPDATE_THRESHOLD = DEG2RAD(20); //no jums >20deg
}
NAMESPACE_CLOSE();


#endif // CONFIG_FILE_GR4_H

