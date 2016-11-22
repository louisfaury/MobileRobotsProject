/*!
 *  @file : config_file_gr4.h
 *  @brief : register macros and robots geometry variables
 */

#ifndef CONFIG_FILE_GR4_H
#define CONFIG_FILE_GR4_H

#include "namespace_ctrl.h"

NAMESPACE_INIT(ctrlGr4);

// PI
#define PI 3.14159265

// macros
#define RAD2DEG(X) (180*X/PI)
#define DEG2RAD(X) (PI*X/180)

#define EPSILON  0.000001  //dealing with float imprecision

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
    static constexpr double KS             = 0.05;      //5% linear slip
    static constexpr double KTHETA         = 0.04;      //4% angular slip
    static constexpr double OBS_VAR_X      = 0.0001;    // (1cm)² cov. for direct input triangulation
    static constexpr double OBS_VAR_Y      = 0.0001;    // (1cm)² cov. for direct input triangulation
    static constexpr double OBS_VAR_THETA  = DEG2RAD(0.1)*DEG2RAD(0.1); // (0.1°)²  cov for direct input triangulation

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
        double xTemp(x);
        double yTemp(y);
        x = xRef + xTemp*cos(thetaRef) - yTemp*sin(thetaRef);
        y = yRef + xTemp*sin(thetaRef) + yTemp*cos(thetaRef);
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

