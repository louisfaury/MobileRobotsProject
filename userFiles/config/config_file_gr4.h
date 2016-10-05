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

//geometry
namespace RobotGeometry
{
    static constexpr double WHEEL_BASE = 0.1125; //m
    static constexpr double WHEEL_RADIUS = 0.03;//m
}


NAMESPACE_CLOSE();


#endif // CONFIG_FILE_GR4_H

