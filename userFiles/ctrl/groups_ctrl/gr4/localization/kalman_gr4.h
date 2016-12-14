/*!
 * \author Group 4
 * \file kalman_gr4.h
 * \brief localization sensors fusion with Kalman
 */

#ifndef _KALMAN_GR4_H_
#define _KALMAN_GR4_H_

#include "CtrlStruct_gr4.h"
#include "init_pos_gr4.h"
#include "useful_gr4.h"

NAMESPACE_INIT(ctrlGr4);

/// Kalman main structure
struct KalmanStruct
{
    KalmanStruct();
    //functions
    void initEst(CtrlStruct *csv);

    // robot pose estimation
    double xEst;
    double yEst;
    double thetaEst;

    // state error's covariance matrix
    covMatrix3D pEst;

    // bool for init and triang. data flag
    bool init;
    bool triang_flag;
    int iter;
    // odometry measurement
    struct OdometryMeasurementStruct
    {
        double dS;
        double dTheta;
        bool odoFlag; // raised when new data is available
    } odo_meas;

};


void kalman(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
