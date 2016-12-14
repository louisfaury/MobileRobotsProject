/*!
 * \author Group 4
 * \file kalman_gr4.h
 * \brief localization sensors fusion with extended Kalman filter
 *        - prediction using odometry model
 *        - inovation using triangulation (identity mapping function)
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
    /*!
     * \brief initEst : initialize estimation mean's and variance at the beginning of the run and when innovation matrix is not in Sn++
     *                  (can happen due to calculus noise)
     * \param csv : ptr to main ctrl struct
     */
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
    int iter; // useful when need to slow down innovation frequency

    // odometry measurement structure
    struct OdometryMeasurementStruct
    {
        double dS;
        double dTheta;
        bool odoFlag; // raised when new data is available
    } odo_meas;

};

/*!
 * \brief kalman : runs the ekf routine
 * \param cvs : ptr to main ctrl struct
 */
void kalman(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
