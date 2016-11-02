#include "kalman_gr4.h"
#include "odometry_gr4.h"
#include "triangulation_gr4.h"
#include "useful_gr4.h"
#include "config_file.h"
#include "config_file_gr4.h"

NAMESPACE_INIT(ctrlGr4);

/*! \brief follow a given path
 * 
 * \param[in,out] cvs controller main structure
 */
void kalman(CtrlStruct *cvs)
{
	// variable declaration
    RobotPosition *rob_pos;
    RobotPosition *triang_pos;
    KalmanStruct *kalman_pos;

    // variables initialization
    rob_pos = cvs->rob_pos;
    triang_pos = cvs->triang_pos;
    kalman_pos = cvs->kalman;

    // init loop
    printf("%d\n",kalman_pos->init);
    if (!kalman_pos->init)
    {
        kalman_pos->initEst(cvs);
        kalman_pos->init = true;
    }

    // prediction step if odometry is available
    if (kalman_pos->odo_meas.odoFlag)
    {
        // buffers
        double pxx = kalman_pos->pEst.xx;
        double pxy = kalman_pos->pEst.xy;
        double pxtheta = kalman_pos->pEst.xtheta;
        double pyy = kalman_pos->pEst.yy;
        double pytheta = kalman_pos->pEst.ytheta;
        double pthetatheta = kalman_pos->pEst.thetatheta;

        // for readability
        double deltaS = kalman_pos->odo_meas.dS;
        double deltaTheta = kalman_pos->odo_meas.dTheta;
        double theta = rob_pos->theta;

        // mean update
        kalman_pos->xEst += kalman_pos->odo_meas.dS * cos(theta + 0.5*kalman_pos->odo_meas.dTheta);
        kalman_pos->yEst += kalman_pos->odo_meas.dS * sin(theta + 0.5*kalman_pos->odo_meas.dTheta);
        kalman_pos->thetaEst += kalman_pos->odo_meas.dTheta;

        // updating cov matrix expression taken from wxmaxima file
        kalman_pos->pEst.xx = (pow(deltaS,2)*pow(sin(deltaTheta/2+theta),2)*pow(RobotGeometry::ENC_RES,2)*pow(RobotGeometry::WHEEL_RADIUS,2)*pow(RobotGeometry::WHEEL_BASE,-2))/24+(pow(cos(deltaTheta/2+theta),2)*pow(RobotGeometry::ENC_RES,2)*pow(RobotGeometry::WHEEL_RADIUS,2))/24+2*RobotGeometry::KS*pxx*abs(cos(theta))*abs(deltaS)+pxx;
        kalman_pos->pEst.xy = -(pow(deltaS,2)*cos(deltaTheta/2+theta)*sin(deltaTheta/2+theta)*pow(RobotGeometry::ENC_RES,2)*pow(RobotGeometry::WHEEL_RADIUS,2)*pow(RobotGeometry::WHEEL_BASE,-2))/24+(cos(deltaTheta/2+theta)*sin(deltaTheta/2+theta)*pow(RobotGeometry::ENC_RES,2)*pow(RobotGeometry::WHEEL_RADIUS,2))/24+pxy;
        kalman_pos->pEst.xtheta = (deltaS*sin(deltaTheta/2+theta)*pow(RobotGeometry::ENC_RES,2)*pow(RobotGeometry::WHEEL_RADIUS,2)*pow(RobotGeometry::WHEEL_BASE,-2))/12+pxtheta;
        kalman_pos->pEst.yy = (pow(deltaS,2)*pow(cos(deltaTheta/2+theta),2)*pow(RobotGeometry::ENC_RES,2)*pow(RobotGeometry::WHEEL_RADIUS,2)*pow(RobotGeometry::WHEEL_BASE,-2))/24+(pow(sin(deltaTheta/2+theta),2)*pow(RobotGeometry::ENC_RES,2)*pow(RobotGeometry::WHEEL_RADIUS,2))/24+2*RobotGeometry::KS*pyy*abs(sin(theta))*abs(deltaS)+pyy;
        kalman_pos->pEst.ytheta = pytheta-(deltaS*cos(deltaTheta/2+theta)*pow(RobotGeometry::ENC_RES,2)*pow(RobotGeometry::WHEEL_RADIUS,2)*pow(RobotGeometry::WHEEL_BASE,-2))/12;
        kalman_pos->pEst.thetatheta = (pow(RobotGeometry::ENC_RES,2)*pow(RobotGeometry::WHEEL_RADIUS,2)*pow(RobotGeometry::WHEEL_BASE,-2))/6+2*RobotGeometry::KTHETA*pthetatheta*abs(deltaTheta)+pthetatheta;

        kalman_pos->odo_meas.odoFlag = false;

        //plots
        set_plot(kalman_pos->xEst, "Kalman x [m]");
        set_plot(kalman_pos->yEst, "Kalman y [m]");
    }

    // update step using triangularisation
    if (kalman_pos->triang_flag = true)
    {
        // buffers
        double pxx = kalman_pos->pEst.xx;
        double pxy = kalman_pos->pEst.xy;
        double pxtheta = kalman_pos->pEst.xtheta;
        double pyy = kalman_pos->pEst.yy;
        double pytheta = kalman_pos->pEst.ytheta;
        double pthetatheta = kalman_pos->pEst.thetatheta;

        // operations
        kalman_pos->triang_flag = false;


    }

}

KalmanStruct::KalmanStruct() : init(false), xEst(0), yEst(0), thetaEst(0), triang_flag(false)
{
}

void KalmanStruct::initEst(CtrlStruct *cvs)
{
    xEst = cvs->rob_pos->x;
    yEst = cvs->rob_pos->y;
    thetaEst = cvs->rob_pos->theta;

    // diagonal initial cov matrix
    pEst.xx = T1_UNCERT*T1_UNCERT;
    pEst.yy = T2_UNCERT*T2_UNCERT;
    pEst.thetatheta = R3_UNCERT*R3_UNCERT;
}

NAMESPACE_CLOSE();
