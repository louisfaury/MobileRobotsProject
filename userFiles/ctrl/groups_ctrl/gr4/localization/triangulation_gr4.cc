#include "triangulation_gr4.h"
#include "useful_gr4.h"
#include "init_pos_gr4.h"
#include <math.h>
#include "config_file.h"
#include "config_file_gr4.h"
#include "kalman_gr4.h"

NAMESPACE_INIT(ctrlGr4);

/*! \brief set the fixed beacons positions, depending on the team
 * 
 * \param[in] team_id ID of the team ('TEAM_A' or 'TEAM_B')
 * \param[out] x_beac_1 first beacon x position [m]
 * \param[out] y_beac_1 first beacon y position [m]
 * \param[out] x_beac_2 second beacon x position [m]
 * \param[out] y_beac_2 second beacon y position [m]
 * \param[out] x_beac_3 third beacon x position [m]
 * \param[out] y_beac_3 third beacon y position [m]
 *
 * This function can be adapted, depending on the map.
 */
void fixed_beacon_positions(int team_id, double *x_beac_1, double *y_beac_1,
	double *x_beac_2, double *y_beac_2, double *x_beac_3, double *y_beac_3)
{
	switch (team_id)
	{
        case TEAM_A:
            *x_beac_1 = TEAM_A_BEACON_1_X;
            *y_beac_1 = TEAM_A_BEACON_1_Y;

            *x_beac_2 = TEAM_A_BEACON_2_X;
            *y_beac_2 = TEAM_A_BEACON_2_Y;

            *x_beac_3 = TEAM_A_BEACON_3_X;
            *y_beac_3 = TEAM_A_BEACON_3_Y;
			break;

        case TEAM_B:
            *x_beac_1 = TEAM_B_BEACON_1_X;
            *y_beac_1 = TEAM_B_BEACON_1_Y;

            *x_beac_2 = TEAM_B_BEACON_2_X;
            *y_beac_2 = TEAM_B_BEACON_2_Y;

            *x_beac_3 = TEAM_B_BEACON_3_X;
            *y_beac_3 = TEAM_B_BEACON_3_Y;
			break;
	
		default:
			printf("Error unknown team ID (%d) !\n", team_id);
			exit(EXIT_FAILURE);
	}
}

/*! \brief get the index of the best angle prediction
 * 
 * \param[in] alpha_predicted angle to reach [rad]
 * \param[in] alpha_a angle computed for A [rad]
 * \param[in] alpha_b angle computed for B [rad]
 * \param[in] alpha_c angle computed for C [rad]
 * \return best index (0, 1, or 2)
 */
int index_predicted(double alpha_predicted, double alpha_a, double alpha_b, double alpha_c)
{
	double pred_err_a, pred_err_b, pred_err_c;

	pred_err_a = fabs(limit_angle(alpha_a - alpha_predicted));
	pred_err_b = fabs(limit_angle(alpha_b - alpha_predicted));
	pred_err_c = fabs(limit_angle(alpha_c - alpha_predicted));
	return (pred_err_a < pred_err_b) ? ((pred_err_a < pred_err_c) ? 0 : 2) : ((pred_err_b < pred_err_c) ? 1 : 2);
}

/*! \brief triangulation main algorithm
 * 
 * \param[in] cvs controller main structure
 *
 * computation found here: http://www.telecom.ulg.ac.be/triangulation/
 */
void triangulation(CtrlStruct *cvs)
{
	// variables declaration
	RobotPosition *pos_tri, *rob_pos;
	CtrlIn *inputs;

	int alpha_1_index, alpha_2_index, alpha_3_index;
	int rise_index_1, rise_index_2, rise_index_3;
	int fall_index_1, fall_index_2, fall_index_3;

	double alpha_a, alpha_b, alpha_c;
	double alpha_1, alpha_2, alpha_3;
	double alpha_1_predicted, alpha_2_predicted, alpha_3_predicted;
    double x_beac_1, y_beac_1, x_beac_2, y_beac_2, x_beac_3, y_beac_3;
    double xRes, yRes, thetaRes;
    double posPeakThreshold;
    double angPeakThreshold;
    double tau;
    double dt;

    static bool init(false); //used to be able to fix an +infty threshold when filtering first position

	// variables initialization
    pos_tri = cvs->triang_pos;
	rob_pos = cvs->rob_pos;
	inputs  = cvs->inputs;

    // for low pass filter increment
    dt = inputs->t - pos_tri->last_t;
    pos_tri->last_t = inputs->t;


	// safety
	if ((inputs->rising_index_fixed < 0) || (inputs->falling_index_fixed < 0))
	{
		return;
	}

	// known positions of the beacons
	fixed_beacon_positions(cvs->team_id, &x_beac_1, &y_beac_1, &x_beac_2, &y_beac_2, &x_beac_3, &y_beac_3);	

	// indexes fot the angles detection
	rise_index_1 = inputs->rising_index_fixed;
	rise_index_2 = (rise_index_1 - 1 < 0) ? NB_STORE_EDGE-1 : rise_index_1 - 1;
	rise_index_3 = (rise_index_2 - 1 < 0) ? NB_STORE_EDGE-1 : rise_index_2 - 1;

	fall_index_1 = inputs->falling_index_fixed;
	fall_index_2 = (fall_index_1 - 1 < 0) ? NB_STORE_EDGE-1 : fall_index_1 - 1;
	fall_index_3 = (fall_index_2 - 1 < 0) ? NB_STORE_EDGE-1 : fall_index_2 - 1;

	// beacons angles measured with the laser (to compute)
    alpha_a = 0.5 * (inputs->last_falling_fixed[fall_index_1] + inputs->last_rising_fixed[rise_index_1]);
    alpha_b = 0.5 * (inputs->last_falling_fixed[fall_index_2] + inputs->last_rising_fixed[rise_index_2]);
    alpha_c = 0.5 * (inputs->last_falling_fixed[fall_index_3] + inputs->last_rising_fixed[rise_index_3]);

	// beacons angles predicted thanks to odometry measurements (to compute)
    alpha_1_predicted = atan2(y_beac_1-rob_pos->y,x_beac_1-rob_pos->x) - rob_pos->theta;
    alpha_2_predicted = atan2(y_beac_2-rob_pos->y,x_beac_2-rob_pos->x) - rob_pos->theta;
    alpha_3_predicted = atan2(y_beac_3-rob_pos->y,x_beac_3-rob_pos->x) - rob_pos->theta;

    // indexes of each beacon
	alpha_1_index = index_predicted(alpha_1_predicted, alpha_a, alpha_b, alpha_c);
	alpha_2_index = index_predicted(alpha_2_predicted, alpha_a, alpha_b, alpha_c);
	alpha_3_index = index_predicted(alpha_3_predicted, alpha_a, alpha_b, alpha_c);

	// safety
	if ((alpha_1_index == alpha_2_index) || (alpha_1_index == alpha_3_index) || (alpha_2_index == alpha_3_index))
	{
		return;
	}

	// angle of the first beacon
	switch (alpha_1_index)
	{
		case 0: alpha_1 = alpha_a; break;
		case 1: alpha_1 = alpha_b; break;
		case 2: alpha_1 = alpha_c; break;
	
		default:
			printf("Error: unknown index %d !\n", alpha_1_index);
			exit(EXIT_FAILURE);
	}

	// angle of the second beacon
	switch (alpha_2_index)
	{
		case 0: alpha_2 = alpha_a; break;
		case 1: alpha_2 = alpha_b; break;
		case 2: alpha_2 = alpha_c; break;
	
		default:
			printf("Error: unknown index %d !\n", alpha_2_index);
			exit(EXIT_FAILURE);
	}

	// angle of the third beacon
	switch (alpha_3_index)
	{
		case 0: alpha_3 = alpha_a; break;
		case 1: alpha_3 = alpha_b; break;
		case 2: alpha_3 = alpha_c; break;
	
		default:
			printf("Error: unknown index %d !\n", alpha_3_index);
			exit(EXIT_FAILURE);
	}
	

	// ----- triangulation computation start ----- //

    double xP1 = x_beac_1 - x_beac_2;
    double yP1 = y_beac_1 - y_beac_2;
    double xP3 = x_beac_3 - x_beac_2;
    double yP3 = y_beac_3 - y_beac_2;

    double T12 = 1/tan(alpha_2 - alpha_1);
    double T23 = 1/tan(alpha_3 - alpha_2);
    double T31 = (1-T12*T23) / (T12 + T23);

    double xP12 = xP1 + T12*yP1;
    double yP12 = yP1 - T12*xP1;
    double xP23 = xP3 - T23*yP3;
    double yP23 = yP3 + T23*xP3;
    double xP31 = xP3 + xP1 + T31*(yP3 - yP1);
    double yP31 = yP3 + yP1 - T31*(xP3 - xP1);

    double kP31 = xP1 * xP3 + yP1 * yP3 + T31*(xP1*yP3 - xP3*yP1);
    double D    = (xP12 - xP23)*(yP23 - yP31) - (yP12 - yP23)*(xP23 - xP31);

    if ( true || fabs(D)  > EPSILON )  //dealing with floating point value imprecision
    {
        xRes = x_beac_2 + (kP31/D) * (yP12 - yP23);
        yRes = y_beac_2 + (kP31/D) * (xP23 - xP12);
        thetaRes = (1./3)*( -alpha_1_predicted + atan2(y_beac_1-rob_pos->y,x_beac_1-rob_pos->x) )
                   + (1./3) * ( -alpha_2_predicted + atan2(y_beac_2-rob_pos->y,x_beac_2-rob_pos->x) )
                   + (1./3) * ( -alpha_3_predicted + atan2(y_beac_3-rob_pos->y,x_beac_3-rob_pos->x) );
      // RobotGeometry::moveToRef(RobotGeometry::TOWER_X, RobotGeometry::TOWER_X, RobotGeometry::TOWER_THETA, xRes, yRes ); //Robot Frame
        xRes -= RobotGeometry::TOWER_X*cos(thetaRes);
        yRes -= RobotGeometry::TOWER_X*sin(thetaRes);

        // some computation to enable x & y to jump at initialization
        // no low pass filter (kalman does that for us)
        pos_tri->x = xRes;
        pos_tri->y = yRes;
        pos_tri->theta = thetaRes;

        //set_plot(pos_tri->x, "TrianX");
        //set_plot(pos_tri->y, "TrianY");
        //set_plot(pos_tri->theta, "TrianTheta");

        // kalman flag rise if there was no absurd peak (discrete way of dealing with it : TODO complete in Kalman with Mahalanobis distance)
        if (norm_dist(pos_tri->x - rob_pos->x, pos_tri->y - rob_pos->y) < 0.4*ConstraintConstant::POS_UPDATE_THRESHOLD)
        {
            cvs->kalman->triang_flag = true;
        }

        if (!init && norm_dist(pos_tri->x - rob_pos->x, pos_tri->y - rob_pos->y) < 5*ConstraintConstant::POS_UPDATE_THRESHOLD)
            init = true; // init set to true only when pos_tri and rob_pos are coherent

    }
    else
        return;


	// ----- triangulation computation end ----- //
}

NAMESPACE_CLOSE();
