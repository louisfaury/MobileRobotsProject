#include "opp_pos_gr4.h"
#include "init_pos_gr4.h"
#include "useful_gr4.h"
#include "config_file_gr4.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr4);

/*! \brief compute the opponents position using the tower
 * 
 * \param[in,out] cvs controller main structure
 */
void opponents_tower(CtrlStruct *cvs)
{
	// variables declaration
	int nb_opp;
	int rise_index_1, rise_index_2, fall_index_1, fall_index_2;

	double delta_t;
	double rise_1, rise_2, fall_1, fall_2;

	CtrlIn *inputs;
	RobotPosition *rob_pos;
	OpponentsPosition *opp_pos;

	// variables initialization
	inputs  = cvs->inputs;
	rob_pos = cvs->rob_pos;
	opp_pos = cvs->opp_pos;

	nb_opp = opp_pos->nb_opp;

	// no opponent
	if (!nb_opp)
	{
		return;
	}

	// safety
	if (nb_opp < 0 || nb_opp > 2)
	{
		printf("Error: number of opponents cannot be %d!\n", nb_opp);
		exit(EXIT_FAILURE);
	}

	// low pass filter time increment ('delta_t' is the last argument of the 'first_order_filter' function)
	delta_t = inputs->t - opp_pos->last_t;
	opp_pos->last_t = inputs->t;

	// indexes
	rise_index_1 = inputs->rising_index;
	fall_index_1 = inputs->falling_index;

	// rise and fall angles of the first opponent
	rise_1 = inputs->last_rising[rise_index_1];
	fall_1 = inputs->last_falling[fall_index_1];

	// rise and fall angles of the second opponent
	if (nb_opp == 2)
	{
		rise_index_2 = (rise_index_1-1 < 0) ? NB_STORE_EDGE-1 : rise_index_1-1;
		fall_index_2 = (fall_index_1-1 < 0) ? NB_STORE_EDGE-1 : fall_index_1-1;

		rise_2 = inputs->last_rising[rise_index_2];
		fall_2 = inputs->last_falling[fall_index_2];

        if (fall_2>fall_1)
        { //dealing with index shifting
            double fall_2_m  = fall_2;
            double rise_2_m = rise_2;

            rise_2 = rise_1;
            fall_2 = fall_1;

            rise_1 = rise_2_m;
            fall_1 = fall_2_m;
        }
	}

	// ----- opponents position computation start ----- //

    // keeping old values in memory for low pass filter
    double oldX = opp_pos->x[0];
    double oldY = opp_pos->y[0];

    // performing opponent tower detection and positioning
    if ( single_opp_tower(rise_1, fall_1, rob_pos->x, rob_pos->y, rob_pos->theta, &(opp_pos->x[0]), &(opp_pos->y[0])) )
    {
        // low-pass filter for opponent position
        opp_pos->x[0] = first_order_filter(oldX, *opp_pos->x, 10*delta_t, delta_t, UINT64_MAX);
        opp_pos->y[0] = first_order_filter(oldY, *opp_pos->y, 10*delta_t, delta_t, UINT64_MAX);
    }


    //set_plot(opp_pos->x[0], "Rx1 ");
    //set_plot(opp_pos->y[0], "Ry1 ");
    //set_plot(0.67, "Ix1");
    //set_plot(0., "Iy1");


    if (nb_opp == 2)
    {
        oldX = opp_pos->x[1];
        oldY = opp_pos->y[1];

        if ( single_opp_tower(rise_2, fall_2, rob_pos->x, rob_pos->y, rob_pos->theta, &(opp_pos->x[1]), &(opp_pos->y[1])) )
        {
            opp_pos->x[1] = first_order_filter(oldX, opp_pos->x[1], 100*delta_t, delta_t, UINT64_MAX);
            opp_pos->y[1] = first_order_filter(oldY, opp_pos->y[1], 100*delta_t, delta_t, UINT64_MAX);
        }
       /*
        set_plot(opp_pos->x[1], "Rx2 ");
        set_plot(opp_pos->y[1], "Ry2 ");
        set_plot(0., "Ix2");
        set_plot(0.8, "Iy2");
        */
    }

    check_opp_front(cvs);

	// ----- opponents position computation end ----- //
}

/*! \brief compute a single opponent position
 * 
 * \param[in] last_rise last rise relative angle [rad]
 * \param[in] last_fall last fall relative angle [rad]
 * \param[in] rob_x robot x position [m]
 * \param[in] rob_y robot y position [m]
 * \param[in] rob_theta robot orientation [rad]
 * \param[out] new_x_opp new known x opponent position
 * \param[out] new_y_opp new known y opponent position
 * \return 1 if computation successful, 0 otherwise
 */
int single_opp_tower(double last_rise, double last_fall, double rob_x, double rob_y, double rob_theta, double *new_x_opp, double *new_y_opp)
{
    double res = 0;
    if ( last_fall > last_rise )
    {
        double d = RobotGeometry::BEACON_RADIUS * ( 1. / tan(0.5*(last_fall - last_rise)) );
        double theta = 0.5 * (last_fall + last_rise);

        //In the frame attached to the base of the tower (TOWER_X,TOWER_Y,TOWER_THETA)
        double x = d * cos(theta);
        double y = d * sin(theta);

        //Moving to the frame attached to the center of the robot
        //RobotGeometry::moveToRef(RobotGeometry::TOWER_X, RobotGeometry::TOWER_Y, RobotGeometry::TOWER_THETA, x, y);
        y += RobotGeometry::TOWER_X;

        //Moving to the main frame
        RobotGeometry::moveToRef(rob_x, rob_y, rob_theta, x, y);


        //Copying
        *new_x_opp = x;
        *new_y_opp = y;

        res = 1;

    }
    return res;
}

/*! \brief check if there is an opponent in front of the robot
 * 
 * \param[in] cvs controller main structure
 * \return 1 if opponent robot in front of the current robot
 */
int check_opp_front(CtrlStruct *cvs)
{
	// variables declaration
	int i, nb_opp;
    int res = 0;

    double distToOppSquare(0); // no sqrt for reducing computation time
    double angToOpp(0);
    bool inFront(false);
    bool tooClose(false);

	OpponentsPosition *opp_pos;
	RobotPosition *rob_pos;

	// variables initialization
	rob_pos = cvs->rob_pos;
	opp_pos = cvs->opp_pos;
	nb_opp = opp_pos->nb_opp;

	// safety
	if (nb_opp < 0 || nb_opp > 2)
	{
		printf("Error: number of opponents cannot be %d!\n", nb_opp);
		exit(EXIT_FAILURE);
	}

    if (nb_opp > 0)
    {
        for(i=0; i<nb_opp; i++)
        {
            distToOppSquare = (opp_pos->x[i]-rob_pos->x)*(opp_pos->x[i]-rob_pos->x) + (opp_pos->y[i]-rob_pos->y)*(opp_pos->y[i]-rob_pos->y);
            angToOpp        = atan2(opp_pos->y[i]-rob_pos->y, opp_pos->x[i]-rob_pos->x);

            inFront  = ( fabs( angToOpp - rob_pos->theta) < ConstraintConstant::ANG_FRONT_WIDTH );
            tooClose = ( distToOppSquare < ConstraintConstant::MIN_OPP_DIST*ConstraintConstant::MIN_OPP_DIST );

            if ( inFront && tooClose )
            {
                res = 1;
                break;
            }

        }
    }
    return res;
}

NAMESPACE_CLOSE();
