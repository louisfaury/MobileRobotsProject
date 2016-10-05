#include "calibration_gr4.h"
#include "speed_regulation_gr4.h"
#include "odometry_gr4.h"
#include "useful_gr4.h"
#include "init_pos_gr4.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr4);

#define DEG_TO_RAD (M_PI/180.0) ///< convertion from degrees to radians

// calibration states
enum {CALIB_START, CALIB_STATE_A, CALIB_STATE_B, CALIB_STATE_C, CALIB_ERROR_STATE, CALIB_FINISH, CALIB_STATE_D, CALIB_STATE_E};

/*! \brief calibration of the robot to calibrate its position
 * 
 * \param[in,out] cvs controller main structure
 * 
 * This FSM can be adapted, depending on the map and on the robots initial position.
 */
void calibration(CtrlStruct *cvs)
{
	// variables declaration
	int team_id;
	double t;

	CtrlIn *inputs;
	RobotCalibration *calib;
	RobotPosition *rob_pos;

	// variables initialization
	inputs = cvs->inputs;
	calib  = cvs->calib;

	rob_pos = cvs->rob_pos;
	
	t = inputs->t;
	team_id = cvs->team_id;

	// finite state machine (FSM)
    switch (calib->flag)
    {
    case CALIB_START: // start calibration
        set_init_position(cvs->robot_id, cvs->rob_pos);
        speed_regulation(cvs, 0.0, 0.0);

        calib->flag = CALIB_STATE_A; // directly go to state A
        calib->t_flag = t;
        break;

    case CALIB_STATE_A: //going backward until both switch are activated
    {
        speed_regulation(cvs, -PI, -PI);

        bool r_activatedSwitch =  (cvs->inputs->u_switch[R_ID] > 0);
        bool l_activatedSwitch =  (cvs->inputs->u_switch[L_ID] > 0);

        // go to state B if both switch are actived
        if ( r_activatedSwitch && l_activatedSwitch )
        {
            calib->flag = CALIB_STATE_B;

            calib->t_flag = t;
        }
        //if we've been going backwards for too long we declare error mode
        else if ( (t - calib->t_flag) > 5.0 )
        {
            calib->flag = CALIB_ERROR_STATE;
        }
        break;
    }

    case CALIB_STATE_B: // keep on going backward for a little while
        speed_regulation(cvs, -PI/2, -PI/2);

        // go to state C after 2 seconds
        if (t - calib->t_flag > 2.0)
        {
            calib->flag = CALIB_STATE_C;

            calib->t_flag = t;
        }
        break;

    case CALIB_STATE_C: // perfectly aligned with the back wall, let's go forward for a bit
        speed_regulation(cvs, PI/2, PI/2);

        // go to D state after 4 seconds
        if (t - calib->t_flag > 4.0)
        {
            calib->flag = CALIB_STATE_D;

            calib->t_flag = t;
        }
        break;

    case CALIB_STATE_D: // rotating of PI/2 angle
    {
        //computing turning time
        double tEnd = 0.5*RobotGeometry::WHEEL_BASE/RobotGeometry::WHEEL_RADIUS;
        speed_regulation(cvs, -PI/2, PI/2);

        // go to final state after 2 seconds
        if (t - calib->t_flag > tEnd)
        {
            speed_regulation(cvs, 0., 0.);
            calib->flag = CALIB_STATE_E;
            calib->t_flag = t;
        }
        break;
    }
    case CALIB_STATE_E: // going backward until both switch are activated
    {
        speed_regulation(cvs, -PI, -PI);

        bool r_activatedSwitch =  (cvs->inputs->u_switch[R_ID] > 0);
        bool l_activatedSwitch =  (cvs->inputs->u_switch[L_ID] > 0);

        // go to state B after 5 seconds
        if ( r_activatedSwitch && l_activatedSwitch )
        {
            calib->flag = CALIB_FINISH;

            calib->t_flag = t;
        }
        //if we've been going backwards for too long we declare error mode
        else if ( (t - calib->t_flag) > 5.0 )
        {
            calib->flag = CALIB_ERROR_STATE;
        }
        break;
    }
    case CALIB_ERROR_STATE :
        //TODO
        calib->flag = CALIB_FINISH;
        break;
    case CALIB_FINISH: // wait before the match is starting
        speed_regulation(cvs, 0.0, 0.0);
        break;

    default:
        printf("Error: unknown state : %d !\n", calib->flag);
        exit(EXIT_FAILURE);
    }
}

NAMESPACE_CLOSE();
