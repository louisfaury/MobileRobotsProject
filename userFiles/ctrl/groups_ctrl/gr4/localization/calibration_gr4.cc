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
        set_triang_init_position(cvs->robot_id, cvs->triang_pos);
        speed_regulation(cvs, 0.0, 0.0);

        calib->flag = CALIB_STATE_A; // directly go to state A
        calib->t_flag = t;
        break;

    case CALIB_STATE_A: //going backward until both switch are activated
    {
        speed_regulation(cvs, -2*PI, -2*PI);

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
        // Before, we calibrate the odometry module
        if (t - calib->t_flag > 1.0)
        {
            //Stopping to be sure that we won't slide anymore
            speed_regulation(cvs, 0, 0);


            switch(team_id)
            {
            case TEAM_A:
                //Telling odometry that we are sure that y = 1.5 and theta = -90
                rob_pos->y=1.5-RobotGeometry::BACK_TO_CENTER;
                rob_pos->theta=-PI/2;
                break;
            case TEAM_B:
                //Telling odometry that we are sure that y = -1.5 and theta = 90
                rob_pos->y=-1.5+RobotGeometry::BACK_TO_CENTER;
                rob_pos->theta=PI/2;
                break;
            default:
                printf("Error: unknown team, calibration impossible : %d !\n", calib->flag);
                exit(EXIT_FAILURE);

            }

            calib->flag = CALIB_STATE_C;
            calib->t_flag = t;


        }
        break;

    case CALIB_STATE_C: // perfectly aligned with the back wall, let's go forward for a bit
        speed_regulation(cvs, PI, PI);

        // go to D state after 4 seconds
        if (t - calib->t_flag > 2.0)
        {
            calib->flag = CALIB_STATE_D;

            calib->t_flag = t;
        }
        break;

    case CALIB_STATE_D: // rotating of PI/2 angle
    {
        //computing turning time
        double tEnd = 0.25*RobotGeometry::WHEEL_BASE/RobotGeometry::WHEEL_RADIUS;
        speed_regulation(cvs, -PI, PI);

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
        speed_regulation(cvs, -2*PI, -2*PI);

        bool r_activatedSwitch =  (cvs->inputs->u_switch[R_ID] > 0);
        bool l_activatedSwitch =  (cvs->inputs->u_switch[L_ID] > 0);

        // go to state Finish state after 5 seconds
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

    case CALIB_FINISH:

        // keep on going backward for a little while (make sure that we are perfectly aligned)
                speed_regulation(cvs, -PI/2, -PI/2);

                // waiting for the match to start
                if (t - calib->t_flag > 1.0)
                {
                    //Stopping to be sure that we won't slide anymore
                    speed_regulation(cvs, 0, 0);

                    switch(team_id)
                    {
                    case TEAM_A:
                        //Telling odometry that we are sure that x = 1.+0.06 and theta = 180
                        rob_pos->x = 1. - RobotGeometry::BACK_TO_CENTER;
                        rob_pos->theta = -PI;
                        break;
                    case TEAM_B:
                        //Telling odometry that we are sure that x = 0.5+0.06 and theta = 0
                        //The robot is back against the interior wall
                        rob_pos->x=0.5+RobotGeometry::BACK_TO_CENTER;
                        rob_pos->theta=0;
                        break;
                    default:
                        printf("Error: unknown team, calibration impossible : %d !\n", calib->flag);
                        exit(EXIT_FAILURE);

                    }



                }
        //Just to make sure we stop and wait for the match to begin
        speed_regulation(cvs, 0.0, 0.0);
        cvs->main_state = WAIT_INIT_STATE;
        printf("waiting for init\n");
        break;

    default:
        printf("Error: unknown state : %d !\n", calib->flag);
        exit(EXIT_FAILURE);
    }
}

NAMESPACE_CLOSE();
