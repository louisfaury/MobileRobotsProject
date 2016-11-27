#include "speed_regulation_gr4.h"
#include "useful_gr4.h"
#include "time.h"

NAMESPACE_INIT(ctrlGr4);

/*! \brief wheel speed regulation
 * 
 * \param[in,out] cvs controller main structure
 * \parem[in] r_sp_ref right wheel speed reference [rad/s]
 * \parem[in] l_sp_ref left wheel speed reference [rad/s]
 */
void speed_regulation(CtrlStruct *cvs, double r_sp_ref, double l_sp_ref)
{
	double r_sp, l_sp;
    double r_error, l_error;
    double* r_int_error = &(cvs->sp_reg->int_error_r);
    double* l_int_error = &(cvs->sp_reg->int_error_l);
    double r_command, l_command;
	double dt;

	// variables declaration
	CtrlIn *inputs;
	CtrlOut *outputs;
	SpeedRegulation *sp_reg;

	// variables initialization
	inputs  = cvs->inputs;
	outputs = cvs->outputs;
	sp_reg  = cvs->sp_reg;

	// wheel speeds
	r_sp = inputs->r_wheel_speed;
	l_sp = inputs->l_wheel_speed;

	// time
    dt = inputs->t - sp_reg->last_t; // time interval since last call

	// ----- Wheels regulation computation start ----- //

    // computing current error and integral error
    r_error = r_sp_ref - r_sp;
    l_error = l_sp_ref - l_sp;

    // saturating the integral error to stay away from non linear behavior
    *r_int_error = limit_range(*r_int_error + dt*r_error, SpeedRegulation::PI_MIN_INT_ERR, SpeedRegulation::PI_MAX_INT_ERR);
    *l_int_error = limit_range(*l_int_error + dt*l_error, SpeedRegulation::PI_MIN_INT_ERR, SpeedRegulation::PI_MAX_INT_ERR);


    // wheel commands, after saturating output for actuators constraints
    // PI controller using the following convention
    //    u = K_P * ( err + (1/T_I)*int_error

    r_command = limit_range(r_error + (1./SpeedRegulation::PI_INT_CHAR_TIME)*(*r_int_error),-SpeedRegulation::PI_OUTPUT_EXTREMUM, SpeedRegulation::PI_OUTPUT_EXTREMUM);
    l_command = limit_range(l_error + (1./SpeedRegulation::PI_INT_CHAR_TIME)*(*l_int_error),-SpeedRegulation::PI_OUTPUT_EXTREMUM, SpeedRegulation::PI_OUTPUT_EXTREMUM);
    r_command *= SpeedRegulation::PI_PROP_GAIN;
    l_command *= SpeedRegulation::PI_PROP_GAIN;

    outputs->wheel_commands[R_ID] = r_command;
    outputs->wheel_commands[L_ID] = l_command;

/*
    set_plot(RobotGeometry::WHEEL_RADIUS*r_sp, "Right output");
    set_plot(RobotGeometry::WHEEL_RADIUS*l_sp, "Left output");
    set_plot(RobotGeometry::WHEEL_RADIUS*r_sp_ref, "Right reference");
    set_plot(RobotGeometry::WHEEL_RADIUS*l_sp_ref, "Left reference");
*/

    cvs->sp_reg->l_sp_ref = l_sp_ref;
    cvs->sp_reg->r_sp_ref = r_sp_ref;

    //TODO ; anti-reset windup


	// ----- Wheels regulation computation end ----- //

	// last update time
	sp_reg->last_t = inputs->t;
}

NAMESPACE_CLOSE();
