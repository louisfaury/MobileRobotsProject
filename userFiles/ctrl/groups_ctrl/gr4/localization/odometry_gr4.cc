#include "odometry_gr4.h"
#include "useful_gr4.h"
#include "init_pos_gr4.h"
#include "config_file_gr4.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr4);

/*! \brief update the robot odometry
 * 
 * \param[in,out] cvs controller main structure
 */
void update_odometry(CtrlStruct *cvs)
{
	// variables declaration
	double r_sp, l_sp;
	double dt;
    double incRight, incLeft;
    double dS, dTheta;
	RobotPosition *rob_pos;
	CtrlIn *inputs;

	// variables initialization
	inputs  = cvs->inputs;
	rob_pos = cvs->rob_pos;

	r_sp = inputs->r_wheel_speed; // right wheel speed
	l_sp = inputs->l_wheel_speed; // left wheel speed

	// time
	dt = inputs->t - rob_pos->last_t; // time increment since last call

	// safety
	if (dt <= 0.0)
	{
		return;
	}

	// ----- odometry computation start ----- //
    incRight = r_sp * dt * RobotGeometry::WHEEL_RADIUS;
    incLeft = l_sp * dt * RobotGeometry::WHEEL_RADIUS;

    dS = 0.5*(incRight+incLeft);
    dTheta = (incRight-incLeft)/RobotGeometry::WHEEL_BASE;


	// ----- odometry computation end ----- //
    rob_pos->x += dS * cos(rob_pos->theta + 0.5*dTheta);
    rob_pos->y += dS * sin(rob_pos->theta + 0.5*dTheta);
    rob_pos->theta += dTheta;

    set_plot(rob_pos->x, "Ideal blue x <[m]");
    set_plot(rob_pos->y, "Ideal blue y <[m]");
   // set_plot(rob_pos->theta, "Ideal blue theta <[m]");


	// last update time
	rob_pos->last_t = inputs->t;
}

NAMESPACE_CLOSE();
