#include "mcl_gr4.h"
#include "odometry_gr4.h"
#include "triangulation_gr4.h"
#include "useful_gr4.h"

NAMESPACE_INIT(ctrlGr4);

/*! \brief update using MCL technic
 * 
 * \param[in,out] cvs controller main structure
 */
void mcl_update(CtrlStruct *cvs)
{
	// variable declaration
	RobotPosition *rob_pos;

	// variables initialization
	rob_pos = cvs->rob_pos;
}

NAMESPACE_CLOSE();
