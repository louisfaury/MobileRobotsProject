#include "init_pos_gr4.h"
#include <math.h>
#include "config_file.h"
#include "config_file_gr4.h"

NAMESPACE_INIT(ctrlGr4);

/*! \brief set the initial robot position
 * 
 * \param[in] robot_id robot ID
 * \param[out] rob_pos robot position structure
 *
 * Adapt these initial positions, depending on the game map.
 */
void set_init_position(int robot_id, RobotPosition *rob_pos)
{
	switch (robot_id)
    {
        case ROBOT_B: // blue robot
            rob_pos->x = BLUE_T1;
            rob_pos->y = BLUE_T2;
            rob_pos->theta = DEG2RAD(BLUE_R3);
            break;

		case ROBOT_R: // red robot
            rob_pos->x = RED_T1;
            rob_pos->y = RED_T2;
            rob_pos->theta = DEG2RAD(RED_R3);
			break;

		case ROBOT_Y: // yellow robot
            rob_pos->x = YELLOW_T1;
            rob_pos->y = YELLOW_T2;
            rob_pos->theta = DEG2RAD(YELLOW_R3);
			break;

		case ROBOT_W: //  white robot
            rob_pos->x = WHITE_T1;
            rob_pos->y = WHITE_T2;
            rob_pos->theta = DEG2RAD(WHITE_R3) ;
			break;
	
		default:
			printf("Error: unknown robot ID: %d !\n", robot_id);
			exit(EXIT_FAILURE);
	}		
}

NAMESPACE_CLOSE();
