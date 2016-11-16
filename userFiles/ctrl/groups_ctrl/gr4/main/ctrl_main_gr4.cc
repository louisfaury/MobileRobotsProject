/*! 
 * \author Group 4
 * \file controller_main_gr4.cc
 * \brief Initialization, loop and finilization of the controller written in C (but compiled as C++)
 */

#include "ctrl_main_gr4.h"
#include "namespace_ctrl.h"
#include "init_pos_gr4.h"
#include "odometry_gr4.h"
#include "opp_pos_gr4.h"
#include "speed_regulation_gr4.h"
#include "calibration_gr4.h"
#include "triangulation_gr4.h"
#include "strategy_gr4.h"
#include "kalman_gr4.h"


//TODO : delete includes when search path tested
#include "SearchGraph_gr4.h"
#include "Cell_gr4.h"
#include "Link_gr4.h"
#include "SearchCell_gr4.h"


NAMESPACE_INIT(ctrlGr4);

/*! \brief initialize controller operations (called once)
 * 
 * \param[in] cvs controller main structure
 */
void controller_init(CtrlStruct *cvs)
{
	// variables declaration
	double t;
	CtrlIn *inputs;

	inputs = cvs->inputs;
	t = inputs->t;

	// robot ID
	cvs->robot_id = inputs->robot_id;

	// robot team
	switch (inputs->robot_id)
	{
		case ROBOT_B: cvs->team_id = TEAM_A; break;
		case ROBOT_R: cvs->team_id = TEAM_A; break;
		case ROBOT_Y: cvs->team_id = TEAM_B; break;
		case ROBOT_W: cvs->team_id = TEAM_B; break;
	
		default:
			printf("Error: unknown robot ID: %d !\n", inputs->robot_id);
			exit(EXIT_FAILURE);
	}

	// number of opponents
	cvs->nb_opp = inputs->nb_opponents;

	// robot initial position
	set_init_position(cvs->robot_id, cvs->rob_pos);
	cvs->rob_pos->last_t = t;

	// speed regulation
	cvs->sp_reg->last_t = t;
}

/*! \brief controller loop (called every time-step)
 * 
 * \param[in] cvs controller main structure
 */
void controller_loop(CtrlStruct *cvs)
{


    //TODO : delete when searchpath tested
//    SearchGraph *searchmap = new SearchGraph();

//    SearchCell* c1 = new SearchCell(0,0,1);
//    SearchCell* c2 = new SearchCell(1,0,1);
//    SearchCell* c3 = new SearchCell(0,1,1);
//    SearchCell* c4 = new SearchCell(1,1,1);

//    searchmap->_addCell(c1);
//    searchmap->_addCell(c2);
//    searchmap->_addCell(c3);
//    searchmap->_addCell(c4);


//    Link* l11 = new Link(1,1);
//    Link* l12 = new Link(3,1.3);
//    Link* l13 = new Link(2,1);

//    Link* l21 = new Link(0,1);
//    Link* l22 = new Link(2,1.3);
//    Link* l23 = new Link(3,1);

//    Link* l31 = new Link(0,1);
//    Link* l32 = new Link(1,1.3);
//    Link* l33 = new Link(3,1);

//    Link* l41 = new Link(1,1);
//    Link* l42 = new Link(0,1.3);
//    Link* l43 = new Link(2,1);

//    c1->addLink(l11);
//    c1->addLink(l12);
//    c1->addLink(l13);

//    c2->addLink(l21);
//    c2->addLink(l22);
//    c2->addLink(l23);

//    c3->addLink(l31);
//    c3->addLink(l32);
//    c3->addLink(l33);

//    c4->addLink(l41);
//    c4->addLink(l42);
//    c4->addLink(l43);


//   std::vector<int> path = searchmap->computePath(120,35);
//   for (std::vector<int>::const_iterator i = path.begin(); i != path.end(); ++i){
//       printf("%d\t", *i);
//    }
//    getchar();



    // variables declaration
    double t;
    CtrlIn *inputs;
    CtrlOut *outputs;

    // variables initialization
    inputs  = cvs->inputs;
    outputs = cvs->outputs;

    // time
    t = inputs->t;

    // update the robot odometry
    update_odometry(cvs);

    // triangulation
    triangulation(cvs);

    // opponents position
    opponents_tower(cvs);

    // tower control
    outputs->tower_command = 15.;

    // kalman
    kalman(cvs);

    /*
     * Test loop for milestone A
    static bool init = false;
    static double t0;
    if (t <= -13)
    {
        speed_regulation(cvs, PI, PI);
    }
    else if (t>-13 && t<=-11)
    {
        speed_regulation(cvs, PI*(1-0.8), PI*(1+0.8));
    }
    else if (t>-11 && t<-9)
    {
        speed_regulation(cvs, PI,PI);
    }
    else
    {
        if (!init)
        {
            t0 = t;
            init = true;
        }
        if (PI*(1-(t-t0)/2)>0)
            speed_regulation(cvs,PI*(1-(t-t0)/2),PI*(1-(t-t0)/2));
        else
            speed_regulation(cvs,0,0);
    }
    */

    switch (cvs->main_state)
    {
        // calibration
        case CALIB_STATE:
            calibration(cvs);
            break;

        // wait before match beginning
        case WAIT_INIT_STATE:
            speed_regulation(cvs, 0.0, 0.0);

            if (t > 0.0)
            {
                cvs->main_state = RUN_STATE;
                cvs->strat->main_state = GAME_STATE_A;
            }
            break;

        // during game
        case RUN_STATE:
            main_strategy(cvs);

            if (t > 89.0) // 1 second safety
            {
                cvs->main_state = STOP_END_STATE;
            }
            break;

        // stop at the end of the game
        case STOP_END_STATE:
            speed_regulation(cvs, 0.0, 0.0);

            outputs->flag_release = 1;
            break;

        case NB_MAIN_STATES:
            printf("Error: state NB_MAIN_STATES should not be reached !\n");
            exit(EXIT_FAILURE);
            break;
	
        default:
            printf("Error:unknown state : %d !\n", cvs->main_state);
            exit(EXIT_FAILURE);
    }
}

/*! \brief last controller operations (called once)
 * 
 * \param[in] cvs controller main structure
 */
void controller_finish(CtrlStruct *cvs)
{

}

NAMESPACE_CLOSE();
