/*!
 * @file LinePath.h
 * @author Louis Faury
 * @date 15/11
 */

#ifndef LINEPATH_H
#define LINEPATH_H

#include "namespace_ctrl.h"
#include "CtrlStruct_gr4.h"
#include "geometric_objects_gr4.h"
#include "Path_gr4.h"
#include "config_file_gr4.h"


NAMESPACE_INIT(ctrlGr4);

class LinePath : public Path
{
public:
    LinePath();
    LinePath(Point start, double length, double angle, int startId, int endId);

    /*! getters !*/
    Point   start(){ return m_start; }
    double  angle(){ return m_angle; }
    double  endSpeed(){ return m_endSpeed; }
    /*!
     * @function nextStep(double,double,CtrlStruct) : bool
     * @brief compute the speed regulation rules for the robot on the next dt time step
     * @returns bool if path is completed
     */
    virtual bool   nextStep(double& s, double dt, CtrlStruct* cvs);
    /*!
     * \brief smoothFromEnd : computes maximum start speed given speed and acceleration constraints
     * \param endSpeed : given from following path
     * \return double : startSpeed
     */
    virtual double smoothFromEnd(double endSpeed);
    /*! setters !*/
    virtual void   setEndSpeed(double endSpeed){ m_endSpeed = endSpeed; }
    /*! verbose function for test purposes !*/
    virtual void   describe(){ printf("%f,%s,%f\n",m_length, m_id.c_str(), m_endSpeed); }

    /*! kinematics constraints !*/
    static const double MAX_ACC   = 2;
    static const double MAX_DESAC = 2;
    static const double MAX_SPEED = 1.5;

protected:
    Point m_start; /// start point for the line
    double m_angle; /// angle wrt to (0,Ox)
    double m_endSpeed; /// endSpeed

};


NAMESPACE_CLOSE();

#endif // LINEPATH_H
