/*!
 * @file Path_gr4.h
 * @author Louis Faury
 * @date 17/11
 * @brief Virtual pure class representing a path, see daughters classes for further descriptions
 */

#ifndef PATH_GR4_H
#define PATH_GR4_H

#include "namespace_ctrl.h"
#include "CtrlStruct_gr4.h"
#include <string>

NAMESPACE_INIT(ctrlGr4);

class Path
{
public:
    Path();
    Path(double length, int startId, int endId);

    virtual double  length(){ return m_length; }
    virtual bool    nextStep(double&, double, CtrlStruct*) = 0;
    virtual void    describe(){ printf("%f,%s\n",m_length, m_id.c_str()); }
    virtual void    setEndSpeed(double endSpeed) = 0;
    virtual double  smoothFromEnd(double endSpeed) = 0;
    virtual int     getEndId(){return m_endId;}
    virtual int     getStartId(){return m_startId;}

protected:
    double m_length;
    std::string m_id;
    int m_endId;
    int m_startId;
};

NAMESPACE_CLOSE();

#endif // PATH_GR4_H
