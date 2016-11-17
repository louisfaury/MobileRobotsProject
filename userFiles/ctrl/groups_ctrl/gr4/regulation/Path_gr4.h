/*!
 * @file Path_gr4.h
 * @author Louis Faury
 * @date 17/11
 */

#ifndef PATH_GR4_H
#define PATH_GR4_H

#include "namespace_ctrl.h"
#include "CtrlStruct_gr4.h"

NAMESPACE_INIT(ctrlGr4);

class Path
{
public:
    Path();
    Path(double length);

    virtual double  length(){ return m_length; }
    virtual bool    nextStep(double&, double, CtrlStruct*) = 0;

protected:
    double m_length;
};

NAMESPACE_CLOSE();

#endif // PATH_GR4_H
