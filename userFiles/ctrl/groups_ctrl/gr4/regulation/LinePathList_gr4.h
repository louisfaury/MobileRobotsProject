/*!
 * @file LinePathlist.h
 * @author Louis Faury
 * @date 15/11
 */

#ifndef LINEPATHLIST_H
#define LINEPATHLIST_H

#include "namespace_ctrl.h"
#include "CtrlStruct_gr4.h"
#include "LinePath_gr4.h"
#include <vector>

NAMESPACE_INIT(ctrlGr4);

using LineList = std::vector<LinePath>;
using LineListIt = LineList::iterator;

class LinePathList
{
public:
    LinePathList();

    void    addLine(LinePath line);
    bool    nextStep(double s, double dt, CtrlStruct* cvs);
    double  length();

private:
    LineList m_lineList;
};

NAMESPACE_CLOSE();

#endif // LINEPATHLIST_H
