/*!
 * @file LinePathlist.h
 * @author Louis Faury
 * @date 15/11
 */

#ifndef PATHLIST_H
#define PATHLIST_H

#include "namespace_ctrl.h"
#include "CtrlStruct_gr4.h"
#include "LinePath_gr4.h"
#include "Path_gr4.h"
#include <vector>

NAMESPACE_INIT(ctrlGr4);

using PathVectIt = std::vector<Path*>::iterator;

class LinePathList
{
public:
    LinePathList();
    ~LinePathList();

    void    addPath(Path* path);
    bool    nextStep(double s, double dt, CtrlStruct* cvs);
    double  length();
    void    clear();

private:
    std::vector<Path*> m_pathVec;
};

NAMESPACE_CLOSE();

#endif // LINEPATHLIST_H
