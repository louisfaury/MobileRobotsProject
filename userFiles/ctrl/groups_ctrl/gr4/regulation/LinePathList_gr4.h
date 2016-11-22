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
using PathVectRit = std::vector<Path*>::reverse_iterator;

class LinePathList
{
public:
    LinePathList();
    ~LinePathList();

    void    addPath(Path* path);
    bool    nextStep(double& s, double dt, CtrlStruct* cvs);
    double  length();
    void    clear();
    void    reverse();
    bool    isEmpty(){return m_pathVec.empty();};
    void    smooth(double);

private:
    std::vector<Path*> m_pathVec;
    bool m_change;
};

NAMESPACE_CLOSE();

#endif // LINEPATHLIST_H
