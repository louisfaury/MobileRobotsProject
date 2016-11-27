#ifndef LINK_H
#define LINK_H

#include "namespace_ctrl.h"
#include "CtrlStruct_gr4.h"
#include "geometric_objects_gr4.h"
#include "LinePath_gr4.h"
#include <vector>

NAMESPACE_INIT(ctrlGr4);

class Link
{
public:
    Link();
    Link(int id, double length, double angle, Point start);

    LinePath* line(){ return &m_line; }
    int goalId(){ return m_goalNodeId; }

private:
    int m_goalNodeId;
    LinePath m_line;
};

NAMESPACE_CLOSE();

#endif // LINK_H
