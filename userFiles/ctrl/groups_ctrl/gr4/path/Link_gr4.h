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
    //Constructors
    Link();
    Link(int startId, int endId, double length, double angle, Point start);

    //Setters and getters
    LinePath* line(){ return &m_line; }
    int goalId(){ return m_goalNodeId; }
    double angle(){return m_line.angle();}

private:
    int m_goalNodeId; //id of the cell the link is pointing to
    LinePath m_line; // represents the trajectory between the two concerned cells
};

NAMESPACE_CLOSE();

#endif // LINK_H
