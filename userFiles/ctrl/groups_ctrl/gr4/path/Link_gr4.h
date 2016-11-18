#ifndef LINK_H
#define LINK_H

#include "namespace_ctrl.h"
#include "CtrlStruct_gr4.h"
#include <vector>

NAMESPACE_INIT(ctrlGr4);

class Link
{
public:
    Link();
    Link(int id, double length);

    int     goalId(){ return m_goalNodeId; }

private:
    int m_goalNodeId;
    double m_length;
};

NAMESPACE_CLOSE();

#endif // LINK_H
