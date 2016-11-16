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

    void    setHrstScore(double score){ m_heuristicalScore = score; }
    double  getHrstScore(){ return m_heuristicalScore; }
    int     goalId(){ return m_goalNodeId; }

private:
    int m_goalNodeId;

    double m_length;
    double m_heuristicalScore;
};

NAMESPACE_CLOSE();

#endif // LINK_H
