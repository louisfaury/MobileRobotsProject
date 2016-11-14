#include "Link_gr4.h"


NAMESPACE_INIT(ctrlGr4);

Link::Link()
{
}

Link::Link(int id1, int id2, double length) : m_firstNodeId(id1), m_secondNodeId(id2), m_length(length), m_heuristicalScore(0.)
{
}

NAMESPACE_CLOSE();
