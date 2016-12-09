#include "Link_gr4.h"


NAMESPACE_INIT(ctrlGr4);

Link::Link()
{
}

Link::Link(int startId, int endId, double length, double angle, Point start) : m_goalNodeId(endId), m_line(start,length,angle,startId, endId)
{
}

NAMESPACE_CLOSE();
