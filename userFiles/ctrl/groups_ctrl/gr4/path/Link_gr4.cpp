#include "Link_gr4.h"


NAMESPACE_INIT(ctrlGr4);

Link::Link()
{
}

Link::Link(int id, double length, double angle, Point start) : m_goalNodeId(id), m_line(start,length,angle,id)
{
}

NAMESPACE_CLOSE();
