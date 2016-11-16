#include "Cell_gr4.h"


NAMESPACE_INIT(ctrlGr4);

Cell::Cell()
{
    m_linkVector.reserve(8); //at most 4 links !
    m_status = OccupancyStatus_t::free;
}

Cell::~Cell()
{
    for (LinkListIt it = m_linkVector.begin(); it != m_linkVector.end(); it++)
    {
        delete(*it);
    }
}

Cell::Cell(double x, double y, double size) : m_x(x), m_y(y), m_size(size)
{
    m_linkVector.reserve(8); //at most 4 links !
    m_status = OccupancyStatus_t::free;
}

void Cell::addLink(Link *link)
{
    m_linkVector.push_back(link);
}

bool Cell::isNeighbor(int id)
{
    bool res = false;
    for (LinkListIt it = m_linkVector.begin(); it != m_linkVector.end(); it++)
    {
        if ( (*it)->goalId()==id )
        {
            res = true;
            break;
        }
    }
    return res;
}

NAMESPACE_CLOSE();
