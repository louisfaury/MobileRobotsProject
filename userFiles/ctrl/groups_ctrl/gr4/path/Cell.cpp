#include "Cell.h"


NAMESPACE_INIT(ctrlGr4);

Cell::Cell()
{
    m_linkVector.reserve(4); //at most 4 links !
}

Cell::~Cell()
{
    for (LinkIt it = m_linkVector.begin(); it != m_linkVector.end(); it++)
    {
        delete(*it);
    }
}

Cell::Cell(double x, double y, double size) : m_x(x), m_y(y), m_size(size)
{
    m_linkVector.reserve(4); //at most 4 links !

}
NAMESPACE_CLOSE();
