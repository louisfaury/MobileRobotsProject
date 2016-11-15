#ifndef CELL_H
#define CELL_H

#include "namespace_ctrl.h"
#include "CtrlStruct_gr4.h"
#include "Link_gr4.h"
#include <vector>

NAMESPACE_INIT(ctrlGr4);

using LinkList = std::vector<Link*>;
using LinkListIt = LinkList::iterator;

class Cell
{
public:
    enum class OccupancyStatus_t
    {
        free = 0,
        occupied
    };

    Cell();
    ~Cell();
    Cell(double x, double y, double size);

    double              x(){ return m_x; }
    double              y();
    double              size(){ return m_size; }
    OccupancyStatus_t   status(){ return m_status; }
    void                setFree(){ m_status = OccupancyStatus_t::free; }
    void                setOccupied(){ m_status = OccupancyStatus_t::occupied; }
    void                addLink(Link* link);

private:
    double m_x;
    double m_y;
    double m_size; // cm, assuming square cells
    OccupancyStatus_t m_status; // is cell occupied or free
    std::vector<Link*> m_linkVector;
};

NAMESPACE_CLOSE();

#endif // CELL_H
