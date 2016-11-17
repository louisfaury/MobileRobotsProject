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

    using LinkIt = std::vector<Link*>::iterator;

    Cell();
    ~Cell();
    Cell(double x, double y, double size);

    virtual double              x(){ return m_x; }
    virtual double              y(){ return m_y; }
    virtual double              size(){ return m_size; }
    virtual OccupancyStatus_t   status(){ return m_status; }
    virtual void                setFree(){ m_status = OccupancyStatus_t::free; }
    virtual void                setOccupied(){ m_status = OccupancyStatus_t::occupied; }
    virtual void                addLink(Link* link);
    virtual bool                isNeighbor(int id);
    virtual std::vector<Link*> getLinks(){return m_linkVector;}


private:
    double m_x;
    double m_y;
    double m_size; // cm, assuming square cells
    OccupancyStatus_t m_status; // is cell occupied or free
    std::vector<Link*> m_linkVector;
};

NAMESPACE_CLOSE();

#endif // CELL_H
