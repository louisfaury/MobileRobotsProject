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

    //Setters and getters
    virtual double              x(){ return m_x; }
    virtual double              y(){ return m_y; }
    virtual double              size(){ return m_size; }
    virtual OccupancyStatus_t   status(){ return m_status; }
    virtual void                setFree(){ m_status = OccupancyStatus_t::free; }
    virtual void                setOccupied(){ m_status = OccupancyStatus_t::occupied; }
    virtual std::vector<Link*>  getLinks(){return m_linkVector;}
    virtual Link*               getLink(int endNodeId);

    /**
     * @brief addLink : add a link to the concerned cell
     * @param link : added link
     */
    virtual void                addLink(Link* link);

    /**
     * @brief isNeighbor : check if cell with id "id" is a neighbor of current object
     * @param id : tested neighbor
     * @return : true if it is a neighbor, false otherwise
     */
    virtual bool                isNeighbor(int id);



private:
    double m_x; //x  coordinate
    double m_y; //y coordinate
    double m_size; // cm, assuming square cells
    OccupancyStatus_t m_status; // is cell occupied by a wall or free
    std::vector<Link*> m_linkVector; //list of links of the cell
};

NAMESPACE_CLOSE();

#endif // CELL_H
