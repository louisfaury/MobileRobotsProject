#ifndef CELL_H
#define CELL_H

#include "namespace_ctrl.h"
#include "CtrlStruct_gr4.h"
#include "Link_gr4.h"
#include <vector>

NAMESPACE_INIT(ctrlGr4);

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

    virtual double getX(){ return m_x; }
    virtual double getY(){ return m_y; }
    virtual double getS(){ return m_size; }
    virtual int isOccupied(){return (int)m_status;}
    virtual void addLink(Link* link);
    virtual std::vector<Link*> getLinks(){return m_linkVector;}

protected:
    double m_x;
    double m_y;
    double m_size; // cm, assuming square cells

    OccupancyStatus_t m_status; // is cell occupied or free

    std::vector<Link*> m_linkVector;
};

NAMESPACE_CLOSE();

#endif // CELL_H
