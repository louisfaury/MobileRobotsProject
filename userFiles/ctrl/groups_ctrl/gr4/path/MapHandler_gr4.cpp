/*!
 * @file MapHandler.cpp
 * @author Louis Faury
 * @date 14/11
 */

#include "MapHandler_gr4.h"
#include "SearchGraph_gr4.h"

NAMESPACE_INIT(ctrlGr4);

MapHandler::MapHandler()
{
    m_geoObjectList.reserve(12); //number of fixed obstacles ..

    // creating obstacles !
    // borders :
    Segment* bottomBorder = new Segment( Point(-1.0,-1.5), Point(1.0,-1.5 ) );
    Segment* leftBorder   = new Segment( Point(-1.0,-1.5), Point(-1.0,1.5) );
    Segment* rightBorder  = new Segment( Point(1.0,-1.5), Point(1.0,1.5) );
    Segment* topBorder    = new Segment( Point(1.0,1.5), Point(-1.0,1.5) );

    // box boundaries
    Rectangle* bottomLeftBox  = new Rectangle( Point(-0.75,-0.84), 0.4, 0.02 );
    Rectangle* bottomRightBox = new Rectangle( Point(0.49,-1.25), 0.02, 0.5 );
    Rectangle* topLeftBox     = new Rectangle( Point(-0.75,0.84), 0.5, 0.02 );
    Rectangle* topRightBox    = new Rectangle( Point(0.49,1.25), 0.02, 0.5 );

    // center obstacle
    Rectangle* bottomRect = new Rectangle( Point(0.,-0.35), 0.4, 0.1);
    Rectangle* topRect    = new Rectangle( Point(0.,0.35), 0.4, 0.1 );
    Rectangle* midVert    = new Rectangle( Point(-0.15,0.), 0.1, 0.6);
    Rectangle* leftHoz    = new Rectangle( Point(-0.35,0.), 0.3, 0.2);

    // TODO : add safety constant to each width and length !

    // adding obstacles
    m_geoObjectList.push_back(bottomBorder);
    m_geoObjectList.push_back(leftBorder);
    m_geoObjectList.push_back(rightBorder);
    m_geoObjectList.push_back(topBorder);

    m_geoObjectList.push_back(bottomLeftBox);
    m_geoObjectList.push_back(bottomRightBox);
    m_geoObjectList.push_back(topLeftBox);
    m_geoObjectList.push_back(topRightBox);

    m_geoObjectList.push_back(bottomRect);
    m_geoObjectList.push_back(topRect);
    m_geoObjectList.push_back(midVert);
    m_geoObjectList.push_back(leftHoz);
}


MapHandler::~MapHandler()
{
   for ( GeoObjListIt it = m_geoObjectList.begin(); it != m_geoObjectList.end(); it++)
       delete(*it);
}

bool MapHandler::isOnObstacle(Cell *cell)
{
    bool res = false;
    int i = 0;
    Rectangle cellRect( Point(cell->x(),cell->y()), SearchGraph::CELL_SIZE, SearchGraph::CELL_SIZE);
    for ( GeoObjListIt it = m_geoObjectList.begin(); it != m_geoObjectList.end(); it++)
    {
        i++;
        res = cellRect.computeIntersection(*it);
        if ( res == true )
            break; // once there is a collision no need to go further in the exploration
    }
    return res;
}

NAMESPACE_CLOSE();
