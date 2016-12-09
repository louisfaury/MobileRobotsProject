/*!
 * @file MapHandler.cpp
 * @author Louis Faury
 * @date 14/11
 */

#include "MapHandler_gr4.h"
#include "SearchGraph_gr4.h"
#include "useful_gr4.h"

NAMESPACE_INIT(ctrlGr4);

MapHandler::MapHandler()
{
    m_fixedObstacleList.reserve(12); //number of fixed obstacles
    m_opponentsList.reserve(2); //two obstacles

    // creating obstacles !
    // borders :
    Segment* bottomBorder = new Segment( Point(-1.0,-1.5), Point(1.0,-1.5 ) );
    Segment* leftBorder   = new Segment( Point(-1.0,-1.5), Point(-1.0,1.5) );
    Segment* rightBorder  = new Segment( Point(1.0,-1.5), Point(1.0,1.5) );
    Segment* topBorder    = new Segment( Point(1.0,1.5), Point(-1.0,1.5) );

    // box boundaries
    Rectangle* bottomLeftBox  = new Rectangle( Point(-0.75,-0.84), 0.5+MAP_SAFETY, 0.02+MAP_SAFETY );
    Rectangle* bottomRightBox = new Rectangle( Point(0.49,-1.25), 0.02+MAP_SAFETY, 0.5+MAP_SAFETY );
    Rectangle* topLeftBox     = new Rectangle( Point(-0.75,0.84), 0.5+MAP_SAFETY, 0.02+MAP_SAFETY );
    Rectangle* topRightBox    = new Rectangle( Point(0.49,1.25), 0.02+MAP_SAFETY, 0.5+MAP_SAFETY );

    // center obstacle
    Rectangle* bottomRect = new Rectangle( Point(0.,-0.35), 0.4+MAP_SAFETY, 0.1+MAP_SAFETY);
    Rectangle* topRect    = new Rectangle( Point(0.,0.35), 0.4+MAP_SAFETY, 0.1+MAP_SAFETY );
    Rectangle* midVert    = new Rectangle( Point(-0.15,0.), 0.1+MAP_SAFETY, 0.6+MAP_SAFETY);
    Rectangle* leftHoz    = new Rectangle( Point(-0.35,0.), 0.3+MAP_SAFETY, 0.2+MAP_SAFETY);

    // adding obstacles
    m_fixedObstacleList.push_back(bottomBorder);
    m_fixedObstacleList.push_back(leftBorder);
    m_fixedObstacleList.push_back(rightBorder);
    m_fixedObstacleList.push_back(topBorder);

    m_fixedObstacleList.push_back(bottomLeftBox);
    m_fixedObstacleList.push_back(bottomRightBox);
    m_fixedObstacleList.push_back(topLeftBox);
    m_fixedObstacleList.push_back(topRightBox);

    m_fixedObstacleList.push_back(bottomRect);
    m_fixedObstacleList.push_back(topRect);
    m_fixedObstacleList.push_back(midVert);
    m_fixedObstacleList.push_back(leftHoz);

    //dynamic allocation (in constructor) for opponents
    //Initially set out of the map allows to be number of opponents agnostic in path checking
    Circle* opp1 = new Circle(Point(-10, -10), 0);
    Circle* opp2 = new Circle(Point(-10, -10), 0);
    m_opponentsList.push_back(opp1);
    m_opponentsList.push_back(opp2);

}


MapHandler::~MapHandler()
{
   for ( GeoObjListIt it = m_fixedObstacleList.begin(); it != m_fixedObstacleList.end(); it++)
       delete(*it);
   for ( GeoObjListIt itO = m_opponentsList.begin(); itO != m_opponentsList.end(); itO++)
       delete(*itO);
}

bool MapHandler::isOnObstacle(Cell *cell)
{
    bool res = false;
    Rectangle cellRect( Point(cell->x(),cell->y()), SearchGraph::CELL_SIZE, SearchGraph::CELL_SIZE);
    for ( GeoObjListIt it = m_fixedObstacleList.begin(); it != m_fixedObstacleList.end(); it++)
    {
        res = cellRect.computeIntersection(*it);
        if ( res == true )
            break; // once there is a collision no need to go further in the exploration
    }
    return res;
}


void MapHandler::updateOpponents(Point rob, Point opp, int index)
{
    if(index<2 && index>=0){
        Circle oppC = Circle(opp, MIN(0.9*RobotGeometry::WHEEL_BASE, 0.7*rob.computeDistance(opp)));
        *((Circle*)(m_opponentsList.at(index))) = oppC;
    }else{
        printf("Error in opponents position update, unknown opponent index");
    }
}

bool MapHandler::isOnOpponent(Cell *cell)
{
    bool res = false;
    Rectangle cellRect( Point(cell->x(),cell->y()), SearchGraph::CELL_SIZE, SearchGraph::CELL_SIZE);
    for ( GeoObjListIt it = m_opponentsList.begin(); it != m_opponentsList.end(); it++)
    {
        res = cellRect.computeIntersection(*it);
        if ( res == true )
            break; // once there is a collision no need to go further in the exploration
    }
    return res;
}

NAMESPACE_CLOSE();
