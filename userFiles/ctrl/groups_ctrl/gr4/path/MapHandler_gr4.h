/*!
 * @file MapHandler.h
 * @author Louis Faury
 * @date 14/11
 */

#ifndef MAPHANDLER_H
#define MAPHANDLER_H

#include "namespace_ctrl.h"
#include "CtrlStruct_gr4.h"
#include "geometric_objects_gr4.h"
#include "Cell_gr4.h"
#include <vector>

NAMESPACE_INIT(ctrlGr4);

class MapHandler
{
public:
    typedef std::vector<GeometricObject*> GeoObjList;
    typedef GeoObjList::iterator GeoObjListIt;

    MapHandler();
    ~MapHandler();

    /*!
     * @function isOnObstacle(cell : Cell) : bool
     * @brief Returns true if the rectangle spawned by the cell is in collision with an obstacle
     */
    bool isOnObstacle(Cell* cell);

    /**
     * @brief updateOpponents Updates opponents position in m_opponentsList, takes into account distance between robot and opponent to adjust opponent representation size on map,
     * Opponents are represented as circles
     * @param rob Current robot position
     * @param opp Current opponent position
     * @param index Opponent index (0 or 1)
     */
    void updateOpponents(Point rob, Point opp, int index);

    /**
     * @brief isOnOpponent Checks if an opponent is present on examined cell
     * @param cell Examined cell
     * @return true if an opponent is on the cell, false otherwise
     */
    bool isOnOpponent(Cell* cell);

    static const double MAP_LENGTH = 2.0; //m
    static const double MAP_WIDTH  = 3.0; //m
    static const double MAP_SAFETY = 0.2; //m

private:
    GeoObjList m_fixedObstacleList; //List of map walls represented as rectangles
    GeoObjList m_opponentsList; //List of opponents of the map (updated regularly by opp_pos module)

};

NAMESPACE_CLOSE();

#endif // MAPHANDLER_H
