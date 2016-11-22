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
    using GeoObjList   = std::vector<GeometricObject*>;
    using GeoObjListIt = GeoObjList::iterator;

    MapHandler();
    ~MapHandler();

    /*!
     * @function isOnObstacle(cell : Cell) : bool
     * @brief Returns true if the rectangle spawned by the cell is in collision with an obstacle
     */
    bool isOnObstacle(Cell* cell);

    static constexpr double MAP_LENGTH = 2.0;
    static constexpr double MAP_WIDTH = 3.0;
    static constexpr double MAP_SAFETY = 0.1;

private:
    GeoObjList m_geoObjectList;
};

NAMESPACE_CLOSE();

#endif // MAPHANDLER_H
