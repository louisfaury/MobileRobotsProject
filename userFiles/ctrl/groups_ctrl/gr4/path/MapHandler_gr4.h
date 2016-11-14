#ifndef MAPHANDLER_H
#define MAPHANDLER_H

#include "namespace_ctrl.h"
#include "CtrlStruct_gr4.h"
#include "geometric_objects_gr4.h"
#include <vector>

NAMESPACE_INIT(ctrlGr4);

class MapHandler
{
public:
    using GeoObjIt = std::vector<GeometricObject*>::iterator;

    MapHandler();
    ~MapHandler();

private:
    std::vector<GeometricObject*> m_geoObjectList;

    double m_width;
    double m_length;
};

NAMESPACE_CLOSE();

#endif // MAPHANDLER_H
