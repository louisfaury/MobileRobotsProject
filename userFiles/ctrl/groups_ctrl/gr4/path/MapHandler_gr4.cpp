#include "MapHandler_gr4.h"

NAMESPACE_INIT(ctrlGr4);

MapHandler::MapHandler()
{
    m_geoObjectList.reserve(12); //number of fixed obstacles ..

    // creating obstacles !
    // borders :
    Segment* bottomBorder = new Segment( Point(-1.062,-1.562), Point(1.062,-1.562 ) );
    Segment* leftBorder   = new Segment( Point(-1.062,-1.562), Point(-1.062,1.562) );
    Segment* rightBorder  = new Segment( Point(1.062,-1.562), Point(1.062,1.562) );
    Segment* topBorder    = new Segment( Point(1.062,1.562), Point(-1.062,1.562) );

    // TODO : add other boxes

    // adding obstacles
    m_geoObjectList.push_back(bottomBorder);
    m_geoObjectList.push_back(leftBorder);
    m_geoObjectList.push_back(rightBorder);
    m_geoObjectList.push_back(topBorder);
}


MapHandler::~MapHandler()
{
   for ( GeoObjIt it = m_geoObjectList.begin(); it != m_geoObjectList.end(); it++)
       delete(*it);
}


NAMESPACE_CLOSE();
