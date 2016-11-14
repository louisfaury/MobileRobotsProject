#include "SearchGraph_gr4.h"


NAMESPACE_INIT(ctrlGr4);

SearchGraph::SearchGraph()
{
    _createMap();
}

SearchGraph::~SearchGraph()
{
    for (CellIt it = m_cellMap.begin(); it != m_cellMap.end(); it++)
    {
        delete((*it).second);
    }
}

void SearchGraph::addCell(Cell *cell)
{
    //TODO : incorporate a free index vector so that index doesn't reach limit
    m_cellCtr++;
    m_cellMap[m_cellCtr] = cell;
}

void SearchGraph::_createMap()
{
    // creates the static map
}

NAMESPACE_CLOSE();

