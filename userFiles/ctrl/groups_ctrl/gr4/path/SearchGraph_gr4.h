#ifndef SEARCHGRAPH_H
#define SEARCHGRAPH_H

#include "namespace_ctrl.h"
#include "CtrlStruct_gr4.h"
#include "Cell_gr4.h"
#include "MapHandler_gr4.h"
#include <map>

NAMESPACE_INIT(ctrlGr4);

using CellIt = std::map<int, Cell*>::iterator;

class SearchGraph
{
public:

    SearchGraph();
    ~SearchGraph();

    static constexpr double CELL_SIZE = 0.2;

protected:
    virtual void _graphInit();
    virtual void _describe();
    virtual void _addCell(Cell* cell);

    MapHandler m_mapHandler;
    std::map<int,Cell*> m_cellMap; //dynamic graph, easier to handle this way than to store ids in Cells
    int m_cellCtr;

};

NAMESPACE_CLOSE();

#endif // SEARCHGRAPH_H


// TODO : extend in a daughter class for dynamic graph handling
