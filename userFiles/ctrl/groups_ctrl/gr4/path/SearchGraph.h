#ifndef SEARCHGRAPH_H
#define SEARCHGRAPH_H

#include "namespace_ctrl.h"
#include "CtrlStruct_gr4.h"
#include "Cell.h"
#include <map>

NAMESPACE_INIT(ctrlGr4);

class SearchGraph
{
public:
    using CellIt = std::map<int, Cell*>::iterator;

    SearchGraph();
    ~SearchGraph();

    void addCell(Cell* cell);

private:
    std::map<int,Cell*> m_cellMap; //dynamic graph, easier to handle this way than to store ids in Cells

    int m_cellCtr;

};

NAMESPACE_CLOSE();

#endif // SEARCHGRAPH_H
