#ifndef SEARCHGRAPH_H
#define SEARCHGRAPH_H

#include "namespace_ctrl.h"
#include "CtrlStruct_gr4.h"
#include "Cell_gr4.h"
#include "SearchCell_gr4.h"
#include "MapHandler_gr4.h"
#include <map>

NAMESPACE_INIT(ctrlGr4);

using CellIt = std::map<int, Cell*>::iterator;
using SCellIt = std::map<int, SearchCell*>::iterator;
using LinkIt = std::vector<Link*>::iterator;

class SearchGraph
{
public:

    SearchGraph();
    ~SearchGraph();

    static constexpr double CELL_SIZE = 0.2;
    virtual void _addCell(SearchCell* cell);

    //Computes A* computed path to reach target identified by targetId, cell identified by sourceId
    //Returns a vector containing list of successive IDs representing the path including sourceId and cellID
    std::vector<int> computePath(int sourceId, int targetId); //A star algortihm to compute optimal path

protected:
    virtual void _graphInit();
    virtual void _describe();


    MapHandler m_mapHandler;
    std::map<int,SearchCell*> m_cellMap; //dynamic graph, easier to handle this way than to store ids in Cells
    int m_cellCtr;

    //Compute A* cost of reachedCell coming from sourceCell and heading to targetCell
    //Updates reachedCell priority attribute
    //Returns 1 if the examined path is the best path to reach the reachedCell until now
    //Returns 0 otherwise
    static bool computeAStarCost(SearchCell *sourceCell, SearchCell *reachedCell, SearchCell *targetCell);

    //Once target has been reached by A* algorithm, recompute the list of cell Ids that optimally leads to the targetCell
    std::vector<int> retrieveBestPath( int sourceId, int targetId);

};

NAMESPACE_CLOSE();

#endif // SEARCHGRAPH_H


// TODO : extend in a daughter class for dynamic graph handling
