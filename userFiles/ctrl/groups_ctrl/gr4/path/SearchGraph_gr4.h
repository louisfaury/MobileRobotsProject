#ifndef SEARCHGRAPH_H
#define SEARCHGRAPH_H

#include "namespace_ctrl.h"
#include "CtrlStruct_gr4.h"
#include "Cell_gr4.h"
#include "SearchCell_gr4.h"
#include <map>

NAMESPACE_INIT(ctrlGr4);

class SearchGraph
{
public:
    using CellIt = std::map<int, Cell*>::iterator;
    using SCellIt = std::map<int, SearchCell*>::iterator;
    using LinkIt = std::vector<Link*>::iterator;

    SearchGraph();
    ~SearchGraph();

    void addCell(SearchCell* cell);

    //Computes A* computed path to reach target identified by targetId, cell identified by sourceId
    //Returns a vector containing list of successive IDs representing the path including sourceId and cellID
    std::vector<int> computePath(int sourceId, int targetId); //A star algortihm to compute optimal path

    static constexpr double CELL_SIZE = 0.1;

private:

    virtual void _createMap(); // called on constructor for dynamic memory allocation

    std::map<int,SearchCell*> m_cellMap; //dynamic graph, easier to handle this way than to store ids in Cells

    int m_cellCtr = {1};

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
