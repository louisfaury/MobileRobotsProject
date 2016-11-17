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
    virtual void _addCell(SearchCell* cell); // TODO : go private

    /*!
     * @function computePath(int,int) : std::vector<int>
     * @brief computes A* computed path to reach target identified by targetId, cell identified by sourceId
     * @return a vector containing list of successive IDs representing the path including sourceId and cellID
     */
    std::vector<int> computePath(int sourceId, int targetId); //A star algortihm to compute optimal path

protected:
    /*!
     * @function _graphInit() : void
     * @brief Called on constructor, responsible of dynamic memory allocations
     */
    virtual void _graphInit();
    /*!
     * @function _describe() : void
     * @brief verbose mode for plot and analyse on Matlab
     */
    virtual void _describe();
    /*!
     * @function _computeAStarCost(..) : bool
     * @brief computes A* cost of reachedCell coming from sourceCell and heading to targetCell
     *        updates reachedCell priority attribute
     * @return 1 if the examined path is the best path to reach the reachedCell until now, 0 otherwise
    */
    virtual bool _computeAStarCost(SearchCell *sourceCell, SearchCell *reachedCell, SearchCell *targetCell);

    /*!
     * @function _retriveBestPath(..) : std::vector<int>
     * @brief once target has been reached by A* algorithm, recompute the list of cell Ids that optimally leads to the targetCell
    */
    virtual std::vector<int> _retrieveBestPath( int sourceId, int targetId);

    MapHandler m_mapHandler;
    std::map<int,SearchCell*> m_cellMap; //dynamic graph, easier to handle this way than to store ids in Cells
    int m_cellCtr;

};

NAMESPACE_CLOSE();

#endif // SEARCHGRAPH_H


// TODO : extend in a daughter class for dynamic graph handling
