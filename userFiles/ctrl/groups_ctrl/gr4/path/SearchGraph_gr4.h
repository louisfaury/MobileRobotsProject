#ifndef SEARCHGRAPH_H
#define SEARCHGRAPH_H

#include "namespace_ctrl.h"
#include "CtrlStruct_gr4.h"
#include "Cell_gr4.h"
#include "SearchCell_gr4.h"
#include "MapHandler_gr4.h"
#include "LinePathList_gr4.h"
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

    /*!
     * @function computePath(int,int) : std::vector<int>
     * @brief computes A* computed path to reach target identified by targetId, cell identified by sourceId
     *        fills path vector with ids of cells constituing the path (including sourcecell and targetcell)
     * @return a boolean (true if a path has been found, false otherwise)
     */
    virtual bool computePath(LinePathList* path, int sourceId, int targetId); //A star algortihm to compute optimal path
    /*!
     * @brief findCell : store id from closest cell to location
     * @param loc : Point
     * @param id : int&
     * @return true if succeeded
     */
    virtual bool findCell(Point loc, int& id);

    static constexpr double CELL_SIZE = 0.05; // cell size for map discretisation

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
     * @function _computeBestDistance(..) : bool
     * @brief computes distance travelled from sourceCell and to reach reachedCell
     *        updates reachedCell weight attribute
     * @return 1 if the examined path is the best path to reach the reachedCell until now, 0 otherwise
    */
    virtual bool _computeBestDistance(SearchCell *sourceCell, SearchCell *reachedCell);
    /*!
     * @function _computeHeuristicalScore(..) :void
     * @brief computes a* heuristical score of cell
     *        updates cell heuristicalScore attribute
     * @return 1 if the examined path is the best path to reach the reachedCell until now, 0 otherwise
    */
    virtual void _computeHeuristicalScore(SearchCell *cell, SearchCell *targetCell);

    /*!
     * @function _retriveBestPath(..) : std::vector<int>
     * @brief once target has been reached by A* algorithm, recompute the list of cell Ids that optimally leads to the targetCell
    */
    virtual void _retrieveBestPath( int sourceId, int targetId, LinePathList* path);

    /*!
     * @brief add cell to graph
     */
    virtual void _addCell(SearchCell* cell);

    virtual void _resetGraph();

    MapHandler m_mapHandler;
    std::map<int,SearchCell*> m_cellMap; //dynamic graph, easier to handle this way than to store ids in Cells
    int m_cellCtr;



};

NAMESPACE_CLOSE();

#endif // SEARCHGRAPH_H
