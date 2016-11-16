#include "SearchGraph_gr4.h"
#include <queue> //std::priority_queue
#include <algorithm>  //std::reverse



NAMESPACE_INIT(ctrlGr4);

SearchGraph::SearchGraph()
{
    _createMap();
}

SearchGraph::~SearchGraph()
{
    for (SCellIt it = m_cellMap.begin(); it != m_cellMap.end(); it++)
    {
        delete((*it).second);
    }
}

void SearchGraph::addCell(SearchCell *cell)
{
    //TODO : incorporate a free index vector so that index doesn't reach limit
    m_cellMap[m_cellCtr] = cell;
    cell->setId(m_cellCtr);
    m_cellCtr++;
}

void SearchGraph::_createMap()
{
    // creates the static map
}

bool SearchGraph::computeAStarCost(SearchCell *sourceCell, SearchCell *reachedCell, SearchCell *targetCell)
{
    bool res = 0;

    //TODO : Implement heuristic : for now it is a Djikstra algorithm with weight = distance(cm)
    double sourceX = sourceCell->getX();
    double sourceY = sourceCell->getY();
    double reachedX = reachedCell->getX();
    double reachedY = reachedCell->getY();

    int weight = (int)(sourceCell->getWeight() + std::sqrt((reachedX-sourceX)*(reachedX-sourceX)+(reachedY-sourceY)*(reachedY-sourceY))*100);
    //Checking if it is the best path that as been found to reach the reachedCell so far
    //printf("%d, %d\n", weight, reachedCell->getWeight());
    if(weight < reachedCell->getWeight())
    {
        reachedCell->setWeight(weight);
        reachedCell->setPreviousCell(sourceCell);
        res = 1;
    }
    return res;
}


std::vector<int> SearchGraph::retrieveBestPath(int sourceId, int targetId){
    int id = targetId;
    std::vector<int> res;
    res.push_back(id);
    while(id != sourceId){
        id = m_cellMap[id]->getPreviousCell()->getId();
        res.push_back(id);
    }
    std::reverse(res.begin(), res.end());
    return res;
}

 std::vector<int> SearchGraph::computePath(int sourceId, int targetId)
 {
    //initializing priority queue
    std::priority_queue<SearchCell, std::vector<SearchCell*>, CompareSearchCells> priorityQueue;

    //Initializing local variables
    //Cell *cell;
    int id, id1, id2, neighborId;
    SearchCell *sCell, *neighborSCell, *targetSCell;
    std::vector<Link*> neighborLinks;
    std::vector<int> res;
    bool bestPath = 0;

    //We begin by exploring the sourceCell and by putting its weight to 0
    sCell = m_cellMap[sourceId];
    sCell->setWeight(0);
    //We identify the target cell
    targetSCell =m_cellMap[targetId];

    //A-star algorithm until we reach destination
    while(true)
    {
        //Retrieve current cell id
        id = sCell->getId();

        //We mark this cell as already visited so that we will never come back to it
        sCell->setStatus(false);

        //Testing if we have reached the destination
        if(id == targetId){
            //If yes we just have to recompute the optimal path
            break;
        }

        //Iterating through all neighboors of this cell
        //Exploring each neighbour of current cell
        neighborLinks = sCell->getLinks();
        for (LinkIt l_it = neighborLinks.begin(); l_it != neighborLinks.end(); l_it++)
        {

            //Neighboor identification (assuming that a cell is never linked to itself)
            id1 = (*l_it)->getFirstId();
            id2 = (*l_it)->getSecondId();

            if(id1 == id )
            {
                neighborId = id2;
            }else
            {
                neighborId = id1;
            }
            neighborSCell = m_cellMap[neighborId];
            //If there is an obstacle or if it has already been visited we avoid it
            if( !neighborSCell->isOccupied()  || neighborSCell->isOpen())
            {

                bestPath = computeAStarCost(sCell, neighborSCell, targetSCell);
                //Adding neighbor in priorityqueue
                if (bestPath)
                {
                    priorityQueue.push(neighborSCell);
                    bestPath = 0;
                }
            }

        }

        //Taking next Cell to be explored in priorityqueue
        if(!priorityQueue.empty())
        {
            //Getting first priority element
            sCell =  priorityQueue.top();
            //Removing it from the priorityQueue
            priorityQueue.pop();
            //while the popped cells have already been visited coming from a shorter path, we ignore them
            while(sCell->isOpen() && !priorityQueue.empty())
            {
                //Getting first priority element
                sCell =  priorityQueue.top();
                //Removing it from the priorityQueue
                priorityQueue.pop();
            }
            if(!sCell->isOpen()){
                //TODO : handle system error in calling function
                printf("Error : not able to reach target\n");
            }
        }else{
            //TODO : handle system error is calling function
            printf("Error : not able to reach target\n");

        }
    }


    //Target destination has been reached
    //We now recompute the optimal path

    //Attention : we want a result in IDs and not Cell* since the SearchCells will all be deleted
    //right after this line of code
    res = retrieveBestPath(sourceId, targetId);


   return res;

 }

NAMESPACE_CLOSE();

