#include "SearchGraph_gr4.h"
#include <math.h>
#include <queue> //std::priority_queue
#include <algorithm>  //std::reverse
#include <utility> //std::pair
#include "Cell_gr4.h"

NAMESPACE_INIT(ctrlGr4);

SearchGraph::SearchGraph() : m_cellCtr(0)
{
    _graphInit();
    //printf("Graph for path planning has been successfully initialized.\n");
    //_describe(); //used for printing map for matlab
}

SearchGraph::~SearchGraph()
{
    for (SCellIt it = m_cellMap.begin(); it != m_cellMap.end(); it++)
    {
        delete((*it).second);
    }
}

void SearchGraph::_graphInit()
{
    double rowCellNumber    = 1 + ceil( MapHandler::MAP_LENGTH / CELL_SIZE );
    double columnCellNumber = 1 + ceil( MapHandler::MAP_WIDTH / CELL_SIZE );

    double xStart = -0.5*(MapHandler::MAP_LENGTH + CELL_SIZE);
    double yStart = -0.5*(MapHandler::MAP_WIDTH + CELL_SIZE);
    double x(xStart), y(yStart);

    // creating cells
    for (int i=0; i<rowCellNumber; i++)
    {
        y = yStart;
        for (int j=0; j<columnCellNumber; j++)
        {
            SearchCell* cell = new SearchCell(x, y, CELL_SIZE);
            // collision check
            if ( m_mapHandler.isOnObstacle(cell) )
                cell->setOccupied();
            _addCell(cell);
            y += CELL_SIZE;
        }
        x += CELL_SIZE;
    }

    // linking cells
    for (int i=0; i< rowCellNumber; i++)
    {
        for (int j=0; j<columnCellNumber; j++)
        {
            int cellId = i*columnCellNumber+j;
            SearchCell* cCell = m_cellMap[cellId];
            if ( i>0 )
            { // add left horizontal link
                SearchCell* nCell = m_cellMap[cellId-columnCellNumber];
                double length( Point(cCell->x(),cCell->y()).computeDistance(Point(nCell->x(),nCell->y())) );
                Link* link = new Link(cellId-columnCellNumber, length);
                cCell->addLink(link);

                if ( j>0 )
                {// add left down diagonal link
                    nCell = m_cellMap[cellId-columnCellNumber-1];
                    double length( Point(cCell->x(),cCell->y()).computeDistance(Point(nCell->x(),nCell->y())) );
                    Link* link1 = new Link(cellId-columnCellNumber-1, length);
                    cCell->addLink(link1);
                }

                if ( j<columnCellNumber-1 )
                {// add left up diagonal link
                    nCell = m_cellMap[cellId-columnCellNumber+1];
                    double length( Point(cCell->x(),cCell->y()).computeDistance(Point(nCell->x(),nCell->y())) );
                    Link* link2 = new Link(cellId-columnCellNumber+1, length);
                    cCell->addLink(link2);
                }
            }

            if ( i<rowCellNumber-1 )
            {// add right horizontal link
                SearchCell* nCell = m_cellMap[cellId+columnCellNumber];
                double length( Point(cCell->x(),cCell->y()).computeDistance(Point(nCell->x(),nCell->y())) );
                Link* link = new Link(cellId+columnCellNumber, length);
                cCell->addLink(link);

                if ( j>0 )
                {// add right down diagonal link
                    nCell = m_cellMap[cellId+columnCellNumber-1];
                    double length( Point(cCell->x(),cCell->y()).computeDistance(Point(nCell->x(),nCell->y())) );
                    Link* link1 = new Link(cellId+columnCellNumber-1, length);
                    cCell->addLink(link1);
                }
                if ( j<columnCellNumber-1 )
                {// add right down diagonal link
                    nCell = m_cellMap[cellId+columnCellNumber+1];
                    double length( Point(cCell->x(),cCell->y()).computeDistance(Point(nCell->x(),nCell->y())) );
                    Link* link1 = new Link(cellId+columnCellNumber+1, length);
                    cCell->addLink(link1);
                }

            }

            if ( j>0 )
            {// add down link
                SearchCell* nCell = m_cellMap[cellId-1];
                double length( Point(cCell->x(),cCell->y()).computeDistance(Point(nCell->x(),nCell->y())) );
                Link* link = new Link(cellId-1, length);
                cCell->addLink(link);
            }

            if ( j<columnCellNumber-1)
            {// add upper link
                SearchCell* nCell = m_cellMap[cellId+1];
                double length( Point(cCell->x(),cCell->y()).computeDistance(Point(nCell->x(),nCell->y())) );
                Link* link = new Link(cellId+1, length);
                cCell->addLink(link);
            }
        }
    }
}

void SearchGraph::_describe()
{

    FILE* adjFile;
    adjFile = fopen("adjfile.txt","w");

    double rowCellNumber    = 1 + ceil( MapHandler::MAP_LENGTH / CELL_SIZE );
    double columnCellNumber = 1 + ceil( MapHandler::MAP_WIDTH / CELL_SIZE );

    for (int i=0; i< rowCellNumber; i++)
    {
        for (int j=0; j<columnCellNumber; j++)
        {
            int cellId = i*columnCellNumber+j; // current cell Id
            SearchCell* cell = m_cellMap[cellId];
            Cell::OccupancyStatus_t status = cell->status();
            printf("%d,",(int)status);
        }
        printf("\n");
    }


    printf("\n");

    int link;
    for (int i=0; i< m_cellCtr; i++)
    {
        for (int j=0; j<m_cellCtr; j++)
        {
            link = (int)(m_cellMap[i]->isNeighbor(j));
            fprintf(adjFile,"%d,",link);
        }
        fprintf(adjFile,"\n");
    }
    fclose(adjFile);
    getchar();
}

void SearchGraph::_addCell(SearchCell *cell)
{
    //TODO : incorporate a free index vector so that index doesn't reach limit (for dynamic graph handling)
    m_cellMap[m_cellCtr] = cell;
    cell->setId(m_cellCtr);
    m_cellCtr++;
}


bool SearchGraph::_computeAStarCost(SearchCell *sourceCell, SearchCell *reachedCell, SearchCell *targetCell)
{
    bool res = 0;

    //TODO : Implement heuristic : for now it is a Djikstra algorithm with weight = distance(cm)
    double sourceX = sourceCell->x();
    double sourceY = sourceCell->y();
    double reachedX = reachedCell->x();
    double reachedY = reachedCell->y();

    int weight = (int)(sourceCell->getWeight() + std::sqrt((reachedX-sourceX)*(reachedX-sourceX)+(reachedY-sourceY)*(reachedY-sourceY))*100);
    //Checking if it is the best path that as been found to reach the reachedCell so far
    if(weight < reachedCell->getWeight())
    {
        reachedCell->setWeight(weight);
        reachedCell->setPreviousCell(sourceCell);
        res = 1;
    }
    return res;
}


std::vector<int> SearchGraph::_retrieveBestPath(int sourceId, int targetId){
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
    std::priority_queue<SearchCell*, std::vector<SearchCell*>, CompareSearchCells> priorityQueue;

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
        getchar();
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
            neighborId = (*l_it)->goalId();

            neighborSCell = m_cellMap[neighborId];
            //If there is an obstacle or if it has already been visited we avoid it
            if( neighborSCell->status() == Cell::OccupancyStatus_t::free  && neighborSCell->isOpen())
            {

                bestPath = _computeAStarCost(sCell, neighborSCell, targetSCell);

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
            while(!sCell->isOpen() && !priorityQueue.empty())
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
    res = _retrieveBestPath(sourceId, targetId);


   return res;

 }

NAMESPACE_CLOSE();
