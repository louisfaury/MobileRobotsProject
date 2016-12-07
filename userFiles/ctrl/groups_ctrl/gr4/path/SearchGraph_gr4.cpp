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
                Point cLoc(cCell->x(),cCell->y());
                double length(cLoc.computeDistance(Point(nCell->x(),nCell->y())) );
                double angle( atan2(nCell->y() - cCell->y(), nCell->x() - cCell->x()) );
                Link* link = new Link(cellId-columnCellNumber, length, angle, cLoc);
                cCell->addLink(link);

                if ( j>0 )
                {// add left down diagonal link
                    nCell = m_cellMap[cellId-columnCellNumber-1];
                    Point cLoc(cCell->x(),cCell->y());
                    double length(cLoc.computeDistance(Point(nCell->x(),nCell->y())) );
                    double angle( atan2(nCell->y() - cCell->y(), nCell->x() - cCell->x()) );
                    Link* link1 = new Link(cellId-columnCellNumber-1,  length, angle, cLoc);
                    cCell->addLink(link1);
                }

                if ( j<columnCellNumber-1 )
                {// add left up diagonal link
                    nCell = m_cellMap[cellId-columnCellNumber+1];
                    Point cLoc(cCell->x(),cCell->y());
                    double length(cLoc.computeDistance(Point(nCell->x(),nCell->y())) );
                    double angle( atan2(nCell->y() - cCell->y(), nCell->x() - cCell->x()) );
                    Link* link2 = new Link(cellId-columnCellNumber+1,  length, angle, cLoc);
                    cCell->addLink(link2);
                }
            }

            if ( i<rowCellNumber-1 )
            {// add right horizontal link
                SearchCell* nCell = m_cellMap[cellId+columnCellNumber];
                Point cLoc(cCell->x(),cCell->y());
                double length(cLoc.computeDistance(Point(nCell->x(),nCell->y())) );
                double angle( atan2(nCell->y() - cCell->y(), nCell->x() - cCell->x()) );
                Link* link = new Link(cellId+columnCellNumber,  length, angle, cLoc);
                cCell->addLink(link);

                if ( j>0 )
                {// add right down diagonal link
                    nCell = m_cellMap[cellId+columnCellNumber-1];
                    Point cLoc(cCell->x(),cCell->y());
                    double length(cLoc.computeDistance(Point(nCell->x(),nCell->y())) );
                    double angle( atan2(nCell->y() - cCell->y(), nCell->x() - cCell->x()) );
                    Link* link1 = new Link(cellId+columnCellNumber-1,  length, angle, cLoc);
                    cCell->addLink(link1);
                }
                if ( j<columnCellNumber-1 )
                {// add right down diagonal link
                    nCell = m_cellMap[cellId+columnCellNumber+1];
                    Point cLoc(cCell->x(),cCell->y());
                    double length(cLoc.computeDistance(Point(nCell->x(),nCell->y())) );
                    double angle( atan2(nCell->y() - cCell->y(), nCell->x() - cCell->x()) );
                    Link* link1 = new Link(cellId+columnCellNumber+1, length, angle, cLoc);
                    cCell->addLink(link1);
                }

            }

            if ( j>0 )
            {// add down link
                SearchCell* nCell = m_cellMap[cellId-1];
                Point cLoc(cCell->x(),cCell->y());
                double length(cLoc.computeDistance(Point(nCell->x(),nCell->y())) );
                double angle( atan2(nCell->y() - cCell->y(), nCell->x() - cCell->x()) );
                Link* link = new Link(cellId-1,  length, angle, cLoc);
                cCell->addLink(link);
            }

            if ( j<columnCellNumber-1)
            {// add upper link
                SearchCell* nCell = m_cellMap[cellId+1];
                Point cLoc(cCell->x(),cCell->y());
                double length(cLoc.computeDistance(Point(nCell->x(),nCell->y())) );
                double angle( atan2(nCell->y() - cCell->y(), nCell->x() - cCell->x()) );
                Link* link = new Link(cellId+1,  length, angle, cLoc);
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

void SearchGraph::clear()
{
    for(SCellIt it = m_cellMap.begin(); it != m_cellMap.end(); it++)
    {
        (it->second)->reset();
    }
}

void SearchGraph::_addCell(SearchCell *cell)
{
    //TODO : incorporate a free index vector so that index doesn't reach limit (for dynamic graph handling)
    m_cellMap[m_cellCtr] = cell;
    cell->setId(m_cellCtr);
    m_cellCtr++;
}
bool SearchGraph::_isOnOpponent(Cell *cell)
{
    return m_mapHandler.isOnOpponent(cell);
}

bool SearchGraph::isOnOpponent(int id)
{
    return m_mapHandler.isOnOpponent(m_cellMap[id]);
}


int SearchGraph::_computeWeight(int pweight, double dx, double dy, double angle)
{
    int penalty = 0;
    if (angle>EPSILON)
        penalty = SearchGraph::ANGLE_PENALTY;
    return ((int)(pweight + std::sqrt(dx*dx+dy*dy)*100 + penalty));
}

bool SearchGraph::_computeBestDistance(SearchCell *sourceCell, SearchCell *reachedCell)
{
    bool res = 0;

    double dx = sourceCell->x()-reachedCell->x();
    double dy = sourceCell->y()-reachedCell->y();
    double angle = sourceCell->getLink(reachedCell->getId())->angle();
    double dangle = fabs(angle-sourceCell->getPreviousAngle());
    int pweight = sourceCell->getWeight();

    int weight = _computeWeight(pweight, dx, dy, dangle);

    // checking if it is the best path that as been found to reach the reachedCell so far
    if(weight < reachedCell->getWeight())
    {
        reachedCell->setWeight(weight);
        reachedCell->setPreviousCell(sourceCell);
        reachedCell->setPreviousAngle(angle);
        res = 1;
    }
    return res;
}


void SearchGraph::_computeHeuristicalScore(SearchCell *cell, SearchCell *targetCell){
    double X = cell->x();
    double Y = cell->y();
    double targetX = targetCell->x();
    double targetY = targetCell->y();

    int score = (int)(std::sqrt((X-targetX)*(X-targetX)+(Y-targetY)*(Y-targetY))*100);
    cell->setHeuristicalScore(score);

}

void SearchGraph::_retrieveBestPath(int sourceId, int targetId, LinePathList *path)
{
    SearchCell* cCell = m_cellMap[sourceId] ;
    SearchCell* nCell;
    int id = sourceId;
    while(id != targetId)
    {
        nCell = cCell->getPreviousCell();
        id = nCell->getId();
        nCell = cCell->getPreviousCell();
        Link* link = cCell->getLink(id);
        if ( link != nullptr)
            path->addPath(link->line());
        cCell = nCell;
    }
}

bool SearchGraph::computePath(LinePathList *path, int sourceId, int targetId)
{
    bool success(true);

    //initializing priority queue
    std::priority_queue<SearchCell*, std::vector<SearchCell*>, CompareSearchCells> priorityQueue;

    //Initializing local variables
    int id, neighborId;
    SearchCell *sCell, *neighborSCell, *sourceSCell;
    std::vector<Link*> neighborLinks;

    bool bestPath = 0;

    //We begin by exploring the targetCell and by putting its weight to 0
    sCell = m_cellMap[targetId];
    sCell->setWeight(0);
    //TODO : integrate robot position
    sCell->setPreviousAngle(0);
    //We identify the target cell
    sourceSCell =m_cellMap[sourceId];

    //A-star algorithm until we reach destination
    while(true)
    {
        //Retrieve current cell id
        id = sCell->getId(); // TODO : compute from exact position

        //We mark this cell as already visited = closed so that we will never come back to it
        sCell->setStatus(SearchCell::SearchStatus_t::closed_);

        //Testing if we have reached the destination
        if(id == sourceId){
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
            if( neighborSCell->status() == Cell::OccupancyStatus_t::free  && neighborSCell->getStatus() != SearchCell::SearchStatus_t::closed_
                    && !_isOnOpponent(neighborSCell))
            {
                //We compute distance between cells and update neighborSCell weight if it is it's best distance found so far
                _computeBestDistance(sCell, neighborSCell);

                //If is has not been seen before, we compute its heuristical score and put it in the priroityqueue
                if(neighborSCell->getStatus() == SearchCell::SearchStatus_t::new_){
                    _computeHeuristicalScore(neighborSCell, sourceSCell);
                    priorityQueue.push(neighborSCell);
                    neighborSCell->setStatus(SearchCell::SearchStatus_t::open_);
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
            while(sCell->getStatus()!= SearchCell::SearchStatus_t::open_ && !priorityQueue.empty())
            {
                //Getting first priority element
                sCell =  priorityQueue.top();
                //Removing it from the priorityQueue
                priorityQueue.pop();
            }
            if(sCell->getStatus()!= SearchCell::SearchStatus_t::open_){
                printf("Error : not able to reach target1\n");
                success = false;
                break;
            }
        }
        else
        {
            printf("Error : not able to reach target\n");
            success = false;
            break;
        }
    }

    //Target destination has been reached, we now recompute the optimal path
    if (success)
        _retrieveBestPath(sourceId, targetId, path);
    return success;
 }

 bool SearchGraph::findCell(Point loc, int &id)
 {
     int columnCellNumber = 1 + ceil( MapHandler::MAP_WIDTH / CELL_SIZE );
     int correspondingI, correspondingJ;

     correspondingI = (int)round((loc.x()+0.5*(MapHandler::MAP_LENGTH+CELL_SIZE))/CELL_SIZE+EPSILON);

     correspondingJ = (int)round((loc.y()+0.5*(MapHandler::MAP_WIDTH+CELL_SIZE))/CELL_SIZE+EPSILON);
     id = correspondingI*columnCellNumber + correspondingJ;

     return (id<m_cellCtr);
 }

 NAMESPACE_CLOSE();


