#include "SearchGraph_gr4.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr4);

SearchGraph::SearchGraph() : m_cellCtr(0)
{
    _graphInit();
    printf("Graph for path planning has been successfully initialized.\n");
    _describe(); //used for printing map for matlab
}

SearchGraph::~SearchGraph()
{
    for (CellIt it = m_cellMap.begin(); it != m_cellMap.end(); it++)
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
            Cell* cell = new Cell(x, y, CELL_SIZE);
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
            Cell* cCell = m_cellMap[cellId];
            if ( i>0 )
            { // add left horizontal link
                Cell* nCell = m_cellMap[cellId-columnCellNumber];
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
                    Link* link2 = new Link(cellId+rowCellNumber-columnCellNumber+1, length);
                    cCell->addLink(link2);
                }
            }

            if ( i<rowCellNumber-1 )
            {
                Cell* nCell = m_cellMap[cellId+columnCellNumber];
                double length( Point(cCell->x(),cCell->y()).computeDistance(Point(nCell->x(),nCell->y())) );
                Link* link = new Link(cellId+columnCellNumber, length);
                cCell->addLink(link);

                if ( j>0 )
                {// add right down diagonal link
                    nCell = m_cellMap[cellId+columnCellNumber-1];
                    double length( Point(cCell->x(),cCell->y()).computeDistance(Point(nCell->x(),nCell->y())) );
                    Link* link1 = new Link(cellId-rowCellNumber+columnCellNumber-1, length);
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
            {
                Cell* nCell = m_cellMap[cellId-1];
                double length( Point(cCell->x(),cCell->y()).computeDistance(Point(nCell->x(),nCell->y())) );
                Link* link = new Link(cellId-1, length);
                cCell->addLink(link);
            }

            if ( j<columnCellNumber-1)
            {
                Cell* nCell = m_cellMap[cellId+1];
                double length( Point(cCell->x(),cCell->y()).computeDistance(Point(nCell->x(),nCell->y())) );
                Link* link = new Link(cellId+1, length);
                cCell->addLink(link);
            }
        }
    }
}

void SearchGraph::_describe()
{
    double rowCellNumber    = 1 + ceil( MapHandler::MAP_LENGTH / CELL_SIZE );
    double columnCellNumber = 1 + ceil( MapHandler::MAP_WIDTH / CELL_SIZE );

    for (int i=0; i< rowCellNumber; i++)
    {
        for (int j=0; j<columnCellNumber; j++)
        {
            int cellId = i*columnCellNumber+j;
            Cell* cell = m_cellMap[cellId];
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
            printf("%d,",link);
        }
        printf("\n");
    }
}

void SearchGraph::_addCell(Cell *cell)
{
    //TODO : incorporate a free index vector so that index doesn't reach limit (for dynamic graph handling)
    m_cellMap[m_cellCtr] = cell;
    m_cellCtr++;
}


NAMESPACE_CLOSE();
