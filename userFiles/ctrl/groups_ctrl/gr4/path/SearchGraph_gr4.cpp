#include "SearchGraph_gr4.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr4);

SearchGraph::SearchGraph() : m_cellCtr(0)
{
    _graphInit();
    printf("Graph for path planning has been successfully initialized.\n");
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
        x += CELL_SIZE;
        for (int j=0; j<columnCellNumber; j++)
        {
            y += CELL_SIZE;
            Cell* cell = new Cell(x, y, CELL_SIZE);
            // collision check
            if ( !m_mapHandler.isOnObstacle(cell) )
                cell->setFree();
            _addCell(cell);
        }
    }

    // linking cells
    for (int i=0; i< rowCellNumber; i++)
    {
        for (int j=0; j<columnCellNumber; j++)
        {
            Cell* cCell = m_cellMap[i+j];
            if (i+j-1>-1)
            {
                Cell* nCell = m_cellMap[i+j-1];
                double length( Point(cCell->x(),cCell->y()).computeDistance(Point(nCell->x(),nCell->y())) );
                Link* link = new Link(i+j-1, length);
            }
            // TODO : continue with adding neighbors ..

        }
    }
}

void SearchGraph::_addCell(Cell *cell)
{
    //TODO : incorporate a free index vector so that index doesn't reach limit (for dynamic graph handling)
    m_cellCtr++;
    m_cellMap[m_cellCtr] = cell;
}


NAMESPACE_CLOSE();
