#include "SearchCell_gr4.h"
#include <limits>


NAMESPACE_INIT(ctrlGr4);

SearchCell::SearchCell() : Cell(), m_weight(std::numeric_limits<int>::max()), m_visitedStatus(1), m_id(-1), m_heuristicalScore(-1)
{
}

SearchCell::~SearchCell()
{
}

SearchCell::SearchCell(double x, double y, double size) : Cell(x, y, size), m_weight(std::numeric_limits<int>::max()), m_visitedStatus(1), m_id(-1), m_heuristicalScore(-1)
{
}


SearchCell::SearchCell(Cell *cell, int id) : Cell(*cell), m_weight(std::numeric_limits<int>::max()),m_visitedStatus(1), m_id(id), m_heuristicalScore(-1)
{
}

void SearchCell::setId(int id)
{
    m_id = id;
}

void SearchCell::setWeight(int weight)
{
    m_weight = weight;
}

void SearchCell::setPreviousCell(SearchCell *previousCell)
{
    m_previousCell = previousCell;
}

void SearchCell::setStatus(bool open)
{
    m_visitedStatus = open;
}

void SearchCell::setHeuristicalScore(int score)
{
    m_heuristicalScore = score;
}

NAMESPACE_CLOSE();
