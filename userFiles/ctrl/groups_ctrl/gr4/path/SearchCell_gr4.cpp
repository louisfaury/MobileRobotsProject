#include "SearchCell_gr4.h"
#include <limits>


NAMESPACE_INIT(ctrlGr4);

SearchCell::SearchCell() : Cell(), m_weight(std::numeric_limits<int>::max()), m_status(new_), m_id(-1), m_heuristicalScore(-1), m_previousAngle(0)
{
}

SearchCell::~SearchCell()
{
}

SearchCell::SearchCell(double x, double y, double size) : Cell(x, y, size), m_weight(std::numeric_limits<int>::max()),m_status(new_), m_id(-1), m_heuristicalScore(-1), m_previousAngle(0)
{
}


SearchCell::SearchCell(Cell *cell, int id) : Cell(*cell), m_weight(std::numeric_limits<int>::max()),m_status(new_), m_id(id), m_heuristicalScore(-1), m_previousAngle(0)
{
}

void SearchCell::reset()
{
    m_weight = std::numeric_limits<int>::max();
    m_status = new_;
    m_heuristicalScore = -1;
    m_previousAngle= 0;
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

void SearchCell::setStatus(SearchStatus_t status)
{
    m_status = status;
}

void SearchCell::setHeuristicalScore(int score)
{
    m_heuristicalScore = score;
}

void SearchCell::setPreviousAngle(double angle)
{
    m_previousAngle = angle;
}

NAMESPACE_CLOSE();
