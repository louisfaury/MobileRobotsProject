#ifndef SEARCHCELL_GR4_H
#define SEARCHCELL_GR4_H

#include "Cell_gr4.h"
#include <utility>

NAMESPACE_INIT(ctrlGr4);

class SearchCell : public Cell
{

public:

    SearchCell();
    ~SearchCell();
    SearchCell(double x, double y, double size);
    SearchCell(Cell *cell, int id);

    // setters and getters
    int getWeight(){ return m_weight; }
    bool notVisited(){ return m_visitedStatus; }
    int getId(){return m_id;}
    SearchCell* getPreviousCell(){return m_previousCell;}
    int getHeuristicalScore(){return m_heuristicalScore;}
    int getScore(){return m_heuristicalScore + m_weight;}
    void setStatus(bool open);
    void setId(int id);
    void setWeight(int weight);
    void setPreviousCell(SearchCell *previousCell);
    void setHeuristicalScore(int);

private:

    int m_id; //Id in searchMap
    int m_weight;//Weight of this cell in the graph search
    bool m_visitedStatus;//Has this cell already been visited during search (1 = OPEN, 0 = CLOSED)
    int m_heuristicalScore;//Cell heuristical score for a star algorithm
    SearchCell* m_previousCell;//Pointer to the SearchCell that this cells is reached from giving it its minimum weight
};


//Comparator struct
//Designed for searchPath Weightqueue
struct CompareSearchCells
{
    bool operator()(SearchCell* c1, SearchCell* c2)
    {
        return c1->getScore() > c2->getScore();
    }
};

NAMESPACE_CLOSE();


#endif // SEARCHCELL_GR4_H
