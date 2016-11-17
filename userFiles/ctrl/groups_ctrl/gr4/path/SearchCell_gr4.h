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
    bool isOpen(){ return m_status; }
    int getId(){return m_id;}
    SearchCell* getPreviousCell(){return m_previousCell;}
    void setStatus(bool open);
    void setId(int id);
    void setWeight(int weight);
    void setPreviousCell(SearchCell *previousCell);

private:

    int m_id; //Id in searchMap
    int m_weight;//Weight of this cell in the graph search
    bool m_status;//Has this cell already been visited during search (1 = OPEN, 0 = CLOSED)
    SearchCell* m_previousCell;//Pointer to the SearchCell that this cells is reached from giving it its minimum weight
};


//Comparator struct
//Designed for searchPath Weightqueue
struct CompareSearchCells
{
    bool operator()(SearchCell* c1, SearchCell* c2)
    {
        return c1->getWeight() > c2->getWeight();
    }
};

NAMESPACE_CLOSE();


#endif // SEARCHCELL_GR4_H
