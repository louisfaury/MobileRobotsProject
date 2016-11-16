#ifndef SEARCHCELL_GR4_H
#define SEARCHCELL_GR4_H

#include "Cell_gr4.h"
#include <utility>

NAMESPACE_INIT(ctrlGr4);

class SearchCell : public Cell
{

public:

    SearchCell(); //Constructor
    ~SearchCell(); //Destructor

    SearchCell(double x, double y, double size); //Constructor
    SearchCell(Cell *cell, int id); //constructor

    int getWeight(){ return m_weight; }
    bool isOpen(){ return m_status; }
    int getId(){return m_id;}
    SearchCell* getPreviousCell(){return m_previousCell;}
    void setStatus(bool open);
    void setId(int id);
    void setWeight(int weight);
    void setPreviousCell(SearchCell *previousCell);

private:

    //Id in searchMap
    int m_id;
    //Weight of this cell in the graph search
    int m_weight;
    //Has this cell already been visited during search (1 = OPEN, 0 = CLOSED)
    bool m_status;
    //Pointer to the SearchCell that this cells is reached from giving it its minimum weight
    SearchCell* m_previousCell;

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
