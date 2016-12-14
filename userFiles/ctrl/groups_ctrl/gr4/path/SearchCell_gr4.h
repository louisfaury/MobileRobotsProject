#ifndef SEARCHCELL_GR4_H
#define SEARCHCELL_GR4_H


#include "namespace_ctrl.h"
#include "Cell_gr4.h"
#include <utility>

NAMESPACE_INIT(ctrlGr4);

class SearchCell : public Cell
{

public:

    enum class SearchStatus_t
    {
        new_,
        open_,
        closed_,
    };

    SearchCell();
    ~SearchCell();
    SearchCell(double x, double y, double size);
    SearchCell(Cell *cell, int id);

    // setters and getters

    int getWeight(){ return m_weight; }
    SearchStatus_t getStatus(){ return m_status; }
    int getId(){return m_id;}
    SearchCell* getPreviousCell(){return m_previousCell;}
    int getHeuristicalScore(){return m_heuristicalScore;}
    double getPreviousAngle(){ return m_previousAngle;}
    void setStatus(SearchStatus_t status);
    void setId(int id);
    void setWeight(int weight);
    void setPreviousCell(SearchCell *previousCell);
    void setHeuristicalScore(int);
    void setPreviousAngle(double angle);

    /**
     * @brief getScore : computes cell score (heuristical score + weight)
     * @return : score
     */
    int getScore(){return m_heuristicalScore + m_weight;}

    /**
     * @brief reset : set weight, heuristical score, previous cell, status to default values
     */
    void reset();

private:

    int m_id; //Id in searchMap
    int m_weight;//Weight of this cell in the graph search
    double m_previousAngle; //angle of the path that links m_previousCell to the current object
    SearchStatus_t m_status;//Has this cell already been visited during search (1 = OPEN, 0 = CLOSED)
    int m_heuristicalScore;//Cell heuristical score for a star algorithm
    SearchCell* m_previousCell;//Pointer to the SearchCell that this cells is reached from giving it its minimum weight
};


//Comparator struct
//Designed for searchPath Weightqueue
struct CompareSearchCells
{

    /**
     * @brief operator () : comparator definition for A_star algorithm priority queue
     * @param c1 : pointer to first cell
     * @param c2 : pointer to second cell
     * @return : true if first cell score is bigger, false otherwise
     */
    bool operator()(SearchCell* c1, SearchCell* c2)
    {
        return c1->getScore() > c2->getScore();
    }
};

NAMESPACE_CLOSE();


#endif // SEARCHCELL_GR4_H
