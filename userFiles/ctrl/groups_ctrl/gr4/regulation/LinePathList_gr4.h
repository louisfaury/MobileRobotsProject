/*!
 * @file LinePathlist.h
 * @author Louis Faury
 * @date 15/11
 * @brief Container class for Path*
 */

#ifndef PATHLIST_H
#define PATHLIST_H

#include "namespace_ctrl.h"
#include "CtrlStruct_gr4.h"
#include "LinePath_gr4.h"
#include "Path_gr4.h"
#include <vector>

NAMESPACE_INIT(ctrlGr4);

typedef std::vector<Path*>::iterator PathVectIt;
typedef std::vector<int>::iterator IntVectIt;
typedef std::vector<Path*>::reverse_iterator PathVectRit;

class LinePathList
{
public:
    LinePathList();
    ~LinePathList();

    /*!
     * \brief addPath : push_back on the container
     * \param path
     */
    void    addPath(Path* path);
    /*!
     * \brief nextStep
     * \param s : reference to the current curvilinear abscissis
     * \param dt : scheduler timing
     * \param cvs : ptr to main controller
     * \return : true if path has ended
     */
    bool    nextStep(double& s, double dt, CtrlStruct* cvs);
    /*!
     * \brief length
     * \return path total length
     */
    double  length();
    /*!
     * \brief clear : clears the container
     */
    void    clear();
    /*!
     * \brief smooth :  smoothes the trajectory from the end (speed targets) to make sure robot is able to stop (0 speed at turns)
     * \param theta : current position theta angle
     * \param id : current cell id
     */
    void    smooth(double theta, int id);
    /*! getters !*/
    std::vector<int> getPathId();
    Segment getCurrentSegment(CtrlStruct *cvs);
    bool    isEmpty(){return m_pathVec.empty();}

private:
    std::vector<Path*> m_pathVec;
    int m_currentPath;
};

NAMESPACE_CLOSE();

#endif // LINEPATHLIST_H
