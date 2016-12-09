/*!
 * @file LinePathlist.cpp
 * @author Louis Faury
 * @date 15/11
 */

#include "LinePathList_gr4.h"
#include "CurvePath_gr4.h"
#include <algorithm>
#include "speed_regulation_gr4.h"
#include "SearchGraph_gr4.h"
#include "path_planning_gr4.h"

NAMESPACE_INIT(ctrlGr4);

LinePathList::LinePathList()
{
    m_pathVec.reserve(100);
    m_change = false;
    m_currentPath=0;
}

LinePathList::~LinePathList()
{
    for ( PathVectIt it = m_pathVec.begin(); it != m_pathVec.end(); it++)
        delete(*it);
    clear();
}

void LinePathList::addPath(Path* path)
{
    m_pathVec.push_back(path);
}

bool LinePathList::nextStep(double& s, double dt, CtrlStruct *cvs)
{
    bool end(false);
    double locLength;
    double locS(0.);
    double resS(0.);
    int iter(0);

    if ( s > length()-EPSILON )
    {//we've reached the end of the path list
        end = true;
        speed_regulation(cvs,0.,0.);
    }
    else
    {
        // finding the current path
        PathVectIt it = m_pathVec.begin();
        for (; it != m_pathVec.end(); it++)
        {
            locLength = (*it)->length();
            if ( s < locS + locLength )
            {
                m_currentPath = iter;
                resS = s - locS;
                m_change = (*it)->nextStep(resS, dt, cvs);
                break;
            }
            else
                iter++;
                locS += locLength; //updating local s
        }
        s = locS + resS;
    }

    return end;
}

double LinePathList::length()
{
    double res(0.);

    for (PathVectIt it = m_pathVec.begin(); it != m_pathVec.end(); it++)
    {
        res += (*it)->length();
    }
    return res;
}

void LinePathList::clear()
{
    m_pathVec.clear();
    m_currentPath=0;
}

void LinePathList::reverse()
{
    std::reverse(m_pathVec.begin(),m_pathVec.end());
}

std::vector<int> LinePathList::getPathId()
{
    std::vector<int> res;
    PathVectIt it = m_pathVec.begin()+m_currentPath;
    for(; it!=m_pathVec.end(); it++)
    {
        res.push_back((*it)->getEndId());
    }
    return res;
}

Path* LinePathList::getCurrentPath(){
    return m_pathVec.at(m_currentPath);
}

Segment LinePathList::getCurrentSegment(CtrlStruct *cvs)
{
    Path* currentPath = getCurrentPath();
    SearchGraph* search_graph = cvs->path->searchGraph;
    return search_graph->toSegment(currentPath->getStartId(), currentPath->getEndId());
}


void LinePathList::smooth(double theta, int id)
{
    PathVectIt it1 = m_pathVec.begin();
    PathVectIt it2 = it1 + 1;
    LinePath *curLine, *nextLine;
    double deltaAngle;
    int sign;

    //First we begin by aligning the robot with the first path line
    curLine = (LinePath*)(*it1);
    deltaAngle = MODULOPI(curLine->angle()-theta);

    if ( fabs(deltaAngle)>EPSILON )
    {
        sign = deltaAngle/fabs(deltaAngle);
        CurvePath* curvePath = new CurvePath(fabs(deltaAngle), sign,id, id);
        m_pathVec.insert(it1,curvePath);
        it1++;
        it2++;
    }

    //Then we do the same for all pathLines
    for (; it2 != m_pathVec.end(); it2++)
    {
        curLine = (LinePath*)(*it1);
        nextLine = (LinePath*)(*it2);

        deltaAngle = MODULOPI(nextLine->angle() -curLine->angle());
        if ( fabs(deltaAngle)>EPSILON )
        {
            sign = deltaAngle/fabs(deltaAngle);

            CurvePath* curvePath = new CurvePath(fabs(deltaAngle), sign,  curLine->getEndId(), curLine->getEndId()); // WARNING : dynamic allocation, object will be destroyed when deleting the vector
            m_pathVec.insert(it2,curvePath);
            it2++;
        }
        else
        {
            (*it1)->setEndSpeed(LinePath::MAX_SPEED);
        }
        it1 = it2;
    }


    // speed smoothing
    double endSpeed = 0.;
    for (PathVectRit rit = m_pathVec.rbegin(); rit != m_pathVec.rend(); rit++)
    {
        (*rit)->setEndSpeed(endSpeed);
        double startSpeed = (*rit)->smoothFromEnd(endSpeed);
        endSpeed = startSpeed;
    }

    /*
    for ( PathVectIt it = m_pathVec.begin(); it != m_pathVec.end(); it++)
        (*it)->describe();
    */
}


NAMESPACE_CLOSE();
