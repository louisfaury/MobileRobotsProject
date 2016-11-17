/*!
 * @file LinePathlist.cpp
 * @author Louis Faury
 * @date 15/11
 */

#include "LinePathList_gr4.h"

NAMESPACE_INIT(ctrlGr4);

LinePathList::LinePathList()
{
    m_pathVec.reserve(20);
}

LinePathList::~LinePathList()
{
    clear();
}

void LinePathList::addPath(Path* path)
{
    m_pathVec.push_back(path);
}

bool LinePathList::nextStep(double s, double dt, CtrlStruct *cvs)
{
    bool end(false);
    double locLength;
    double locS(0.);
    double resS(0.);

    // finding the current pathLine
    PathVectIt it = m_pathVec.begin();
    for (; it != m_pathVec.end(); it++)
    {
        locLength = (*it)->length();
        if ( s < locS + locLength && s > locS + EPSILON )
        {
            if ( !nextStep(locS, dt, cvs) )
                break;
        }
        else
            locS += locLength;
    }
    resS = s - locLength;

    end = ( fabs( length() - s)<EPSILON );
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
    for (PathVectIt it = m_pathVec.begin(); it != m_pathVec.end(); it++)
        delete(*it);
}

NAMESPACE_CLOSE();

