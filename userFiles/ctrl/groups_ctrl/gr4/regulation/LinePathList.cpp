#include "LinePathList.h"

NAMESPACE_INIT(ctrlGr4);

LinePathList::LinePathList()
{
    m_lineList.reserve(20);
}


void LinePathList::addLine(LinePath line)
{
    m_lineList.push_back(line);
}

bool LinePathList::nextStep(double s, double dt, CtrlStruct *cvs)
{
    bool end(false);
    double locLength;
    double locS(0.);
    double resS(0.);

    // finding the current pathLine
    LineListIt it = m_lineList.begin();
    for (; it != m_lineList.end(); it++)
    {
        locLength = it->length();
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

    for (LineListIt it = m_lineList.begin(); it != m_lineList.end(); it++)
    {
        res += it->length();
    }
    return res;
}

NAMESPACE_CLOSE();
