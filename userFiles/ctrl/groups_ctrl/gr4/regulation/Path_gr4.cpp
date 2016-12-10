/*!
 * @file Path_gr4.h
 * @author Louis Faury
 * @date 17/11
 */

#include "Path_gr4.h"

NAMESPACE_INIT(ctrlGr4);

Path::Path()
{
}

Path::Path(double length, int startId, int endId) : m_length(length), m_startId(startId), m_endId(endId)
{
}


NAMESPACE_CLOSE();

