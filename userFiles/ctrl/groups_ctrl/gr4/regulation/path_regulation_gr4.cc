#include "path_regulation_gr4.h"
#include "useful_gr4.h"
#include "speed_regulation_gr4.h"
#include "time.h"

NAMESPACE_INIT(ctrlGr4);

PathRegulation *init_path_regulation()
{
    PathRegulation* path_regulation = (PathRegulation*)malloc(sizeof(PathRegulation));
    path_regulation->refPath = new LinePathList();
    path_regulation->reached = false;
    path_regulation->s = 0.;

    clock_gettime(CLOCK_MONOTONIC,&(path_regulation->start));

    return path_regulation;
}

/*! \brief follow a given path
 * 
 * \param[in,out] cvs controller main structure
 */
void follow_path(CtrlStruct *cvs)
{
    PathRegulation* path_reg = cvs->path_reg;
    LinePathList* refPath = path_reg->refPath;



    clock_gettime(CLOCK_MONOTONIC,&(path_reg->endr));
    double dt = path_reg->endr.tv_sec - path_reg->start.tv_sec + (path_reg->endr.tv_nsec-path_reg->start.tv_nsec)/(1000.*1000*1000);
    if (dt<0.01 & dt > 0.)
    {
        if ( !path_reg->reached )
        {
            path_reg->reached = refPath->nextStep(path_reg->s, dt, cvs);
        }
    }
    clock_gettime(CLOCK_MONOTONIC,&(path_reg->start));

}

void free_path_regulation(PathRegulation* pr)
{
    delete(pr->refPath);
    free(pr);
}

void reset(CtrlStruct *cvs)
{
    PathRegulation* path_reg = cvs->path_reg;

    path_reg->refPath->clear();
    path_reg->s = 0.;
}

NAMESPACE_CLOSE();


