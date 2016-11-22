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
    path_regulation->last_t = -1000;

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

    double t = cvs->inputs->t;

    double dt = t-path_reg->last_t;
    if (dt<0.1 && dt > EPSILON)
    {
        if ( !path_reg->reached )
        {
            path_reg->reached = refPath->nextStep(path_reg->s, dt, cvs);
        }
        else
            speed_regulation(cvs,0.,0.);
    }
    path_reg->last_t=t;

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


