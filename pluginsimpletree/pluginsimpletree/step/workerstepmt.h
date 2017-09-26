#ifndef WORKERSTEPMT_H
#define WORKERSTEPMT_H

#include <QRunnable>
#include <QSharedPointer>
#include <QVector>
#include <SimpleTree4/method/spherefollowingrecursive.h>
#include <step/st_stepabstractmodellingmt.h>

class ST_StepAbstractModellingMT;

class WorkerStepMT: public QRunnable
{
    int _i;
    PointCloudS::Ptr _cloud;
    MethodCoefficients _coeff;
    bool _subdivide_stem_and_branch_points;
    ST_StepAbstractModellingMT* _step;
public:
    WorkerStepMT(PointCloudS::Ptr cloud, MethodCoefficients coeff, bool subdivide_stem_and_branch_points, int i, ST_StepAbstractModellingMT* step);

    void run();

};

#endif // WORKERSTEPMT_H
