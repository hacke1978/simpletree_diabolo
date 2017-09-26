#ifndef OPTIMIZATIONSPHEREFOLLOWINGST_H
#define OPTIMIZATIONSPHEREFOLLOWINGST_H

#include <QMutexLocker>
#include <QMutex>
#include <QEnableSharedFromThis>
#include <QVector>

#include <SimpleTree4/model/pointsimpletree.h>
#include <SimpleTree4/method/method_coefficients.h>
#include <SimpleTree4/model/tree.h>
#include <SimpleTree4/method/computedistancecylinderscloud.h>
#include <SimpleTree4/method/spherefollowing2.h>

class OptimizationSphereFollowingST : public QObject, public QEnableSharedFromThis<OptimizationSphereFollowingST>
{
    Q_OBJECT

    static QMutex lock;
    PointCloudS::Ptr _cloud;
    MethodCoefficients _coeff;

    static float _best_distance;

    QVector<MethodCoefficients>
    generate_coefficients();

    bool _subdivide_stem_and_branch_points;
    bool _is_multithreaded;




public:

    static MethodCoefficients _coeff_end;

    OptimizationSphereFollowingST(PointCloudS::Ptr cloud, MethodCoefficients coeff, bool subdivide_stem_and_branch_points, bool is_multithreaded = true);

    void
    update_coeff(MethodCoefficients coeff, float distance);

    void
    optimize();
};

#endif // OPTIMIZATIONSPHEREFOLLOWINGST_H
