#ifndef OPTIMIZATIONFITST_H
#define OPTIMIZATIONFITST_H

#include <QMutexLocker>
#include <QMutex>
#include <QEnableSharedFromThis>
#include <QVector>
#include <QDebug>

#include <SimpleTree4/model/pointsimpletree.h>
#include <SimpleTree4/method/method_coefficients.h>
#include <SimpleTree4/model/tree.h>
#include "SimpleTree4/method/computedistancecylinderscloud.h"
#include <SimpleTree4/method/spherefollowing2.h>

#include "SimpleTree4/model/tree.h"
#include "SimpleTree4/method/optimizationfit.h"
#include "SimpleTree4/model/build_tree/improvefit.h"

#include "SimpleTree4/model/build_tree/buildtree.h"
#include "SimpleTree4/model/build_tree/improvebranchjunctions.h"
#include "SimpleTree4/model/build_tree/improvebymedian.h"
#include "SimpleTree4/model/build_tree/improvefit.h"
#include "SimpleTree4/model/build_tree/removefalsecylinders.h"
#include "SimpleTree4/model/build_tree/reordertree.h"
#include "SimpleTree4/model/build_tree/improvebymerge.h"
#include "SimpleTree4/method/optimizationspherefollowing.h"
#include "SimpleTree4/method/spherefollowingrecursive.h"
#include "SimpleTree4/model/build_tree/improvebyallometry.h"
#include "SimpleTree4/model/build_tree/improvebypipemodel.h"
#include "SimpleTree4/method/point_cloud_operations/computemeanandstandarddeviation.h"
#include "SimpleTree4/model/build_tree/improvedbyadvancedmedian.h"
#include "SimpleTree4/method/computeallometry.h"

class OptimizationFitST: public QObject, QEnableSharedFromThis<OptimizationFitST>
{
    Q_OBJECT

    static QMutex lock;
    PointCloudS::Ptr _cloud;
    MethodCoefficients _coeff;



    QVector<pcl::ModelCoefficients> _cylinder_coeff;

    static float _best_distance;

    bool _is_multithreaded;

    QVector<MethodCoefficients>
    generate_coefficients();

public:

    static
    QSharedPointer<Tree> _tree;

    static MethodCoefficients _coeff_end;

    OptimizationFitST(PointCloudS::Ptr cloud, MethodCoefficients coeff,  QVector<pcl::ModelCoefficients> cylinder_coef, bool is_multithreaded = true);

    void
    update_coeff(MethodCoefficients coeff, float distance, QSharedPointer<Tree> tree);

    void
    optimize();
};

#endif // OPTIMIZATIONFITST_H
