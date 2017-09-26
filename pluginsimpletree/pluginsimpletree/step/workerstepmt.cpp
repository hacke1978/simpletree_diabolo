#include "workerstepmt.h"

WorkerStepMT::WorkerStepMT(PointCloudS::Ptr cloud, MethodCoefficients coeff, bool subdivide_stem_and_branch_points, int i, ST_StepAbstractModellingMT *step)
{
    _cloud = cloud;
    _coeff = coeff;
    _subdivide_stem_and_branch_points = subdivide_stem_and_branch_points;
    _i = i;
    _step = step;

}

void WorkerStepMT::run()
{
    qDebug() << "worker" << _i;
    QSharedPointer<SphereFollowingRecursive> spherefollowing(new SphereFollowingRecursive (_cloud,_cloud, _coeff, true,false));

    QObject::connect(spherefollowing.data(), SIGNAL(emit_qstring_spherefollowingrecursive(QString)), _step, SLOT(receive_qstring_abstract_modelling(QString)) );
    QObject::connect(spherefollowing.data(), SIGNAL(emit_counter_spherefollowingrecursive(int)), _step, SLOT (receive_counter_abstract_modelling(int)) );
    QObject::connect(spherefollowing.data(), SIGNAL(emit_finished()), _step, SLOT (receive_counter_finished()) );


    spherefollowing->do_recursion();

    _coeff = spherefollowing->get_coeff();
    QVector<pcl::ModelCoefficients> cylinder_coeff = spherefollowing->get_cylinders();

    _step->update_vec(_i,_coeff,cylinder_coeff);

}
