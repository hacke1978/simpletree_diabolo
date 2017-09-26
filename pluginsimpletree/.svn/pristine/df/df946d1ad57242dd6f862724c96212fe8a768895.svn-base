#ifndef SPHEREFOLLOWINGRECURSIVEST_H
#define SPHEREFOLLOWINGRECURSIVEST_H

#include "optimizationspherefollowingst.h"
#include "SimpleTree4/method/geometrical_operations/extractfittedpoints.h"
#include "SimpleTree4/method/point_cloud_operations/clustercloud.h"
#include "SimpleTree4/method/point_cloud_operations/voxelgridfilter.h"
#include <SimpleTree4/model/tree.h>
#include "SimpleTree4/method/computedistancecylinderscloud.h"
#include <SimpleTree4/method/spherefollowing2.h>
#include "SimpleTree4/math/simplemath.h"
#include "optimizationfitst.h"
#include "optimizationdownhillsimplexst.h"

#include <pcl/common/transforms.h>
#include <pcl/common/geometry.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/impl/octree_search.hpp>

#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Core>

#include <QTime>





struct TripletST
{
    PointS attr;
    PointS end;
    float dist;
};


Q_DECLARE_METATYPE(TripletST)

class SphereFollowingRecursiveST : public QObject
{
    Q_OBJECT

    QList<QVariant> _list;

    void
    remove_attractor(PointS &attr);

    void
    initiate_list();

    void
    update_list(PointS &p);

    QPair<PointS, PointS> find_pair();

    PointCloudS::Ptr _cloud;

    MethodCoefficients _coeff;

    pcl::KdTreeFLANN<PointS>::Ptr _kdtree;

    QSharedPointer<pcl::octree::OctreePointCloudSearch<PointS> > _octree;

    PointCloudS::Ptr _end_pts;

    PointCloudS::Ptr _attractors;

    QVector<pcl::ModelCoefficients> _cylinders;

    float _radius = 0.02f;

    bool _subdivide_stem_and_branch_points;

    QSharedPointer<Tree> _tree;

    void find_connection_cylinder();

    Eigen::Vector3f get_connection_vector();

    QPair<PointS, PointS> find_closest_pair(PointCloudS::Ptr cloud_model, PointCloudS::Ptr cloud_attractor);

    PointCloudS::Ptr generate_end_point_cloud(QVector<pcl::ModelCoefficients> coeff);

    PointCloudS::Ptr generate_start_point_cloud(QVector<pcl::ModelCoefficients> coeff);

    bool _is_multithreaded;

public slots:

    void receive_qstring(QString qstr);

    void receive_counter(int counter);

public:

    void do_recursion();

    SphereFollowingRecursiveST(PointCloudS::Ptr cloud, MethodCoefficients coeff, bool subdivide_stem_and_branch_points, bool is_multithreaded = true);

    QVector<pcl::ModelCoefficients> get_cylinders() const;

    MethodCoefficients get_coeff() const;

    void set_coeff(const MethodCoefficients &coeff);

    QSharedPointer<Tree> get_tree();

signals:

    void emit_qstring_spherefollowingrecursive(QString qstr);

    void emit_counter_spherefollowingrecursive(int counter);
};

#endif // SPHEREFOLLOWINGRECURSIVEST_H
