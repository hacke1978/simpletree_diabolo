/****************************************************************************

 Copyright (C) 2016-2017 INRA (Institut National de la Recherche Agronomique, France) and IGN (Institut National de l'information Géographique et forestière, France)
 All rights reserved.

 Contact : jan.hackenberg@posteo.de

 Developers : Jan Hackenberg

 This file is part of Simpletree plugin Version 4 for Computree.

 Simpletree plugin is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Simpletree plugin is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Simpletree plugin.  If not, see <http://www.gnu.org/licenses/>.

*****************************************************************************/


#ifndef SPHEREFOLLOWINGRECURSIVE_H
#define SPHEREFOLLOWINGRECURSIVE_H

#include "optimizationspherefollowing.h"
#include "SimpleTree4/method/geometrical_operations/extractfittedpoints.h"
#include "SimpleTree4/method/point_cloud_operations/clustercloud.h"
#include "SimpleTree4/method/point_cloud_operations/voxelgridfilter.h"
#include "SimpleTree4/math/simplemath.h"
#include "SimpleTree4/method/optimizationfit.h"
#include "SimpleTree4/method/optimizationdownhillsimplex.h"
#include "SimpleTree4/method/optimizationgap.h"
#include "SimpleTree4/model/build_tree/improvebyattractor.h"

#include <pcl/common/transforms.h>
#include <pcl/common/geometry.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/impl/octree_search.hpp>

#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Core>

#include <QTime>


class SphereFollowingRecursive : public QObject
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

    PointCloudS::Ptr _cloud_tree;

    PointCloudS::Ptr _cloud_noise;

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

    SphereFollowingRecursive(PointCloudS::Ptr cloud, PointCloudS::Ptr cloud_noise, MethodCoefficients coeff, bool subdivide_stem_and_branch_points, bool is_multithreaded = true);

    QVector<pcl::ModelCoefficients> get_cylinders() const;

    MethodCoefficients get_coeff() const;

    void set_coeff(const MethodCoefficients &coeff);

    QSharedPointer<Tree> get_tree();

signals:

    void emit_qstring_spherefollowingrecursive(QString qstr);

    void emit_counter_spherefollowingrecursive(int counter);

    void emit_finished();
};

#endif // SPHEREFOLLOWINGRECURSIVE_H
