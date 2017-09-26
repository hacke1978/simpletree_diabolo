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

#ifndef SPHEREFOLLOWING_H
#define SPHEREFOLLOWING_H

#include "SimpleTree4/method/method_coefficients.h"
#include "SimpleTree4/model/pointsimpletree.h"
#include "SimpleTree4/math/simplemath.h"
#include "SimpleTree4/method/point_cloud_operations/clustercloud.h"
#include "SimpleTree4/method/geometrical_operations/circlefit.h"
#include "SimpleTree4/method/geometrical_operations/cylinderfit.h"
#include "SimpleTree4/method/point_cloud_operations/spheresurfaceextraction.h"
#include "SimpleTree4/method/point_cloud_operations/subdividestemandbranchpoints.h"

#include <QString>
#include <QVector>
#include <QQueue>
#include <QSharedPointer>
#include <QDebug>

#include <pcl/ModelCoefficients.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/impl/octree_search.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/common/common.h>



class SphereFollowing2
{
private:


    MethodCoefficients _coeff;
    QVector<pcl::ModelCoefficients> _processed_circles;
    QVector<pcl::ModelCoefficients> _processed_spheres;
    QQueue<pcl::ModelCoefficients> _sphere_queue_stem;
    QQueue<pcl::ModelCoefficients> _sphere_queue_branch;
    pcl::ModelCoefficients _current_sphere;
    QVector<pcl::ModelCoefficients> _cylinders;
    QSharedPointer<pcl::octree::OctreePointCloudSearch<PointS> > _octree;

    PointS _min;
    PointS _max;

    bool _subdivide_stem_and_branch_points;




    PointCloudS::Ptr _cloud;

    PointCloudS::Ptr extract_lowest_cluster();

    bool compute_start_sphere();

    void
    convert_circle_to_sphere(pcl::ModelCoefficients & circle);

    bool
    contains_stem(PointCloudS::Ptr cloud);






public:
    SphereFollowing2(MethodCoefficients coeff, PointCloudS::Ptr cloud, bool subdivide_stem_and_branch_points = false);

    void
    sphere_following();

    PointCloudS::Ptr
    get_remaining_points();


    const QVector<pcl::ModelCoefficients> get_cylinders()
    {
        return _cylinders;
    }

};

#endif // SPHEREFOLLOWING_H
