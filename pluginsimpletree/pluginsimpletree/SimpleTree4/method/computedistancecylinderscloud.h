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

#ifndef COMPUTEDISTANCECYLINDERSCLOUD_H
#define COMPUTEDISTANCECYLINDERSCLOUD_H
#include "SimpleTree4/model/pointsimpletree.h"
#include "SimpleTree4/model/cylinder.h"
#include "SimpleTree4/math/simplemath.h"
#include "SimpleTree4/model/tree.h"


#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>

#include <QVector>
#include <QSharedPointer>
#include <QtGlobal>

class ComputeDistanceCylindersCloud
{

    const float _MIN_DIST_TO_CYLINDER = 0.03f;

    const float _CYLINDER_SIZE_MULTIPLIER = 1.1f;

    PointCloudS::Ptr _cloud;

    QVector<QSharedPointer<Cylinder> > _cylinders;

    QSharedPointer<Cylinder> _cylinder;

    QVector<double> _distances_double_sqrd;

 int _number_inliers = 0;

 float _total_surface_area;

 const float _MAX_DIST = 0.015;


    void
    compute_distances();

    void compute_distances_double_sqrd_with_power();

    float _distance = 0;

    float _distance_sqrd = 0;

    float _distance_sqrd_normal = 0;

    float _standard_deviation = 0;

    float _standard_deviation_sqrd = 0;

    float _min_inlier_distance = 30;


    /**
     * @brief extract_points_near_cylinder Returns a point cloud with the points near the cylinder
     * @param cylinder the input cylinder
     * @return the cloud
     */
    std::vector<int> extract_points_near_cylinder(QSharedPointer<Cylinder> cylinder,  QSharedPointer<pcl::octree::OctreePointCloudSearch<PointS> > octree);
public:
    ComputeDistanceCylindersCloud(QVector<pcl::ModelCoefficients> cylinder_coeff, PointCloudS::Ptr cloud, float inlier_distance = 30);

    ComputeDistanceCylindersCloud(QSharedPointer<Tree> tree, PointCloudS::Ptr cloud);

   ComputeDistanceCylindersCloud(QSharedPointer<Cylinder> cylinder, PointCloudS::Ptr cloud);

    QVector<double> get_distances_double_sqrd() const;

    float get_mean_sqrd_dist() const;

    float get_inliers() const;

    float get_inliers_per_area() ;


};

#endif // COMPUTEDISTANCECYLINDERSCLOUD_H
