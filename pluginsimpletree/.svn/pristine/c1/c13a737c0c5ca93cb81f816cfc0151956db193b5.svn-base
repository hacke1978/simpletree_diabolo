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

#ifndef IMPROVEBYATTRACTOR_H
#define IMPROVEBYATTRACTOR_H

#include "SimpleTree4/method/geometrical_operations/extractfittedpoints.h"
#include "SimpleTree4/method/point_cloud_operations/clustercloud.h"
#include "SimpleTree4/method/point_cloud_operations/voxelgridfilter.h"

struct Triplet
{
    PointS attr;
    PointS end;
    float dist;
};


Q_DECLARE_METATYPE(Triplet)

class ImproveByAttractor
{
    float _clustering_distance = 0.075f;

    float _min_dist_downscale = 0.2f;

    int _min_pts_cluster = 9;

    QList<QVariant> _list;

    void remove_attractor(PointS &attr);

    void initiate_list();

    PointCloudS::Ptr generate_end_point_cloud(QVector<pcl::ModelCoefficients> coeff);

    PointCloudS::Ptr generate_attractor_cloud(PointCloudS::Ptr cloud);

    void update_list(PointS &p);

    QPair<PointS, PointS> find_pair();

    PointCloudS::Ptr _end_pts;

    PointCloudS::Ptr _attractors;

    MethodCoefficients _coeff;

    PointCloudS::Ptr _cloud;

    PointCloudS::Ptr _remaining_cloud;

    QVector<pcl::ModelCoefficients> _cylinders;

    QSharedPointer<pcl::octree::OctreePointCloudSearch<PointS> > _octree;

public:
    ImproveByAttractor(PointCloudS::Ptr cloud,PointCloudS::Ptr remaining_cloud, MethodCoefficients coeff,
                       QVector<pcl::ModelCoefficients> get_cylinders, int _min_pts_cluster = 9);

    QVector<pcl::ModelCoefficients> get_cylinders() const;

    MethodCoefficients get_coeff() const;
};

#endif // IMPROVEBYATTRACTOR_H
