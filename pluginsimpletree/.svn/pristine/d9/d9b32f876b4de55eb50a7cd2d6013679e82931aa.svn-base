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

#ifndef GENERATESKELETONCLOUD_H
#define GENERATESKELETONCLOUD_H

#include "SimpleTree4/model/pointsimpletree.h"
#include "SimpleTree4/method/point_cloud_operations/bincloud.h"
#include "SimpleTree4/method/point_cloud_operations/clustercloud.h"
#include "dijkstra_coefficients.h"
#include <pcl/common/centroid.h>
#include <QDebug>

class GenerateSkeletonCloud
{
    float _bin_width;

    PointCloudS::Ptr _cloud_in;

    PointCloudS::Ptr _cloud_out;

    PointCloudS::Ptr _skeleton;

    QVector<PointCloudS::Ptr> _bin_clusters;

    QVector<QVector<PointCloudS::Ptr> > _clusters;

    DijkstraCoefficients _coeff;

public:
    void compute();

    GenerateSkeletonCloud(PointCloudS::Ptr cloud_in,  DijkstraCoefficients coeff);

    PointCloudS::Ptr get_cloud_out() const;

    PointCloudS::Ptr get_skeleton() const;
};

#endif // GENERATESKELETONCLOUD_H
