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

#ifndef CLUSTERANDSORT_H
#define CLUSTERANDSORT_H

#include "SimpleTree4/model/pointsimpletree.h"
#include <QVector>
#include <QPair>
#include <QDebug>
#include <pcl/kdtree/kdtree.h>
#include "dijkstra_coefficients.h"

class ClusterAndSort
{
    DijkstraCoefficients _coeff;

    QVector<QPair<pcl::search::KdTree<PointS>::Ptr, PointCloudS::Ptr> > _pairs;

    PointCloudS::Ptr _cloud_in;

    PointCloudS::Ptr _cloud_processed;

    PointCloudS::Ptr _cloud_unprocessed;

    PointS _next_point;

    QVector<PointCloudS::Ptr> _cluster_large;

    pcl::search::KdTree<PointS>::Ptr _tree;

    int _total_size;

    void subdivide_clouds();

    void next_pt();

//    void cluster_cloud();

//    void sort();

public:
    ClusterAndSort(PointCloudS::Ptr cloud_in, DijkstraCoefficients coeff);

    PointS get_next_point() const;
};

#endif // CLUSTERANDSORT_H
