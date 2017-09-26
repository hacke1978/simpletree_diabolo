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

#ifndef CLUSTERCLOUD_H
#define CLUSTERCLOUD_H

#include "SimpleTree4/model/pointsimpletree.h"
#include "SimpleTree4/math/simplemath.h"


#include <QVector>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>

class ClusterCloud
{
    static const int _MIN_PTS = 3;
public:
    ClusterCloud();


    /**
     * @brief cluster Performs an euclidean clustering
     * @param cloud_in the input cloud
     * @param distance the min distance between two clusters
     * @return All clusters with at least _MIN_PTS points
     */
    static const QVector<PointCloudS::Ptr> cluster(PointCloudS::Ptr cloud_in, float distance);

    /**
     * @brief cluster Performs an euclidean clustering
     * @param cloud_in the input cloud
     * @param distance the min distance between two clusters
     * @return All clusters with at least _MIN_PTS points
     */
    static const QVector<PointCloudS::Ptr> cluster(PointCloudS::Ptr cloud_in, float distance, int min_pts);
};

#endif // CLUSTERCLOUD_H
