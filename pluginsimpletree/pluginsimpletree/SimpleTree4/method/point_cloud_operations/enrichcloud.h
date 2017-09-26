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

#ifndef ENRICHCLOUD_H
#define ENRICHCLOUD_H




#include <QThread>
#include <QDebug>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include "SimpleTree4/model/pointsimpletree.h"


class EnrichCloud
{
private:

    /**
     * @brief _cloud_in The cloud to be enriched with curvature information
     */
    PointCloudS::Ptr _cloud_in;

    /**
     * @brief _kdtree The KDtree used for the computation
     */
    pcl::KdTreeFLANN<PointS>::Ptr _kdtree;

    /**
     * @brief _k The K for knn search
     */
    int _k = 15;
    /**
     * @brief _range The radius for radius search
     */
    float _range = 0.03;
    /**
     * @brief _use_knn_search True if knn search is applied, false for radius search
     */
    bool _use_knn_search = true;

    /**
     * @brief compute All methods are wrapped here
     */
    void
    compute();

    /**
     * @brief compute_normals Compute the normals, multithreaded routine if possible
     */
    void
    compute_normals();

public:
    /**
     * @brief EnrichCloud Enriches a cloud with curvature information
     * @param cloud_in the input cloud
     * @param k the number of k nearest neighbors for knn search
     * @param range the range for range search
     * @param use_knn true if knn search is to be used, false for range search
     */
    EnrichCloud(PointCloudS::Ptr cloud_in, int k = 15, float range = 0.03f, bool use_knn = true );



};

#endif // ENRICHCLOUD_H
