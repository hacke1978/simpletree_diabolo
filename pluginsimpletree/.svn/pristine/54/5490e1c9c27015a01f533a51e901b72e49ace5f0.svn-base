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

#ifndef STEMPOINTDETECTION_H
#define STEMPOINTDETECTION_H

#include "SimpleTree4/model/pointsimpletree.h"
#include "SimpleTree4/method/point_cloud_operations/voxelgridfilter.h"
#include "SimpleTree4/math/simplemath.h"
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>

#include <QVector>

class StemPointDetection
{
private:

    PointCloudS::Ptr _cloud;
    PointCloudS::Ptr _down_scaled_cloud;
    PointCloudS::Ptr _largest_clusters;
    float _min1;
    float _max1;
    float _min2;
    float _max2;
    float _min3;
    float _max3;

    float _min_height_cluster;
    float _max_height_cluster;

    float _min_height_vegetation;

    int _percentage = 0;

    void
    compute_min_height_vegetation();


    float _voxel_size;
    int _number_trees;

    int
    compute_mean_cluster_size(std::vector<pcl::PointIndices> cluster_indices);


    bool
    check_cluster_extension(pcl::PointIndices indices);

    void
    extract_clusters();

    void
    back_scale_stem_points();

    void
    detect_stem_points_by_eigen() const;

    void
    down_scale_cloud();

    void
    detect_stem_points_by_threshold();

    void
    count_percentage();


public:
    StemPointDetection(float min1, float max1, float min2, float max2, float min3, float max3, float voxel_size,
                       PointCloudS::Ptr cloud, int number_of_trees = 1, float min_height_cluster = -100, float max_height_cluster  = 100);

    void
    compute();

    int get_percentage() const;
};

#endif // STEMPOINTDETECTION_H
