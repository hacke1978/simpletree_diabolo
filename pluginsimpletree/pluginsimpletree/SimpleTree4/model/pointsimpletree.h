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






#ifndef POINTSIMPLETREE_H
#define POINTSIMPLETREE_H

#define PCL_NO_PRECOMPILE
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/impl/convex_hull.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/impl/convex_hull.hpp>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/impl/concave_hull.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/impl/statistical_outlier_removal.hpp>

#include <pcl/filters/shadowpoints.h>
#include <pcl/filters/impl/shadowpoints.hpp>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/impl/radius_outlier_removal.hpp>

#include <pcl/surface/mls.h>
#include <pcl/surface/impl/mls.hpp>



#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/impl/normal_3d_omp.hpp>

#include <pcl/common/geometry.h>
#include <pcl/common/impl/common.hpp>


#include <pcl/search/search.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>

#include<pcl/search/organized.h>
#include<pcl/search/impl/organized.hpp>




struct PointSimpleTree
{
    inline PointSimpleTree (float _x, float _y, float _z):x(_x),y(_y),z(_z)
    {
    }
    inline PointSimpleTree ():x(0),y(0),z(0)
    {
    }

    bool operator < (const PointSimpleTree & other){
        return true_distance < other.true_distance;
    }


    PCL_ADD_POINT4D;
    PCL_ADD_NORMAL4D;
    float curvature;
    int cluster;
    int rangebin;
    float was_visited;
    float distance;
    float true_distance;
    float is_stem;
    float eigen1;
    float eigen2;
    float eigen3;
    int treeID;
    int ID;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointSimpleTree,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, normal_x, normal_x)
                                   (float, normal_y, normal_y)
                                   (float, normal_z, normal_z)
                                   (float, curvature, curvature)
                                   (float, is_stem, is_stem)
                                   (float, eigen1, eigen1)
                                   (float, eigen2, eigen2)
                                   (float, eigen3, eigen3)
                                   (int, treeID, treeID)
                                   (int, ID,ID)
                                   (int, cluster, cluster)
                                   (int, rangebin, rangebin)
                                   (float, distance, distance)
                                   (float, true_distance, true_distance)
                                   (float, was_visited, was_visited)
                                   )


typedef PointSimpleTree PointS;
typedef pcl::PointCloud<PointS> PointCloudS;

#endif // POINTSIMPLETREE_H
