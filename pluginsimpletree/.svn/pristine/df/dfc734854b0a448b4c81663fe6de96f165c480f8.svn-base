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

#ifndef POINTDIJKSTRA_H
#define POINTDIJKSTRA_H

#define PCL_NO_PRECOMPILE
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/geometry.h>
#include <pcl/common/impl/common.hpp>
#include <pcl/search/search.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>




struct PointDijkstra
{
    inline PointDijkstra (float _x, float _y, float _z):x(_x),y(_y),z(_z)
    {
    }
    inline PointDijkstra ():x(0),y(0),z(0)
    {
    }




    PCL_ADD_POINT4D;
    float distance;
    float squared_distance;
    bool is_mutable;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointDijkstra,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, distance, distance)
                                   (float, squared_distance, squared_distance)
                                   (bool, is_mutable, is_mutable)
                                   )


typedef PointDijkstra PointD;
typedef pcl::PointCloud<PointD> PointCloudD;

#endif // POINTDIJKSTRA_H
