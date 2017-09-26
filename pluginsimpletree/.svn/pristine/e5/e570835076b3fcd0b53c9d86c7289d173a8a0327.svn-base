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


#ifndef VOXELGRIDFILTER_H
#define VOXELGRIDFILTER_H

// Define all point types that include XYZ data


#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/filter.h>
#include <pcl/for_each_type.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>



#include <QVector>

#include "SimpleTree4/model/pointsimpletree.h"
#include "SimpleTree4/math/simplemath.h"

class VoxelGridFilter
{
private:
    /**
     * @brief _cloud_in The input cloud
     */
    PointCloudS::Ptr _cloud_in;

    /**
     * @brief _cloud_out The down_sampled cloud
     */
    PointCloudS::Ptr _cloud_out;

    /**
     * @brief _cell_size The desired cell size of the voxel cells
     */
    float _cell_size;

    /**
     * @brief _split_size Before the down_sampling the cloud needs to be split up in cells of width _split_size
     * to avoid an integer overflow, leave to 1m if you do not know
     */
    float _split_size;

    /**
     * @brief _octree An octree to store the input cloud
     */
    pcl::octree::OctreePointCloudSearch<PointS>::Ptr _octree; //you should remove this one and all related stuff if extract_cloud_in_box2 works

    /**
     * @brief extract_sub_clouds Generates a vector of subclouds with cell width _split_size
     * @return
     */
    const QVector<PointCloudS::Ptr> extract_sub_clouds();

    /**
     * @brief extract_cloud_in_box Extracts from _cloud_in the subcloud of the cell denoted by min and max
     * @param min the minPoint of the cell
     * @param max the maxPoint of the cell
     * @return A Sub Point cloud
     */
    PointCloudS::Ptr extract_cloud_in_box(PointS min, PointS max);

    /**
     * @brief extract_cloud_in_box Extracts from _cloud_in the subcloud of the cell denoted by min and max
     * @param min the minPoint of the cell
     * @param max the maxPoint of the cell
     * @return A Sub Point cloud
     */
    PointCloudS::Ptr extract_cloud_in_box2(PointS min, PointS max);

    /**
     * @brief down_sample voxelgrid downsamples all the clouds contained in the vector
     * @param clouds  the vector of clouds
     */
    void down_sample(QVector<PointCloudS::Ptr> clouds);



public:
    /**
     * @brief VoxelGridFilter standard constructor
     * @param cloud the input cloud
     * @param cell_size the desired voxelgrid voxel size
     * @param split_size the size of voxels to produce a vector of subsamples to prevent an integer overflow
     */
    VoxelGridFilter(PointCloudS::Ptr cloud, float cell_size = 0.025f, float split_size = 20.0f);

    /**
     * @brief get_cloud_out the getter for the output
     * @return  the downsampled cloud
     */
    PointCloudS::Ptr get_cloud_out() const;

    /**
     * @brief compute Computes the downscaling routine
     */
    void
    compute();
};

#endif // VOXELGRIDFILTER_H
