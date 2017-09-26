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

#include "stempointdetection.h"
#include <QDebug>

int StemPointDetection::get_percentage() const
{
    return _percentage;
}

void StemPointDetection::compute_min_height_vegetation()
{
    float zMin = std::numeric_limits<float>::max();
    for(size_t i = 0; i < _cloud->points.size(); i++)
    {
        if(_cloud->points[i].z < zMin)
        {
            zMin = _cloud->points[i].z;
        }
    }
    _min_height_vegetation = zMin;
}



int StemPointDetection::compute_mean_cluster_size(std::vector<pcl::PointIndices> cluster_indices)
{ QVector<int> sizes;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointIndices indices= *it;
        sizes.push_back(indices.indices.size());
    }
    int mean = SimpleMath<int>::get_mean(sizes);
    return mean;
}

bool StemPointDetection::check_cluster_extension(pcl::PointIndices indices)
{
    bool is_stem_cluster = false;
    float zMin = std::numeric_limits<float>::max();
    float zMax = std::numeric_limits<float>::lowest();
    for(size_t i = 0; i < indices.indices.size(); i++)
    {
        int index = indices.indices.at(i);
        if(_down_scaled_cloud->points[index].z < zMin)
        {
            zMin = _down_scaled_cloud->points[index].z;
        }
        if(_down_scaled_cloud->points[index].z > zMax)
        {
            zMax = _down_scaled_cloud->points[index].z;
        }
        if(zMin < _max_height_cluster && zMax > _min_height_cluster)
        {
            is_stem_cluster = true;
        }
    }
    //return is_stem_cluster;
    return true;
}

void StemPointDetection::extract_clusters()
{
    _largest_clusters.reset(new PointCloudS());

    pcl::search::KdTree<PointS>::Ptr tree (new pcl::search::KdTree<PointS>);
    tree->setInputCloud (_down_scaled_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointS> ec;
    float range = std::sqrt( 3*(_voxel_size)*(_voxel_size));
    ec.setClusterTolerance (range);

    ec.setSearchMethod (tree);
    ec.setInputCloud (_down_scaled_cloud);
    ec.extract (cluster_indices);
    int j = 0;
    size_t size = cluster_indices.size();
    size = std::min<size_t>(size, _number_trees);
    for(size_t i = 0; i < size; i++)
    {

        if(cluster_indices.size()>0)
        {
            pcl::PointIndices indices = cluster_indices.at(i);
            if(check_cluster_extension(indices))
            {
                for (std::vector<int>::const_iterator pit = indices.indices.begin (); pit != indices.indices.end (); ++pit)
                {
                    {
                        _down_scaled_cloud->points[*pit].treeID = j+1;
                        _largest_clusters->points.push_back (_down_scaled_cloud->points[*pit]);
                    }
                }
            }
        }
    }
}

void StemPointDetection::back_scale_stem_points()
{
    float range = std::sqrt((2*_voxel_size)*(2*_voxel_size));
    pcl::octree::OctreePointCloudSearch<PointS> octree (SimpleMath<float>::_OCTREE_RESOLUTION);
    octree.setInputCloud (_cloud);
    octree.addPointsFromInputCloud ();
    size_t size = _largest_clusters->points.size();
    for(int i = 0; i < size; i++)
    {
        if(_largest_clusters->points.at(i).is_stem==1)
        {
            PointS searchPoint = _largest_clusters->points.at(i);
            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            if (octree.radiusSearch (searchPoint, range, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
            {
                for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
                {
                    _cloud->points[ pointIdxRadiusSearch[i] ].is_stem = 1;
                }
            }
        }
    }
}

void StemPointDetection::detect_stem_points_by_eigen() const
{
    size_t size = _cloud->points.size();
    int stem_count = 0;
    int twig_count = 0;
    for(int i = 0; i < size; i++)
    {
        PointS p = _cloud->points.at(i);
        if(p.eigen1>=_min1&&p.eigen1<=_max1&&p.eigen2>=_min2&&p.eigen2<=_max2&&p.eigen3>=_min3&&p.eigen3<_max3)
        {
            _cloud->points.at(i).is_stem = 1;
            stem_count++;
        } else
        {
            twig_count++;
            _cloud->points.at(i).is_stem = 0;
        }
    }
}

void StemPointDetection::down_scale_cloud()
{
    VoxelGridFilter voxel_grid (_cloud,_voxel_size);
    voxel_grid.compute();
    _down_scaled_cloud = voxel_grid.get_cloud_out();
    size_t size = _cloud->points.size();
    for(int i = 0; i < size; i++)
    {
        _cloud->points.at(i).is_stem = 0;
    }
}

void StemPointDetection::detect_stem_points_by_threshold()
{
    int stem_count = 0;
    int twig_count = 0;
    PointCloudS::Ptr down_scaled_cloud_new (new PointCloudS);
    size_t size = _down_scaled_cloud->points.size();
    for(int i = 0; i < size; i++)
    {

        if(_down_scaled_cloud->points.at(i).is_stem<0.5)
        {
            _down_scaled_cloud->points.at(i).is_stem = 0;

            twig_count++;
        }
        else
        {
                 stem_count++;
            _down_scaled_cloud->points.at(i).is_stem = 1;
            down_scaled_cloud_new->push_back(_down_scaled_cloud->points.at(i));
        }
    }
    _down_scaled_cloud = down_scaled_cloud_new;

}

void StemPointDetection::count_percentage()
{
    int size_total = _cloud->points.size();
    int size_stem = 0;
    for(size_t i = 0; i < _cloud->points.size(); i++)
    {
        PointS p = _cloud->points.at(i);
        if(p.is_stem)
        {
            size_stem ++;
        }
    }
    _percentage = (((double) size_stem) / ((double) size_total) *100);

}

StemPointDetection::StemPointDetection(float min1, float max1, float min2, float max2, float min3, float max3, float voxel_size, PointCloudS::Ptr cloud, int number_of_trees, float min_height_cluster, float max_height_cluster)
{
    _cloud = cloud;
    compute_min_height_vegetation();

    _min_height_cluster = min_height_cluster - _min_height_vegetation;
    _max_height_cluster = max_height_cluster - _min_height_vegetation;

    _min1 = min1;
    _max1 = max1;

    _min2 = min2;
    _max2 = max2;

    _min3 = min3;
    _max3 = max3;

    _voxel_size = voxel_size;
    _number_trees = number_of_trees;
}

void StemPointDetection::compute()
{
    detect_stem_points_by_eigen();
    down_scale_cloud();
    detect_stem_points_by_threshold();
    extract_clusters();
    back_scale_stem_points();
    count_percentage();
}
