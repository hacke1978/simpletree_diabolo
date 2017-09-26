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

#include "improvebyattractor.h"

void ImproveByAttractor::remove_attractor(PointS &attr)
{
    int index = -1;
    for(int i = 0; i < _attractors->points.size(); i++)
    {
        if(SimpleMath<float>::are_equal(_attractors->points.at(i),attr))
        {
            index = i;
        }
    }
    std::vector<PointS, Eigen::aligned_allocator<PointS> >::const_iterator it = _attractors->points.begin() + index;
    _attractors->points.erase(it);
    _list.removeAt(index);
}

void ImproveByAttractor::initiate_list()
{
    _list.clear();
    for(int i = 0; i< _attractors->points.size(); i++)
    {
        PointS attr = _attractors->points.at(i);
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        _octree->nearestKSearch(attr,1, pointIdxRadiusSearch, pointRadiusSquaredDistance);
        PointS end_point = _end_pts->points.at(pointIdxRadiusSearch.at(0));
        float dist = std::sqrt(pointRadiusSquaredDistance.at(0));
        Triplet trip;
        trip.attr = attr;
        trip.end  = end_point;
        trip.dist = dist;
        _list.push_back(QVariant::fromValue(trip));
    }
}

PointCloudS::Ptr ImproveByAttractor::generate_end_point_cloud(QVector<pcl::ModelCoefficients> coeff)
{
    PointCloudS::Ptr end_pts (new PointCloudS);
    end_pts->points.resize(coeff.size());
    for(size_t i = 0; i< coeff.size(); i++)
    {
        pcl::ModelCoefficients cf = coeff.at(i);
        float x = cf.values[0] + cf.values[3];
        float y = cf.values[1] + cf.values[4];
        float z = cf.values[2] + cf.values[5];
        PointS p;
        p.x = x;
        p.y = y;
        p.z = z;
        end_pts->points[i] = p;
    }
    return end_pts;
}

PointCloudS::Ptr ImproveByAttractor::generate_attractor_cloud(PointCloudS::Ptr cloud)
{
    pcl::search::KdTree<PointS>::Ptr tree (new pcl::search::KdTree<PointS>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointS> ec;
    ec.setClusterTolerance (_clustering_distance); // 2cm
    ec.setMinClusterSize (_min_pts_cluster);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);
    QVector<PointCloudS::Ptr> clusters;
    int size = cluster_indices.size();
    for(int i = 0; i < size; i++)
    {
        pcl::PointIndices indi = cluster_indices.at(i);

        PointCloudS::Ptr cloud_cluster (new PointCloudS);
        for (std::vector<int>::const_iterator pit = indi.indices.begin (); pit != indi.indices.end (); ++pit)
            cloud_cluster->points.push_back (cloud->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
    }

    PointCloudS::Ptr attractors (new PointCloudS);

    for(int i = 0; i < size; i++)
    {
        PointCloudS::Ptr cluster = clusters.at(i);

        PointS min;
        PointS max;

        SimpleMath<float>::get_min_max_pts(cluster,min,max);
        float dist = SimpleMath<float>::get_distance(min,max);
        if(dist < _min_dist_downscale)
        {
            PointS center = SimpleMath<float>::get_center_of_mass(cluster);
            attractors->points.push_back(center);
        } else {
            PointCloudS::Ptr downscaled (new PointCloudS);
            pcl::VoxelGrid<PointS> sor;
            sor.setInputCloud (cluster);
            sor.setLeafSize (_min_dist_downscale, _min_dist_downscale, _min_dist_downscale);
            sor.filter (*downscaled);
            for(int j = 0; j < downscaled->points.size(); j++)
            {
                attractors->points.push_back(downscaled->points.at(j));
            }
        }
    }


    return attractors;
}

void ImproveByAttractor::update_list(PointS &p)
{
    for(int i = 0; i < _list.size(); i++)
    {
        Triplet trip = _list.at(i).value<Triplet>();
        PointS attr = trip.attr;
        float dist = SimpleMath<float>::get_distance(p,attr);
        if(dist<trip.dist)
        {
            Triplet trip_new;
            trip_new.attr = attr;
            trip_new.end = p;
            trip_new.dist = dist;
            _list.replace(i,QVariant::fromValue(trip_new));
        }

    }
}

QPair<PointS, PointS> ImproveByAttractor::find_pair()
{
    int index = -1;
    float min_dist = std::numeric_limits<float>::max();
    for(int i = 0; i < _list.size(); i++)
    {
        Triplet trip = _list.at(i).value<Triplet>();
        if(trip.dist<min_dist)
        {
            min_dist = trip.dist;
            index =i;
        }
    }
    Triplet trip = _list.at(index).value<Triplet>();
    QPair<PointS,PointS> pair;
    pair.first = trip.end;
    pair.second = trip.attr;
    return pair;
}

MethodCoefficients ImproveByAttractor::get_coeff() const
{
    return _coeff;
}

QVector<pcl::ModelCoefficients> ImproveByAttractor::get_cylinders() const
{
    return _cylinders;
}

ImproveByAttractor::ImproveByAttractor(PointCloudS::Ptr cloud, PointCloudS::Ptr remaining_cloud, MethodCoefficients coeff,
                                       QVector<pcl::ModelCoefficients> cylinders, int min_pts_cluster):
    _cloud(cloud), _coeff(coeff), _cylinders(cylinders), _remaining_cloud(remaining_cloud)
{
    _min_pts_cluster = min_pts_cluster;
    _end_pts.reset(new PointCloudS);
    _attractors.reset(new PointCloudS);
    _end_pts = generate_end_point_cloud(_cylinders);
    PointCloudS::Ptr remaining_points (new PointCloudS);
    remaining_points = _remaining_cloud;
    _attractors = generate_attractor_cloud(remaining_points);
    int attractor_size = _attractors->points.size();
    float percentage = ((float) attractor_size)/((float) (attractor_size + _end_pts->points.size()));
    _octree.reset(new pcl::octree::OctreePointCloudSearch<PointS>(SimpleMath<float>::_OCTREE_RESOLUTION));
    _octree->setInputCloud (_end_pts);
    _octree->addPointsFromInputCloud ();
    initiate_list();
    while(!(_attractors->points.empty()))
    {
        QPair<PointS, PointS> pair = find_pair();
        PointS model = pair.first;
        PointS attractor = pair.second;
        remove_attractor(attractor);
        update_list(attractor);
        float x = attractor.x - model.x;
        float y = attractor.y - model.y;
        float z = attractor.z - model.z;
        PointS between;
        between.x = model.x + (x/2);
        between.y = model.y + (y/2);
        between.z = model.z + (z/2);
        _octree->addPointToCloud(attractor,_end_pts);
        pcl::ModelCoefficients coeff1;
        coeff1.values.resize(7);
        coeff1.values[0] = model.x;
        coeff1.values[1] = model.y;
        coeff1.values[2] = model.z;
        coeff1.values[3] = x/2;
        coeff1.values[4] = y/2;
        coeff1.values[5] = z/2;
        coeff1.values[6] = 0;
        pcl::ModelCoefficients coeff2;
        coeff2.values.resize(7);
        coeff2.values[0] = between.x;
        coeff2.values[1] = between.y;
        coeff2.values[2] = between.z;
        coeff2.values[3] = x/2;
        coeff2.values[4] = y/2;
        coeff2.values[5] = z/2;
        coeff2.values[6] = 0;
        _cylinders.push_back(coeff1);
        _cylinders.push_back(coeff2);
    }
}
