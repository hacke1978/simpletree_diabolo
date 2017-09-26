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

#include "clustercloud.h"

ClusterCloud::ClusterCloud()
{

}



const QVector<PointCloudS::Ptr> ClusterCloud::cluster(PointCloudS::Ptr cloud_in, float distance)
{
    QVector<PointCloudS::Ptr> clusters;
    if(cloud_in->points.size()>2)
    {
        pcl::search::KdTree<PointS>::Ptr tree (new pcl::search::KdTree<PointS>);
        tree->setInputCloud (cloud_in);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointS> ec;
        ec.setClusterTolerance (distance); // 2cm
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud_in);
        ec.extract (cluster_indices);

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            pcl::PointCloud<PointS>::Ptr cloud_cluster (new pcl::PointCloud<PointS>);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                cloud_cluster->points.push_back (cloud_in->points[*pit]); //*
            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            if(cloud_cluster->points.size()>_MIN_PTS)
            {
                clusters.push_back(cloud_cluster);
            }
        }
    }
    return clusters;
}

const QVector<PointCloudS::Ptr> ClusterCloud::cluster(PointCloudS::Ptr cloud_in, float distance, int min_pts)
{
    QVector<PointCloudS::Ptr> clusters;
    if(cloud_in->points.size()>2)
    {
        pcl::search::KdTree<PointS>::Ptr tree (new pcl::search::KdTree<PointS>);
        tree->setInputCloud (cloud_in);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointS> ec;
        ec.setClusterTolerance (distance); // 2cm
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud_in);
        ec.extract (cluster_indices);

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            pcl::PointCloud<PointS>::Ptr cloud_cluster (new pcl::PointCloud<PointS>);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                cloud_cluster->points.push_back (cloud_in->points[*pit]); //*
            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            if(cloud_cluster->points.size()>min_pts)
            {
                clusters.push_back(cloud_cluster);
            }
        }
    }
    return clusters;
}
