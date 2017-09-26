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

#include "generateskeletoncloud.h"

PointCloudS::Ptr GenerateSkeletonCloud::get_cloud_out() const
{
    return _cloud_out;
}

PointCloudS::Ptr GenerateSkeletonCloud::get_skeleton() const
{
    return _skeleton;
}

void GenerateSkeletonCloud::compute()
{
    BinCloud bc (_cloud_in, _coeff.bin_width);
    _bin_clusters = bc.get_clusters();

    QVectorIterator<PointCloudS::Ptr> it(_bin_clusters);
    while(it.hasNext())
    {
        PointCloudS::Ptr bin_cluster = it.next();
        QVector<PointCloudS::Ptr> sub_clusters = ClusterCloud::cluster(bin_cluster,_coeff.cluster_for_bin,1);
        _clusters.push_back(sub_clusters);
    }
    int cluster_ID = 0;
    _cloud_out.reset(new PointCloudS);
    _skeleton.reset(new PointCloudS);
    for(size_t i = 0; i < _clusters.size(); i++)
    {
        QVector<PointCloudS::Ptr> sub_clusters = _clusters[i];
        for(size_t j = 0; j < sub_clusters.size(); j++)
        {
            PointCloudS::Ptr cluster = sub_clusters[j];
            size_t size = cluster->points.size();
            for(size_t k = 0; k < size; k++)
            {
                cluster->points[k].cluster = cluster_ID;
                _cloud_out->push_back(cluster->points[k]);
            }
            Eigen::Matrix<float,4,1> centroid;
            pcl::compute3DCentroid(*cluster,centroid);
            PointS center;
            center.x = centroid.coeff(0);
            center.y = centroid.coeff(1);
            center.z = centroid.coeff(2);
            center.ID = cluster->points[0].ID;
            center.rangebin = cluster->points[0].rangebin;
            center.treeID = cluster->points[0].treeID;
            center.cluster = cluster->points[0].cluster;
            center.ID = cluster->points[0].ID;
            center.distance = cluster->points[0].distance;
            cluster_ID++;
            _skeleton->push_back(center);
        }
    }
}

GenerateSkeletonCloud::GenerateSkeletonCloud(PointCloudS::Ptr cloud_in, DijkstraCoefficients coeff):
    _cloud_in(cloud_in), _coeff(coeff)
{
    compute();
}
