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

#include "clusterandsort.h"

PointS ClusterAndSort::get_next_point() const
{
    return _next_point;
}

void ClusterAndSort::subdivide_clouds()
{
    _cloud_processed.reset(new PointCloudS);
    _cloud_unprocessed.reset(new PointCloudS);
    _total_size = _cloud_in->points.size();
    for(size_t i = 0; i < _total_size; i++)
    {
        PointS p = _cloud_in->points[i];
        if(p.distance<std::numeric_limits<float>::max())
        {
            _cloud_processed->push_back(p);
        } else {
            _cloud_unprocessed->push_back(p);
        }
    }
    qDebug() << "ClusterAndSort::subdivide_clouds()" << _total_size;
    qDebug() << "ClusterAndSort::subdivide_clouds()" << _cloud_processed->points.size();
    qDebug() << "ClusterAndSort::subdivide_clouds()" << _cloud_unprocessed->points.size();
}

void ClusterAndSort::next_pt()
{
        pcl::search::KdTree<PointS>::Ptr tree (new pcl::search::KdTree<PointS>);
        tree->setInputCloud (_cloud_processed);
        size_t size = _cloud_unprocessed->points.size();
        PointS p;
        float dist = std::numeric_limits<float>::max();
        int index = -1;
        for(size_t i = 0; i < size; i++)
        {
            PointS unprocessed = _cloud_unprocessed->points.at(i);
            std::vector<int> pointIdxNKNSearch(1);
            std::vector<float> pointNKNSquaredDistance(1);
            tree->nearestKSearch(unprocessed,1,pointIdxNKNSearch,pointNKNSquaredDistance);
            float dist_temp = std::sqrt(pointNKNSquaredDistance[0]);
            if(dist>dist_temp)
            {
                dist = dist_temp;
                p = unprocessed;
                index = pointIdxNKNSearch[0];
            }
        }
        PointS parent = _cloud_processed->points[index];
        p.distance = parent.distance;
        p.true_distance = parent.true_distance+dist;
        _next_point = p;

}



ClusterAndSort::ClusterAndSort(PointCloudS::Ptr cloud_in, DijkstraCoefficients coeff): _cloud_in(cloud_in), _coeff(coeff)
{
    subdivide_clouds();
    next_pt();
}
