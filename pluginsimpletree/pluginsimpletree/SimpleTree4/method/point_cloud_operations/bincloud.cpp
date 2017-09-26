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

#include "bincloud.h"

QVector<PointCloudS::Ptr> BinCloud::get_clusters() const
{
    return _clusters;
}

void BinCloud::fill_clusters()
{
    size_t size = _cloud_in->points.size();
    for(size_t i = 0; i < size; i++)
    {
        float dist = _cloud_in->points.at(i).distance;
        int cluster_ID = qFloor(dist/_bin_distance);

        _clusters[cluster_ID]->push_back(_cloud_in->points.at(i));
        _cloud_in->points[i].rangebin = cluster_ID;
    }
}

void BinCloud::generate_empty_clusters()
{
    float max_dist = 0;
    size_t size = _cloud_in->points.size();
    for(size_t i = 0; i < size; i++)
    {
        float dist = _cloud_in->points.at(i).distance;
        if(_cloud_in->points.at(i).distance > max_dist)
        {
            max_dist = _cloud_in->points.at(i).distance;
        }
    }
    int number = qFloor(max_dist/_bin_distance)+1;
    for(int i = 0; i < number; i++)
    {
        PointCloudS::Ptr cloud (new PointCloudS);
        _clusters.push_back(cloud);
    }

}

BinCloud::BinCloud(PointCloudS::Ptr cloud_in, float bin_distance): _cloud_in(cloud_in), _bin_distance(bin_distance)
{
    generate_empty_clusters();
    fill_clusters();
}
