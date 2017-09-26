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

#include "buildtopology.h"

QVector<pcl::ModelCoefficients> BuildTopology::get_skeleton_coeff() const
{
    return _skeleton_coeff;
}

void BuildTopology::initialize()
{
    BinCloud bc(_cloud_in, _coeff.bin_width);
    _bin_clusters = bc.get_clusters();
}

void BuildTopology::compute()
{
    if(_bin_clusters.size()>1)
    {
        size_t size = _bin_clusters.size();
        for(size_t i = 1; i < size; i++)
        {
            PointCloudS::Ptr cluster = _bin_clusters.at(i);
            PointCloudS::Ptr parent_cluster = _bin_clusters.at(i-1);
            for(size_t j = 0; j < cluster->points.size(); j++)
            {
                PointS end = cluster->points.at(j);
                float dist = std::numeric_limits<float>::max();
                PointS start;
                for(size_t k = 0; k < parent_cluster->points.size(); k++)
                {
                    PointS start_cand = parent_cluster->points.at(k);
                    float dist_cand = SimpleMath<float>::get_distance(start_cand,end);
                    if(dist_cand<dist)
                    {
                        dist = dist_cand;
                        start = start_cand;
                    }
                }
                pcl::ModelCoefficients cf;
                cf.values.resize(7);
                cf.values[0] = start.x;
                cf.values[1] = start.y;
                cf.values[2] = start.z;
                cf.values[3] = end.x-start.x;
                cf.values[4] = end.y-start.y;
                cf.values[5] = end.z-start.z;
                cf.values[6] = 0.01;
                _skeleton_coeff.push_back(cf);
            }
        }
    }
    else
    {
        qDebug() << "critical error in BuildTopology::compute()";
    }
}

BuildTopology::BuildTopology(PointCloudS::Ptr cloud_in, DijkstraCoefficients coeff): _cloud_in(cloud_in), _coeff(coeff)
{
    initialize();
    compute();
}
