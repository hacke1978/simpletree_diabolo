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

#include "computemeanandstandarddeviation.h"


ComputeMeanAndStandardDeviation::ComputeMeanAndStandardDeviation(PointCloudS::Ptr cloud)
{
    _cloud = cloud;

    _kdtree.reset(new pcl::KdTreeFLANN<PointS>);
    _kdtree->setInputCloud(_cloud);

    for(size_t i = 0; i < _cloud->points.size(); i++)
    {
        PointS p = _cloud->points.at(i);
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        _kdtree->nearestKSearch(p,5,pointIdxRadiusSearch, pointRadiusSquaredDistance);
        std::vector<double> v;
        for(size_t i = 0; i< pointRadiusSquaredDistance.size(); i++)
        {
            v.push_back(std::sqrt(pointRadiusSquaredDistance[i]));
        }



        float sum = std::accumulate(v.begin(), v.end(), 0.0);
        float mean = sum / v.size();
        _average_distances.push_back(mean);

    }
    double sum = std::accumulate(_average_distances.begin(), _average_distances.end(), 0.0);
    _mean = sum / _average_distances.size();
    std::vector<double> diff(_average_distances.size());
    std::transform(_average_distances.begin(), _average_distances.end(), diff.begin(),
                   std::bind2nd(std::minus<double>(), _mean));
    double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    _sd = std::sqrt(sq_sum / _average_distances.size());
}
