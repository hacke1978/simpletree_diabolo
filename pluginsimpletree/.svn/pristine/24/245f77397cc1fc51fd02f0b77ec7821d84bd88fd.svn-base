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

#include "extractfittedpoints.h"

ExtractFittedPoints::ExtractFittedPoints(PointCloudS::Ptr cloud, QVector<pcl::ModelCoefficients> coeff)
{
//    ComputeDistanceCylindersCloud cdc (coeff, cloud);
//    QVector<float> distances = cdc.get_distances_float();
//    float max_rad = 0;
//    QVectorIterator<pcl::ModelCoefficients> it(coeff);
//    while(it.hasNext())
//    {
//        pcl::ModelCoefficients cf = it.next();
//        if(cf.values[6]>max_rad)
//        {
//            max_rad = cf.values[6];
//        }

//    }
//    float min_dist = std::min(SimpleMath<float>::_UNFITTED_DISTANCE,max_rad);
//    _cloud_out.reset(new PointCloudS);
//    for(size_t i = 0; i < distances.size(); i++)
//    {
//        if(distances[i]>min_dist)
//        {
//            _cloud_out->points.push_back(cloud->points.at(i));
//        }
//    }

}

PointCloudS::Ptr ExtractFittedPoints::get_cloud_out() const
{
    return _cloud_out;
}


