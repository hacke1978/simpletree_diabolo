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

#include "extractlowestclusters.h"

QVector<PointCloudS::Ptr> ExtractLowestClusters::get_clusters() const
{
    return clusters;
}

ExtractLowestClusters::ExtractLowestClusters(PointCloudS::Ptr _cloud, float height, float thresh)
{
    float minZ;
    minZ = std::numeric_limits<float>::max();

    for(int i = 0; i< _cloud->points.size(); i++)
    {


        if(minZ > _cloud->points[i].z)
        {
            minZ = _cloud->points[i].z;
        }
    }

    PointCloudS::Ptr _lowest_cloud_1 (new PointCloudS);
    PointCloudS::Ptr _lowest_cloud_2 (new PointCloudS);
    pcl::PassThrough<PointS> pass_1;
    pass_1.setInputCloud(_cloud);
    pass_1.setFilterFieldName("z");
    pass_1.setFilterLimits(minZ, minZ+height);
    pass_1.filter(*_lowest_cloud_1);

//    pcl::PassThrough<PointS> pass_2;
//    pass_2.setInputCloud(_cloud);
//    pass_2.setFilterFieldName("z");
//    pass_2.setFilterLimits((minZ+ height, minZ+2*height));
//    pass_2.filter(_lowest_cloud_2);

//    if(_lowest_cloud_1->points.size()<)


    clusters = ClusterCloud::cluster(_lowest_cloud_1, thresh);

}
