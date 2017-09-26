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

#include "extractcloudnearmodel.h"

PointCloudS::Ptr ExtractCloudNearModel::get_output_cloud() const
{
    return _output_cloud;
}

bool ExtractCloudNearModel::is_close_to_cylinders(PointS p, std::vector<int> pointIdxRadiusSearch)
{
    bool is_close = false;
    size_t size = pointIdxRadiusSearch.size();
    for(size_t i = 0; i < size; i++)
    {
        QSharedPointer<Cylinder> cylinder = _cylinders.at(i);
        float dist = cylinder->dist_to_point(p);
        if(dist<_max_dist_to_cylinder)
        {
            is_close = true;
        }
    }
    return is_close;
}

void ExtractCloudNearModel::compute_max_distance()
{
    _max_half_length = get_max_halflength();
    _MAX_DIST_TO_ALL_CYLINDERS = _max_half_length + _max_dist_to_cylinder;
}

float ExtractCloudNearModel::get_max_halflength()
{
    float max_length = 0;
    QVectorIterator<QSharedPointer<Cylinder> > it (_cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        float half_length = cylinder->get_half_length();
        if(half_length> max_length)
        {
            max_length = half_length;
        }
    }
    return max_length;
}

PointCloudS::Ptr ExtractCloudNearModel::get_center_point_cloud(QSharedPointer<Tree> tree)
{
    PointCloudS::Ptr center_cloud (new PointCloudS);
    QVector<QSharedPointer<Cylinder> > cylinders = tree->get_all_cylinders();
    QVectorIterator<QSharedPointer<Cylinder> > it (cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        PointS center = cylinder->get_center();
        center_cloud->push_back(center);
    }
    center_cloud->width = center_cloud->points.size();
    center_cloud->height = 1;
    return center_cloud;
}

void ExtractCloudNearModel::extract_points_near_cylinder()
{
    QSharedPointer<pcl::octree::OctreePointCloudSearch<PointS> > octree (new  pcl::octree::OctreePointCloudSearch<PointS> (0.1));
    PointCloudS::Ptr center_cloud = get_center_point_cloud(_tree);
    _output_cloud.reset(new PointCloudS);
    octree->setInputCloud(center_cloud);
    octree->addPointsFromInputCloud();
    size_t size = _input_cloud->points.size();
    compute_max_distance();
    for(size_t i = 0; i < size; i++)
    {
        PointS p = _input_cloud->points.at(i);
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        octree->radiusSearch (p, _MAX_DIST_TO_ALL_CYLINDERS, pointIdxRadiusSearch, pointRadiusSquaredDistance);
        if(is_close_to_cylinders(p,pointIdxRadiusSearch))
        {
            _output_cloud->points.push_back(p);
        }
    }
}

ExtractCloudNearModel::ExtractCloudNearModel(QSharedPointer<Tree> tree, PointCloudS::Ptr cloud): _tree(tree), _input_cloud(cloud)
{
    _cylinders = _tree->get_all_cylinders();
    extract_points_near_cylinder();
}
