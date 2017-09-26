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

#include "detectfalsecylinders.h"

PointCloudS::Ptr DetectFalseCylinders::get_center_point_cloud(QSharedPointer<Tree> tree)
{
    QVector<QSharedPointer<Cylinder> > cylinders = tree->get_all_cylinders();
    QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);
    PointCloudS::Ptr center_point_cloud (new PointCloudS);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cyl = it.next();
        PointS center = cyl->get_center();
        center_point_cloud->push_back(center);
    }
    return center_point_cloud;
}

void DetectFalseCylinders::compute_distances_point_cylinder(PointS p, QSharedPointer<Cylinder> cylinder, float &distance, float &distance_sqrd, float &distance_sqrd_angle)
{
    distance = std::abs(cylinder->dist_to_point(p));
    distance_sqrd = distance*distance;
    Eigen::Vector3f cylinder_axis;
    cylinder_axis[0] = cylinder->values[3];
    cylinder_axis[1] = cylinder->values[4];
    cylinder_axis[2] = cylinder->values[5];
    Eigen::Vector3f point_normal;
    point_normal[0] = p.normal_x;
    point_normal[1] = p.normal_y;
    point_normal[2] = p.normal_z;
    double angle_deg = SimpleMath<float>::angle_between(cylinder_axis,point_normal);
    angle_deg = std::abs(angle_deg -90);
    if(isnan(angle_deg))
    {
        angle_deg = 90;
    }
//    if(angle_deg == 0)
//    {
//        angle_deg = 0.00001;
//    }
    distance_sqrd_angle = distance_sqrd*angle_deg;
}

void DetectFalseCylinders::compute()
{
    PointCloudS::Ptr center_point_cloud = get_center_point_cloud(_tree);
    pcl::KdTreeFLANN<PointS> kdtree_center_cloud;
    kdtree_center_cloud.setInputCloud (center_point_cloud);
    pcl::KdTreeFLANN<PointS> kdtree_cloud;
    kdtree_cloud.setInputCloud (_cloud);
    QVector<QSharedPointer<Cylinder> > cylinders = _tree->get_all_cylinders();
    QVectorIterator<QSharedPointer<Cylinder> > hit(cylinders);
    while(hit.hasNext())

    {
        QSharedPointer<Cylinder> cyl = hit.next();
        PointS center  = cyl->get_center();
        float halfsize = cyl->get_half_size();
        float radius = std::max(halfsize,_MAX_DIST);
        std::vector<int> pointIdxNKNSearch;
        std::vector<float> pointNKNSquaredDistance;
        kdtree_cloud.radiusSearch(center,radius, pointIdxNKNSearch, pointNKNSquaredDistance);
        QVector<float> dist;
        QVector<float> dist_sqrd;
        QVector<float> dist_sqrd_angle;
        int number = 0;
        size_t neighborhood_size = pointIdxNKNSearch.size();
        for(size_t i = 0; i < neighborhood_size; i++)
        {

            PointS neighbor = _cloud->points.at(pointIdxNKNSearch[i]);
            int K = 5;
            std::vector<int> pointIdxNKNSearch_center(K);
            std::vector<float> pointNKNSquaredDistance_center(K);
            kdtree_center_cloud.nearestKSearch(neighbor,K, pointIdxNKNSearch_center, pointNKNSquaredDistance_center);


            float min_dist = std::numeric_limits<float>::max();
            float min_dist_sqrd = std::numeric_limits<float>::max();
            float min_dist_sqrd_angle = std::numeric_limits<float>::max();
            int index = -1;
            for(int j = 0; j < pointIdxNKNSearch_center.size(); j++)
            {
                QSharedPointer<Cylinder> cylinder = cylinders.at(pointIdxNKNSearch_center[j]);
                float distance, distance_sqrd, distance_sqrd_angle;
                compute_distances_point_cylinder(neighbor,cylinder,distance, distance_sqrd, distance_sqrd_angle);
                if(distance < min_dist)
                {
                    min_dist_sqrd_angle = distance_sqrd_angle;
                    min_dist_sqrd = distance_sqrd;
                    min_dist = distance;
                    index = pointIdxNKNSearch_center[j];
                }
            }
            if(index>=0)
            {
                QSharedPointer<Cylinder> nearest_cylinder = cylinders.at(index);
                if(nearest_cylinder == cyl)
                {
                    dist.push_back(min_dist);
                    dist_sqrd.push_back(min_dist_sqrd);
                    dist_sqrd_angle.push_back(min_dist_sqrd_angle);
                    number++;
                }
            }
        }

        float d    = SimpleMath<float>::get_mean(dist);
        float ds   = SimpleMath<float>::get_mean(dist_sqrd);
        float dsa  = SimpleMath<float>::get_mean(dist_sqrd_angle);
        cyl->setMean_distance(d);
        cyl->setMean_distance_sqrd(ds);
        cyl->setMean_distance_sqrd_angle(dsa);
        cyl->setNumber_pts(number);
    }
}




DetectFalseCylinders::DetectFalseCylinders(PointCloudS::Ptr cloud, QSharedPointer<Tree> tree): _cloud(cloud), _tree(tree)
{
    compute();
}
