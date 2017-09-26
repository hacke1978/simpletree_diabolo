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

#include "computedistancecylinderscloud.h"





QVector<double> ComputeDistanceCylindersCloud::get_distances_double_sqrd() const
{
    return _distances_double_sqrd;
}





float ComputeDistanceCylindersCloud::get_mean_sqrd_dist() const
{
    if(_distance_sqrd>1e-40)
    {
        return _distance_sqrd;
    }
    float x = 1.0+ ((qrand()/100.0))/1000.0;
    qDebug() << "computedistancecylinderscloud the distance" << x << "number cylinders " << _cylinders.size();
    return x;
}

float ComputeDistanceCylindersCloud::get_inliers() const
{
    return _number_inliers;
}

float ComputeDistanceCylindersCloud::get_inliers_per_area()
{
        return _number_inliers;

//    _total_surface_area = 0;
//    QVectorIterator<QSharedPointer<Cylinder> > it(_cylinders);
//    while(it.hasNext())
//    {
//        QSharedPointer<Cylinder> cylinder = it.next();
//        float length = cylinder->get_length();
//        float circumference = SimpleMath<float>::_PI*2*cylinder->get_radius()*length;
//        _total_surface_area += circumference;
//    }
//    if(_total_surface_area==0)
//    {
//        qDebug() << "ComputeDistance_cylinder_cloud ";
//        return 0;
//    } else {
//        return ((float) _number_inliers/_total_surface_area);
//    }
}














void ComputeDistanceCylindersCloud::compute_distances()
{
    double max_unfitted_distance = SimpleMath<float>::_UNFITTED_DISTANCE;
    _distances_double_sqrd.fill(max_unfitted_distance*max_unfitted_distance,_cloud->points.size());
    QSharedPointer<pcl::octree::OctreePointCloudSearch<PointS> > octree(new  pcl::octree::OctreePointCloudSearch<PointS> ( SimpleMath<float>::_OCTREE_RESOLUTION )) ;
    octree->setInputCloud ( _cloud );
    octree->addPointsFromInputCloud ();
    QVectorIterator<QSharedPointer<Cylinder> > it (_cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        std::vector<int> indices = extract_points_near_cylinder( cylinder,  octree);
        for ( size_t i = 0; i < indices.size (); i++ ) {
            PointS point = _cloud->points[indices[i]];
            QSharedPointer<PointS> p (new PointS);
            p->x = point.x;
            p->y = point.y;
            p->z = point.z;

            float dist = cylinder->dist_to_point_ptr(p);
                if(dist<_MAX_DIST && dist>=std::sqrt(_distances_double_sqrd[indices[i]]))
                {
                    _number_inliers ++ ;
                }

                if ( std::abs ( dist*dist ) < std::abs ( _distances_double_sqrd[indices[i]] ) ) {
                    _distances_double_sqrd[indices[i]] = std::abs(dist*dist);
                }

        }
    }
    _distance_sqrd = SimpleMath<double>::get_mean(_distances_double_sqrd);
    _standard_deviation_sqrd = SimpleMath<double>::get_standard_deviation(_distances_double_sqrd);
}




std::vector<int> ComputeDistanceCylindersCloud::extract_points_near_cylinder(QSharedPointer<Cylinder> cylinder, QSharedPointer<pcl::octree::OctreePointCloudSearch<PointS> > octree)
{
    PointCloudS::Ptr output_cloud (new PointCloudS);
    QSharedPointer<PointS> query_point = cylinder->get_center_ptr();
    float cylinder_size = cylinder->get_half_size();
    float radius = qMax<float>(cylinder_size + _MIN_DIST_TO_CYLINDER, cylinder_size* _CYLINDER_SIZE_MULTIPLIER);
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    octree->radiusSearch (*query_point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    output_cloud->points.resize(pointIdxRadiusSearch.size());
    return pointIdxRadiusSearch;

}

ComputeDistanceCylindersCloud::ComputeDistanceCylindersCloud(QVector<pcl::ModelCoefficients> cylinder_coeff, PointCloudS::Ptr cloud, float inlier_distance)
{
    _min_inlier_distance= 5;
    _cylinders.resize(cylinder_coeff.size());
    for(int i =0 ; i< cylinder_coeff.size(); i++)
    {
        QSharedPointer<Cylinder> cylinder (new Cylinder(cylinder_coeff.at(i)));
        _cylinders[i] = cylinder;
    }
    _cloud = cloud;
    compute_distances();
}

ComputeDistanceCylindersCloud::ComputeDistanceCylindersCloud(QSharedPointer<Tree> tree, PointCloudS::Ptr cloud)
{
    _cylinders = tree->get_all_cylinders();
    _cloud = cloud;
    compute_distances();
}



ComputeDistanceCylindersCloud::ComputeDistanceCylindersCloud(QSharedPointer<Cylinder> cylinder, PointCloudS::Ptr cloud)
{
    _cylinder = cylinder;
    _distances_double_sqrd.clear();

    for(size_t i = 0; i < cloud->points.size(); i++)
    {
        PointS point = cloud->points[i];
        QSharedPointer<PointS> p (new PointS);
        p->x = point.x;
        p->y = point.y;
        p->z = point.z;
        double dist = cylinder->dist_to_point_ptr(p);
        _distances_double_sqrd.push_back(dist*dist);
    }

    _distance_sqrd = SimpleMath<double>::get_mean(_distances_double_sqrd);
    _standard_deviation_sqrd = SimpleMath<double>::get_standard_deviation(_distances_double_sqrd);
}




