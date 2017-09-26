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

#include "improvefit.h"

//const float ImproveFit::_OCTREE_RESOLUTION = 0.02f;
//const int ImproveFit::_MIN_PTS_CYLINDER_IMPROVEMENT = 3;
const float ImproveFit::_MIN_DIST_TO_CYLINDER = 0.05f;
const float ImproveFit::_CYLINDER_SIZE_MULTIPLIER = 1.3f;
//const  float ImproveFit::_MIN_RADIUS_MULTIPLIER = 0.33f;
//const float ImproveFit::_MAX_RADIUS_MULTIPLIER = 3.0f;

const bool ImproveFit::is_perpendicular(PointS p, const QSharedPointer<Cylinder> cylinder)
{
    Eigen::Vector3f v1(p.normal_x,p.normal_y,p.normal_z);
    Eigen::Vector3f v2((cylinder->get_end_ptr()->x-cylinder->get_start_ptr()->x),(cylinder->get_end_ptr()->y-cylinder->get_start_ptr()->y),(cylinder->get_end_ptr()->z-cylinder->get_start_ptr()->z));
    v1.normalize();
    v2.normalize();
    float cos = v1.dot(v2);
    float rad = acos(cos);
    float deg = (rad*180)/SimpleMath<float>::_PI;
    if(deg > 80 && deg < 100)
    {
        return true;
    }
    return false;
}

ImproveFit::ImproveFit(QSharedPointer<Tree> tree, PointCloudS::Ptr cloud, MethodCoefficients coeff)
{
    _tree = tree;
    _cloud = cloud;
    _coeff = coeff;
    _octree.reset (new  pcl::octree::OctreePointCloudSearch<PointS> (SimpleMath<float>::_OCTREE_RESOLUTION));
    _octree->setInputCloud(_cloud);
    _octree->addPointsFromInputCloud();
    improve();
}


QVector<PointCloudS::Ptr> ImproveFit::allocate_segment_points()
{


    QVector<QSharedPointer<Cylinder> > cylinders = _tree->get_all_cylinders();
    QVector<PointCloudS::Ptr> output;
    output.resize(cylinders.size());
    PointCloudS::Ptr center_points (new PointCloudS);
    center_points->points.resize(cylinders.size());

    for(size_t i = 0; i < cylinders.size(); i++)
    {
        PointCloudS::Ptr new_cloud(new PointCloudS);
        output[i] = new_cloud;
    }

    pcl::KdTreeFLANN<PointS> kdtree;
    kdtree.setInputCloud (center_points);

    for(size_t i = 0; i < _cloud->points.size(); i++)
    {
        int k = 6;
        PointS search_point = _cloud->points[i];
        std::vector<int> pointIdxNKNSearch(k);
        std::vector<float> pointNKNSquaredDistance(k);
        kdtree.nearestKSearch (search_point, k, pointIdxNKNSearch, pointNKNSquaredDistance);
        int index = -1;
        float min_dist = std::numeric_limits<float>::max();
        for(int j = 0; j < k; j++)
        {
            QSharedPointer<Cylinder> cylinder = cylinders[pointIdxNKNSearch[j]];
            float dist = SimpleMath<float>::dist_to_line(search_point,*cylinder->get_start_ptr(),*cylinder->get_end_ptr());
            if(dist<min_dist)
            {
                min_dist = dist;
                index = pointIdxNKNSearch[j];
            }
        }
        search_point.ID = index;
        output[index]->points.push_back(search_point);

    }
    return output;
}

QVector<PointCloudS::Ptr> ImproveFit::allocate_points()
{


    QVector<QSharedPointer<Cylinder> > cylinders = _tree->get_all_cylinders();
    QVector<PointCloudS::Ptr> output;
    output.resize(cylinders.size());
    PointCloudS::Ptr center_points (new PointCloudS);
    center_points->points.resize(cylinders.size());

    for(size_t i = 0; i < cylinders.size(); i++)
    {
        PointCloudS::Ptr new_cloud(new PointCloudS);
        output[i] = new_cloud;
        center_points->points[i] = *(cylinders.at(i)->get_center_ptr());
    }

    pcl::KdTreeFLANN<PointS> kdtree;
    kdtree.setInputCloud (center_points);

    for(size_t i = 0; i < _cloud->points.size(); i++)
    {
        int k = 6;
        PointS search_point = _cloud->points[i];
        std::vector<int> pointIdxNKNSearch(k);
        std::vector<float> pointNKNSquaredDistance(k);
        kdtree.nearestKSearch (search_point, k, pointIdxNKNSearch, pointNKNSquaredDistance);
        int index = -1;
        float min_dist = std::numeric_limits<float>::max();
        for(int j = 0; j < k; j++)
        {
            QSharedPointer<Cylinder> cylinder = cylinders[pointIdxNKNSearch[j]];
            float dist = SimpleMath<float>::dist_to_line(search_point,*cylinder->get_start_ptr(),*cylinder->get_end_ptr());
            if(dist<min_dist)
            {
                min_dist = dist;
                index = pointIdxNKNSearch[j];
            }
        }
        search_point.ID = index;
        output[index]->points.push_back(search_point);

    }
    return output;
}

PointCloudS::Ptr ImproveFit::extract_points_near_cylinder(QSharedPointer<Cylinder> cylinder)
{
    PointCloudS::Ptr output_cloud (new PointCloudS);
    QSharedPointer<PointS> query_point = cylinder->get_center_ptr();
    float cylinder_size = cylinder->get_half_size();
    float radius = qMax<float>(cylinder_size+_MIN_DIST_TO_CYLINDER, cylinder_size*_CYLINDER_SIZE_MULTIPLIER);
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    _octree->radiusSearch (*query_point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
    {
        PointS p = _cloud->points.at(pointIdxRadiusSearch.at(i));
        output_cloud->push_back(p);

    }
    return output_cloud;
}

void ImproveFit::improve()
{
    QVector<QSharedPointer<Cylinder> > cylinders = _tree->get_all_cylinders();
    QVectorIterator<QSharedPointer<Cylinder> > it (cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        PointCloudS::Ptr cloud = extract_points_near_cylinder(cylinder);
        if(cylinder->get_segment()->get_mean_radius()>SimpleMath<float>::_MIN_RADIUS_FOR_CYLINDER_IMPROVEMENT)
        {

            CylinderFit::general_cylinder_fit(cloud,cylinder,_coeff.minPts_ransac_branch,_coeff.ransac_type,_coeff.ransac_inlier_distance,_coeff.ransac_iterations);
        }

    }
}

pcl::ModelCoefficients ImproveFit::RANSAC(pcl::PointCloud<PointS>::Ptr cloud)
{
    pcl::ModelCoefficients coeff;
    pcl::SACSegmentationFromNormals<PointS, PointS> seg;
    seg.setOptimizeCoefficients ( true );
    seg.setModelType ( pcl::SACMODEL_CYLINDER );
    seg.setMethodType (_coeff.ransac_type);
    seg.setNormalDistanceWeight ( 0.12 );
    seg.setMaxIterations ( 100 );
    seg.setDistanceThreshold (_coeff.ransac_inlier_distance);
    seg.setRadiusLimits ( 0, 2.2 );
    seg.setInputCloud ( cloud );
    seg.setInputNormals ( cloud );
    pcl::PointIndices::Ptr inliers_cylinder ( new pcl::PointIndices );
    seg.segment ( *inliers_cylinder, coeff );
    return coeff;
}

void ImproveFit::improve_with_RANSAC(QSharedPointer<Cylinder> cylinder, pcl::ModelCoefficients coeff)
{
    QSharedPointer<PointS> start = cylinder->get_start_ptr();
    QSharedPointer<PointS> end  = cylinder->get_end_ptr();

    QSharedPointer<Cylinder> new_cylinder (new Cylinder(coeff));
    QSharedPointer<PointS> start2 =  new_cylinder->projection_on_axis_ptr(start);
    QSharedPointer<PointS> end2 =  new_cylinder->projection_on_axis_ptr(end);

    float x, y, z, x2, y2, z2, r;

    x = start2->x;
    y = start2->y;
    z = start2->z;
    x2 = end2->x - start2->x;
    y2 = end2->y - start2->y;
    z2 = end2->z - start2->z;
    r = coeff.values.at(6);

    cylinder->values.resize(7);
    cylinder->values[0] =  x ;
    cylinder->values[1] =  y ;
    cylinder->values[2] =  z ;
    cylinder->values[3] =  x2 ;
    cylinder->values[4] =  y2 ;
    cylinder->values[5] =  z2 ;
    cylinder->values[6] =  r ;
}

void ImproveFit::improve_with_median(QSharedPointer<Cylinder> cylinder, pcl::PointCloud<PointS>::Ptr cloud)
{
    QVector<float> distances;
    distances.resize(cloud->points.size());
    for(size_t i = 0; i < cloud->points.size(); i++)
    {
        PointS p = (cloud->points[i]);
        QSharedPointer<PointS> pptr (new PointS(p.x,p.y,p.z));
        //        pptr->x = p.x;
        //        pptr->y = p.y;
        //        pptr->z = p.z;

        distances[i] = cylinder->dist_to_axis(pptr);
    }
    float median = SimpleMath<float>::get_median(distances);
    cylinder->set_radius(median);
}


