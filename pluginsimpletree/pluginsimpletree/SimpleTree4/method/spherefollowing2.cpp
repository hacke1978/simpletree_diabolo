/****************************************************************************

 Copyright (C) 2016-2017 Jan Hackenberg
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

#include "spherefollowing2.h"


SphereFollowing2::SphereFollowing2(MethodCoefficients coeff, PointCloudS::Ptr cloud, bool subdivide_stem_and_branch_points)
{
    _octree.reset(new pcl::octree::OctreePointCloudSearch<PointS> (SimpleMath<float>::_OCTREE_RESOLUTION));
    _subdivide_stem_and_branch_points = subdivide_stem_and_branch_points;
    _octree->setInputCloud (cloud);
    _octree->addPointsFromInputCloud ();
    _cloud = cloud;
    _coeff = coeff;


}

PointCloudS::Ptr SphereFollowing2::extract_lowest_cluster()
{
    float minX,maxX,minY,maxY,minZ,maxZ;
    minX = std::numeric_limits<float>::max();
    maxX = std::numeric_limits<float>::lowest();
    minY = std::numeric_limits<float>::max();
    maxY = std::numeric_limits<float>::lowest();
    minZ = std::numeric_limits<float>::max();
    maxZ = std::numeric_limits<float>::lowest();

    for(int i = 0; i< _cloud->points.size(); i++)
    {
        if(minX > _cloud->points[i].x)
        {
            minX = _cloud->points[i].x;
        }

        if(maxX < _cloud->points[i].x)
        {
            maxX = _cloud->points[i].x;
        }

        if(minY > _cloud->points[i].y)
        {
            minY = _cloud->points[i].y;
        }

        if(maxY < _cloud->points[i].y)
        {
            maxY = _cloud->points[i].y;
        }

        if(minZ > _cloud->points[i].z)
        {
            minZ = _cloud->points[i].z;
        }

        if(maxZ < _cloud->points[i].z)
        {
            maxZ = _cloud->points[i].z;
        }
    }
    _min.x = minX;
    _max.x = maxX;
    _min.y = minY;
    _max.y = maxY;
    _min.z = minZ;
    _max.z = maxZ;
    PointCloudS::Ptr _lowest_cloud (new PointCloudS);
    pcl::PassThrough<PointS> pass;
    pass.setInputCloud(_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(_min.z, _min.z+_coeff.height_start_sphere);
    pass.filter(*_lowest_cloud);
    if(_lowest_cloud->points.size() > 20)
    {

    }
    else
    {
        _coeff.height_start_sphere = _coeff.height_start_sphere*2;
        _lowest_cloud = extract_lowest_cluster();
    }
    return _lowest_cloud;
}

bool SphereFollowing2::compute_start_sphere()
{
    _sphere_queue_stem.clear();
    _sphere_queue_branch.clear();
    PointCloudS::Ptr lowest_cluster = extract_lowest_cluster();
    if(lowest_cluster->points.size()>=3)
    {
        pcl::ModelCoefficients circle  = CircleFit::general_circle_fit(lowest_cluster,_coeff.ransac_median_factor,_coeff.ransac_circle_type,_coeff.ransac_circle_inlier_distance,_coeff.ransac_iterations);
        circle.values[2] = _min.z;
        _processed_circles.push_back(circle);
        convert_circle_to_sphere(circle);
        _processed_spheres.push_back(circle);
        if(_coeff.optimze_stem)
        {
            _sphere_queue_stem.push_back(circle);
        }
        else
        {
            _sphere_queue_branch.push_back(circle);
        }
        return true;
    }
    return false;

}

void SphereFollowing2::convert_circle_to_sphere(pcl::ModelCoefficients &circle)
{
    float radius = circle.values[3];
    radius = std::max(radius*_coeff.sphere_radius_multiplier,_coeff.min_radius_sphere);
    circle.values[3] = radius;
}

bool SphereFollowing2::contains_stem(PointCloudS::Ptr cloud)
{
    int counter = 0;

    for(int i = 0; i < cloud->points.size(); i++)
    {
        if(cloud->points[i].is_stem)
        {
            counter++;
        }
    }
    if(counter>(cloud->points.size()/2))
    {
        return true;
    }
    return false;
}

void SphereFollowing2::sphere_following()
{

    int total_stem = 0;
    compute_start_sphere();
    while(!_sphere_queue_stem.empty())
    {
        total_stem++;
        _current_sphere = _sphere_queue_stem.dequeue();
        while(_current_sphere.values.size()==4)
        {
            PointCloudS::Ptr surface = SphereSurfaceExtraction::extract_sphere_surface(_octree,_current_sphere,_cloud,_coeff.epsilon_sphere);
            bool is_stem = contains_stem(surface);
            QVector<PointCloudS::Ptr> clusters = ClusterCloud::cluster(surface, _coeff.epsilon_cluster_stem);
            QVectorIterator<PointCloudS::Ptr> it (clusters);
            int fitted_circles = 0;
            while(it.hasNext())
            {
                PointCloudS::Ptr cluster = it.next();
                if(cluster->points.size()>=3)
                {

                    pcl::ModelCoefficients circle = CircleFit::general_circle_fit(cluster,_coeff.ransac_median_factor,_coeff.ransac_circle_type,_coeff.ransac_circle_inlier_distance, _coeff.ransac_iterations);
                    pcl::ModelCoefficients cylinder = CylinderFit::get_cylinder_from_circles(_current_sphere, circle);
                    _cylinders.push_back(cylinder);
                    if(is_stem)
                    {
                        if(fitted_circles ==0)
                        {
                            _processed_circles.push_back(circle);
                            convert_circle_to_sphere(circle);
                            _processed_spheres.push_back(circle);
                            _current_sphere = circle;
                        }
                        else
                        {
                            _processed_circles.push_back(circle);
                            convert_circle_to_sphere(circle);
                            _processed_spheres.push_back(circle);
                            _sphere_queue_stem.push_back(circle);
                        }


                        fitted_circles ++;
                    }
                    else
                    {
                        _processed_circles.push_back(circle);
                        convert_circle_to_sphere(circle);
                        _processed_spheres.push_back(circle);
                        _sphere_queue_branch.push_back(circle);
                    }

                }
            }
            if(fitted_circles == 0)
            {
                _current_sphere.values.clear();
            }






        }
    }


    while(!_sphere_queue_branch.empty())
    {
        _current_sphere = _sphere_queue_branch.dequeue();
        while(_current_sphere.values.size()==4)
        {

            PointCloudS::Ptr surface = SphereSurfaceExtraction::extract_sphere_surface(_octree,_current_sphere,_cloud,_coeff.epsilon_sphere);

            QVector<PointCloudS::Ptr> clusters = ClusterCloud::cluster(surface, _coeff.epsilon_cluster_branch);
            QVectorIterator<PointCloudS::Ptr> it (clusters);
            int fitted_circles = 0;
            while(it.hasNext())
            {
                PointCloudS::Ptr cluster = it.next();
                if(cluster->points.size()>=3)
                {
                    pcl::ModelCoefficients circle = CircleFit::general_circle_fit(cluster,_coeff.ransac_median_factor,_coeff.ransac_circle_type,_coeff.ransac_circle_inlier_distance, _coeff.ransac_iterations);
                    pcl::ModelCoefficients cylinder = CylinderFit::get_cylinder_from_circles(_current_sphere, circle);
                                        _cylinders.push_back(cylinder);
                    if(fitted_circles ==0)
                    {
                        _processed_circles.push_back(circle);
                        convert_circle_to_sphere(circle);
                        _processed_spheres.push_back(circle);
                        _current_sphere = circle;
                    } else
                    {
                        _processed_circles.push_back(circle);
                        convert_circle_to_sphere(circle);
                        _processed_spheres.push_back(circle);
                        _sphere_queue_branch.push_back(circle);
                    }
                    fitted_circles++;
                }
            }
            if(fitted_circles==0)
            {
                _current_sphere.values.clear();
            }
        }
    }
}

PointCloudS::Ptr SphereFollowing2::get_remaining_points()
{
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    float dist = SimpleMath<float>::get_distance(_min,_max);
    _octree->radiusSearch (_min, dist, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    PointCloudS::Ptr remain (new PointCloudS);
    for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
    {
        remain->points.push_back(_cloud->points[ pointIdxRadiusSearch[i]] );
    }
    return remain;
}
