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

#include "predictstablevolume.h"

MethodCoefficients PredictStableVolume::get_coeff() const
{
    return _coeff;
}

void PredictStableVolume::predict_height()
{
    float height = 0;

    PointS lowest;
    PointS heighest;

    pcl::getMinMax3D(*_cloud, lowest, heighest);

    height = heighest.z - lowest.z;

    _minZ = lowest.z;

    _coeff.tree_height = height;
}

void PredictStableVolume::predict_circumference()
{
    PointCloudS::Ptr lowest_cluster = extract_low_cluster();
    PointCloudS::Ptr filtered_cluster = filter_low_cluster(lowest_cluster);
    float rad = predict_cylinder_radius(filtered_cluster);
    float cf   = get_circumference_from_radius(rad);
    _coeff.tree_circumference = cf;
}

void PredictStableVolume::predict_volume()
{
    float volume = 0;
    volume = 0.496*((_coeff.tree_height*_coeff.tree_circumference*_coeff.tree_circumference)/(4*SimpleMath<float>::_PI*(1-(1.3f/_coeff.tree_height) )*(1-(1.3f/_coeff.tree_height) ) ));
    _coeff.tree_predicted_volume = volume;
}

PointCloudS::Ptr PredictStableVolume::extract_low_cluster()
{
    PointCloudS::Ptr output (new PointCloudS);
    pcl::PassThrough<PointS> pass;
    pass.setInputCloud (_cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (_minZ, (_minZ+ 0.1f) );
    pass.filter (*output);
    return output;
}

PointCloudS::Ptr PredictStableVolume::filter_low_cluster(PointCloudS::Ptr cloud_unfiltered)
{
    PointCloudS::Ptr filtered (new PointCloudS);
    for(size_t i = 0; i < cloud_unfiltered->points.size(); i++)
    {
        PointS p = cloud_unfiltered->points[i];
        Eigen::Vector3f normal;
        normal(0) = p.normal_x;
        normal(1) = p.normal_y;
        normal(2) = p.normal_z;
        Eigen::Vector3f z_axis;
        z_axis(0) = 0;
        z_axis(1) = 0;
        z_axis(2) = 1;
        float angle = SimpleMath<float>::angle_between(normal,z_axis);
        if(!(angle < _coeff.tree_max_angle))
        {
            filtered->points.push_back(p);
        }
    }
    return filtered;
}

float PredictStableVolume::predict_cylinder_radius(PointCloudS::Ptr cloud)
{
    float radius = 0;
    pcl::ModelCoefficients coeff;
    pcl::SACSegmentationFromNormals<PointS, PointS> seg;
    seg.setOptimizeCoefficients ( true );
    seg.setModelType ( pcl::SACMODEL_CYLINDER );
    seg.setMethodType (_coeff.ransac_type);
    seg.setNormalDistanceWeight (0 );
    seg.setMaxIterations ( 100 );
    seg.setDistanceThreshold ( _coeff.ransac_inlier_distance );
    seg.setRadiusLimits ( 0, 2.2 );
    seg.setInputCloud ( cloud );
    seg.setInputNormals ( cloud );
    pcl::PointIndices::Ptr inliers_cylinder ( new pcl::PointIndices );
    seg.segment ( *inliers_cylinder, coeff );
    if(coeff.values.size()!=7)
    {
       // qDebug() << "critical error in predictstablevolume cylinder fit " << cloud->points.size();
    }
    else
    {
        radius = coeff.values[6];
    }
    return radius;
}

float PredictStableVolume::get_circumference_from_radius(float radius)
{
    float radius_cm = radius *1;
    float cf = 2*radius_cm*SimpleMath<float>::_PI;
    return cf;
}

PredictStableVolume::PredictStableVolume(PointCloudS::Ptr cloud, MethodCoefficients coeff):_cloud(cloud), _coeff(coeff)
{
    predict_height();
    predict_circumference();
    predict_volume();

}
