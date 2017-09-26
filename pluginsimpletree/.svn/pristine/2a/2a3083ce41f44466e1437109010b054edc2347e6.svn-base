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

#include "circlefit.h"
#include <QDebug>

const QVector<float> CircleFit::get_distances(Eigen::Vector4f centroid, PointCloudS::Ptr cloud)
{
    QVector<float> distances;
    distances.resize((int) cloud->points.size());
    for(int i = 0; i < cloud->points.size(); i++)
    {
        float distance;
        float x = cloud->points[i].x-centroid.coeff(0);
        float y = cloud->points[i].y-centroid.coeff(1);
        float z = cloud->points[i].z-centroid.coeff(2);
        distance = std::sqrt(x*x + y*y + z*z);
        distances[i] = distance;
    }
    return distances;
}

PointS CircleFit::get_largest_dist_point(const PointS &p, PointCloudS::Ptr cloud)
{
    PointS q = p;
    //    float max_dist = 0;
    //    for(size_t i = 0; i < cloud->points.size(); i++)
    //    {
    //        PointS cand = cloud->points.at(i);
    //        float dist = SimpleMath<float>::get_distance(p,cand);
    //        if(dist> max_dist)
    //        {
    //            max_dist = dist;
    //            q = cand;
    //        }
    //    }
    return q;
}

PointS CircleFit::check_sphere(const pcl::ModelCoefficients &coeff, PointCloudS::Ptr cloud)
{
    PointS point;
    //    point.x = 0;
    //    point.y = 0;
    //    point.z = 0;
    //    PointS center;
    //    center.x = coeff.values.at(0);
    //    center.y = coeff.values.at(1);
    //    center.z = coeff.values.at(2);
    //    float radius = coeff.values.at(3);
    //    for(size_t i = 0; i < cloud->points.size(); i++)
    //    {
    //        PointS cand = cloud->points.at(i);
    //        float dist = SimpleMath<float>::get_distance(center,cand);
    //        if(dist> radius)
    //        {
    //            max_dist = dist;
    //            q = cand;
    //        }
    //    }

    return point;
}

//pcl::ModelCoefficients CircleFit::ritter_sphere(pcl::PointCloud::Ptr cloud)
//{
//    bool found_sphere = false;
//    PointS p1 = cloud->points.at(0);
//    PointS p2 = get_largest_dist_point(p1, cloud);
//    PointS p3 = get_largest_dist_point(p2, cloud);
//    pcl::ModelCoefficients circle = compute_sphere(p2,p3);
//    PointS out = check_sphere(circle, cloud);
//    found_sphere = check_out(out);
//    while(!found_sphere)
//    {

//    }
//}

bool CircleFit::check_out(const PointS &out)
{
    bool check = true;
    if(out.x!=0)
    {
        check = false;
    }
    if(out.y!=0)
    {
        check = false;
    }
    if(out.z!=0)
    {
        check = false;
    }
    return check;
}

const PointCloudS::Ptr CircleFit::convex_hull(PointCloudS::Ptr cloud_in)
{
    PointCloudS::Ptr hull_cloud(new PointCloudS);
    //    std::vector<pcl::Vertices> polygons;
    //    pcl::ConvexHull<PointS> chull;
    //    chull.setInputCloud(cloud_in);
    //    chull.reconstruct(*hull_cloud, polygons);
    return hull_cloud;
}

const pcl::ModelCoefficients CircleFit::bounding_sphere(PointCloudS::Ptr cloud_in)
{
    PointS p1;
    PointS p2;
    float dist = 0;
    PointCloudS::Ptr hull = cloud_in;
    for(int i = 0; i < hull->points.size()-1; i++)
    {
        for(int j = i+1; j < hull->points.size(); j++)
        {
            PointS p1x = hull->points.at(i);
            PointS p2x = hull->points.at(j);

            float distance = SimpleMath<float>::get_distance(p1x,p2x);
            if(distance>dist)
            {
                dist = distance;
                p1 = p1x;
                p2 = p2x;
            }
        }
    }
    float x = (p1.x + p2.x)/2;
    float y = (p1.y + p2.y)/2;
    float z = (p1.z + p2.z)/2;
    pcl::ModelCoefficients coeff;
    coeff.values.push_back(x);
    coeff.values.push_back(y);
    coeff.values.push_back(z);
    coeff.values.push_back(dist/2);
    return coeff;

}

bool CircleFit::is_good_fit(pcl::ModelCoefficients coeff, PointCloudS::Ptr cloud_in, pcl::PointIndices::Ptr inliers)
{
     pcl::ExtractIndices<PointS> extract;
     PointCloudS::Ptr cloud_out(new PointCloudS);
     extract.setInputCloud (cloud_in);
     extract.setIndices (inliers);
     extract.setNegative (false);
     extract.filter (*cloud_out);
     size_t size_before = cloud_in->points.size();
     size_t size_after  = cloud_out->points.size();
     float percent_fit;
     if(size_before == 0)
     {
         percent_fit = 0;
     }
     else
     {
         percent_fit = ((float) size_after/(float) size_before );
     }
     bool is_good = true;
     if(percent_fit<0.66)
     {
         is_good = false;
     }
     if(size_after < 50)
     {
         is_good = false;
     }
     float mean = 0;
     QVector<float> distances;
     return is_good;
}

CircleFit::CircleFit()
{

}

const pcl::ModelCoefficients CircleFit::ransac_circle_fit(PointCloudS::Ptr cloud, int type, float distance, int max_iterations)
{
    pcl::ModelCoefficients circle;

    pcl::SACSegmentationFromNormals<PointS, PointS> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.setNormalDistanceWeight (1 );
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CIRCLE3D);
    seg.setMethodType(type);
    seg.setMaxIterations(max_iterations);
    seg.setDistanceThreshold(distance);
    seg.setInputCloud(cloud);
    seg.setInputNormals(cloud);
    seg.segment(*inliers, circle);

    return circle;
}

const pcl::ModelCoefficients CircleFit::median_circle_fit(PointCloudS::Ptr cloud)
{
    Eigen::Vector4f centroid;

    pcl::compute3DCentroid<PointS> (*cloud, centroid);

    QVector<float> distances = get_distances(centroid, cloud);

    float median = SimpleMath<float>::get_median(distances);

    pcl::ModelCoefficients circle;
    circle.values.resize(4);
    circle.values[0] = centroid(0);
    circle.values[1] = centroid(1);
    circle.values[2] = centroid(2);
    circle.values[3] = median;

    return circle;
}
const pcl::ModelCoefficients CircleFit::general_circle_fit(PointCloudS::Ptr cloud, float radius_multiplier, int type, float distance, int max_iterations)
{
    pcl::ModelCoefficients coeff_empty;
    if(cloud->points.size()>2)
    {
        pcl::ModelCoefficients circle_median = median_circle_fit(cloud);
        pcl::ModelCoefficients circle_ransac = ransac_circle_fit(cloud, type, distance, max_iterations);

        if(circle_ransac.values.size()==3)
        {
            if(circle_ransac.values[3]< circle_median.values[3]*radius_multiplier)
            {
                return circle_ransac;
            }
        }
        return circle_median;
    }
    return coeff_empty;
}

const pcl::ModelCoefficients CircleFit::advanced_circle_fit(PointCloudS::Ptr cloud, float radius_multiplier, int type, float distance, int max_iterations)
{
    pcl::ModelCoefficients coeff_empty;
    if(cloud->points.size()>2)
    {
        pcl::ModelCoefficients circle_median = median_circle_fit(cloud);
        pcl::ModelCoefficients circle_ransac = ransac_circle_fit(cloud, type, distance, max_iterations);

        if(circle_ransac.values.size()==3)
        {
            if(circle_ransac.values[3]< circle_median.values[3]*radius_multiplier)
            {
                return circle_ransac;           }
        }
        return circle_median;
    }
    return coeff_empty;

}




