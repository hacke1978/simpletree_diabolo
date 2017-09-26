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

#ifndef SIMPLEMATH_HPP
#define SIMPLEMATH_HPP

#include "simplemath.h"



template <typename T>
const float SimpleMath<T>::_MIN_RADIUS_FOR_CYLINDER_IMPROVEMENT = 0.005f;

template <typename T>
const int SimpleMath<T>::_METHOD_TYPE = pcl::SAC_MLESAC;

template <typename T>
const float SimpleMath<T>::_PI = 3.1415926f;

template <typename T>
const float SimpleMath<T>::_OCTREE_RESOLUTION = 0.025f;

template <typename T>
const float SimpleMath<T>::_MIN_CYLINDER_LENGTH = 0.35f;

template <typename T>
const float SimpleMath<T>::_UNFITTED_DISTANCE = 0.05f;

template <typename T>
SimpleMath<T>::SimpleMath()
{

}

template <typename T>
void SimpleMath<T>::get_min_max_pts(PointCloudS::Ptr cloud, PointS & min, PointS & max)
{
    float minX = std::numeric_limits<float>::max();
    float minY = std::numeric_limits<float>::max();
    float minZ = std::numeric_limits<float>::max();


    float maxX = std::numeric_limits<float>::lowest();
    float maxY = std::numeric_limits<float>::lowest();
    float maxZ = std::numeric_limits<float>::lowest();

    for(size_t i = 0; i < cloud->points.size(); i++)
    {
        PointS p = cloud->points.at(i);
        if(p.x < minX)
            minX = p.x;
        if(p.y < minY)
            minY = p.y;
        if(p.z < minZ)
            minZ = p.z;


        if(p.x > maxX)
            maxX = p.x;
        if(p.y > maxY)
            maxY = p.y;
        if(p.z > maxZ)
            maxZ = p.z;
    }

    min.x = minX;
    min.y = minY;
    min.z = minZ;

    max.x = maxX;
    max.y = maxY;
    max.z = maxZ;
}

template <typename T>
bool SimpleMath<T>::are_equal(PointS & p1, PointS & p2)
{
    float dist = get_distance(p1, p2);
    return (qFuzzyCompare(dist,0));
}

template <typename T>
float SimpleMath<T>::dist_to_line(PointS &p, PointS & l1, PointS & l2)
{
    Eigen::Vector4f pt;
    pt[0] = p.x;
    pt[1] = p.y;
    pt[2] = p.z;
    pt[3] = 0;

    Eigen::Vector4f lp;
    lp[0] = l1.x;
    lp[1] = l1.y;
    lp[2] = l1.z;
    lp[3] = 0;

    Eigen::Vector4f la;
    la[0] = l2.x - l1.x;
    la[1] = l2.y - l1.y;
    la[2] = l2.z - l1.z;
    la[3] = 0;

    double dist = pcl::sqrPointToLineDistance(pt,lp,la);
    return sqrt(dist);
}


template <typename T>
float SimpleMath<T>::get_distance(PointS & p1, PointS & p2)
{
    float x = p2.x - p1.x;
    float y = p2.y - p1.y;
    float z = p2.z - p1.z;
    float dist = std::sqrt(x*x + y*y + z*z);
    return dist;
}

template <typename T>
float SimpleMath<T>::get_distance(pcl::PointXYZ & p1, pcl::PointXYZ & p2)
{
    float x = p2.x - p1.x;
    float y = p2.y - p1.y;
    float z = p2.z - p1.z;
    float dist = std::sqrt(x*x + y*y + z*z);
    return dist;
}

template <typename T>
float SimpleMath<T>::angle_between(Eigen::Vector3f axis1, Eigen::Vector3f axis2)
{
    axis1.normalize();
    axis2.normalize();
    float cos = axis1.dot(axis2);
    float rad = acos(cos);
    float deg = (rad*180)/SimpleMath<float>::_PI;
    return deg;
}

template <typename T>
float SimpleMath<T>::angle_between_in_rad(Eigen::Vector3f axis1, Eigen::Vector3f axis2)
{
    axis1.normalize();
    axis2.normalize();
    float cos = axis1.dot(axis2);
    float rad = acos(cos);
    return rad;
}



template <typename T>
T SimpleMath<T>::get_max(QVector<T> & number)
{
    qSort(number);
    T max = number.last();
    return max;
}

template <typename T>
T SimpleMath<T>::get_min(QVector<T> & number)
{
    qSort(number);
    T min= number.first();
    return min;
}

template <typename T> PointS
SimpleMath<T>::get_center_of_mass(PointCloudS::Ptr cloud)
{
    PointS center(0,0,0);
    for(size_t i = 0; i < cloud->points.size(); i++)
    {
        center.x += cloud->points[i].x;
        center.y += cloud->points[i].y;
        center.z += cloud->points[i].z;
    }
    if(cloud->points.size() <= 0)
    {
        return center;
    }
    center.x = center.x / cloud->points.size();
    center.y = center.y / cloud->points.size();
    center.z = center.z / cloud->points.size();
    return center;
}

template <typename T>
T SimpleMath<T>::get_median(QVector<T> & number)
{
    qSort(number);
    T median = 0;
    int size = number.size();
    if(size>0)
    {
        if(size%2==0)
        {
            median = (number.at(size/2-1)+number.at(size/2))/2;
        } else
        {
            median = number.at(size/2);
        }
    }
    return median;
}

template <typename T>
T SimpleMath<T>::get_mean(QVector<T> & number)
{
    int size = number.size();
    if(size>0)
    {
        T sum = 0;
        QVectorIterator<T> it(number);
        while(it.hasNext())
        {
            sum += it.next();
        }
        return (sum/size);
    }
    return 0;
}

template <typename T>
T SimpleMath<T>::get_standard_deviation(QVector<T> number)
{
    int size = number.size();
    T mean = SimpleMath<T>::get_mean(number);
    if(size>1)
    {
        T sum = 0;
        QVectorIterator<T> it(number);
        while(it.hasNext())
        {
            T num = it.next();
            T res = num-mean;

            sum += res*res;
        }
        T sd = std::sqrt(sum/(size-1));
        return sd;
    }
    return 0;
}


#endif // SIMPLEMATH_HPP
