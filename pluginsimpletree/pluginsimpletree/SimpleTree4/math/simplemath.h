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

#ifndef SIMPLEMATH_H
#define SIMPLEMATH_H


#include <QVector>
#include <QtGlobal>
#include <pcl/sample_consensus/method_types.h>
#include <SimpleTree4/model/pointsimpletree.h>
 #include <pcl/common/distances.h>

template <typename T>
class SimpleMath
{
public:
    static const float _OCTREE_RESOLUTION;

    static const float _MIN_CYLINDER_LENGTH;

    static const float _MIN_RADIUS_FOR_CYLINDER_IMPROVEMENT;

    static PointS get_center_of_mass(PointCloudS::Ptr cloud);

    static void get_min_max_pts(PointCloudS::Ptr cloud,  PointS & min, PointS & max);



    SimpleMath();
    /**
     * @brief get_median Returns the median of a templated vector of numbers
     * @param number the vector of templated numbers
     * @return the median
     */
    static T get_median(QVector<T> & number);

    /**
     * @brief get_max Returns the max value of a templated vector of numbers
     * @param number the vector of templated numbers
     * @return the max value
     */
    static T get_max(QVector<T> & number);

    /**
     * @brief get_min Returns the min of a templated vector of numbers
     * @param number the vector of templated numbers
     * @return the min value
     */
    static T get_min(QVector<T> & number);

    /**
     * @brief get_mean Returns the mean of a templated vector of numbers
     * @param number the vector of templated numbers
     * @return  the mean
     */
    static T get_mean(QVector<T> & number);


    /**
     * @brief get_standard_deviation the standarddeviation of the vector
     * @param number the vector of templated numbers
     * @return the standard deviation
     */
    static T get_standard_deviation(QVector<T> number);

    /**
     * @brief getdistance returns Euclidean distance between two points
     * @param p1 the first point
     * @param p2 the second point
     * @return  the distance
     */
    static float get_distance(PointS &p1, PointS & p2);

    /**
     * @brief getdistance returns Euclidean distance between two points
     * @param p1 the first point
     * @param p2 the second point
     * @return  the distance
     */
    static float get_distance(pcl::PointXYZ &p1, pcl::PointXYZ & p2);



    static float dist_to_line(PointS &p, PointS & l1, PointS & l2);


    /**
     * @brief are_equal returns true if the two points are equal
     * @param p1 the first point
     * @param p2 the second point
     * @return true if points are equal regarding their spatial position
     */
    static bool are_equal(PointS & p1, PointS & p2);


    static const float _PI;

    static const int _METHOD_TYPE;

    static const float _UNFITTED_DISTANCE;

    static float angle_between(Eigen::Vector3f axis1, Eigen::Vector3f axis2);

        static float angle_between_in_rad(Eigen::Vector3f axis1, Eigen::Vector3f axis2);



};

#include "simplemath.hpp"

#endif // SIMPLEMATH_H
