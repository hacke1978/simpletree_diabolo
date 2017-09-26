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

#ifndef CIRCLEFIT_H
#define CIRCLEFIT_H

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/impl/sac_segmentation.hpp>
//#include <pcl/surface/convex_hull.h>
//#include <pcl/surface/impl/convex_hull.hpp>
#include <pcl/point_types.h>
//#include <pcl/Vertices.h>


#include <SimpleTree4/model/pointsimpletree.h>
#include <SimpleTree4/math/simplemath.h>

#include <SimpleTree4/method/method_coefficients.h>

#include <QVector>


class CircleFit
{
private:

    /**
     * @brief get_distances computes the distances from a subcloud to its center of mass
     * @param centroid the center of mass
     * @param cloud the sub cloud
     * @return the vector of distances
     */
    const static QVector<float> get_distances(Eigen::Vector4f centroid, PointCloudS::Ptr cloud);

    /**
     * @brief get_largest_dist_point Return from the input cloud the point with the largest distance to the input point
     * @param p the input point
     * @param cloud the input cloud
     * @return the point with the largest distance
     */
    PointS get_largest_dist_point(const PointS &p, PointCloudS::Ptr cloud);

    /**
     * @brief compute_sphere Calculates a sphere from two opposite points
     * @param p1 the first point
     * @param p2 the second point
     * @return the sphere in from of pcl::ModelCoefficients
     */
    pcl::ModelCoefficients compute_sphere(const PointS &p1, const PointS &p2);

    /**
     * @brief check_sphere checks if the sphere contains all points
     * @param coeff the spehere coefficients
     * @param cloud the input cloud
     * @return  (0,0,0) if all points are contained or an outlier otherwise
     */
    PointS check_sphere(const pcl::ModelCoefficients &coeff, PointCloudS::Ptr cloud);

    /**
     * @brief ritter_sphere Implementation of the Ritter algorithm to compute a bounding sphere
     * @param cloud the input cloud
     * @return a bounding sphere
     */
    pcl::ModelCoefficients ritter_sphere(PointCloudS::Ptr cloud);

    /**
     * @brief check_out checks if the point is (0,0,0)
     * @param out the input point
     * @return true if point is not the origin
     */
    bool check_out(const PointS & out);

    /**
     * @brief convex_hull computes the convex hull
     * @param cloud_in the input cloud
     * @return the vertices of the hull
     */
    const static PointCloudS::Ptr convex_hull(PointCloudS::Ptr cloud_in);

    /**
     * @brief bounding_sphere Computes a bounding sphere
     * @param cloud_in The input cloud
     * @return  The bounding sphere as model Coefficients
     */
    const static pcl::ModelCoefficients bounding_sphere(PointCloudS::Ptr cloud_in);

    /**
     * @brief is_good_fit checks if a cylinder represents a cloud well
     * @param coeff The cylinder coefficients
     * @param cloud_in the input cloud
     * @param inliers the inliers of the input data
     * @return true if the fit quality is good
     */
    bool is_good_fit(pcl::ModelCoefficients coeff, PointCloudS::Ptr cloud_in, pcl::PointIndices::Ptr inliers);


public:

    CircleFit();
    /**
     * @brief ransac_circle_fit fits a circle into a cloud
     * @param cloud the intput cloud
     * @param type the applied ransac method, see PCL documentation
     * @param distance the distance for inliers
     * @param max_iterations the number of iterations
     * @return a circle in form of pcl::ModelCoefficients
     */
    const static pcl::ModelCoefficients ransac_circle_fit(PointCloudS::Ptr cloud, int type = SimpleMath<int>::_METHOD_TYPE , float distance = 0.05, int max_iterations = 100);

    /**
     * @brief median_circle_fit Fits a circle into a cloud by computing the center of mass and its median distance to the cloud
     * @param cloud the cloud
     * @return a circle in form of pcl::ModelCoefficients
     */
    const static pcl::ModelCoefficients median_circle_fit(PointCloudS::Ptr cloud);


    /**
     * @brief general_circle_fit Applies first a both a ransac and a median fit and chooses ransac only if the radius does not deviate too much from the median circle radius
     * @param cloud the input cloud
     * @param radius_multiplier the allowed multiplier for the median radius to let a ransac circle be accepted
     * @param type the applied ransac method, see PCL documentation
     * @param distance the distance for inliers
     * @param max_iterations the number of iterations
     * @return a circle in form of pcl::ModelCoefficients
     */
    const static pcl::ModelCoefficients general_circle_fit(PointCloudS::Ptr cloud, float radius_multiplier = 3, int type = SimpleMath<int>::_METHOD_TYPE , float distance = 0.05, int max_iterations = 100);

    /**
     * @brief advanced_circle_fit Applies first  a ransac and a median fit and chooses ransac only if the cylinder fit is considered good
     * @param cloud the input cloud
     * @param radius_multiplier the allowed multiplier for the median radius to let a ransac circle be accepted
     * @param type the applied ransac method, see PCL documentation
     * @param distance the distance for inliers
     * @param max_iterations the number of iterations
     * @return a circle in form of pcl::ModelCoefficients
     */
    const static pcl::ModelCoefficients advanced_circle_fit(PointCloudS::Ptr cloud, float radius_multiplier = 3, int type = SimpleMath<int>::_METHOD_TYPE , float distance = 0.05, int max_iterations = 100);
};

#endif // CIRCLEFIT_H

