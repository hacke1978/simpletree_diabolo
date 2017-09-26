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

#ifndef CYLINDERFIT_H
#define CYLINDERFIT_H


#include <SimpleTree4/model/pointsimpletree.h>
#include <SimpleTree4/model/cylinder.h>
#include <SimpleTree4/model/segment.h>
#include <SimpleTree4/math/simplemath.h>
#include <SimpleTree4/method/computedistancecylinderscloud.h>
#include <SimpleTree4/method/point_cloud_operations/computemeanandstandarddeviation.h>
#include <SimpleTree4/method/point_cloud_operations/voxelgridfilter.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/impl/sac_segmentation.hpp>


#include <QSharedPointer>

class CylinderFit
{
    const static float _MIN_RADIUS_MULTIPLIER;
    const static float _MAX_RADIUS_MULTIPLIER;
public:
    CylinderFit();

    const static pcl::ModelCoefficients get_cylinder_from_circles(pcl::ModelCoefficients circle1, pcl::ModelCoefficients circle2);

    const static pcl::ModelCoefficients ransac_cylinder_fit(PointCloudS::Ptr cloud, int type = SimpleMath<int>::_METHOD_TYPE , float distance = 0.03, int max_iterations = 100);

//     const static pcl::ModelCoefficients ransac_cylinder_fit(PointCloudS::Ptr cloud, int type = SimpleMath<int>::_METHOD_TYPE , float distance = 0.03, int max_iterations = 100,
  //                                                           const Eigen::Vector3f &ax = Eigen::Vector3f(), double rad = SimpleMath<float>::_PI);

    const static void median_cylinder_fit(QSharedPointer<Cylinder> cylinder, PointCloudS::Ptr cloud);


    const static void general_cylinder_fit(PointCloudS::Ptr cloud, QSharedPointer<Cylinder> cylinder, int minPTS, int type = SimpleMath<int>::_METHOD_TYPE
                                        , float distance = 0.03, int max_iterations = 100);

    const static void advanced_cylinder_fit(PointCloudS::Ptr cloud, QSharedPointer<Cylinder> cylinder, MethodCoefficients coeff);

};
#endif // CYLINDERFIT_H
