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

#ifndef IMPROVEFIT_H
#define IMPROVEFIT_H

#include "../tree.h"

#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <Eigen/Dense>

#include <SimpleTree4/math/simplemath.h>
#include <SimpleTree4/method/method_coefficients.h>
#include <SimpleTree4/method/geometrical_operations/cylinderfit.h>

class ImproveFit
{
private:

    MethodCoefficients _coeff;
    QSharedPointer<Tree> _tree;
    QSharedPointer<pcl::octree::OctreePointCloudSearch<PointS> > _octree;
    PointCloudS::Ptr _cloud;


    const static float _MIN_DIST_TO_CYLINDER;
    const static float _CYLINDER_SIZE_MULTIPLIER;


    const bool
    is_perpendicular(PointS p,  QSharedPointer<Cylinder> const cylinder);

public:
    ImproveFit(QSharedPointer<Tree> tree, PointCloudS::Ptr cloud, MethodCoefficients coeff);

    /**
     * @brief allocate_points Writes an ID for each point if fitted by a cylinder;
     */
    QVector<PointCloudS::Ptr>   allocate_points();

    /**
     * @brief allocate_points Writes an ID for each point if fitted by a cylinder;
     */
    QVector<PointCloudS::Ptr>   allocate_segment_points();




    /**
     * @brief extract_points_near_cylinder Returns a point cloud with the points near the cylinder
     * @param cylinder the input cylinder
     * @return the cloud
     */
    PointCloudS::Ptr
    extract_points_near_cylinder(QSharedPointer<Cylinder> cylinder);
    /**
     * @brief improve Iterates through all the cylinders and improves those by either RANSAC or Median method
     */
    void
    improve();

    /**
     * @brief RANSAC does a RANSAC Improvement
     * @param cloud the input cloud
     * @return the cylinder RANSAC coefficients
     */
    pcl::ModelCoefficients
    RANSAC(PointCloudS::Ptr cloud);

    /**
     * @brief improve_with_RANSAC Applies the RANSAC coeffiencts to the cylinder
     * @param cylinder the original cylinder
     * @param coeff the RANSAC coefficients
     */
    void
    improve_with_RANSAC(QSharedPointer<Cylinder> cylinder, pcl::ModelCoefficients coeff);

    /**
     * @brief improve_with_median Improves the radius of the cylinder with the median method
     * @param cylinder the input cylinder
     * @param cloud the cloud to compute the median from.
     */
    void
    improve_with_median(QSharedPointer<Cylinder> cylinder, PointCloudS::Ptr cloud);
};

#endif // IMPROVEFIT_H
