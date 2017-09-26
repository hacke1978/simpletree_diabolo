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


#ifndef EXTRACTCLOUDNEARMODEL_H
#define EXTRACTCLOUDNEARMODEL_H

#include "SimpleTree4/model/pointsimpletree.h"
#include "SimpleTree4/model/tree.h"

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>

class ExtractCloudNearModel
{
    /**
     * @brief _input_cloud the input cloud
     */
    PointCloudS::Ptr _input_cloud;

    /**
     * @brief _output_cloud the output cloud
     */
    PointCloudS::Ptr _output_cloud;

    /**
     * @brief _tree the tree
     */
    QSharedPointer<Tree> _tree;

    /**
     * @brief _cylinders the qsm cylinders
     */
    QVector<QSharedPointer<Cylinder> > _cylinders;

    /**
     * @brief _max_half_length the maximum half length of all cylinders
     */
    float _max_half_length;

    /**
     * @brief _max_dist_to_cylinderthe maxmimum allowed distance between a point and the qsm
     */
    const float _max_dist_to_cylinder = 0.03;

    /**
     * @brief _MAX_DIST_TO_ALL_CYLINDERS the maximum allowed distance to all cylinders center points
     */
    float _MAX_DIST_TO_ALL_CYLINDERS;

    /**
     * @brief is_close_to_cylinders
     * @param p
     * @param pointIdxRadiusSearch
     * @return
     */
    bool is_close_to_cylinders(PointS p, std::vector<int> pointIdxRadiusSearch);

    /**
     * @brief get_max_halflength returns the maximum half length of all cylinders
     * @return the maximum half length
     */
    float get_max_halflength();

    /**
     * @brief compute_max_distance computes the maximum allowed distance to all cylinders
     */
    void compute_max_distance();

    /**
     * @brief get_center_point_cloud makes out of the QSM cylinders center points a point cloud
     * @param the QSM
     * @return the point cloud made out of the cylinders center points
     */
    PointCloudS::Ptr get_center_point_cloud(QSharedPointer<Tree> tree);

    /**
     * @brief extract_points_near_cylinder the main method used to extract the good points
     */
    void extract_points_near_cylinder();




public:
    /**
     * @brief ExtractCloudNearModel Can be used after a robust driven parameter optimization
     * to remove the non representing points
     * @param tree the input QSM
     * @param cloud a point cloud representing the QSM
     */
    ExtractCloudNearModel(QSharedPointer<Tree> tree, PointCloudS::Ptr cloud);
    /**
     * @brief get_output_cloud getter for the output cloud
     * @return the points near the QSM
     */
    PointCloudS::Ptr get_output_cloud() const;
};

#endif // EXTRACTCLOUDNEARMODEL_H
