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

#ifndef SIMPLETREESTEP_H
#define SIMPLETREESTEP_H

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/common.h>
#include <pcl/common/impl/common.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/time.h>

#include "SimpleTree4/model/pointsimpletree.h"
#include "SimpleTree4/method/point_cloud_operations/voxelgridfilter.h"
#include "SimpleTree4/method/point_cloud_operations/convertcttost.h"

#include <ct_step/abstract/ct_abstractstep.h>
#include <ct_itemdrawable/abstract/ct_abstractitemdrawablewithpointcloud.h>
#include <ct_itemdrawable/ct_scene.h>
#include <ct_itemdrawable/abstract/ct_abstractstandarditemgroup.h>
#include <ct_tools/model/ct_autorenamemodels.h>
#include <ct_itemdrawable/ct_standarditemgroup.h>
#include <ct_itemdrawable/ct_grid3d_sparse.h>
#include <ct_itemdrawable/ct_pointcluster.h>
// Inclusion of standard result class
#include "ct_result/ct_resultgroup.h"

#include "ct_view/ct_stepconfigurabledialog.h"

#include <QMessageBox>
enum PointAtrributeType {EIGEN1, EIGEN2, EIGEN3, EIGENALL, NORMALX, NORMALY, NORMALZ,
                         NORMAL, TREEID, ID, CLUSTER, RANGEBIN, DISTANCE, TRUEDISTANCE, VISITED, ALL};
class SimpleTreeStep
{


public:
    SimpleTreeStep();
private:
    /**
     * @brief copy_attribute Copies a specified attribute from one point to another
     * @param from the point to copy from
     * @param to the point to copy to
     * @param type the specification of the attribute
     */
    void copy_attribute(PointS &from, PointS &to, PointAtrributeType type);



    /**
     * @brief get_percentage returns a percentage number of two different sizes
     * @param before the size before
     * @param after the size after
     * @return the reduction in percent
     */
    float get_percentage(const int before, const int after) const;

    /**
     * @brief voxelize_cloud produces a vector of subclouds contained in bboxes with size res
     * @param itemCpy_cloud_in the input cloud
     * @param res the grid resolution
     * @return  A vector of subclouds
     */
    QVector<PointCloudS::Ptr> voxelize_cloud_from_ct(CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in, float res = 10.0f);


    /**
     * @brief voxelize_cloud produces a vector of subclouds contained in bboxes with size res
     * @param itemCpy_cloud_in the input cloud
     * @param res the grid resolution
     * @return  A vector of subclouds
     */
    QVector<PointCloudS::Ptr> voxelize_cloud(PointCloudS::Ptr cloud_in, float res = 10.0f);

    /**
     * @brief get_center_of_mass returns the center of mass of a given point cloud
     * @param cloud the input cloud
     * @return the center of mass
     */
    PointS get_center_of_mass(PointCloudS::Ptr cloud);

protected:

    void dialog_simple_tree( CT_StepConfigurableDialog *configDialog);

    /**
      @param cloud the input cloud the input cloud to be checked if georeferenced
     * @brief warning_geo_referenced Pops up a QT Dialog with a Warning in case of georeferenced data.
     */
    void warning_geo_referenced(PointCloudS::Ptr cloud);

    /**
     * @brief toQString formats an int to a QString
     * @param i the int
     * @return the formatted QString
     */
    QString toQString(const int i) const;

    /**
     * @brief toQString formats a float to a QString
     * @param f the float
     * @param dec_places the number of decimal places
     * @return a formatted string
     */
    QString tofQString(const float  f, int dec_places = 2) const;

    /**
     * @brief downscale_cloud_ct downscales the cloud in the following manner:
     * A Voxelgrid is computed, each voxel gets x,y,z from its center coordinates and an additional counter for the points contained
     * @param itemCpy_cloud_in The input cloud
     * @param res the resolution of the grid
     * @return A downscaled cloud, number of points are written in field ID
     */
    PointCloudS::Ptr downscale_cloud_ct(CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in, float res);

    /**
     * @brief make_two_dimensional_pcl builds a new cloud and sets for each point z coordinate as 0
     * @param cloud The input cloud
     * @return A flattened cloud (all points have z = 0)
     */
    PointCloudS::Ptr make_two_dimensional_pcl(PointCloudS::Ptr cloud);

    /**
     * @brief pcl_voxel_grid_filter voxel_grid_filters the in cloud with a voxel of size leaf_size
     * @param step the step form which the function in called from
     * @param cloud_in the input cloud
     * @param leaf_size the voxel size
     * @return the downscaled cloud
     */
    PointCloudS::Ptr pcl_voxel_grid_filter_depricated(const PointCloudS::Ptr cloud_in, CT_AbstractStep * step, float leaf_size, bool print_console = true)  const;


    /**
     * @brief pcl_voxel_grid_filter voxel_grid_filters the in cloud with a voxel of size leaf_size
     * @param step the step form which the function in called from
     * @param cloud_in the input cloud
     * @param leaf_size the voxel size
     * @return the downscaled cloud
     */
    PointCloudS::Ptr pcl_voxel_grid_filter(const PointCloudS::Ptr cloud_in, CT_AbstractStep * step, float leaf_size, bool print_console = true);

    /**
     * @brief compute_normals Computes the normals for a given cloud
     * @param cloud_in the input cloud
     * @param k the number of neighbors to be accounted to
     */
    void pcl_compute_normals(PointCloudS::Ptr cloud_in, int k);

    /**
     * @brief pcl_CT_to_PCL_cloud converts a Computree cloud into a PCL cloud
     * @param itemCpy_cloud_in The computree cloud
     * @param step the step from which this function is called
     * @param knn the size of knn neighborhood
     * @param convert_2D set to true if points should be projected into XY plane
     * @return the converted pcl cloud
     */
    PointCloudS::Ptr pcl_CT_to_PCL_cloud(CT_AbstractItemDrawableWithPointCloud *itemCpy_cloud_in,
                                         CT_AbstractStep *step, int knn, bool convert_2D = false, bool print_console = true);

    /**
     * @brief pcl_copy_PCL_cloud Copies a pcl cloud
     * @param cloud_in the input cloud
     * @return a copy of the input cloud
     */
    PointCloudS::Ptr pcl_copy_PCL_cloud(PointCloudS::Ptr cloud_in) const;

    /**
     * @brief pcl_PCL_to_CT_cloud converts a pcl cloud to a CT cloud and adds this to the result
     * @param cloud_in the pcl input cloud
     * @param resCpy_res the result grp
     * @param grpCpy_grp the grp
     * @param model the model name
     * @param step the step from which this function is called
     */
    void pcl_PCL_to_CT_cloud(const PointCloudS::Ptr cloud_in, CT_AbstractResult *resCpy_res,
                             CT_StandardItemGroup* grpCpy_grp, const QString model_name, CT_AbstractStep *step, bool print_console = true);

    /**
     * @brief pcl_transfer_attribute Transfers one attribute from a low resoluted cloud to a high resoluted cloud
     * @param cloud_low_res the low resoluted cloud
     * @param cloud_high_res the high resoluted cloud
     * @param type an enum about the attribute which is to be copied
     */
    void pcl_transfer_attribute(PointCloudS::Ptr cloud_low_res, PointCloudS::Ptr cloud_high_res, PointAtrributeType type);

    void recursiveRemoveGroupIfEmpty(CT_AbstractItemGroup *parent, CT_AbstractItemGroup *group) const;

};

#endif // SIMPLETREESTEP_H
