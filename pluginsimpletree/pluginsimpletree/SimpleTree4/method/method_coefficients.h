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

#ifndef METHOD_COEFFICIENTS_H
#define METHOD_COEFFICIENTS_H
#include <pcl/sample_consensus/method_types.h>
#include <QString>
struct MethodCoefficients{

    float min_fitted_distance = 0.03f; //The minimum fitted distance to a cylinder for a point to be considered fitted

    /**
      * Spherefollowing parameters
     */
    float sphere_radius_multiplier=2;
    float epsilon_cluster_stem=0.02f;
    float epsilon_cluster_branch=0.02f;
    float epsilon_sphere=0.035f;
    int minPts_improve_cylinder = 20;
    int minPts_ransac_branch=3;
    int minPts_cluster_stem=3;
    int minPts_cluster_branch=3;
    float min_radius_sphere= 0.07f;    
    float height_start_sphere = 0.1f;
    int ransac_circle_type = pcl::SAC_MLESAC;
    float ransac_circle_inlier_distance = 0.03f;
    float cut_height = 0.32f;
    int number_of_merges = 1;
    float taper_a;
    float taper_b;
    float length_a;
    float length_b;

    bool use_simple_error_term_sphere = true;

    bool use_remove_false_cylinders = true;

    bool use_improve_by_median = true;

    bool use_stem_taper = true;

    bool use_improve_branch_junctions = true;

    bool use_simple_error_term_fit = true;

    bool use_median_filter_later = true;

    bool use_pipe_check = true;


    float min_rad = 0.005f;





    int number_clusters_for_spherefollowing = 10;

    /**
      *
     **/
    bool use_dhs = true;

    /**
      * for splitting up the points
      * */
     float clustering_distance = 0.05f;


    /**
     * Parameters for general tree volume estimation
     *
     */
    float tree_height = 0.0f;
    float tree_circumference = 0.0f;
    float tree_predicted_volume   = 0.0f;
    float tree_max_angle = 45;
    float percentage_for_attractor = 0.4f;
    bool use_allom = true;


    float a=35.671f;
    float b=2.204f;
    float minRad=0.0025f;
    int ransac_type = pcl::SAC_MLESAC;
    float ransac_inlier_distance = 0.02f;
    float ransac_iterations = 100;
    float ransac_median_factor = 5;
    double min_dist=0.0001;



    float factor = 3.0f;
    float sd;
    float mean;
    int sd_mult = 3;
    bool optimze_stem = false;
    float radius_multiplier_cylinder_test = 3.0f;

    /**
     *Parameters used for Gap Optimization
     */
    int times_cluster_extension = 0;
    int max_times_cluster_extension = 2;
    float min_ratio_pype = 0.5f;
    float max_ratio_pype = 1.5f;
    float min_radius_for_pype_test = 0.035f;
    int max_number_failure_segments = 0;

    QString id = "Tree01";
    QString species = "unknown species";
    QString outputpath = "D:/";

    /**
      * Allometry parameters, when to use allometry, when not
     */

    float ratio_min = 2.0f;



    /**
     * @brief slize_ground_multiplicator how to decide between lowest and second lowest slice.
     */
    float slize_ground_multiplicator = 2.0f;

    /**
     * @brief volume_3d the volume of the 3d convex crown hull
     */
    float volume_3d = -1.0f;


    /**
     * @brief area_2d the crown projection area
     */
    float area_2d = -1.0f;

    /**
     * @brief area_3d the crown hull area
     */
    float area_3d = -1.0f;


};

#endif // METHOD_COEFFICIENTS_H
