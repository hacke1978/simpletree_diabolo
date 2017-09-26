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

#include "optimizationspherefollowing.h"

OptimizationSphereFollowing::OptimizationSphereFollowing(PointCloudS::Ptr cloud, MethodCoefficients coeff,  bool subdivide_stem_and_branch_points, bool is_multithreaded)
{
    _cloud = cloud;
    _coeff = coeff;
    _coeff_end = coeff;
    _subdivide_stem_and_branch_points = subdivide_stem_and_branch_points;
    _is_multithreaded = is_multithreaded;
}

QVector<MethodCoefficients> OptimizationSphereFollowing::generate_coefficients()
{
    QVector<MethodCoefficients> coefficients;
    MethodCoefficients coeff_a = _coeff;
    for(int i = 0; i < 3; i++)
    {
        MethodCoefficients coeff_b = coeff_a;
        switch (i) {
        case 0:
        {
            coeff_b.sphere_radius_multiplier = 1.75;
            break;
        }
        case 1:
        {
            coeff_b.sphere_radius_multiplier = 2.00;
            break;
        }
        case 2:
        {
            coeff_b.sphere_radius_multiplier = 2.25;
            break;
        }
        default:
            break;
        }
        for(int j = 0; j < 3; j++)
        {
            MethodCoefficients coeff_c = coeff_b;
            switch (j) {
            case 0:
                coeff_c.epsilon_cluster_branch = coeff_b.epsilon_cluster_branch*1;
                break;
            case 1:
                coeff_c.epsilon_cluster_branch = coeff_b.epsilon_cluster_branch*2;
                break;
            case 2:
                coeff_c.epsilon_cluster_branch = coeff_b.epsilon_cluster_branch*4;
                break;
            default:
                break;
            }

            for(int k = 0; k < 3; k++)
            {
                MethodCoefficients coeff_d = coeff_c;
                switch (k) {
                case 0:
                    coeff_d.epsilon_sphere = 0.01;
                    break;
                case 1:
                    coeff_d.epsilon_sphere = 0.02;
                    break;
                case 2:
                    coeff_d.epsilon_sphere = 0.04;
                    break;

                default:
                    break;
                }
                if(!coeff_d.optimze_stem)
                {
                    coeff_d.epsilon_cluster_stem = 0.04f;
                    coefficients.push_back(coeff_d);
                } else
                {
                    for(int l = 0; l < 3; l++ )
                    {
                        MethodCoefficients coeff_e = coeff_d;
                        switch (l) {
                        case 0:
                            coeff_e.epsilon_cluster_stem = coeff_b.epsilon_cluster_branch*1;
                            break;
                        case 1:
                            coeff_e.epsilon_cluster_stem = coeff_b.epsilon_cluster_branch*2;
                            break;
                        case 2:
                            coeff_e.epsilon_cluster_stem = coeff_b.epsilon_cluster_branch*4;
                            break;

                        default:
                            break;
                        }
                        coefficients.push_back(coeff_e);
                    }
                }
            }
        }
    }
    return coefficients;
}

void OptimizationSphereFollowing::update_coeff(MethodCoefficients coeff, float distance)
{
    if(_best_distance > distance)
    {
        _best_distance = distance;
        _coeff_end = coeff;
    }
}

void OptimizationSphereFollowing::optimize()
{
    _best_distance = 50;
    QVector<MethodCoefficients> coeff = generate_coefficients();
    QVectorIterator<MethodCoefficients> it (coeff);
    while(it.hasNext())
    {
        MethodCoefficients coefficients = it.next();
        SphereFollowing2 sf(coefficients, _cloud, _subdivide_stem_and_branch_points);
        sf.sphere_following();
        QVector<pcl::ModelCoefficients> coeff = sf.get_cylinders();
        float dist  = 50;
        if(coeff.size()>3)
        {
            BuildTree builder(coeff);
            QSharedPointer<Tree> tree (new Tree(builder.getRoot_segment(),_coeff_end.id));
            RemoveFalseCylinders remove(tree);
            ImproveByMedian improve_by_median(tree);
            ImproveByMerge improve_merge(tree);
            ReorderTree reorder(tree);
            coeff = tree->get_pcl_coefficients();
            ComputeDistanceCylindersCloud cd (coeff,_cloud);

            dist = cd.get_mean_sqrd_dist();

           // dist = (double) 1.0/cd.get_inliers_per_area();
        }
        update_coeff(coefficients, dist);
    }
}

