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

#include "optimizationfit.h"

QVector<MethodCoefficients> OptimizationFit::generate_coefficients()
{
    QVector<MethodCoefficients> coefficients;
    MethodCoefficients coeff_a = _coeff;
    for(int i = 0; i < 4; i++)
    {
        MethodCoefficients coeff_b = coeff_a;
        switch (i) {
        case 0:
            coeff_b.number_of_merges = 0;
            break;
        case 1:
            coeff_b.number_of_merges = 1;
            break;
        case 2:
            coeff_b.number_of_merges = 2;
            break;
        case 3:
            coeff_b.number_of_merges = 3;
            break;
        default:
            qDebug() << "critical error in optimization fit";
            break;
        }

        for(int j = 0; j < 5; j++)
        {
            MethodCoefficients coeff_c = coeff_b;
            switch (j) {
            case 0:
                coeff_c.ransac_inlier_distance = 0.01;
                break;
            case 1:
                coeff_c.ransac_inlier_distance = 0.02;
                break;
            case 2:
                coeff_c.ransac_inlier_distance = 0.03;
                break;
            case 3:
                coeff_c.ransac_inlier_distance = 0.04;
                break;
            case 4:
                coeff_c.ransac_inlier_distance = 0.05;
                break;
            default:
                qDebug() << "critical error in optimization fit";
                break;
            }
            coefficients.push_back(coeff_c);

        }
    }
    return coefficients;

}

OptimizationFit::OptimizationFit(PointCloudS::Ptr cloud, MethodCoefficients coeff,  QVector<pcl::ModelCoefficients> cylinder_coef, bool is_multithreaded)
{
    _cylinder_coeff = cylinder_coef;
    _cloud = cloud;
    _coeff = coeff;
    _coeff_end = coeff;
    _is_multithreaded = is_multithreaded;
}

void OptimizationFit::update_coeff(MethodCoefficients coeff, float distance, QSharedPointer<Tree> tree)
{
    if(_best_distance > distance)
    {
        _best_distance = distance;
        _coeff_end = coeff;
        _tree = tree;
    }
}

void OptimizationFit::optimize()
{
    _best_distance = 100;
    QVector<MethodCoefficients> coeff = generate_coefficients();
    QVectorIterator<MethodCoefficients> it (coeff);
    BuildTree builder(_cylinder_coeff);
    QSharedPointer<Tree> tree (new Tree(builder.getRoot_segment(), _coeff_end.id));
    RemoveFalseCylinders remove(tree);
    ImproveByMedian improve_by_median(tree);




    while(it.hasNext())
    {
        MethodCoefficients coefficients = it.next();


        int i = 0;
        while(i < coefficients.number_of_merges )
        {
                ImproveByMerge improve_merge(tree);
                i++;
        }
        AllocatePoints alloc (tree, _cloud);
        alloc.compute();
        QVector<PointCloudS::Ptr> clouds = alloc.get_sub_clouds();
        QVector<pcl::ModelCoefficients> coeffs = alloc.get_cylinder_coeff();
        QVector<QSharedPointer<Cylinder> > cylinders = tree->get_all_cylinders();

        for(size_t i = 0; i < clouds.size(); i++)
        {
            PointCloudS::Ptr sub_cloud = clouds.at(i);
            QSharedPointer<Cylinder> cylinder = cylinders.at(i);
            CylinderFit::general_cylinder_fit(sub_cloud,cylinder,
                                              coefficients.minPts_ransac_branch,coefficients.ransac_type,
                                              coefficients.ransac_inlier_distance,coefficients.ransac_iterations);
        }

        QVector<QSharedPointer<Cylinder> > cylinders_tree = tree->get_all_cylinders();
        QVector<pcl::ModelCoefficients> cylinder_coefficients;
        QVectorIterator<QSharedPointer<Cylinder> > git(cylinders_tree);

        while(git.hasNext())
        {
            QSharedPointer<Cylinder> cylinder = git.next();
            pcl::ModelCoefficients coefficients;
            coefficients.values = cylinder->values;
            cylinder_coefficients.push_back(coefficients);
        }

        ComputeDistanceCylindersCloud cd (cylinder_coefficients,_cloud, coefficients.ransac_inlier_distance);
        float dist = 50;

        dist = cd.get_mean_sqrd_dist();
        update_coeff(coefficients, dist,tree);

        for(size_t i = 0; i < clouds.size(); i++)
        {
            pcl::ModelCoefficients original_coefficients = coeffs.at(i);
            QSharedPointer<Cylinder> cylinder = cylinders.at(i);
            cylinder->set_coeff(original_coefficients);
        }
    }
}
