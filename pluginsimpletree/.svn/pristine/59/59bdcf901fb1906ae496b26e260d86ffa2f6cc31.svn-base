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

#include "workerfit.h"

WorkerFit::WorkerFit(MethodCoefficients coeff, PointCloudS::Ptr cloud, QSharedPointer<OptimizationFit> optim,  QVector<pcl::ModelCoefficients> cylinder_coeff)
{
    _cloud = cloud;
    _coeff = coeff;
    _optim = optim;
    _cylinder_coeff = cylinder_coeff;
}


void
WorkerFit::run()
{


    BuildTree builder(_cylinder_coeff);
    QSharedPointer<Tree> tree (new Tree(builder.getRoot_segment(), _coeff.id));
    RemoveFalseCylinders remove(tree);
    ImproveByMedian improve_by_median(tree);
    ImproveByMerge improve_merge(tree);
    ImproveByPipeModel pype(tree,false, 0.1);
    ImproveFit fit(tree,_cloud,_coeff);



    QVector<QSharedPointer<Cylinder> > cylinders = tree->get_all_cylinders();
    QVector<pcl::ModelCoefficients> cylinder_coefficients;
    QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        pcl::ModelCoefficients coefficients;
        coefficients.values = cylinder->values;
        cylinder_coefficients.push_back(coefficients);
    }

    ComputeDistanceCylindersCloud cd (cylinder_coefficients,_cloud);
    float dist = cd.get_mean_sqrd_dist();
    _optim->update_coeff(_coeff, dist,tree);
}
