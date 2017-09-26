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

#include "workerdownhillsimplex.h"

WorkerDownhillSimplex::WorkerDownhillSimplex(MethodCoefficients coeff, PointCloudS::Ptr cloud,
                                             QSharedPointer<OptimizationDownHillSimplex> optim, bool subdivide_stem_and_branch_points, int i, bool is_dual)
{
    _coeff = coeff;
    _cloud = cloud;
    _optim = optim;
    _subdivide_stem_and_branch_points = subdivide_stem_and_branch_points;
    _i = i;
    _is_dual = is_dual;
}

void WorkerDownhillSimplex::run()
{
    SphereFollowing2 spherefollowing(_coeff,_cloud, _subdivide_stem_and_branch_points);
    spherefollowing.sphere_following();


    BuildTree builder(spherefollowing.get_cylinders());
    QSharedPointer<Tree> tree (new Tree(builder.getRoot_segment(),_coeff.id));
    RemoveFalseCylinders remove(tree);

    //QVector<pcl::ModelCoefficients> coeff_cylinder = spherefollowing.get_cylinders();
    ComputeDistanceCylindersCloud cd (tree,_cloud);
    float dist = cd.get_mean_sqrd_dist();;

    _optim->set_distance(_i,dist);


}
