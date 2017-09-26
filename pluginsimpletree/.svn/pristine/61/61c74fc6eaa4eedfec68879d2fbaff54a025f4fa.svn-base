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

#include "workerspherefollowing.h"

WorkerSphereFollowing::WorkerSphereFollowing(MethodCoefficients coeff, PointCloudS::Ptr cloud, QSharedPointer<OptimizationSphereFollowing> optim, bool subdivide_stem_and_branch_points)
{
    _cloud = cloud;
    _coeff = coeff;
    _optim = optim;
    _subdivide_stem_and_branch_points = subdivide_stem_and_branch_points;
}

void
WorkerSphereFollowing::run()
{
    SphereFollowing2 sf(_coeff, _cloud, _subdivide_stem_and_branch_points);
    sf.sphere_following();
    QVector<pcl::ModelCoefficients> coeff = sf.get_cylinders();
    ComputeDistanceCylindersCloud cd (coeff,_cloud);
    float dist = cd.get_mean_sqrd_dist();
    _optim->update_coeff(_coeff, dist);
}
