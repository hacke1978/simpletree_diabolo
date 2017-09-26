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

#ifndef OPTIMIZATIONSPHEREFOLLOWING_H
#define OPTIMIZATIONSPHEREFOLLOWING_H


#include <QEnableSharedFromThis>
#include <QVector>

#include <SimpleTree4/model/pointsimpletree.h>
#include <SimpleTree4/method/method_coefficients.h>
#include <SimpleTree4/method/workerspherefollowing.h>
#include <SimpleTree4/model/build_tree/buildtree.h>
#include <SimpleTree4/model/build_tree/improvebymedian.h>
#include <SimpleTree4/model/build_tree/removefalsecylinders.h>
#include <SimpleTree4/model/build_tree/improvebymerge.h>
#include <SimpleTree4/model/build_tree/reordertree.h>


class OptimizationSphereFollowing : public QObject, public QEnableSharedFromThis<OptimizationSphereFollowing>
{
    Q_OBJECT

    PointCloudS::Ptr _cloud;

    MethodCoefficients _coeff;

    float _best_distance = 50;

    QVector<MethodCoefficients>
    generate_coefficients();

    bool _subdivide_stem_and_branch_points;

    bool _is_multithreaded;




public:

    MethodCoefficients _coeff_end;

    OptimizationSphereFollowing(PointCloudS::Ptr cloud, MethodCoefficients coeff, bool subdivide_stem_and_branch_points, bool is_multithreaded = true);

    void
    update_coeff(MethodCoefficients coeff, float distance);

    void
    optimize();
};

#endif // OPTIMIZATIONSPHEREFOLLOWING_H
