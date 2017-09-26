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

#ifndef OPTIMIZATIONFIT_H
#define OPTIMIZATIONFIT_H

#include <QEnableSharedFromThis>
#include <QVector>
#include <QDebug>

#include <SimpleTree4/model/pointsimpletree.h>
#include <SimpleTree4/method/method_coefficients.h>
#include <SimpleTree4/method/workerfit.h>
#include <SimpleTree4/method/geometrical_operations/allocatepoints.h>

class OptimizationFit: public QObject, public QEnableSharedFromThis<OptimizationFit>
{
    Q_OBJECT

  //  static QMutex lock;
    PointCloudS::Ptr _cloud;
    MethodCoefficients _coeff;



    QVector<pcl::ModelCoefficients> _cylinder_coeff;

    float _best_distance = 1;

    bool _is_multithreaded;

    QVector<MethodCoefficients>
    generate_coefficients();

public:

    QSharedPointer<Tree> _tree;

    MethodCoefficients _coeff_end;

    OptimizationFit(PointCloudS::Ptr cloud, MethodCoefficients coeff,  QVector<pcl::ModelCoefficients> cylinder_coef, bool is_multithreaded = true);

    void
    update_coeff(MethodCoefficients coeff, float distance, QSharedPointer<Tree> tree);

    void
    optimize();
};

#endif // OPTIMIZATIONFIT_H
