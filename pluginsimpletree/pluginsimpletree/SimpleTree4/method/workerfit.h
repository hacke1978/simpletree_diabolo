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

#ifndef WORKERFIT_H
#define WORKERFIT_H


#include <QRunnable>
#include <QSharedPointer>
#include <QVector>
#include "SimpleTree4/model/tree.h"
#include "SimpleTree4/method/optimizationfit.h"
#include "SimpleTree4/model/build_tree/improvefit.h"

#include "SimpleTree4/model/build_tree/buildtree.h"
#include "SimpleTree4/model/build_tree/improvebranchjunctions.h"
#include "SimpleTree4/model/build_tree/improvebymedian.h"
#include "SimpleTree4/model/build_tree/improvefit.h"
#include "SimpleTree4/model/build_tree/removefalsecylinders.h"
#include "SimpleTree4/model/build_tree/reordertree.h"
#include "SimpleTree4/model/build_tree/improvebymerge.h"
#include "SimpleTree4/method/optimizationspherefollowing.h"
#include "SimpleTree4/method/spherefollowingrecursive.h"
#include "SimpleTree4/model/build_tree/improvebyallometry.h"
#include "SimpleTree4/model/build_tree/improvebypipemodel.h"
#include "SimpleTree4/method/point_cloud_operations/computemeanandstandarddeviation.h"
#include "SimpleTree4/model/build_tree/improvedbyadvancedmedian.h"
#include "SimpleTree4/method/computeallometry.h"




class OptimizationFit;
class WorkerFit: public QRunnable
{
    QSharedPointer<OptimizationFit> _optim;
    MethodCoefficients _coeff;
    PointCloudS::Ptr _cloud;
    QVector<pcl::ModelCoefficients> _cylinder_coeff;

public:
    WorkerFit(MethodCoefficients coeff, PointCloudS::Ptr cloud, QSharedPointer<OptimizationFit> optim, QVector<pcl::ModelCoefficients> cylinder_coeff);

    void
    run();
};

#endif // WORKERFIT_H
