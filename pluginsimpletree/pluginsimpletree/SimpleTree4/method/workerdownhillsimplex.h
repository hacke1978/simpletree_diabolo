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

#ifndef WORKERDOWNHILLSIMPLEX_H
#define WORKERDOWNHILLSIMPLEX_H

#include <QRunnable>
#include <QSharedPointer>
#include <QVector>
#include "SimpleTree4/method/spherefollowing2.h"
#include "SimpleTree4/method/optimizationdownhillsimplex.h"
#include "SimpleTree4/method/computedistancecylinderscloud.h"

class OptimizationDownHillSimplex;

class WorkerDownhillSimplex: public QRunnable
{
    MethodCoefficients _coeff;
     PointCloudS::Ptr _cloud;
     QSharedPointer<OptimizationDownHillSimplex>  _optim;
     bool _subdivide_stem_and_branch_points;
     int _i;
     bool _is_dual;
public:
    WorkerDownhillSimplex(MethodCoefficients coeff, PointCloudS::Ptr cloud,
                          QSharedPointer<OptimizationDownHillSimplex> optim, bool subdivide_stem_and_branch_points, int i, bool is_dual =  false);

    void run();
};

#endif // WORKERDOWNHILLSIMPLEX_H
