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

#ifndef PREDICTSTARTPARAMETERSERIC_H
#define PREDICTSTARTPARAMETERSERIC_H

#include "SimpleTree4/model/pointsimpletree.h"
#include "SimpleTree4/method/method_coefficients.h"
#include "SimpleTree4/method/point_cloud_operations/computemeanandstandarddeviation.h"
#include "SimpleTree4/import/readcsv.h"
#include "SimpleTree4/method/point_cloud_operations/predictstablevolume.h"


class PredictStartParameters_Eric
{
    PointCloudS::Ptr _cloud;

    MethodCoefficients _coeff;

private:
    int get_percentage();

    float get_height();

    float _cut_height;

public:
    PredictStartParameters_Eric(PointCloudS::Ptr cloud, float cut_height);

    MethodCoefficients get_coeff() const;
};

#endif // PREDICTSTARTPARAMETERSERIC_H
