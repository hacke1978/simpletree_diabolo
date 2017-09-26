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

#ifndef IMPROVEBYALLOMETRY_H
#define IMPROVEBYALLOMETRY_H

#include "SimpleTree4/math/simplemath.h"
#include "SimpleTree4/model/tree.h"


class ImproveByAllometry
{
    float _min_rad = 0.0025f;
    float _fac = 2.0f;

    QSharedPointer<Tree> _tree;

    float _a;
    float _b;

    bool _inverse;

    void
    improve();

    void
    improve_cropped();

    float get_y_from_x(float x);
    float get_x_from_y(float y);
    MethodCoefficients _cf;

    bool _is_cropped = false;

//    float
//    get_lower_x_from_y(float y);

//    float
//    get_upper_x_from_y(float y);

//    void
//    get_model_data(QVector<float> & x, QVector<float> & y,QVector<float> & x_model, QVector<float> & y_model,float max_y, float max_x);
public:
    ImproveByAllometry(QSharedPointer<Tree> tree, MethodCoefficients cf, float a, float b, float fac = 2.0, bool inverse = false, bool is_cropped = false);
};

#endif // IMPROVEBYALLOMETRY_H
