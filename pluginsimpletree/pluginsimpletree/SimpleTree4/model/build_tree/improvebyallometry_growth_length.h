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

#ifndef IMPROVEBYALLOMETRYLENGTH_H
#define IMPROVEBYALLOMETRYLENGTH_H

#include "SimpleTree4/math/simplemath.h"
#include "SimpleTree4/model/tree.h"


class ImproveByAllometryLength
{

    QSharedPointer<Tree> _tree;

    float _a;
    float _b;

    float _a_length;
    float _b_length;

    float _fac;
    float _min_rad;


    void
    improve();

    float get_y_from_x(float x);
    float get_x_from_y(float y);
    MethodCoefficients _cf;

public:
    ImproveByAllometryLength(QSharedPointer<Tree> tree, MethodCoefficients cf, float a, float b, float a_length, float b_length, float fac = 2.0f, float min_rad = 0.0025f);
};

#endif // IMPROVEBYALLOMETRYLENGTH_H
