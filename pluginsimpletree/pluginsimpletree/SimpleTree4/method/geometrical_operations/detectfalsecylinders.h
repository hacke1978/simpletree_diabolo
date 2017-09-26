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

#ifndef DETECTFALSECYLINDERS_H
#define DETECTFALSECYLINDERS_H

#include "SimpleTree4/model/pointsimpletree.h"
#include "SimpleTree4/model/tree.h"




class DetectFalseCylinders
{
    PointCloudS::Ptr _cloud;

    PointCloudS::Ptr get_center_point_cloud(QSharedPointer<Tree> tree);

    void compute_distances_point_cylinder(PointS p, QSharedPointer<Cylinder> cylinder, float & distance, float & distance_sqrd, float & distance_sqrd_angle);

    QSharedPointer<Tree> _tree;

    void compute();

    float  _MAX_DIST = 0.12f;



public:
    DetectFalseCylinders(PointCloudS::Ptr cloud, QSharedPointer<Tree> tree);
};

#endif // DETECTFALSECYLINDERS_H
