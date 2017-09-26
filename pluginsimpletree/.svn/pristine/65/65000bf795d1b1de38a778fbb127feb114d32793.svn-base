/****************************************************************************

 Copyright (C) 2010-2012 Dr. Jan Hackenberg.
                     All rights reserved.

 Contact : jan.hackenberg@posteo.de

 Developers : Jan Hackenberg

 This file is part of SimpleTree library 4.0.

 SimpleTree is free library: you can redistribute it and/or modify
 it under the terms of the GNU Lesser General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 SimpleTree is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU Lesser General Public License
 along with SimpleTree.  If not, see <http://www.gnu.org/licenses/lgpl.html>.

*****************************************************************************/

#ifndef COMPUTEPRINCIPALDIRECTION_H
#define COMPUTEPRINCIPALDIRECTION_H

#include <QDebug>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include "SimpleTree4/model/pointsimpletree.h"

class ComputePrincipalDirection
{
    PointCloudS::Ptr _cloud_in;

    float _search_radius = 0.2f;

    void compute();

public:
    ComputePrincipalDirection(PointCloudS::Ptr cloud_in);

};

#endif // COMPUTEPRINCIPALDIRECTION_H
