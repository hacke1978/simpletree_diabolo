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

#ifndef ALLOCATEPOINTS_H
#define ALLOCATEPOINTS_H

#include "SimpleTree4/model/tree.h"
#include "SimpleTree4/model/pointsimpletree.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>


class AllocatePoints
{

    QVector<QSharedPointer<Cylinder> > _cylinders;

    QVector<pcl::ModelCoefficients> _cylinder_coeff;

    QVector<PointCloudS::Ptr> _sub_clouds;

    QSharedPointer<Tree> _tree;

    PointCloudS::Ptr _cloud;

    void initiate();

    void allocate();

    QSharedPointer<pcl::KdTreeFLANN<PointS> > _kdtree;

    bool only_sphere_following = true;




public:
    AllocatePoints(QSharedPointer<Tree> tree, PointCloudS::Ptr cloud);

    void compute();

    QVector<PointCloudS::Ptr> get_sub_clouds() const;

    QVector<pcl::ModelCoefficients> get_cylinder_coeff() const;
};

#endif // ALLOCATEPOINTS_H
