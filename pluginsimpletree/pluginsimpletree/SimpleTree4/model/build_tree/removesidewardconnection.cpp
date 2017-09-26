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

#include "removesidewardconnection.h"

RemoveSideWardConnection::RemoveSideWardConnection(QSharedPointer<Tree> tree)
{
    _tree = tree;
    improve();
}

void RemoveSideWardConnection::improve()
{
    QVector<QSharedPointer<Cylinder> > cylinders = _tree->get_all_cylinders();
    QVectorIterator<QSharedPointer<Cylinder> > it (cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cyl = it.next();
        Eigen::Vector3f normal = cyl->get_axis();
        Eigen::Vector3f zAxis;
        zAxis(0) = 0;
        zAxis(1) = 0;
        zAxis(2) = 1;
        float angle = SimpleMath<float>::angle_between(normal,zAxis);
        float length = cyl->get_length();
        if( ( (angle < (90+_min_angle)) && (angle > (90-_min_angle)) && (length > _max_distance/3) ) || (length > _max_distance) )
        {
            QSharedPointer<Segment> seg = cyl->get_segment();
            if(seg!=NULL&&seg->get_parent_segment()!=NULL)
            {
                QSharedPointer<Segment> parent = seg->get_parent_segment();
                parent->remove_child(seg);
            }
        }

    }
}
