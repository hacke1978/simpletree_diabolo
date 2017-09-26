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

#include "improvebranchjunctions.h"

ImproveBranchJunctions::ImproveBranchJunctions(QSharedPointer<Tree> tree)
{
    _tree = tree;
    improve();
}

void ImproveBranchJunctions::improve()
{
    QVector<QSharedPointer<Segment> > segments = _tree->get_all_segments();
    QVectorIterator<QSharedPointer<Segment> > it(segments);
    while(it.hasNext())
    {
        QSharedPointer<Segment> segment = it.next();
        if (segment->get_cylinders().size()>1)
        {
            QSharedPointer<Cylinder> first = segment->get_cylinders().at (0);
            QSharedPointer<Cylinder> second = segment->get_cylinders().at (1);

            QSharedPointer<Cylinder> parent = _tree->get_parent(first);
            QSharedPointer<Cylinder> grand_parent = _tree->get_parent(parent);

            QSharedPointer<PointS> l1 = second->get_start_ptr();
            QSharedPointer<PointS> l2 = second->get_end_ptr();

            QSharedPointer<PointS> p1 = first->get_start_ptr();
            QSharedPointer<PointS> p2 = parent->get_start_ptr();
            QSharedPointer<PointS> p3 = grand_parent->get_start_ptr();

            float dist1 = SimpleMath<float>::dist_to_line(*p1,*l1,*l2);
            float dist2 = SimpleMath<float>::dist_to_line(*p2,*l1,*l2);
            float dist3 = SimpleMath<float>::dist_to_line(*p3,*l1,*l2);

            //     qDebug() << "ImproveBranchJunctions::improve()" << dist1 << dist2 << dist3;

            if(dist3<dist2 && dist3 < dist1)
            {
                Eigen::Vector3f axis1;
                axis1(0) = first->values[3];
                axis1(1) = first->values[4];
                axis1(2) = first->values[5];

//                Eigen::Vector3f axis2;
//                axis2(0) = parent->values[3];
//                axis2(1) = parent->values[4];
//                axis2(2) = parent->values[5];

                Eigen::Vector3f axis3;
                axis3(0) = grand_parent->values[3];
                axis3(1) = grand_parent->values[4];
                axis3(2) = grand_parent->values[5];

//                float angle1 = SimpleMath<float>::angle_between(axis1,axis2);
                float angle2 = SimpleMath<float>::angle_between(axis1,axis3);
                if(angle2>45)

                    first->set_start_end(p3,first->get_end_ptr());
                //  parent->get_segment()->remove_cylinder(parent);
                //  grand_parent->get_segment()->remove_cylinder(grand_parent);
            }
            else if (dist2 < dist1)
            {
                Eigen::Vector3f axis1;
                axis1(0) = first->values[3];
                axis1(1) = first->values[4];
                axis1(2) = first->values[5];

                Eigen::Vector3f axis2;
                axis2(0) = parent->values[3];
                axis2(1) = parent->values[4];
                axis2(2) = parent->values[5];


                float angle1 = SimpleMath<float>::angle_between(axis1,axis2);
                if(angle1>45)
                    first->set_start_end(p2,first->get_end_ptr());
                //  parent->get_segment()->remove_cylinder(parent);
            }
        }
    }
}
