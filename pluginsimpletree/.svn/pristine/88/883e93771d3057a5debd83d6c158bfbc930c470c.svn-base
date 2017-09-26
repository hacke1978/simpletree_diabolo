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

#include "improvebymerge.h"

void ImproveByMerge::improve()
{
    QVector<QSharedPointer<Segment> > segments = _tree->get_all_segments();
    QVectorIterator<QSharedPointer<Segment> > it(segments);
    while(it.hasNext())
    {
        QSharedPointer<Segment> segment = it.next();
        while(segment->get_mean_length()<1*(SimpleMath<float>::_MIN_CYLINDER_LENGTH)&&segment->get_cylinders().size()>1)
//        while(segment->get_cylinders().size()>1)
        {
            QVector<QSharedPointer<Cylinder> > cylinders = segment->get_cylinders();
            QVector<QSharedPointer<Cylinder> >  cylinders_new;
            QVectorIterator<QSharedPointer<Cylinder> > git (cylinders);
            while(git.hasNext())
            {
                QSharedPointer<Cylinder> cylinder1 = git.next();
                if(git.hasNext())
                {
                    QSharedPointer<Cylinder> cylinder2 = git.next();

                    float x1_start = cylinder1->get_start_ptr()->x;
                    float y1_start = cylinder1->get_start_ptr()->y;
                    float z1_start = cylinder1->get_start_ptr()->z;
                    float x2_end = cylinder2->get_end_ptr()->x;
                    float y2_end = cylinder2->get_end_ptr()->y;
                    float z2_end = cylinder2->get_end_ptr()->z;
                    float radius = (cylinder1->get_radius()+cylinder2->get_radius())/2;
                    float x_axis = x2_end-x1_start;
                    float y_axis = y2_end-y1_start;
                    float z_axis = z2_end-z1_start;

                    pcl::ModelCoefficients cf;
                    cf.values.push_back(x1_start);
                    cf.values.push_back(y1_start);
                    cf.values.push_back(z1_start);
                    cf.values.push_back(x_axis);
                    cf.values.push_back(y_axis);
                    cf.values.push_back(z_axis);
                    cf.values.push_back(radius);
                    QSharedPointer<Cylinder> cylinder_new (new Cylinder(cf));
                    cylinders_new.push_back(cylinder_new);
                } else
                {
                    cylinders_new.push_back(cylinder1);
                }

            }
            segment->set_cylinders(cylinders_new);
        }
    }
}

ImproveByMerge::ImproveByMerge(QSharedPointer<Tree> tree)
{
    _tree = tree;
    improve();
}
