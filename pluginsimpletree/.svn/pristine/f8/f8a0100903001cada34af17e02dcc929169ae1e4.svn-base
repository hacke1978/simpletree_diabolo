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

#include "removefalsecylinders.h"


RemoveFalseCylinders::RemoveFalseCylinders(QSharedPointer<Tree> tree)
{
    _tree = tree;
    //qDebug() << "RemoveFalseCylinders " << "01";
    remove();
    //qDebug() << "RemoveFalseCylinders " << "02";
    merge();
    //qDebug() << "RemoveFalseCylinders " << "03";
}

void RemoveFalseCylinders::remove()
{
    QVector<QSharedPointer<Segment> > segments = _tree->get_leave_segments(_tree->get_root_segment());
    //qDebug() << "RemoveFalseCylinders" << "0a";
    QVectorIterator<QSharedPointer<Segment> > it (segments);
    while(it.hasNext())
    {
        QSharedPointer<Segment> segment = it.next();
        QVector<QSharedPointer<Cylinder> > cylinders = segment->get_cylinders();
     //   qDebug() << "RemoveFalseCylinders" << "0b";
        if(!(cylinders.size()>1))
        {
            segment->remove();
        }
    }
   // qDebug() << "RemoveFalseCylinders" << "0c";
}

void RemoveFalseCylinders::merge()
{
    bool do_merge = true;
    while(do_merge)
    {
        do_merge = false;
        QVector<QSharedPointer<Segment> > segments = _tree->get_all_segments();
        QVectorIterator<QSharedPointer<Segment> > it (segments);
        while(it.hasNext())
        {
            QSharedPointer<Segment> segment = it.next();
            if(segment->get_child_segments().size()==1)
            {
                do_merge = true;
                QSharedPointer<Segment> child = segment->get_child_segments().at(0);
                QVector<QSharedPointer<Segment> > grand_children = child->get_child_segments();
                child->remove();
                it.toBack();
                QVector<QSharedPointer<Cylinder> > cylinders = child->get_cylinders();
                QVectorIterator<QSharedPointer<Cylinder> > it2(cylinders);
                while(it2.hasNext())
                {
                    QSharedPointer<Cylinder> cylinder = it2.next();
                    segment->add_cylinder(cylinder);
                }
                QVectorIterator<QSharedPointer<Segment> > it3(grand_children);
                while(it3.hasNext())
                {
                    QSharedPointer<Segment> grand_child = it3.next();
                    segment->add_child_segment(grand_child);
                }
            }
        }
    }
}
