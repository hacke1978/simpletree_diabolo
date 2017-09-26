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

#include "improvebymedian.h"

ImproveByMedian::ImproveByMedian(QSharedPointer<Tree> tree)
{
    _tree = tree;
    QVector<QSharedPointer<Segment> > segments = tree->get_all_segments();
    QVectorIterator<QSharedPointer<Segment> > it (segments);
    while(it.hasNext())
    {
        QSharedPointer<Segment> segment = it.next();
        if(!segment->is_root())
        {
            if(!(segment->get_branch_order()==0 && segment->get_median_radius()>0.035))
            median_smooth(segment);
        }
    }
}

const float ImproveByMedian::_MAX_PERCENTAGE = 1.2f;
const float ImproveByMedian::_MIN_PERCENTAGE = 0.8f;

void ImproveByMedian::median_smooth(QSharedPointer<Segment> segment)
{
    float median = segment->get_median_radius();
    QVector<QSharedPointer<Cylinder> > cylinders = segment->get_cylinders();
    QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        float radius  = cylinder->get_radius();
        if(radius>median*_MAX_PERCENTAGE)
        {
            cylinder->set_radius(median);

        }
        if(radius<median*_MIN_PERCENTAGE)
        {
            cylinder->set_radius(median);

        }
    }
}

void ImproveByMedian::mean_smooth(QSharedPointer<Segment> segment)
{
    float mean = segment->get_mean_radius();
    QVector<QSharedPointer<Cylinder> > cylinders = segment->get_cylinders();
    QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        float radius  = cylinder->get_radius();
        if(radius>mean*_MAX_PERCENTAGE)
        {
            cylinder->set_radius(mean);

        }
        if(radius<mean*_MIN_PERCENTAGE)
        {
            cylinder->set_radius(mean);
        }
    }
}
