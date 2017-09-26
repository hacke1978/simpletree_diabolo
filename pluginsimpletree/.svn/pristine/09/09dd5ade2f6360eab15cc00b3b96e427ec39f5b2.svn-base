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

#include "improvedbyadvancedmedian.h"

const float ImprovedByAdvancedMedian::_MAX_PERCENTAGE = 1.2f;
const float ImprovedByAdvancedMedian::_MIN_PERCENTAGE = 0.8f;

void ImprovedByAdvancedMedian::improve_segment(QSharedPointer<Segment> segment)
{
    if(segment->get_cylinders().size()>1)
    {
        _indices.clear();
        float median_radius = segment->get_median_radius();
        QVector<QSharedPointer<Cylinder> > cylinders = segment->get_cylinders();
        for(size_t i = 0; i < cylinders.size(); i++)
        {
            if(!(segment->get_branch_order()==0 && segment->get_median_radius()>0.035)){
                QSharedPointer<Cylinder> cylinder = cylinders.at(i);
                if(cylinder->get_radius()> median_radius*_MAX_PERCENTAGE || cylinder->get_radius()<median_radius*_MIN_PERCENTAGE)
                {
                    _indices.push_back(i);
                }
            } else
            {
                QSharedPointer<Cylinder> cylinder = cylinders.at(i);
                if(cylinder->get_radius()> median_radius*1.5 || cylinder->get_radius()<median_radius*0.5)
                {
                    _indices.push_back(i);
                }
            }
        }
        for(size_t i = 0; i < _indices.size(); i++)
        {
            int index = _indices.at(i);
            if(index == 0)
            {
                QSharedPointer<PointS> start = cylinders.at(index)->get_start_ptr();
                QSharedPointer<PointS> end = cylinders.at(index+1)->get_start_ptr();
                cylinders.at(index)->set_start_end(start,end);
                cylinders.at(index)->set_radius(median_radius);
                cylinders.at(index)->set_allometry_improvement(AllometryImproved::ALLOM_MED);
            } else if (index == (cylinders.size()-1))
            {
                QSharedPointer<PointS> start = cylinders.at(index-1)->get_end_ptr();
                QSharedPointer<PointS> end = cylinders.at(index)->get_end_ptr();
                cylinders.at(index)->set_start_end(start,end);
                cylinders.at(index)->set_radius(median_radius);
                cylinders.at(index)->set_allometry_improvement(AllometryImproved::ALLOM_MED);
            } else
            {
                QSharedPointer<PointS> start = cylinders.at(index-1)->get_end_ptr();
                QSharedPointer<PointS> end = cylinders.at(index+1)->get_start_ptr();
                cylinders.at(index)->set_start_end(start,end);
                cylinders.at(index)->set_radius(median_radius);
                cylinders.at(index)->set_allometry_improvement(AllometryImproved::ALLOM_MED);
            }
        }
    }
}

void ImprovedByAdvancedMedian::smooth_segment(QSharedPointer<Segment> segment)
{
    QVector<QSharedPointer<Cylinder> > cylinders = segment->get_cylinders();
    if(!segment->is_root())
    {

        if(cylinders.size()>1)
        {

            for(size_t i = 0; i < cylinders.size(); i++)
            {
                if(i == 0)
                {


                    QVector<QSharedPointer<Cylinder> > cylinders_parent =  segment->get_parent_segment()->get_cylinders();
                    QSharedPointer<Cylinder>  cylinder = cylinders_parent.last();
                    QSharedPointer<PointS> start = cylinder->get_end_ptr();
                    QSharedPointer<PointS> end1 = cylinders.at(i+1)->get_start_ptr();
                    QSharedPointer<PointS> end2 = cylinders.at(i)->get_end_ptr();
                    QSharedPointer<PointS> end(new PointS);
                    end->x = (end1->x + end2->x)/2;
                    end->y = (end1->y + end2->y)/2;
                    end->z = (end1->z + end2->z)/2;
                    cylinders.at(i)->set_start_end(start,end);
                } else if (i == (cylinders.size()-1))
                {

                    QSharedPointer<PointS> start1 = cylinders.at(i-1)->get_end_ptr();
                    QSharedPointer<PointS> start2 = cylinders.at(i)->get_start_ptr();
                    QSharedPointer<PointS> start(new PointS);
                    start->x = (start1->x + start2->x)/2;
                    start->y = (start1->y + start2->y)/2;
                    start->z = (start1->z + start2->z)/2;
                    QSharedPointer<PointS> end = cylinders.at(i)->get_end_ptr();
                    cylinders.at(i)->set_start_end(start,end);
                } else
                {
                    QSharedPointer<PointS> start1 = cylinders.at(i-1)->get_end_ptr();
                    QSharedPointer<PointS> start2 = cylinders.at(i)->get_start_ptr();
                    QSharedPointer<PointS> end1 = cylinders.at(i)->get_end_ptr();
                    QSharedPointer<PointS> end2 = cylinders.at(i+1)->get_start_ptr();
                    QSharedPointer<PointS> end(new PointS);
                    end->x = (end1->x + end2->x)/2;
                    end->y = (end1->y + end2->y)/2;
                    end->z = (end1->z + end2->z)/2;

                    QSharedPointer<PointS> start(new PointS);
                    start->x = (start1->x + start2->x)/2;
                    start->y = (start1->y + start2->y)/2;
                    start->z = (start1->z + start2->z)/2;
                    cylinders.at(i)->set_start_end(start,end);
                }
            }
        } else
        {
            QSharedPointer<Cylinder> cylinder = cylinders.at(0);

            QVector<QSharedPointer<Cylinder> > cylinders_parent =  segment->get_parent_segment()->get_cylinders();
            QSharedPointer<Cylinder>  cylinder_last = cylinders_parent.last();
            QSharedPointer<PointS> start = cylinder_last->get_end_ptr();
            QSharedPointer<PointS> end = cylinders.at(0)->get_end_ptr();

            cylinder->set_start_end(start,end);


        }

    } else {


        if(cylinders.size()>1)
        {

            for(size_t i = 0; i < cylinders.size(); i++)
            {

                if(i == 0)
                {



                    QSharedPointer<PointS> start = cylinders.at(0)->get_start_ptr();

                    QSharedPointer<PointS> end1 = cylinders.at(i+1)->get_start_ptr();

                    QSharedPointer<PointS> end2 = cylinders.at(i)->get_end_ptr();

                    QSharedPointer<PointS> end (new PointS);

                    end->x = (end1->x + end2->x)/2;
                    end->y = (end1->y + end2->y)/2;
                    end->z = (end1->z + end2->z)/2;

                    cylinders.at(i)->set_start_end(start,end);

                } else if (i == (cylinders.size()-1))
                {

                    QSharedPointer<PointS> start1 = cylinders.at(i-1)->get_end_ptr();
                    QSharedPointer<PointS> start2 = cylinders.at(i)->get_start_ptr();
                    QSharedPointer<PointS> start (new PointS);
                    start->x = (start1->x + start2->x)/2;
                    start->y = (start1->y + start2->y)/2;
                    start->z = (start1->z + start2->z)/2;
                    QSharedPointer<PointS> end = cylinders.at(i)->get_end_ptr();
                    cylinders.at(i)->set_start_end(start,end);
                } else
                {

                    QSharedPointer<PointS> start1 = cylinders.at(i-1)->get_end_ptr();
                    QSharedPointer<PointS> start2 = cylinders.at(i)->get_start_ptr();
                    QSharedPointer<PointS> end1 = cylinders.at(i)->get_end_ptr();
                    QSharedPointer<PointS> end2 = cylinders.at(i+1)->get_start_ptr();
                    QSharedPointer<PointS> end (new PointS);
                    end->x = (end1->x + end2->x)/2;
                    end->y = (end1->y + end2->y)/2;
                    end->z = (end1->z + end2->z)/2;

                    QSharedPointer<PointS> start (new PointS);
                    start->x = (start1->x + start2->x)/2;
                    start->y = (start1->y + start2->y)/2;
                    start->z = (start1->z + start2->z)/2;
                    cylinders.at(i)->set_start_end(start,end);
                }
            }
        }

    }

}

void ImprovedByAdvancedMedian::improve()
{
    QVector<QSharedPointer<Segment> > segments = _tree->get_all_segments();
    QVectorIterator<QSharedPointer<Segment> > it (segments);
    while(it.hasNext())
    {
        QSharedPointer<Segment> segment = it.next();
        if(!segment->is_root())
        {
            improve_segment(segment);
        }
    }
}

void ImprovedByAdvancedMedian::smooth()
{
    QVector<QSharedPointer<Segment> > segments = _tree->get_all_segments();
    QVectorIterator<QSharedPointer<Segment> > it (segments);
    while(it.hasNext())
    {
        QSharedPointer<Segment> segment = it.next();
        smooth_segment(segment);
    }
}

ImprovedByAdvancedMedian::ImprovedByAdvancedMedian(QSharedPointer<Tree> tree)
{
    _tree = tree;
    improve();
    smooth();
}
