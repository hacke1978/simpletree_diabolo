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

#include "improvebypipemodel.h"




void ImproveByPipeModel::improve_cylinders()
{
    QVector<QSharedPointer<Cylinder> > cylinders = _tree->get_all_cylinders();
    float min_rad = 0.0001;
    if(_interpolate_all)
    {
        min_rad = 4;
    }
    QVector<QSharedPointer<Cylinder> > improve_cylinders;
    QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        if(cylinder->get_segment()!=  _tree->get_root_segment())
        {
            if(_interpolate_all)
            {
                improve_cylinders.push_back(cylinder);
            } else {
                if(cylinder->get_allometry_improvement()!=AllometryImproved::NOALLOM||cylinder->get_radius()<= 0.0001)
                {
                    improve_cylinders.push_back(cylinder);
                }
            }
        }
    }
    QVectorIterator<QSharedPointer<Cylinder> > git(improve_cylinders);
    while(git.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = git.next();
        if(cylinder != cylinder->get_segment()->get_cylinders().first())
        {
            improve_inside_segment(cylinder);
        }
        else
        {
            improve_over_branch_junction(cylinder);
        }
    }
}

void ImproveByPipeModel::improve_inside_segment(QSharedPointer<Cylinder> cylinder)
{

    QSharedPointer<Segment> segment = cylinder->get_segment();
    QVector<QSharedPointer<Cylinder> > cylinders = segment->get_cylinders();
    int index = -1;
    for(int i = 0; i< cylinders.size(); i++)
    {
        if(cylinder == cylinders.at(i))
        {
            index = i;
        }
    }
    QSharedPointer<Cylinder> parent = cylinders.at(index-1);
    QSharedPointer<Cylinder> child = cylinder;
    float radius_parent = parent->get_radius();
    float radius_cylinder_original = child->get_radius();
    float length_parent = _tree->get_length_to_leave_save(parent);
    float length_child  = _tree->get_length_to_leave_save(child);
    if(length_parent > 0)
    {
        float ratio = length_child/length_parent;
        float radius_child_predicted = std::pow(ratio,1.5)*radius_parent;
        if( (radius_cylinder_original < ((1-_percentage) * radius_child_predicted ))
                || (radius_cylinder_original > ((1+_percentage) * radius_child_predicted)))
        {
            cylinder->set_radius(radius_child_predicted);
            cylinder->set_allometry_improvement(AllometryImproved::ALLOM_PIPE);
            if(radius_cylinder_original<=0.0001)
            {
                cylinder->set_detection(DetectionType::ATTRACTOR);
            }
        }
    }
}

void ImproveByPipeModel::improve_over_branch_junction(QSharedPointer<Cylinder> cylinder)
{
    QSharedPointer<Segment> seg = cylinder->get_segment();
    QSharedPointer<Cylinder> parent = seg->get_parent_segment()->get_cylinders().last();

    float radius_parent = parent->get_radius();
    float radius_cylinder_orginal = cylinder->get_radius();
    float length_cylinder;/*
    length_cylinder = _tree->get_length_to_leave_save(cylinder);
    QVector<QSharedPointer<Segment> > siblings = seg->get_parent_segment()->get_child_segments();
    QVectorIterator<QSharedPointer<Segment> > bit(siblings);
    float sum_length = 0;
    while(bit.hasNext())
    {
        QSharedPointer<Segment> sibling = bit.next();
        QSharedPointer<Cylinder> cylinder = sibling->get_cylinders().at(0);
        sum_length += _tree->get_length_to_leave_save(cylinder);
    }
    float length_all_children;// = _tree->get_growth_length(seg->get_parent_segment()) - seg->get_parent_segment()->get_length();*/
    float length_all_children = _tree->get_growth_length(parent);
    length_cylinder = _tree->get_growth_length(cylinder);
    if(length_all_children==0)
    {
        qDebug() << "ImproveByPipeModel::improve()";
    }

    float radius_child_predicted = radius_parent*(std::pow( (length_cylinder/length_all_children),(1/2.49) )  );
  //  qDebug() << "branch junction" << radius_parent << ";" << radius_cylinder_orginal << ";" << radius_child_predicted;
    if( (radius_cylinder_orginal < ((1-_percentage) * radius_child_predicted ))
            || (radius_cylinder_orginal > ((1+_percentage) * radius_child_predicted)))
    {
        cylinder->set_radius(radius_child_predicted);
        cylinder->set_allometry_improvement(AllometryImproved::ALLOM_PIPE);
        if(radius_cylinder_orginal<=0.0001)
        {
            cylinder->set_detection(DetectionType::ATTRACTOR);
        }
    }
}

ImproveByPipeModel::ImproveByPipeModel(QSharedPointer<Tree> tree, bool interpolate_all, float percentage)
{
    _tree = tree;
    _interpolate_all = interpolate_all;
    _percentage = percentage;
    improve_cylinders();
}
