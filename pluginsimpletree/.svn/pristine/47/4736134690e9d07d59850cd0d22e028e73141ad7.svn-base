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

#include "reordertree.h"

float ReorderTree::get_maximum_volume_to_leave(QSharedPointer<Segment> seg)
{
    QSharedPointer<Tree> tree (new Tree(seg,"tree"));
    QVector<float> volumina;
    QVector<QSharedPointer<Segment> > segments = tree->get_leave_segments(seg);
    QVectorIterator<QSharedPointer<Segment> > it (segments);
    while(it.hasNext())
    {
        QSharedPointer<Segment>  leaf = it.next();
        float volume = tree->get_volume_to_base(leaf, seg);
        volumina.push_back(volume);
    }
    float result = *(std::max_element(volumina.begin(), volumina.end()));
    return result;
}
bool ReorderTree::compareSegments0(QSharedPointer<Segment> seg1, QSharedPointer<Segment> seg2)
{
    float volume1;
    float volume2;

    volume1 = seg1->get_median_radius();
    volume2 = seg2->get_median_radius();
    return(volume2<volume1);
}
bool ReorderTree::compareSegments1(QSharedPointer<Segment> seg1, QSharedPointer<Segment> seg2)
{
    float volume1;
    float volume2;
    volume1 = get_maximum_volume_to_leave(seg1);
    volume2 = get_maximum_volume_to_leave(seg2);
    return(volume2<volume1);
}
bool ReorderTree::compareSegments2(QSharedPointer<Segment> seg1, QSharedPointer<Segment> seg2)
{
    float volume1;
    float volume2;

    QVector<QSharedPointer<Segment> > segments1 = seg1->get_child_segments_recursively();
    QVector<QSharedPointer<Segment> > segments2 = seg2->get_child_segments_recursively();
    QVectorIterator<QSharedPointer<Segment> > it1 (segments1);
    while(it1.hasNext())
    {
        QSharedPointer<Segment> seg = it1.next();
        volume1 += seg->get_volume();
    }

    QVectorIterator<QSharedPointer<Segment> > it2 (segments2);
    while(it2.hasNext())
    {
        QSharedPointer<Segment> seg = it2.next();
        volume2 += seg->get_volume();
    }
    return(volume2<volume1);
}


bool ReorderTree::compareSegments3(QSharedPointer<Segment> seg1, QSharedPointer<Segment> seg2)
{
    float angle1 = 0;
    float angle2 = 0;
    if(!seg1->is_root())
    {
    QSharedPointer<Segment> parent = seg1->get_parent_segment();
    Eigen::Vector3f axis_1;
    axis_1[0] = seg1->get_cylinders().last()->get_end().x-seg1->get_cylinders().first()->get_start().x;
    axis_1[1] = seg1->get_cylinders().last()->get_end().y-seg1->get_cylinders().first()->get_start().y;
    axis_1[2] = seg1->get_cylinders().last()->get_end().z-seg1->get_cylinders().first()->get_start().z;

    Eigen::Vector3f axis_2;
    axis_2[0] = seg2->get_cylinders().last()->get_end().x-seg2->get_cylinders().first()->get_start().x;
    axis_2[1] = seg2->get_cylinders().last()->get_end().y-seg2->get_cylinders().first()->get_start().y;
    axis_2[2] = seg2->get_cylinders().last()->get_end().z-seg2->get_cylinders().first()->get_start().z;


    Eigen::Vector3f axis_3;
    axis_3[0] = parent->get_cylinders().last()->get_end().x-parent->get_cylinders().first()->get_start().x;
    axis_3[1] = parent->get_cylinders().last()->get_end().y-parent->get_cylinders().first()->get_start().y;
    axis_3[2] = parent->get_cylinders().last()->get_end().z-parent->get_cylinders().first()->get_start().z;
    angle1 = SimpleMath<float>::angle_between(axis_1,axis_3);
    angle2 = SimpleMath<float>::angle_between(axis_2,axis_3);





    }
    return(angle1<angle2);
}



bool ReorderTree::compareSegments(QSharedPointer<Segment> seg1, QSharedPointer<Segment> seg2)
{
    float volume1;
    float volume2;

    volume1 = get_maximum_volume_to_leave(seg1);
    volume2 = get_maximum_volume_to_leave(seg2);

    return(volume2<volume1);
}

bool ReorderTree::compareStemSegments(QSharedPointer<Segment> seg1, QSharedPointer<Segment> seg2)
{
    return(seg2->get_branch_order()<seg1->get_branch_order());
}

void ReorderTree::reorder()
{

    QVector<QSharedPointer<Segment> > segments = _tree->get_all_segments();
    QVectorIterator<QSharedPointer<Segment> > it (segments);
    while(it.hasNext())
    {
        QSharedPointer<Segment> segment = it.next();
        segment->set_branch_id(-1);
        segment->set_branch_order(-1);
        segment->set_id(-1);

    }
//    QSharedPointer<Segment> top_segment = detect_top_segment();
//    recursive_stem_tagging(top_segment);


    QVectorIterator<QSharedPointer<Segment> > git (segments);
    while(git.hasNext())
    {
        QSharedPointer<Segment> segment = git.next();

        if(segment->get_branch_order() == -1)
        {
            if(segment->get_child_segments().size()>1)
            {
                QVector<QSharedPointer<Segment> > children = segment->get_child_segments();
                if(_compare_type == 0)
                {
                    std::sort(children.begin(), children.end(), compareSegments0);
                }
                else if (_compare_type == 1)
                {
                    std::sort(children.begin(), children.end(), compareSegments1);
                }
                else if (_compare_type == 2)
                {
                    std::sort(children.begin(), children.end(), compareSegments2);
                }
                else if (_compare_type == 3)
                {
                    std::sort(children.begin(), children.end(), compareSegments3);
                }
                segment->set_child_segments(children);
            }
        }
    }
}

void ReorderTree::fill_IDs()
{
    QVector<QSharedPointer<Segment> > segments = _tree->get_all_segments();
    QVectorIterator<QSharedPointer<Segment> > it (segments);
    int segment_ID = 0;
    while(it.hasNext())
    {
        QSharedPointer<Segment> segment = it.next();
        segment->set_id(segment_ID);
        segment_ID++;
    }
    generate_branch_order(_tree->get_root_segment(),0);
    generate_branch_ID();
}

void ReorderTree::set_cylinder_ID()
{
    QVector<QSharedPointer<Cylinder> > cylinders = _tree->get_all_cylinders();
    size_t index = 0;
    for(index = 0; index < cylinders.size(); index++)
    {
        QSharedPointer<Cylinder> cylinder = cylinders.at(index);
        cylinder->setID(index);
    }
    for(index = 0; index < cylinders.size(); index++)
    {
        QSharedPointer<Cylinder> cylinder = cylinders.at(index);
        //        cylinder->setID(index);
        QSharedPointer<Cylinder> parent = _tree->get_parent(cylinder);
        if(parent != cylinder)
            cylinder->setParent_ID(parent->getID());
    }
}

void ReorderTree::recursive_stem_tagging(QSharedPointer<Segment> seg)
{
    seg->set_branch_order(0);
    if(seg!=_tree->get_root_segment())
        recursive_stem_tagging(seg->get_parent_segment());
}

void ReorderTree::generate_reverse_pipe_bo(QSharedPointer<Segment> seg)
{
    if(seg->is_leave())
    {
        seg->setReverse_pipe_order(1);
    } else {
        QVector<QSharedPointer<Segment> > children = seg->get_child_segments();

        float order = 0;
        QVectorIterator<QSharedPointer<Segment> > it (children);
        while(it.hasNext())
        {
            QSharedPointer<Segment> child = it.next();
            generate_reverse_pipe_bo(child);
            order += child->getReverse_pipe_order()*child->getReverse_pipe_order();
        }
        order = std::sqrt(order);
        seg->setReverse_pipe_order(order);
    }
}

QSharedPointer<Segment> ReorderTree::detect_top_segment()
{
    QVector<QSharedPointer<Segment> > leave_segments = _tree->get_leave_segments(_tree->get_root_segment());
      //  qDebug() << "number leaves Reorder Tree" << leave_segments.size() ;
    QVectorIterator<QSharedPointer<Segment> > it (leave_segments);
    QSharedPointer<Segment> top_segment;
    float max_height = std::numeric_limits<float>::lowest();
    while(it.hasNext())
    {
        QSharedPointer<Segment> leave = it.next();
        float height = leave->get_cylinders().last()->get_end().z;
        if(height > max_height)
        {
            max_height = height;
            top_segment = leave;
        }
    }
    //qDebug() << "max_height Reorder Tree" << max_height ;
    return top_segment;
}

void ReorderTree::generate_branch_order(QSharedPointer<Segment> segment, int branch_order)
{
    segment->set_branch_order(branch_order);
    QVector<QSharedPointer<Segment> > children = segment->get_child_segments();
    QVectorIterator<QSharedPointer<Segment> > it (children);
    bool first = true;
    while(it.hasNext())
    {
        QSharedPointer<Segment> child = it.next();
        if(first)
        {
            generate_branch_order(child, branch_order);
        } else
        {
            generate_branch_order(child, branch_order+1);
        }
        first = false;
    }
}

QVector<QSharedPointer<Segment> > ReorderTree::get_stem()
{
    QVector<QSharedPointer<Segment> > segments = _tree->get_all_segments();
    QVector<QSharedPointer<Segment> > stem_segments;
    QVectorIterator<QSharedPointer<Segment> > it (segments);
    while(it.hasNext())
    {
        QSharedPointer<Segment> segment = it.next();
        if(segment->get_branch_order()==0)
        {
            stem_segments.push_back(segment);
        }


    }
    return stem_segments;
}

QVector<QSharedPointer<Segment> > ReorderTree::get_branch_roots()
{
    QVector<QSharedPointer<Segment> > stem_segments = get_stem();
    QVector<QSharedPointer<Segment> > branch_roots;
    QVectorIterator<QSharedPointer<Segment> > it(stem_segments);
    while(it.hasNext())
    {
        QSharedPointer<Segment> stem_segment = it.next();
        QVector<QSharedPointer<Segment> > children = stem_segment->get_child_segments();
        QVectorIterator<QSharedPointer<Segment> > git(children);
        bool first = true;
        while(git.hasNext())
        {
            QSharedPointer<Segment> child = git.next();
            if(!first)
            {
                branch_roots.push_back(child);
            }
            first = false;
        }
    }
    return branch_roots;
}

void ReorderTree::generate_branch_ID()
{
    QVector<QSharedPointer<Segment> > branch_roots = get_branch_roots();
    int index = 1;
    QVectorIterator<QSharedPointer<Segment> > it(branch_roots);
    while(it.hasNext())
    {
        QSharedPointer<Segment> branch_root = it.next();
        branch_root->set_branch_id(index);
        QVector<QSharedPointer<Segment> > complete_branch = branch_root->get_child_segments_recursively();
        QVectorIterator<QSharedPointer<Segment> >  git (complete_branch);
        while(git.hasNext())
        {
            QSharedPointer<Segment> branch = git.next();
            branch->set_branch_id(index);

        }
        index++;

    }
}





ReorderTree::ReorderTree(QSharedPointer<Tree> tree, int compare_type)
{
    _compare_type = compare_type;
    _tree = tree;
    RemoveSideWardConnection rswc(_tree);
    reorder();
    fill_IDs();
    set_cylinder_ID();
    generate_reverse_pipe_bo(tree->get_root_segment());
}
