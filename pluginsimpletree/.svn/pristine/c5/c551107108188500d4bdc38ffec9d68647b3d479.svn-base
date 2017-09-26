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

#include "optimizationgap.h"

OptimizationGap::OptimizationGap(PointCloudS::Ptr cloud, MethodCoefficients coeff, bool subdivide_stem_and_branch_points): _cloud(cloud), _coeff(coeff), _subdivide_stem_and_branch_points(subdivide_stem_and_branch_points)
{
    SphereFollowing2 spherefollowing(_coeff,_cloud, _subdivide_stem_and_branch_points);
    spherefollowing.sphere_following();

    BuildTree builder(spherefollowing.get_cylinders());
    QSharedPointer<Tree> tree (new Tree(builder.getRoot_segment(),_coeff.id));
    _tree = tree;
}

MethodCoefficients OptimizationGap::get_coeff() const
{
    return _coeff;
}

QVector<QSharedPointer<Segment> > OptimizationGap::extract_segments(QSharedPointer<Tree> tree)
{
    QVector<QSharedPointer<Segment> > all_segments = tree->get_all_segments();
    QVector<QSharedPointer<Segment> > result;
    QVectorIterator<QSharedPointer<Segment> > it(all_segments);
    while(it.hasNext())
    {
        QSharedPointer<Segment> seg = it.next();
        if(seg->get_mean_radius()>_coeff.min_radius_for_pype_test)
        {
            result.push_back(seg);
        }
    }
    return result;
}

float OptimizationGap::segment_area(QSharedPointer<Segment> segment)
{
    float radius = segment->get_mean_radius();
    float area = SimpleMath<float>::_PI*radius*radius;
    return area;
}

float OptimizationGap::sum_children_area(QSharedPointer<Segment> segment)
{
    QVector<QSharedPointer<Segment> > children = segment->get_child_segments();
    QVectorIterator<QSharedPointer<Segment> > it (children);
    float sum = 0;
    while(it.hasNext())
    {
        QSharedPointer<Segment> seg = it.next();
        sum += segment_area(seg);
    }
    return sum;
}

bool OptimizationGap::check_pype(QVector<QSharedPointer<Segment> > segments)
{
    int failure_segments = 0;
    QVectorIterator<QSharedPointer<Segment> > it(segments);
    while(it.hasNext())
    {
        QSharedPointer<Segment> seg = it.next();
        float area_parent = segment_area(seg);
        float area_children = sum_children_area(seg);
        if(area_children == 0)
        {
            failure_segments ++;
        }
        else
        {
            float ratio = area_parent/area_children;
            if(ratio<_coeff.min_ratio_pype||ratio>_coeff.max_ratio_pype)
            {
                failure_segments++;
            }
        }

    }
    if(failure_segments>_coeff.max_number_failure_segments)
    {
        return false;
    }
    return true;
}

void OptimizationGap::optimize()
{
    QVector<QSharedPointer<Segment> > segments = extract_segments(_tree);
    int number_iterations = 0;
    while(number_iterations<_coeff.max_times_cluster_extension && (!check_pype(segments)))
    {
        number_iterations ++;
        _coeff.epsilon_cluster_stem   = _coeff.epsilon_cluster_stem*2;
        _coeff.times_cluster_extension = number_iterations;
        SphereFollowing2 spherefollowing(_coeff,_cloud, _subdivide_stem_and_branch_points);
        spherefollowing.sphere_following();

        BuildTree builder(spherefollowing.get_cylinders());
        QSharedPointer<Tree> tree (new Tree(builder.getRoot_segment(),_coeff.id));
        _tree = tree;
        segments = extract_segments(_tree);
        qDebug() << "optimization gap doubled...";
    }
}
