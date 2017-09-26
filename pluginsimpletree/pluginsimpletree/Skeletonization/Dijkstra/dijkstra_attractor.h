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
#ifndef DIJKSTRA_ATTRACTOR_H
#define DIJKSTRA_ATTRACTOR_H


#include "SimpleTree4/model/pointsimpletree.h"
#include "SimpleTree4/math/simplemath.h"
#include "SimpleTree4/model/tree.h"

#include <pcl/kdtree/kdtree.h>
#include <pcl/common/geometry.h>
#include <pcl/octree/octree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

#include <QSharedPointer>
#include <QVector>
#include <QDebug>

#include <boost/heap/fibonacci_heap.hpp>

struct heap_data2;
using Heap2 = boost::heap::fibonacci_heap<heap_data2>;

struct compare_point2
{
    bool operator()(const PointS& n1, const PointS n2) const
    {
        return n1.true_distance > n2.true_distance;
    }
};

struct heap_data2
{
    PointS _p;
    Heap2::handle_type handle;

    heap_data2(PointS p) : _p(p), handle() {}

    bool operator<(heap_data2 const & rhs) const {
        return _p.true_distance > rhs._p.true_distance;
    }
};

class Dijkstra_Attractor
{

    QSharedPointer<Tree> _tree;

    PointCloudS::Ptr _cloud_attractor;

    PointCloudS::Ptr _cloud_tree;

    QVector<pcl::ModelCoefficients> _attractor_cylinders;

    pcl::octree::OctreePointCloudSearch<PointS>::Ptr _octree_tree;

    pcl::octree::OctreePointCloudSearch<PointS>::Ptr _octree_attractor;

    Heap2 _priority_queue;

    QVector<Heap2::handle_type> handle;

    QSharedPointer<pcl::KdTreeFLANN<PointS> >_kdtree;

    void initialize();

    void initialize_segment(QSharedPointer<Segment> segment, float distance, float squared_distance);

    void initialize_attractors();

    void initialize_heap();

    void compute();

int get_index(PointS p);


public:

    Dijkstra_Attractor(PointCloudS::Ptr cloud_attractor, QSharedPointer<Tree> tree);

    QVector<pcl::ModelCoefficients> attractor_cylinders() const;
};

#endif // DIJKSTRA_ATTRACTOR_H
