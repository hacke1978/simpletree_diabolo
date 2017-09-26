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



#ifndef DIJKSTRA_H
#define DIJKSTRA_H
#include "SimpleTree4/model/pointsimpletree.h"
#include "SimpleTree4/math/simplemath.h"
#include "dijkstra_coefficients.h"
#include <QSharedPointer>
#include <QVector>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/geometry.h>
#include <boost/heap/fibonacci_heap.hpp>
#include "clusterandsort.h"
#include <pcl/octree/octree.h>

#include <QObject>
#include <QDebug>

struct heap_data;
using Heap = boost::heap::fibonacci_heap<heap_data>;

struct compare_point
{
    bool operator()(const PointS& n1, const PointS n2) const
    {
        return n1.true_distance > n2.true_distance; //check if sign switch
    }
};

struct heap_data
{
    PointS _p;
    Heap::handle_type handle;

    heap_data(PointS p) : _p(p), handle() {}

    bool operator<(heap_data const & rhs) const {
        return _p.true_distance > rhs._p.true_distance;
    }
};

class Dijkstra
{

    float _min_dist_init = 0.05f;

    pcl::octree::OctreePointCloudSearch<PointS>::Ptr _octree;

    DijkstraCoefficients _coeff;

    // typedef boost::heap::fibonacci_heap<PointS, boost::heap::compare<compare_point>  >::handle_type handle_t;

    std::vector<int> get_neighbors(PointS p);

    PointS get_nearest_point();

    Heap _priority_queue;

    //boost::heap::fibonacci_heap<PointS, boost::heap::compare<compare_point> > _priority_queue;

    QVector<Heap::handle_type> handle;

    PointCloudS::Ptr _cloud;

    QVector<PointCloudS::Ptr> _start_clouds;

    PointCloudS::Ptr _processed;

    PointCloudS::Ptr _unprocessed;

    QSharedPointer<pcl::KdTreeFLANN<PointS> >_kdtree;

    void initialize();

    void initialize_start_clusters();

    void initialize_heap();

    void compute();

    int get_index(PointS p);

    void initialize_cluster_jump();

    int _counter_queue = 0;

//    std::vector<int> get_next_cluster(PointS &p);




public:

    Dijkstra(PointCloudS::Ptr cloud, QVector<PointCloudS::Ptr> start_clouds, DijkstraCoefficients coeff);
};

#endif // DIJKSTRA_H
