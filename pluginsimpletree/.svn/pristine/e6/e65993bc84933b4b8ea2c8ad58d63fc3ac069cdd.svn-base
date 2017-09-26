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

#include "dijkstra_attractor.h"







QVector<pcl::ModelCoefficients> Dijkstra_Attractor::attractor_cylinders() const
{
    return _attractor_cylinders;
}

void Dijkstra_Attractor::initialize()
{

    _octree_tree.reset(new pcl::octree::OctreePointCloudSearch<PointS> (0.07));
    _cloud_tree.reset(new PointCloudS);
    initialize_segment(_tree->get_root_segment(),0,0);


    _octree_tree->setInputCloud(_cloud_tree);
    _octree_tree->addPointsFromInputCloud();

    initialize_attractors();
    initialize_heap();

}

void Dijkstra_Attractor::initialize_attractors()
{
    _kdtree.reset(new pcl::KdTreeFLANN<PointS>);
    _kdtree->setInputCloud(_cloud_tree);


    _octree_attractor.reset(new pcl::octree::OctreePointCloudSearch<PointS> (0.07));
    size_t size = _cloud_attractor->points.size();
    for(size_t i = 0; i < size ;  i++)
    {
        PointS p = _cloud_attractor->points[i];
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);
        _kdtree->nearestKSearch (p, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
        PointS connector = _cloud_tree->points[pointIdxNKNSearch[0] ];
        float dist_orig = connector.distance;
        float dist_sqrd_orig = connector.true_distance;
        _cloud_attractor->points[i].distance = (dist_orig + std::sqrt(pointNKNSquaredDistance[0]));
        _cloud_attractor->points[i].true_distance = (dist_sqrd_orig + pointNKNSquaredDistance[0]);
        _cloud_attractor->points[i].was_visited = false;
    }

    _octree_attractor->setInputCloud(_cloud_attractor);
    _octree_attractor->addPointsFromInputCloud();
}

void Dijkstra_Attractor::initialize_heap()
{
    _priority_queue.clear();
    size_t size = _cloud_attractor->points.size();
    handle.clear();
    for(size_t i = 0; i < size; i++)
    {
        Heap2::handle_type h = _priority_queue.push(_cloud_attractor->points[i]);
        handle.push_back(h);
        (*h).handle = h;
    }
}

void Dijkstra_Attractor::compute()
{
    int size = _cloud_tree->points.size();
    int max_size = std::min(size,10);

    while(!_priority_queue.empty())
    {
        PointS pt_attractor = _priority_queue.top()._p;
        _priority_queue.pop();

        std::vector<int> pointIdxNKNSearch(max_size);
        std::vector<float> pointNKNSquaredDistance(max_size);
        _octree_tree->nearestKSearch (pt_attractor, max_size, pointIdxNKNSearch, pointNKNSquaredDistance);

        pcl::ModelCoefficients cylinder;
        int ind = 0;



        float dist = pt_attractor.distance;
        float dist_sqrd = pt_attractor.true_distance;
        for(int i = 0; i < max_size; i++)
        {
            int index = pointIdxNKNSearch[i];
            PointS origin =_cloud_tree->points[index];


            if((origin.true_distance + pointNKNSquaredDistance[i]) <= dist_sqrd)
            {
                dist_sqrd = (origin.true_distance + pointNKNSquaredDistance[i]);
                dist = (origin.true_distance + std::sqrt(pointNKNSquaredDistance[i]));
                ind = i;
            }
        }


        pt_attractor.distance = dist;
        pt_attractor.true_distance = dist_sqrd;
        int index_attractor = get_index(pt_attractor);
        _cloud_attractor->points[index_attractor].was_visited = true;

        std::vector<int> pointIdxNKNSearch_attr(max_size);
        std::vector<float> pointNKNSquaredDistance_attr(max_size);
        _octree_attractor->nearestKSearch (pt_attractor, max_size, pointIdxNKNSearch_attr, pointNKNSquaredDistance_attr);
        for(int i = 0; i < max_size; i++)
        {
            int index = pointIdxNKNSearch_attr[i];
            PointS atr = _cloud_attractor->points[index];
            if(!atr.was_visited)
            {
                float dist_sqrd = pointNKNSquaredDistance_attr[i];
                float dist = pointNKNSquaredDistance_attr[i];

                float potential_dist_sqrd = pt_attractor.true_distance + dist_sqrd;
                if(potential_dist_sqrd < atr.true_distance)
                {
                    atr.true_distance = potential_dist_sqrd;
                    atr.distance      = pt_attractor.distance + dist;
                    _cloud_attractor->points[index].true_distance = potential_dist_sqrd;
                    _cloud_attractor->points[index].distance = atr.distance;
                    _priority_queue.decrease(handle[index],atr);
                }
            }
        }

        PointS origin =_cloud_tree->points[pointIdxNKNSearch[ind] ];
        cylinder.values.resize(7);
        cylinder.values[0] = origin.x;
        cylinder.values[1] = origin.y;
        cylinder.values[2] = origin.z;
        cylinder.values[3] = pt_attractor.x - origin.x;
        cylinder.values[4] = pt_attractor.y - origin.y;
        cylinder.values[5] = pt_attractor.z - origin.z;
        cylinder.values[6] = 0;

        _attractor_cylinders.push_back(cylinder);

    }
}

int Dijkstra_Attractor::get_index(PointS p)
{
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);
    _octree_attractor->nearestKSearch (p, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
    int index = pointIdxNKNSearch[0];
    return index;
}

Dijkstra_Attractor::Dijkstra_Attractor(PointCloudS::Ptr cloud_attractor, QSharedPointer<Tree> tree)
{
    _cloud_attractor = cloud_attractor;
    _tree = tree;
    if(_tree->get_all_cylinders().size()> 10)
    {
        if(_cloud_attractor->points.size() > 10)
        {
            initialize();
            compute();
        } else {
            qDebug () << "Dijkstra_Attractor: not enough attractor";
        }

    } else {
        qDebug() << "Dijkstra_Attractor: not enough cylinders";
    }
}





void Dijkstra_Attractor::initialize_segment(QSharedPointer<Segment> segment, float distance, float squared_distance)
{
    QVector<QSharedPointer<Cylinder> >cylinders = segment->get_cylinders();
    QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        float l = cylinder->get_length();
        float sl = l*l;
        PointS end = cylinder->get_end();
        distance += l;
        squared_distance += sl;
        end.distance = distance;
        end.true_distance = squared_distance;
        _cloud_tree->points.push_back(end);
    }
    QVector<QSharedPointer<Segment> > children = segment->get_child_segments();
    QVectorIterator<QSharedPointer<Segment> > git (children);
    while(git.hasNext())
    {
        QSharedPointer<Segment> child = git.next();
        initialize_segment(child, distance, squared_distance);

    }
}
