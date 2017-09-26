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



#include "dijkstra.h"




std::vector<int> Dijkstra::get_neighbors(PointS p)
{
    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;
    _kdtree->radiusSearch(p,_coeff.search_range,pointIdxNKNSearch,pointNKNSquaredDistance);
    return pointIdxNKNSearch;
}

PointS Dijkstra::get_nearest_point()
{
    float distance = std::numeric_limits<float>::max();
    int index = -1;
    PointS p_next ;
    boost::heap::fibonacci_heap<heap_data>::iterator it = _priority_queue.begin();
    while(it != _priority_queue.end())
    {
        heap_data hd = *it;
        PointS point_unkndown_distance = hd._p;
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);
        _octree->nearestKSearch(point_unkndown_distance,1,pointIdxNKNSearch,pointNKNSquaredDistance);
        float dist_of_parent= _processed->points[pointIdxNKNSearch[0]].true_distance;
        //        float dist_to_parent = std::sqrt(pointNKNSquaredDistance[0]);
        float dist_candidate = dist_of_parent +  _coeff.punishment_multiplicative*( pointNKNSquaredDistance[0]) + _coeff.punishment_additative;

//        float dist_candidate = dist_of_parent +  pointNKNSquaredDistance[0];

        if(dist_candidate<distance)
        {
            distance = dist_candidate;
            index = pointIdxNKNSearch[0];
            p_next = point_unkndown_distance;
        }
        it++;
    }
    //float dist = std::sqrt(distance);
    PointS parent = _processed->points[index];
    p_next.distance = distance;
    p_next.true_distance = distance;
    p_next.treeID = parent.treeID;
    return p_next;




}

void Dijkstra::initialize()
{
    size_t size = _cloud->points.size();
    for(size_t i = 0; i < size; i++)
    {
        _cloud->points[i].was_visited = false;
        _cloud->points[i].true_distance = std::numeric_limits<float>::max();
        _cloud->points[i].distance = std::numeric_limits<float>::max();
        _cloud->points[i].rangebin = -1;
        _cloud->points[i].cluster = -1;
        _cloud->points[i].treeID = -1;
    }
    initialize_start_clusters();
    initialize_heap();
    _octree.reset(new pcl::octree::OctreePointCloudSearch<PointS> (_coeff.OCTREE_CELL_SIZE));
    _processed.reset(new PointCloudS);
    _unprocessed.reset(new PointCloudS);
    _octree->setInputCloud(_processed);
    _octree->addPointsFromInputCloud();
}

void Dijkstra::initialize_start_clusters()
{
    _kdtree.reset(new pcl::KdTreeFLANN<PointS>);
    _kdtree->setInputCloud(_cloud);
    QVectorIterator<PointCloudS::Ptr> it(_start_clouds);
    int treeID = 0;
//    qDebug() << "a";
    while(it.hasNext())
    {
        PointCloudS::Ptr start_cloud = it.next();
        size_t size = start_cloud->points.size();
        for(size_t i = 0; i < size; i++)
        {
            PointS query = start_cloud->points[i];
            std::vector<int> pointIdxNKNSearch(1);
            std::vector<float> pointNKNSquaredDistance(1);
            _kdtree->nearestKSearch (query, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
            int index = pointIdxNKNSearch[0];
            float dist  = std::sqrt(pointNKNSquaredDistance[0]);
            if(dist< 0.05)
            {
                _cloud->points[index].true_distance = dist;
                _cloud->points[index].distance = 0;
                _cloud->points[index].treeID = treeID;
            }

        }
        treeID++;
    }
    size_t size = _cloud->points.size();
    for(size_t i = 0; i < size; i++)
    {
        if(_cloud->points[i].distance == 0)
            _cloud->points[i].true_distance = 0;
    }


    //    _kdtree.reset(new pcl::KdTreeFLANN<PointS>);
    //    _kdtree->setInputCloud(_cloud);
    //    QVectorIterator<PointCloudS::Ptr> it(_start_clouds);
    //    int treeID = 0;
    //    while(it.hasNext())
    //    {
    //        PointCloudS::Ptr start_cloud = it.next();
    //        size_t size = start_cloud->points.size();
    //        for(size_t i = 0; i < size; i++)
    //        {
    //            PointS query = start_cloud->points[i];
    //            std::vector<int> pointIdxNKNSearch(1);
    //            std::vector<float> pointNKNSquaredDistance(1);
    //            _kdtree->nearestKSearch (query, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
    //            int index = pointIdxNKNSearch[0];
    //            float dist  = std::sqrt(pointNKNSquaredDistance[0]);
    //            if(dist< _min_dist_init)
    //            {
    //            _cloud->points[index].true_distance = 0;
    //            _cloud->points[index].distance = 0;
    //            _cloud->points[index].treeID = treeID;
    //            }

    //        }
    //        treeID++;
    //    }
}

void Dijkstra::initialize_heap()
{
    _priority_queue.clear();
    size_t size = _cloud->points.size();
    handle.clear();
    for(size_t i = 0; i < size; i++)
    {
        Heap::handle_type h = _priority_queue.push(_cloud->points[i]);
        handle.push_back(h);
        (*h).handle = h;
        //        _counter_queue++;
    }
}

void Dijkstra::compute()
{
    bool not_finished = true;
    while(!_priority_queue.empty() && not_finished)
    {
        PointS p = _priority_queue.top()._p;
        int index = get_index(p);


        if(_cloud->points[index].distance > 100000)
        {
            p = get_nearest_point();
        }

        else
        {
            _priority_queue.pop();
            //            _counter_queue--;
            _cloud->points[index].was_visited = true;
            _octree->addPointToCloud(p,_processed);
        }
        std::vector<int> neighborindex = get_neighbors(p);
        //        if(neighborindex.size()<=0)
        //        {
        //            qDebug () <<  "Dijkstra::compute() critical error";
        ////            neighborindex = get_next_cluster(p);
        //        }
        size_t size = neighborindex.size();
        if(size<=0)
        {
            qDebug() << "Dijkstra::compute() critical error";
        }
        for(size_t i = 0; i < size; i++)
        {
            int index = neighborindex.at(i);
            //            qDebug() << "a2" << i << ";" << index << ";" <<_cloud->points.size() << ";" << size;
            PointS p2 = _cloud->points[index];
            //            qDebug() << "a2" << i << ";" << _cloud->points.size();
            if(!p2.was_visited)
            {
                float d = SimpleMath<float>::get_distance(p,p2);
                float dist = d*d;
                float alt  = p.true_distance + dist;
                if(alt < p2.true_distance)
                {
                    p2.true_distance = alt;
                    p2.distance = p.distance+dist;
                    p2.treeID = p.treeID;
                    _cloud->points[index].true_distance = alt;
                    _cloud->points[index].distance = p2.distance;
                    _cloud->points[index].treeID = p.treeID;
                    _priority_queue.decrease(handle[index],p2);
                }
            }

        }
    }
}

int Dijkstra::get_index(PointS p)
{
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);
    _kdtree->nearestKSearch (p, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
    int index = pointIdxNKNSearch[0];
    return index;
}





Dijkstra::Dijkstra(PointCloudS::Ptr cloud,QVector<PointCloudS::Ptr> start_clouds, DijkstraCoefficients coeff): _cloud(cloud), _start_clouds(start_clouds), _coeff(coeff)
{
    initialize();
    compute();
}
