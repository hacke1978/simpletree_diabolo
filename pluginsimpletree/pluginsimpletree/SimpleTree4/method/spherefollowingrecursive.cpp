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

#include "spherefollowingrecursive.h"

QVector<pcl::ModelCoefficients> SphereFollowingRecursive::get_cylinders() const
{
    return _cylinders;
}



MethodCoefficients SphereFollowingRecursive::get_coeff() const
{
    return _coeff;
}

void SphereFollowingRecursive::set_coeff(const MethodCoefficients &coeff)
{
    _coeff = coeff;
}

QSharedPointer<Tree> SphereFollowingRecursive::get_tree()
{
    return _tree;
}


void SphereFollowingRecursive::remove_attractor(PointS &attr)
{
    int index = -1;
    for(int i = 0; i < _attractors->points.size(); i++)
    {
        if(SimpleMath<float>::are_equal(_attractors->points.at(i),attr))
        {
            index = i;
        }
    }
    std::vector<PointS, Eigen::aligned_allocator<PointS> >::const_iterator it = _attractors->points.begin() + index;
    _attractors->points.erase(it);
    _list.removeAt(index);
}

void SphereFollowingRecursive::initiate_list()
{
    _list.clear();
    for(int i = 0; i< _attractors->points.size(); i++)
    {
        PointS attr = _attractors->points.at(i);
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        _octree->nearestKSearch(attr,1, pointIdxRadiusSearch, pointRadiusSquaredDistance);
        PointS end_point = _end_pts->points.at(pointIdxRadiusSearch.at(0));
        float dist = std::sqrt(pointRadiusSquaredDistance.at(0));
        Triplet trip;
        trip.attr = attr;
        trip.end  = end_point;
        trip.dist = dist;
        _list.push_back(QVariant::fromValue(trip));
    }
}

void SphereFollowingRecursive::update_list(PointS &p)
{
    for(int i = 0; i < _list.size(); i++)
    {
        Triplet trip = _list.at(i).value<Triplet>();
        PointS attr = trip.attr;
        float dist = SimpleMath<float>::get_distance(p,attr);
        if(dist<trip.dist)
        {
            Triplet trip_new;
            trip_new.attr = attr;
            trip_new.end = p;
            trip_new.dist = dist;
            _list.replace(i,QVariant::fromValue(trip_new));
        }

    }
}

QPair<PointS, PointS> SphereFollowingRecursive::find_pair()
{
    int index = -1;
    float min_dist = std::numeric_limits<float>::max();
    for(int i = 0; i < _list.size(); i++)
    {
        Triplet trip = _list.at(i).value<Triplet>();
        if(trip.dist<min_dist)
        {
            min_dist = trip.dist;
            index =i;
        }
    }
    Triplet trip = _list.at(index).value<Triplet>();
    QPair<PointS,PointS> pair;
    pair.first = trip.end;
    pair.second = trip.attr;
    return pair;
}

void SphereFollowingRecursive::do_recursion()
{
    QTime time;
    time.restart();
    OptimizationSphereFollowing optim (_cloud_tree, _coeff, _subdivide_stem_and_branch_points, _is_multithreaded);
    optim.optimize();
    int msec = time.elapsed();
    int minutes = msec/60000;
    QString str2 = "";
    str2.append(_coeff.id);
    str2.append(" the raster optimization took ");
    str2.append(QString::number(minutes));
    str2.append(" minutes");
    emit emit_qstring_spherefollowingrecursive(str2);
    time.restart();
    _coeff = optim._coeff_end;
    if(_coeff.use_dhs)
    {

        QSharedPointer<OptimizationDownHillSimplex> downhill
                (new OptimizationDownHillSimplex(0.00000001,_coeff,_cloud_tree,_subdivide_stem_and_branch_points, _is_multithreaded));
        QObject::connect(downhill.data(), SIGNAL (emit_counter(int)), this, SLOT (receive_counter(int)));
        QObject::connect(downhill.data(), SIGNAL (emit_qstring(QString)), this, SLOT (receive_qstring(QString)));
        if(_coeff.optimze_stem)
        {
            downhill->set_ndim(6);
        } else
        {
            downhill->set_ndim(5);
        }
        downhill->minimize();
        msec = time.elapsed();
        minutes = msec/60000;
        str2 = "";
        str2.append(_coeff.id);
        str2.append(" the downhill simplex optimization took ");
        str2.append(QString::number(minutes));
        str2.append(" minutes");
        emit emit_qstring_spherefollowingrecursive(str2);
        time.restart();
        _coeff = downhill->get_end_coeff();
    }
    SphereFollowing2 spherefollowing(_coeff,_cloud_tree, _subdivide_stem_and_branch_points);
    spherefollowing.sphere_following();
    _cylinders = spherefollowing.get_cylinders();
    ImproveByAttractor ia (_cloud_tree,spherefollowing.get_remaining_points(),_coeff,_cylinders);
    _cylinders = ia.get_cylinders();
    _coeff = ia.get_coeff();
    ImproveByAttractor ia2 (_cloud_tree,_cloud_noise,_coeff,_cylinders);
    _cylinders = ia2.get_cylinders();
    _coeff = ia2.get_coeff();
    msec = time.elapsed();
    minutes = msec/60000;
    str2 = "";
    str2.append(_coeff.id);
    str2.append(" the attractor stuff took ");
    str2.append(QString::number(minutes));
    str2.append(" minutes with ");
    emit emit_qstring_spherefollowingrecursive(str2);
    time.restart();
    emit emit_finished();
}

QPair<PointS, PointS> SphereFollowingRecursive::find_closest_pair(PointCloudS::Ptr cloud_model, PointCloudS::Ptr cloud_attractor)
{
    PointS model = cloud_model->points.at(0);
    PointS attractor = cloud_attractor->points.at(0);
    QPair<PointS, PointS> pair (model,attractor);
    float min_dist = SimpleMath<float>::get_distance(model,attractor);
    size_t index = 0;
    size_t index_model = 0;
    for(size_t i = 0; i < cloud_attractor->points.size(); i++)
    {
        attractor = cloud_attractor->points.at(i);
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        _octree->nearestKSearch(attractor,1, pointIdxRadiusSearch, pointRadiusSquaredDistance);
        model = cloud_model->points.at(pointIdxRadiusSearch.at(0));
        float dist = std::sqrt(pointRadiusSquaredDistance.at(0));
        if(dist<min_dist)
        {
            min_dist = dist;
            index = i;
            index_model = pointIdxRadiusSearch.at(0);
            pair = QPair<PointS, PointS>(model,attractor);
        }
    }
    std::vector<PointS, Eigen::aligned_allocator<PointS> >::const_iterator it = cloud_attractor->points.begin() + index;
    cloud_attractor->points.erase(it);
    _radius = _cylinders.at(index_model).values.at(6);
    return pair;




}

PointCloudS::Ptr SphereFollowingRecursive::generate_end_point_cloud(QVector<pcl::ModelCoefficients> coeff)
{
    PointCloudS::Ptr end_pts (new PointCloudS);
    end_pts->points.resize(coeff.size());
    for(size_t i = 0; i< coeff.size(); i++)
    {
        pcl::ModelCoefficients cf = coeff.at(i);
        float x = cf.values[0] + cf.values[3];
        float y = cf.values[1] + cf.values[4];
        float z = cf.values[2] + cf.values[5];
        PointS p;
        p.x = x;
        p.y = y;
        p.z = z;
        end_pts->points[i] = p;
    }
    return end_pts;
}

PointCloudS::Ptr SphereFollowingRecursive::generate_start_point_cloud(QVector<pcl::ModelCoefficients> coeff)
{
    PointCloudS::Ptr start_pts (new PointCloudS);
    start_pts->points.resize(coeff.size());
    for(size_t i = 0; i< coeff.size(); i++)
    {
        pcl::ModelCoefficients cf = coeff.at(i);
        float x = cf.values[0];
        float y = cf.values[1];
        float z = cf.values[2];
        PointS p;
        p.x = x;
        p.y = y;
        p.z = z;
        start_pts->points[i] = p;
    }
    return start_pts;
}

void SphereFollowingRecursive::receive_qstring(QString qstr)
{
    emit emit_qstring_spherefollowingrecursive(qstr);
}

void SphereFollowingRecursive::receive_counter(int counter)
{
    if(_is_multithreaded)
    {
        emit emit_counter_spherefollowingrecursive(counter);
    }
}

SphereFollowingRecursive::SphereFollowingRecursive(PointCloudS::Ptr cloud, PointCloudS::Ptr cloud_noise, MethodCoefficients coeff, bool subdivide_stem_and_branch_points, bool is_multithreaded)

{
    _cloud_noise = cloud_noise;
    _cloud_tree = cloud;
    _coeff = coeff;
    _subdivide_stem_and_branch_points = subdivide_stem_and_branch_points;
    _is_multithreaded = is_multithreaded;

}
