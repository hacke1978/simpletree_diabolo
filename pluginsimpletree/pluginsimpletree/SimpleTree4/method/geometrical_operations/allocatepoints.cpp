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

#include "allocatepoints.h"

QVector<PointCloudS::Ptr> AllocatePoints::get_sub_clouds() const
{
    return _sub_clouds;
}



QVector<pcl::ModelCoefficients> AllocatePoints::get_cylinder_coeff() const
{
    return _cylinder_coeff;
}

void AllocatePoints::initiate()
{
    /**
     * Sets the cylinder ID of each point to 0;
    **/
    for(size_t i = 0; i < _cloud->points.size(); i++)
    {
        _cloud->points[i].ID = -1;
    }
    _cylinders = _tree->get_all_cylinders();


    PointCloudS::Ptr center_points (new PointCloudS);
    _cylinder_coeff.clear();
    _sub_clouds.clear();
    QVectorIterator<QSharedPointer<Cylinder> > git (_cylinders);
    while(git.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = git.next();
        center_points->points.push_back(cylinder->get_center());
        _cylinder_coeff.push_back(cylinder->get_coeff());
        PointCloudS::Ptr sub_cloud (new PointCloudS);
        _sub_clouds.push_back(sub_cloud);
    }

    _kdtree.reset(new pcl::KdTreeFLANN<PointS>);
    _kdtree->setInputCloud(center_points);
}

void AllocatePoints::allocate()
{
    MethodCoefficients cf;
    for(size_t i = 0; i < _cloud->points.size(); i++)
    {
        PointS p = _cloud->points.at(i);
        int K = 6;

        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        _kdtree->nearestKSearch (p, K, pointIdxNKNSearch, pointNKNSquaredDistance);
        QSharedPointer<Cylinder> cylinder = _cylinders.at(pointIdxNKNSearch[0]);
        float distance = std::abs(cylinder->dist_to_point(p));
        int index = pointIdxNKNSearch[0];
        for(size_t j = 1; j < K; j++)
        {
            QSharedPointer<Cylinder> cylinder2 = _cylinders.at(pointIdxNKNSearch[j]);
            float distance2 = std::abs(cylinder2->dist_to_point(p));
            if(distance2 < distance)
            {
                distance = distance2;
                index = pointIdxNKNSearch[j];
            }
        }
        if(distance<cf.ransac_inlier_distance&&_cylinders[index]->get_detection()==DetectionType::SPHEREFOLLOWING)
        {
            _sub_clouds[index]->points.push_back(p);
        }
        _cloud->points[i].ID == index;
    }

}

AllocatePoints::AllocatePoints(QSharedPointer<Tree> tree, PointCloudS::Ptr cloud):_tree(tree), _cloud(cloud)
{
}

void AllocatePoints::compute()
{

    initiate();
    allocate();
}
