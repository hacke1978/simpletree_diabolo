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

#include "enrichcloud.h"

void EnrichCloud::compute()
{
    compute_normals();
    _kdtree.reset(new pcl::KdTreeFLANN<PointS>);
    _kdtree->setInputCloud(_cloud_in);

    for(size_t i = 0; i < _cloud_in->points.size(); i++)
    {
        PointS p = _cloud_in->points.at(i);
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        if(_use_knn_search)
        {
            _kdtree->nearestKSearch(p,_k,pointIdxRadiusSearch, pointRadiusSquaredDistance);
        }
        else
        {
            _kdtree->radiusSearch(p,_range,pointIdxRadiusSearch,pointRadiusSquaredDistance);
        }


        Eigen::Matrix3f covariance_matrix;
        Eigen::Vector4f xyz_centroid;
        pcl::compute3DCentroid<PointS> (*_cloud_in, pointIdxRadiusSearch, xyz_centroid);
        pcl::computeCovarianceMatrix (*_cloud_in, pointIdxRadiusSearch, xyz_centroid, covariance_matrix);

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen (covariance_matrix);
        float lambda1,lambda2,lambda3,sum;
        Eigen::Matrix<float,3,1> lambda = eigen.eigenvalues();
        eigen.eigenvectors();
        lambda1 = lambda[0];
        lambda2 = lambda[1];
        lambda3 = lambda[2];

        sum = lambda1+lambda2+lambda3;
        lambda1 /= sum;
        lambda2 /= sum;
        lambda3 /= sum;

        _cloud_in->points.at(i).eigen1 = lambda1;
        _cloud_in->points.at(i).eigen2 = lambda2;
        _cloud_in->points.at(i).eigen3 = lambda3;

    }
}

void EnrichCloud::compute_normals()
{
    if(QThread::idealThreadCount()>1)
    {
        // Create the normal estimation class, and pass the input dataset to it
        pcl::NormalEstimationOMP<PointS, PointS> ne;
        ne.setInputCloud (_cloud_in);

        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<PointS>::Ptr tree (new pcl::search::KdTree<PointS> ());
        ne.setSearchMethod (tree);
        if(_use_knn_search)
        {
            ne.setKSearch(_k);
        }
        else
        {
            ne.setRadiusSearch(_range);
        }
        // Compute the features
        ne.compute (*_cloud_in);
    }
    else
    {
        // Create the normal estimation class, and pass the input dataset to it
        pcl::NormalEstimation<PointS, PointS> ne;
        ne.setInputCloud (_cloud_in);
        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<PointS>::Ptr tree (new pcl::search::KdTree<PointS> ());
        ne.setSearchMethod (tree);
        if(_use_knn_search)
        {
            qDebug() << "enrich wrong";
            ne.setKSearch(_k);
        }
        else
        {
            qDebug() << "enrich right";
            ne.setRadiusSearch(_range);
        }
        ne.compute (*_cloud_in);
    }

}

EnrichCloud::EnrichCloud(PointCloudS::Ptr cloud_in, int k , float range, bool use_knn )
{
    _cloud_in = cloud_in;
    _k = k;
    _range = range;
    _use_knn_search = use_knn;
    compute();
}
