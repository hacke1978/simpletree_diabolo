/****************************************************************************

 Copyright (C) 2016-2017 Jan Hackenberg
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

#include "worker_thread_std_out_multithread.h"

void Worker_Std_out_multithread::run()
{
    PointCloudS::Ptr cloud = _param.cloud;
    CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in = _param.itemCpy_cloud_in;

    int number_iterations = 0;
    _iterations = _param.iterations;
    _knn        = _param.knn;
    _std_mult   = _param.std_mult;
    while(number_iterations < _iterations)
    {
        PointCloudS::Ptr cloud_filtered (new PointCloudS);
        if(cloud->points.size()>10)
        {


            pcl::StatisticalOutlierRemoval<PointS> sor;
            sor.setInputCloud (cloud);
            sor.setMeanK (_knn);
            sor.setStddevMulThresh (_std_mult);
            sor.filter (*cloud_filtered);
        }
        cloud = cloud_filtered;
        number_iterations++;
    }

    const CT_AbstractPointCloudIndex *pointCloudIndex = itemCpy_cloud_in->getPointCloudIndex();
    CT_PointCloudIndexVector *extractedCloud = new CT_PointCloudIndexVector();
    CT_PointIterator itP(pointCloudIndex);
    pcl::KdTreeFLANN<PointS> kdtree;
    kdtree.setInputCloud (cloud);
    size_t i = 0;
    while (itP.hasNext())
    {
        PointS p = _param.cloud->points.at(i);
        itP.next();
        size_t index = itP.currentGlobalIndex();
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);
        kdtree.nearestKSearch(p,1,pointIdxNKNSearch,pointNKNSquaredDistance);
        if ( pointNKNSquaredDistance[0] < 0.000001)
        {
            extractedCloud->addIndex(index);
        }
        i++;
    }
    CT_Scene* outScene = new CT_Scene(_param._cloud_out_name, _param.resCpy_res, PS_REPOSITORY->registerPointCloudIndex(extractedCloud));
    outScene->updateBoundingBox();
    _param.grpCpy_grp->addItemDrawable(outScene);
}




Worker_Std_out_multithread::Worker_Std_out_multithread(St_step_std_out_param params, QSharedPointer<ThreadPoool_std_out_multithread> tp_std_out):
    _param(params),_tp_std_out(tp_std_out)
{
setAutoDelete(true);
}

