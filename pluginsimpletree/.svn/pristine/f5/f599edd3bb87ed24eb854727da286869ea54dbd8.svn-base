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


#include "st_stepcomputedtm.h"


#ifdef USE_OPENCV
#include "ct_global/ct_context.h"

#include "ct_result/model/inModel/ct_inresultmodelgrouptocopy.h"
#include "ct_result/model/outModel/ct_outresultmodelgroupcopy.h"
#include "ct_result/model/outModel/tools/ct_outresultmodelgrouptocopypossibilities.h"

#include "ct_result/ct_resultgroup.h"

#include "ct_itemdrawable/ct_scene.h"
#include "ct_itemdrawable/abstract/ct_abstractareashape2d.h"
#include "ct_pointcloudindex/ct_pointcloudindexvector.h"
#include "ct_iterator/ct_pointiterator.h"
#include "ct_view/ct_stepconfigurabledialog.h"

#include "ct_itemdrawable/tools/image2dtools/ct_image2dnaturalneighboursinterpolator.h"

#include <QTimer>
#include <math.h>
#include <stdlib.h>
#include <limits>

#define DEF_SearchInResult   "ires"
#define DEF_SearchInGroup   "igrp"
#define DEF_SearchInScene   "isc"

#define EPSILON 0.000001

ST_StepComputeDTM::ST_StepComputeDTM(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{
    _gridsize   = 0.8;
    _nCells = 10;
}

QString ST_StepComputeDTM::getStepDescription() const
{
    return tr("Create DTM");
}

QString ST_StepComputeDTM::getStepDetailledDescription() const
{
    return tr("This step creates a digital terrain model. It performs better when it takes input points"
              "which consist only of ground points rather than the complete scene. ");
}

CT_VirtualAbstractStep* ST_StepComputeDTM::createNewInstance(CT_StepInitializeData &dataInit)
{
    // cree une copie de cette etape
    return new ST_StepComputeDTM(dataInit);
}

float ST_StepComputeDTM::get_percentage_under_plane(pcl::ModelCoefficients::Ptr coeff, PointCloudS::Ptr cloud)
{
    int size = cloud->points.size();
    int under_pts = 0;
    for(int i = 0; i < size; i ++)
    {
        PointS p = cloud->points.at(i);
        float z = (p.x*coeff->values[0] + p.y*coeff->values[1] + coeff->values[3])/-coeff->values[2];
        if(p.z <= (z-_min_dist_below))
        {
            under_pts ++;
        }

    }
    if(under_pts == 0)
        return 0;
    float s = size;
    float u = under_pts;
    return (u/s);
}

/////////////////////// PROTECTED ///////////////////////

void ST_StepComputeDTM::createInResultModelListProtected()
{  
    CT_InResultModelGroupToCopy *resultModel = createNewInResultModelForCopy(DEF_SearchInResult, tr("Points sol"));

    resultModel->setZeroOrMoreRootGroup();
    resultModel->addGroupModel( "", DEF_SearchInGroup, CT_AbstractItemGroup::staticGetType(), tr("grp_in"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    resultModel->addItemModel(DEF_SearchInGroup, DEF_SearchInScene, CT_Scene::staticGetType(), tr("Points sol"));
    //    resultModel->addItemModel(DEF_SearchInGroup, DEF_SearchInArea, CT_AbstractAreaShape2D::staticGetType(),
    //                              tr("Emprise"), "", CT_InAbstractModel::C_ChooseOneIfMultiple, CT_InAbstractModel::F_IsOptional);
}

void ST_StepComputeDTM::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();
    configDialog->addDouble(tr("Raster resolution :"), "m", 0, 1000, 2, _gridsize, 1);
    configDialog->addDouble(tr("Clip min z for extrapolation :"), "m",-100000000 , +100000000, 2, _clip_min, 1);
    configDialog->addDouble(tr("Clip max z for extrapolation :"), "m",-100000000 , +100000000, 2, _clip_max, 1);
    configDialog->addBool(tr("Clip by bounding box "), "", "", _clip_by_bbox,"min z and max z will be ignored.");
}

void ST_StepComputeDTM::createOutResultModelListProtected()
{
    CT_OutResultModelGroupToCopyPossibilities *resultModel = createNewOutResultModelToCopy(DEF_SearchInResult);

    if(resultModel != NULL)
    {
        resultModel->addItemModel(DEF_SearchInGroup, _outDTMModelName50, new CT_Image2D<float>(), tr("DTM_ST"));
    }
}

void ST_StepComputeDTM::compute()
{
    QList<CT_AbstractItemGroup*> groupsToBeRemoved;
    if(_als)
    {

        _dif200 = 10000;
        _dif100 = 10000;
        _dif50 = 10000;

        _minPts = 1;
        _min_perc_under = 0;
        _min_dist_below = 1;

        _nCells ;
        _max_distance = 100;
        _max_angle_plane_normal = 45;

        _clip_min = -10;
        _clip_max = 10;

        _max_deviation = 100;

        _clip_by_bbox = true;

    }

    const QList<CT_ResultGroup*> &outResList = getOutResultList();
    CT_ResultGroup *outResult = outResList.at(0);

    CT_ResultGroupIterator it(outResult, this, DEF_SearchInGroup);
    while (!isStopped() && it.hasNext())
    {
        CT_StandardItemGroup* group = (CT_StandardItemGroup*) it.next();

        if (group != NULL)
        {
            const CT_Scene *scene = (const CT_Scene*)group->firstItemByINModelName(this, DEF_SearchInScene);
            CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in
                    = (CT_AbstractItemDrawableWithPointCloud*)group->firstItemByINModelName(this, DEF_SearchInScene);
            if(itemCpy_cloud_in!=NULL)
            {
                double minX = scene->minX();
                double minY = scene->minY();
                double maxX = scene->maxX();
                double maxY = scene->maxY();

                if(_clip_by_bbox)
                {
                    _clip_min = scene->minZ();
                    _clip_max = scene->maxZ();
                }

                _grid_size200 = _gridsize*4;
                _grid_size100 = _gridsize*2;
                _grid_size50  = _gridsize;

                CT_Image2D<float>* mnt200 = CT_Image2D<float>::createImage2DFromXYCoords
                        ( _outDTMModelName200.completeName(), outResult, minX, minY, maxX, maxY, _grid_size200, scene->minZ(), -9999, -9999);
                CT_Image2D<float>* mnt100 = CT_Image2D<float>::createImage2DFromXYCoords
                        (_outDTMModelName100.completeName(), outResult, minX, minY, maxX, maxY, _grid_size100, scene->minZ(), -9999, -9999);
                CT_Image2D<float>* mnt50 = CT_Image2D<float>::createImage2DFromXYCoords
                        (_outDTMModelName50.completeName(), outResult, minX, minY, maxX, maxY, _grid_size50, scene->minZ(), -9999, -9999);

                setProgress(20);
                PointCloudS::Ptr cloud_in = pcl_CT_to_PCL_cloud(itemCpy_cloud_in,this,16,false,true);
                setProgress(22);
                PointCloudS::Ptr cloud_in_downscaled_for_ground = pcl_voxel_grid_filter(cloud_in,this,_gridsize/9,true);
                setProgress(24);
                QVector<PointCloudS::Ptr> clouds200 = clouds_rasterized(cloud_in_downscaled_for_ground,mnt200);
                setProgress(30);
                QVector<PointCloudS::Ptr> clouds100 = clouds_rasterized(cloud_in_downscaled_for_ground,mnt100);
                setProgress(35);
                QVector<PointCloudS::Ptr> clouds50 = clouds_rasterized(cloud_in_downscaled_for_ground,mnt50);
                setProgress(40);

                fill_DTM_a(clouds200,  mnt200, mnt100, mnt50);
                setProgress(50);
                fill_DTM_b(clouds100 , mnt100, mnt50);
                setProgress(60);
                fill_DTM_c(clouds50 , mnt50);
                setProgress(70);



                interpolate_DTM(mnt200,_grid_size200);
                setProgress(80);
                interpolate_DTM(mnt100,_grid_size100);
                setProgress(90);
                interpolate_DTM(mnt50,_grid_size50);
                setProgress(95);
                int number_cells =  mnt50->nCells();

                group->addItemDrawable(mnt50);
                mnt50->computeMinMax();
            } else {
                groupsToBeRemoved.push_back(group);
            }


        }
        setProgress(100);
    }
    while (!groupsToBeRemoved.isEmpty())
    {
        CT_AbstractItemGroup *group = groupsToBeRemoved.takeLast();
        recursiveRemoveGroupIfEmpty(group->parentGroup(), group);
    }
}

void ST_StepComputeDTM::interpolate_DTM(CT_Image2D<float> *dtm200, double res)
{
    int number_cells =  dtm200->nCells();
    int number_cells_nan = 0;

    PointCloudS::Ptr cell_cloud (new PointCloudS);
    PointCloudS::Ptr cell_cloud_2D (new PointCloudS);

    for(int i = 0; i < number_cells; i++)
    {
        Eigen::Vector2d bot;
        Eigen::Vector2d top;
        dtm200->getCellCoordinates(i,bot,top);
        float centerX = (bot(0)+top(0))/2;
        float centerY = (bot(1)+top(1))/2;
        float value = dtm200->valueAtIndex(i);
        if(value != -9999)
        {
            PointS p;
            p.x = centerX;
            p.y = centerY;
            p.z = value;
            cell_cloud->points.push_back(p);
            PointS p2;
            p2.x = centerX;
            p2.y = centerY;
            p2.z = 0;
            cell_cloud_2D->points.push_back(p2);
        }
    }





    for(int i =0; i< number_cells; i++)
    {
        Eigen::Vector2d bot;
        Eigen::Vector2d top;
        dtm200->getCellCoordinates(i,bot,top);
        double value_at = dtm200->valueAtIndex(i);
        if(value_at == -9999)
        {
            number_cells_nan++;
        } else {
            //            qDebug() << "no NAN";
        }

    }
    while(number_cells_nan > 0)
    {
        int max_filled_neighbors = 0;
        for(int i =0; i< number_cells; i++)
        {
            Eigen::Vector2d bot;
            Eigen::Vector2d top;
            dtm200->getCellCoordinates(i,bot,top);
            float centerX = (bot(0)+top(0))/2;
            float centerY = (bot(1)+top(1))/2;
            double value_at = dtm200->valueAtIndex(i);
            if(value_at == -9999)
            {

                int neibors = 0;
                for(int j = -1 ; j < 2; j++)
                {
                    for(int k = -1 ; k < 2; k++)
                    {
                        float centerX_1 = centerX +  j*res;
                        float centerY_1 = centerY +  k*res;


                        float centerX_2 = centerX +  2*j*res;
                        float centerY_2 = centerY +  2*k*res;
                        float val1 = -9999;
                        float val2 = -9999;
                        val1 = dtm200->valueAtCoords(centerX_1,centerY_1);
                        val2 = dtm200->valueAtCoords(centerX_2,centerY_2);


                        if(val1 != -9999 && val2 != -9999 && val1 != 0 && val2 != 0)
                        {
                            neibors++;
                        }

                    }
                }
                if(neibors>max_filled_neighbors)
                    max_filled_neighbors = neibors;



            }
        }



        for(int i =0; i< number_cells; i++)
        {
            Eigen::Vector2d bot;
            Eigen::Vector2d top;
            dtm200->getCellCoordinates(i,bot,top);
            float centerX = (bot(0)+top(0))/2;
            float centerY = (bot(1)+top(1))/2;
            double value_at = dtm200->valueAtIndex(i);
            if(value_at == -9999)
            {
                int neibors = 0;
                for(int j = -1 ; j < 2; j++)
                {
                    for(int k = -1 ; k < 2; k++)
                    {
                        float centerX_1 = centerX +  j*res;
                        float centerY_1 = centerY +  k*res;
                        float centerX_2 = centerX +  2*j*res;
                        float centerY_2 = centerY +  2*k*res;
                        float val1 = -9999;
                        float val2 = -9999;
                        val1 = dtm200->valueAtCoords(centerX_1,centerY_1);
                        val2 = dtm200->valueAtCoords(centerX_2,centerY_2);
                        if(val1 != -9999 && val2 != -9999 && val1 != 0 && val2 != 0)
                        {
                            neibors++;
                        }
                    }
                }




                QVector<float> heights;
                if(neibors>=max_filled_neighbors)
                {

                    for(int j = -1 ; j < 2; j++)
                    {
                        for(int k = -1 ; k < 2; k++)
                        {
                            float centerX_1 = centerX +  j*res;
                            float centerY_1 = centerY +  k*res;
                            float centerX_2 = centerX +  2*j*res;
                            float centerY_2 = centerY +  2*k*res;
                            float val1 = -9999;
                            float val2 = -9999;
                            val1 = dtm200->valueAtCoords(centerX_1,centerY_1);
                            val2 = dtm200->valueAtCoords(centerX_2,centerY_2);
                            if(val1 != -9999 && val2 != -9999)
                            {
                                float h = val1 + val1 -val2;
                                heights.push_back(h);
                            }
                        }
                    }
                    float med_height = SimpleMath<float>::get_median(heights);
                    dtm200->setValueAtIndex(i,med_height);
                    number_cells_nan--;
                }
            }
        }
    }


    pcl::KdTreeFLANN<PointS> kdtree;
    kdtree.setInputCloud (cell_cloud_2D);
    for(int i = 0; i < number_cells; i++)
    {
        Eigen::Vector2d bot;
        Eigen::Vector2d top;
        dtm200->getCellCoordinates(i,bot,top);
        float centerX = (bot(0)+top(0))/2;
        float centerY = (bot(1)+top(1))/2;
        float value = dtm200->valueAtIndex(i);
        //        if(value == -9999)
        {
            PointS p;
            p.x = centerX;
            p.y = centerY;
            p.z = 0;

            int K = 4;
            std::vector<int> pointIdxNKNSearch(K);
            std::vector<float> pointNKNSquaredDistance(K);
            float dist1;
            float dist2;
            float dist3;
            float dist4;
            int index1;
            int index2;
            int index3;
            int index4;

            if ( kdtree.nearestKSearch (p, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
            {
                dist1 = std::sqrt(pointNKNSquaredDistance[0]);
                dist2 = std::sqrt(pointNKNSquaredDistance[1]);
                dist3 = std::sqrt(pointNKNSquaredDistance[2]);
                dist4 = std::sqrt(pointNKNSquaredDistance[3]);

                if(dist1 == 0)
                {
                    dist1 = 1000000000;
                } else {
                    dist1 = 1/dist1;
                }

                if(dist2 == 0)
                {
                    dist2 = 1000000000;
                } else {
                    dist2 = 1/dist2;
                }

                if(dist3 == 0)
                {
                    dist3 = 1000000000;
                } else {
                    dist3 = 1/dist3;
                }

                if(dist4 == 0)
                {
                    dist4 = 1000000000;
                } else {
                    dist4 = 1/dist4;
                }

                index1 = pointIdxNKNSearch[0];
                index2 = pointIdxNKNSearch[1];
                index3 = pointIdxNKNSearch[2];
                index4 = pointIdxNKNSearch[3];
            }
            float dist_total = dist1 + dist2 + dist3 + dist4;
            float val = cell_cloud->points[index1].z * dist1/dist_total +
                    cell_cloud->points[index2].z * dist2/dist_total +
                    cell_cloud->points[index3].z * dist3/dist_total +
                    cell_cloud->points[index4].z * dist4/dist_total;
            if(std::abs(val-value)>_max_deviation)
                dtm200->setValueAtIndex(i,val);

        }
    }
}

void ST_StepComputeDTM::median_filter(CT_Image2D<float> *dtm200, float res)
{
    size_t size = dtm200->nCells();
    for(size_t i = 0; i < size; i++)
    {
        Eigen::Vector2d bot;
        Eigen::Vector2d top;
        dtm200->getCellCoordinates(i,bot,top);
        float centerX = (bot(0)+top(0))/2;
        float centerY = (bot(1)+top(1))/2;
        double value_at = dtm200->valueAtIndex(i);
        if(value_at != -9999)
        {



            QVector<float> heights;

            for(int j = -1 ; j < 2; j++)
            {
                for(int k = -1 ; k < 2; k++)
                {
                    float centerX_1 = centerX +  j*res;
                    float centerY_1 = centerY +  k*res;


                    float centerX_2 = centerX +  2*j*res;
                    float centerY_2 = centerY +  2*k*res;
                    float val1 = -9999;
                    float val2 = -9999;
                    val1 = dtm200->valueAtCoords(centerX_1,centerY_1);
                    val2 = dtm200->valueAtCoords(centerX_2,centerY_2);
                    if(val1 != -9999 && val2 != -9999)
                    {
                        float h = val1 + val1 -val2;
                        heights.push_back(h);
                    }

                }
            }
            float med_height = SimpleMath<float>::get_median(heights);
            dtm200->setValueAtIndex(i,med_height);
        }
    }
}

void ST_StepComputeDTM::cross_check_dtm(CT_Image2D<float> *dtm200, CT_Image2D<float> *dtm)
{
    size_t size = dtm->nCells();
    for(size_t i = 0; i < size; i++)
    {
        Eigen::Vector2d bot;
        Eigen::Vector2d top;
        dtm->getCellCoordinates(i,bot,top);
        float centerX = (bot(0)+top(0))/2;
        float centerY = (bot(1)+top(1))/2;
        double value_at = dtm->valueAtIndex(i);
        if(value_at != -9999)
        {
            double value_at_large = dtm200->valueAtCoords(centerX,centerY);
            if(std::abs(value_at - value_at_large)>= _max_distance)
            {
                dtm->setValueAtCoords(centerX,centerY,value_at_large);
            }
        }
    }
}

void ST_StepComputeDTM::fill_DTM_a(QVector<PointCloudS::Ptr> clouds200, CT_Image2D<float> *dtm200, CT_Image2D<float> *dtm100, CT_Image2D<float> *dtm50)
{
    for(size_t i = 0; i < clouds200.size(); i++)
    {
        PointCloudS::Ptr cloud = clouds200.at(i);
        if(cloud->points.size()>= _minPts*9)
        {
            float val_origin = dtm200->valueAtIndex(i);
            Eigen::Vector2d bot;
            Eigen::Vector2d top;
            dtm200->getCellCoordinates(i,bot,top);
            float centerX = (bot(0)+top(0))/2;
            float centerY = (bot(1)+top(1))/2;

            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            pcl::SACSegmentation<PointS> seg;
            seg.setOptimizeCoefficients (true);
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setEpsAngle(SimpleMath<float>::_PI/8);
            seg.setMethodType (pcl::SAC_MLESAC);
            seg.setDistanceThreshold (0.2);
            seg.setMaxIterations(100);
            seg.setInputCloud (cloud);
            seg.segment (*inliers, *coefficients);



            if (inliers->indices.size () == 0)
            {
                PCL_ERROR ("Could not estimate a planar model for the given dataset.");
            } else {
                Eigen::Vector3f normal;
                normal(0) = coefficients->values[0];
                normal(1) = coefficients->values[1];
                normal(2) = coefficients->values[2];

                Eigen::Vector3f zAxis;
                zAxis(0) = 0;
                zAxis(1) = 0;
                zAxis(2) = 1;

                float angle = SimpleMath<float>::angle_between(normal,zAxis);

                if ( (inliers->indices.size () >= (cloud->points.size()/6)) && (angle < _max_angle_plane_normal || (angle > (180 - _max_angle_plane_normal))))

                {
                    if(get_percentage_under_plane(coefficients,cloud)<=_min_perc_under)
                    {

                        float z = (centerX*coefficients->values[0] + centerY*coefficients->values[1] + coefficients->values[3])/-coefficients->values[2];
                        float dif = std::abs(z-val_origin);
                        dif = 0;
                        if(dif < _dif200 )
                        {
                            dtm200->setValueAtIndex(i,z);
                            for(int k = 0; k < 2; k++)
                            {
                                for(int l = 0; l < 2; l++)
                                {
                                    float centerX100 = centerX;
                                    float centerY100 = centerY;
                                    if( k == 0)
                                    {
                                        centerX100 -= _grid_size100/2;
                                    }
                                    else if
                                            (k == 1)
                                    {
                                        centerX100 += _grid_size100/2;
                                    }


                                    if( l == 0)
                                    {
                                        centerY100 -= _grid_size100/2;
                                    } else if (l == 1)
                                    {
                                        centerY100 += _grid_size100/2;
                                    }

                                    size_t ind;
                                    dtm100->indexAtCoords(centerX100, centerY100, ind);
                                    float z = (centerX100*coefficients->values[0] + centerY100*coefficients->values[1]
                                            + coefficients->values[3])/-coefficients->values[2];
                                    dtm100->setValueAtIndex(ind,z);



                                    for(int m = 0; m < 2; m++)
                                    {
                                        for(int n = 0; n < 2; n++)
                                        {
                                            float centerX50 = centerX100;
                                            float centerY50 = centerY100;
                                            if( m == 0)
                                            {
                                                centerX50 -= _grid_size50/2;
                                            } else if (m == 1)
                                            {
                                                centerX50 += _grid_size50/2;
                                            } else {
                                                //                                            qDebug() <<"foo3";
                                            }


                                            if( n == 0)
                                            {
                                                centerY50 -= _grid_size50/2;
                                            } else if (n == 1)
                                            {
                                                centerY50 += _grid_size50/2;
                                            } else {
                                                //                                            qDebug() <<"foo4";
                                            }
                                            size_t ind;
                                            dtm50->indexAtCoords(centerX50, centerY50, ind);
                                            float z =
                                                    (centerX50*coefficients->values[0] + centerY50*coefficients->values[1]
                                                    + coefficients->values[3])/-coefficients->values[2];
                                            dtm50->setValueAtIndex(ind,z);
                                        }
                                    }
                                }
                            }
                        }
                        else
                        {
                            //                            qDebug() << "fail dif 200" << dif;
                        }
                    } else {
                        //                        qDebug() << " get_percentage_under_plane(coefficients,cloud)>=_min_perc_under false ";
                    }
                } else {
                    //                    qDebug() << " inliers->indices.size () >= (cloud->points.size()/2) false ";
                }
            }
        }
        //        qDebug() << " (cloud->points.size()>= _minPts*4) false ";
    }
}

void ST_StepComputeDTM::fill_DTM_b(QVector<PointCloudS::Ptr> clouds100, CT_Image2D<float> *dtm100, CT_Image2D<float> *dtm50)
{
    for(size_t i = 0; i < clouds100.size(); i++)
    {
        PointCloudS::Ptr cloud = clouds100.at(i);
        if(cloud->points.size()>= _minPts*3)
        {
            float val_origin = dtm100->valueAtIndex(i);
            Eigen::Vector2d bot;
            Eigen::Vector2d top;
            dtm100->getCellCoordinates(i,bot,top);
            float centerX = (bot(0)+top(0))/2;
            float centerY = (bot(1)+top(1))/2;

            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            pcl::SACSegmentation<PointS> seg;
            seg.setOptimizeCoefficients (true);
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setEpsAngle(SimpleMath<float>::_PI/4);
            seg.setMethodType (pcl::SAC_MLESAC);
            seg.setDistanceThreshold (0.15);
            seg.setMaxIterations(100);
            seg.setInputCloud (cloud);
            seg.segment (*inliers, *coefficients);



            if (inliers->indices.size () == 0)
            {
                PCL_ERROR ("Could not estimate a planar model for the given dataset.");
            } else {

                Eigen::Vector3f normal;
                normal(0) = coefficients->values[0];
                normal(1) = coefficients->values[1];
                normal(2) = coefficients->values[2];

                Eigen::Vector3f zAxis;
                zAxis(0) = 0;
                zAxis(1) = 0;
                zAxis(2) = 1;
                float angle = SimpleMath<float>::angle_between(normal,zAxis);
                if ((inliers->indices.size () >= (cloud->points.size()/6))&&(angle < _max_angle_plane_normal || (angle > (180 - _max_angle_plane_normal))))
                {
                    float z = (centerX*coefficients->values[0] + centerY*coefficients->values[1] + coefficients->values[3])/-coefficients->values[2];
                    float dif = std::abs(z-val_origin);
                    if(dif < _dif100 )
                    {
                        dtm100->setValueAtIndex(i,z);
                        for(int k = 0; k < 2; k++)
                        {
                            for(int l = 0; l < 2; l++)
                            {
                                float centerX50 = centerX;
                                float centerY50 = centerY;
                                if( k == 0)
                                {
                                    centerX50 -= _grid_size50/2;
                                } else if (k == 1)
                                {
                                    centerX50 += _grid_size50/2;
                                } else {
                                    //                                    qDebug() <<"foo";
                                }


                                if( l == 0)
                                {
                                    centerY50 -= _grid_size50/2;
                                } else if (l == 1)
                                {
                                    centerY50 += _grid_size50/2;
                                } else {
                                    //                                    qDebug() <<"foo2";
                                }
                                size_t ind;
                                dtm50->indexAtCoords(centerX50, centerY50, ind);

                                float z = (centerX50*coefficients->values[0] + centerY50*coefficients->values[1] + coefficients->values[3])/-coefficients->values[2];
                                dtm50->setValueAtIndex(ind,z); }
                        }
                    } else {
                        //                                                qDebug() << "fail dif 100" << dif ;
                    }
                }
            }
        }
    }
}

void ST_StepComputeDTM::fill_DTM_c(QVector<PointCloudS::Ptr> clouds50, CT_Image2D<float> *dtm50)
{
    for(size_t i = 0; i < clouds50.size(); i++)
    {
        PointCloudS::Ptr cloud = clouds50.at(i);
        if(cloud->points.size()>= _minPts/2)
        {
            float val_origin = dtm50->valueAtIndex(i);
            Eigen::Vector2d bot;
            Eigen::Vector2d top;
            dtm50->getCellCoordinates(i,bot,top);
            float centerX = (bot(0)+top(0))/2;
            float centerY = (bot(1)+top(1))/2;

            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            pcl::SACSegmentation<PointS> seg;
            seg.setOptimizeCoefficients (true);
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setMethodType (pcl::SAC_MLESAC);
            seg.setEpsAngle(SimpleMath<float>::_PI/4);
            seg.setDistanceThreshold (0.1);
            seg.setMaxIterations(100);
            seg.setInputCloud (cloud);
            seg.segment (*inliers, *coefficients);




            if (inliers->indices.size () == 0)
            {
                PCL_ERROR ("Could not estimate a planar model for the given dataset.");
            } else {
                Eigen::Vector3f normal;
                normal(0) = coefficients->values[0];
                normal(1) = coefficients->values[1];
                normal(2) = coefficients->values[2];

                Eigen::Vector3f zAxis;
                zAxis(0) = 0;
                zAxis(1) = 0;
                zAxis(2) = 1;
                float angle = SimpleMath<float>::angle_between(normal,zAxis);
                if ((inliers->indices.size () >= (cloud->points.size()/6))&& (angle < _max_angle_plane_normal || (angle > (180 - _max_angle_plane_normal))) )
                {
                    float z = (centerX*coefficients->values[0] + centerY*coefficients->values[1] + coefficients->values[3])/-coefficients->values[2];
                    float dif = std::abs(z-val_origin);
                    //                                        qDebug() << "dif DTM 50" << dif;
                    if(dif < _dif50 )
                    {
                        dtm50->setValueAtIndex(i,z);
                    }
                    else
                    {
                        //                                               qDebug() << "fail dif 50" << dif ;
                    }
                }
            }
        }
    }
}

QVector<PointCloudS::Ptr> ST_StepComputeDTM::clouds_rasterized(PointCloudS::Ptr cloud, CT_Image2D<float> *DTM)
{

    pcl::NormalEstimation<PointS, PointS> ne;
    ne.setInputCloud (cloud);
    pcl::search::KdTree<PointS>::Ptr tree (new pcl::search::KdTree<PointS> ());
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (_gridsize/2.9);
    ne.compute (*cloud);


    QVector<PointCloudS::Ptr> out_clouds;
    size_t dtm_size = DTM->nCells();

    for(size_t i = 0; i < dtm_size; i++)
    {
        PointCloudS::Ptr new_cloud(new PointCloudS);
        out_clouds.push_back(new_cloud);
    }



    size_t cloud_size = cloud->points.size();
    for(size_t i = 0; i < cloud_size; i++)
    {
        PointS p = cloud->points.at(i);
        size_t index = -1;
        DTM->indexAtCoords(p.x,p.y,index);
        PointCloudS::Ptr cloud = out_clouds.at(index);
        cloud->push_back(p);
    }
    return out_clouds;
}


#endif
