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

#include "st_stepclearsky.h"

// Alias for indexing models
#define DEFin_res "res"
#define DEFin_grp "grp"
#define DEFin_cloud_in "cloud_in"

//#define DEF_resultIn_inputResult "inputResult"
//#define DEF_groupIn_inputScene "inputGroup"
//#define DEF_itemIn_scene "inputScene"


#define DEFin_normals_in "no_in"
#define DEFin_stem_in "stem_in"


//// Alias for indexing out models
#define DEF_resultOut_translated "extractedResult"
#define DEF_groupOut_pointCloud "extractedGroup"
#define DEF_itemOut_scene "extractedScene"



// Constructor : initialization of parameters
ST_StepClearSky::ST_StepClearSky(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{
}

ST_StepClearSky::~ST_StepClearSky()
{

}

// Step description (tooltip of contextual menu)
QString ST_StepClearSky::getStepDescription() const
{
    return tr("Clear Sky filter.");
}

// Step detailled description
QString ST_StepClearSky::getStepDetailledDescription() const
{
    return tr("Clear Sky filter ." );
}

// Step URL
QString ST_StepClearSky::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
}

// Step copy method
CT_VirtualAbstractStep* ST_StepClearSky::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepClearSky(dataInit);
}


// Creation and affiliation of IN models
void ST_StepClearSky::createInResultModelListProtected()
{



    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("Result_In"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("Grp_In"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    resIn_res->addItemModel(DEFin_grp, DEFin_cloud_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Point Cloud with Sky noise"));
}

// Creation and affiliation of OUT models
void ST_StepClearSky::createOutResultModelListProtected()
{

    CT_OutResultModelGroupToCopyPossibilities *res = createNewOutResultModelToCopy(DEFin_res);

    if(res != NULL)
    {
        res->addItemModel(DEFin_grp, _scene_sky_denoised, new CT_Scene(), tr("Point Cloud, denoised from Sky noise"));
        res->addItemModel(DEFin_grp, _scene_sky_noise, new CT_Scene(), tr("Point Cloud, the Sky noise"));
    }
}

// Semi-automatic creation of step parameters DialogBox
void ST_StepClearSky::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();
    configDialog->addTitle(tr("Select a minimum number of points (approxiamted):"));
    // configDialog->addInt( "The number of clusters  ","numCluster",1,10000,_number_clusters);
    configDialog->addInt("minPts ","",0,100000,_minPts);
    configDialog->addTitle(tr("within a range"));
    configDialog->addDouble("range","",0.5,10,1,_range);
    dialog_simple_tree(configDialog);

}

void ST_StepClearSky::compute()
{
    QList<CT_AbstractItemGroup*> groupsToBeRemoved;
    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res = outResultList.at(0);

    CT_ResultGroupIterator itCpy_grp(resCpy_res, this, DEFin_grp);
    while (itCpy_grp.hasNext() && !isStopped())
    {
        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp.next();
        CT_AbstractItemDrawableWithPointCloud* itemCpy_cloud_in
                = (CT_AbstractItemDrawableWithPointCloud*)grpCpy_grp->firstItemByINModelName(this, DEFin_cloud_in);
        if(itemCpy_cloud_in!=NULL)
        {
            _cloud_in_downscaled = downscale_cloud_ct(itemCpy_cloud_in, _range/6);
            if(_cloud_in_downscaled->points.size()!=0)
            {
                warning_geo_referenced(_cloud_in_downscaled);
                setProgress(5);
                QVector<float> numbers;
                {
                    pcl::KdTreeFLANN<PointS> kdtree;
                    kdtree.setInputCloud (_cloud_in_downscaled);
                    setProgress(15);
                    for(size_t i = 0; i < _cloud_in_downscaled->points.size(); i++)
                    {
                        PointS p = _cloud_in_downscaled->points.at(i);
                        std::vector<int> pointIdxRadiusSearch;
                        std::vector<float> pointRadiusSquaredDistance;
                        kdtree.radiusSearch (p, _range, pointIdxRadiusSearch, pointRadiusSquaredDistance);
                        int number = 0;
                        for (size_t j = 0; j < pointIdxRadiusSearch.size (); ++j)
                        {
                            number += _cloud_in_downscaled->points[pointIdxRadiusSearch[j]].ID;
                        }
                        numbers.push_back(number);
                    }
                }
                qDebug() << "06";
//                float mean = SimpleMath<float>::get_mean(numbers);
//                float sd   = SimpleMath<float>::get_standard_deviation(numbers);
//                float minPts = mean - 4*sd;
                {
                    pcl::KdTreeFLANN<PointS> kdtree;
                    kdtree.setInputCloud (_cloud_in_downscaled);
                    setProgress(25);
                    for(size_t i = 0; i < _cloud_in_downscaled->points.size(); i++)
                    {
                        PointS p = _cloud_in_downscaled->points.at(i);
                        std::vector<int> pointIdxRadiusSearch;
                        std::vector<float> pointRadiusSquaredDistance;
                        kdtree.radiusSearch (p, _range, pointIdxRadiusSearch, pointRadiusSquaredDistance);
                        int number = 0;
                        for (size_t j = 0; j < pointIdxRadiusSearch.size (); ++j)
                        {
                            number += _cloud_in_downscaled->points[pointIdxRadiusSearch[j]].ID;
                        }
                        if(number>_minPts)
                        {
                            _cloud_in_downscaled->points[i].treeID = 1;
                        } else {
                            _cloud_in_downscaled->points[i].treeID = 0;
                        }
                    }
                    setProgress(30);
                }
                CT_PointCloudIndexVector *extractedCloud1 = new CT_PointCloudIndexVector(); // 1) cretation of output pointcloud index
                CT_PointCloudIndexVector *extractedCloud2 = new CT_PointCloudIndexVector(); // 1) cretation of output pointcloud index

                {
                    const CT_AbstractPointCloudIndex *pointCloudIndex = itemCpy_cloud_in->getPointCloudIndex();
                    CT_PointIterator itP(pointCloudIndex);
                    pcl::KdTreeFLANN<PointS> kdtree;
                    kdtree.setInputCloud (_cloud_in_downscaled);
                    setProgress(30);
                    int i = 0;
                    size_t n_points = pointCloudIndex->size();
                    while (itP.hasNext() && !isStopped())
                    {
                        itP.next();
                        const CT_Point &point = itP.currentPoint();
                        size_t index = itP.currentGlobalIndex();

                        PointS query(point(0),point(1),point(2));
                        std::vector<int> pointIdxNKNSearch(1);
                        std::vector<float> pointNKNSquaredDistance(1);

                        if(kdtree.nearestKSearch (query, 1, pointIdxNKNSearch, pointNKNSquaredDistance))
                        {
                        int is_valid = _cloud_in_downscaled->points[pointIdxNKNSearch.at(0)].treeID;
                        if(is_valid == 0)
                        {
                            extractedCloud1->addIndex(index);
                        } else {
                            extractedCloud2->addIndex(index);
                        }
                        }
                        setProgress(30.0 + 60.0*i/n_points);
                        ++i;
                    }
                }
                setProgress(95);



                if(extractedCloud2->size()>0)
                {
                    CT_Scene* outScene = new CT_Scene(_scene_sky_denoised.completeName(),
                                                      resCpy_res, PS_REPOSITORY->registerPointCloudIndex(extractedCloud2));
                    outScene->updateBoundingBox();
                    grpCpy_grp->addItemDrawable(outScene);
                }
                if(extractedCloud1->size()>0)
                {
                    CT_Scene* outScene = new CT_Scene(_scene_sky_noise.completeName(),
                                                      resCpy_res, PS_REPOSITORY->registerPointCloudIndex(extractedCloud1));
                    outScene->updateBoundingBox();
                    grpCpy_grp->addItemDrawable(outScene);
                }
                setProgress(99);
            } else {
                groupsToBeRemoved.push_back(grpCpy_grp);
            }


        } else {
            groupsToBeRemoved.push_back(grpCpy_grp);
        }


    }
    _cloud_in.reset(new PointCloudS);
    _cloud_in_downscaled.reset(new PointCloudS);
    _cloud_out_denoised.reset(new PointCloudS);
    while (!groupsToBeRemoved.isEmpty())
    {
        CT_AbstractItemGroup *group = groupsToBeRemoved.takeLast();
        recursiveRemoveGroupIfEmpty(group->parentGroup(), group);
    }
}













