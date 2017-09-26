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

#include "st_stepgmm.h"

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


#include "opencv2/opencv.hpp"
#include "opencv2/ml.hpp"


// Constructor : initialization of parameters
ST_StepGMM::ST_StepGMM(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{
    _range = 0.05;

}

ST_StepGMM::~ST_StepGMM()
{

}

// Step description (tooltip of contextual menu)
QString ST_StepGMM::getStepDescription() const
{
    return tr("SimpleTree Denoising routine.");
}

// Step detailled description
QString ST_StepGMM::getStepDetailledDescription() const
{
    return tr("SimpleTree Denoising routine" );
}

// Step URL
QString ST_StepGMM::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
}

// Step copy method
CT_VirtualAbstractStep* ST_StepGMM::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepGMM(dataInit);
}

QString ST_StepGMM::getStepName() const
{
    return ("ST_denoising");
}

//////////////////// PROTECTED METHODS //////////////////

// Creation and affiliation of IN models
void ST_StepGMM::createInResultModelListProtected()
{



    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("cloud_in"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("grp_in"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);

    resIn_res->addItemModel(DEFin_grp, DEFin_cloud_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Cloud In"));
}

// Creation and affiliation of OUT models
void ST_StepGMM::createOutResultModelListProtected()
{

    CT_OutResultModelGroupToCopyPossibilities *res = createNewOutResultModelToCopy(DEFin_res);

    if(res != NULL)
    {

        res->addItemModel(DEFin_grp, _outScene_cluster1, new CT_Scene(), tr("cluster 1"));
        res->addItemModel(DEFin_grp, _outScene_cluster2, new CT_Scene(), tr("cluster 2"));
        res->addItemModel(DEFin_grp, _outScene_cluster3, new CT_Scene(), tr("cluster 3"));
        res->addItemModel(DEFin_grp, _outScene_cluster4, new CT_Scene(), tr("cluster 4"));
        res->addItemModel(DEFin_grp, _outScene_cluster5, new CT_Scene(), tr("cluster 5"));
        res->addItemModel(DEFin_grp, _outScene_cluster6, new CT_Scene(), tr("cluster 6"));
        res->addItemModel(DEFin_grp, _outScene_cluster7, new CT_Scene(), tr("cluster 7"));
        res->addItemModel(DEFin_grp, _outScene_cluster8, new CT_Scene(), tr("cluster 8"));
        res->addItemModel(DEFin_grp, _outScene_cluster9, new CT_Scene(), tr("cluster 9"));
        res->addItemModel(DEFin_grp, _outScene_cluster10, new CT_Scene(), tr("cluster 10"));
    }
}

// Semi-automatic creation of step parameters DialogBox
void ST_StepGMM::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();
    configDialog->addInt(tr("number of clusters"), "", 2,10,_num,"");
    configDialog->addDouble(tr("range"), "", 0.02,1,4,_range,1 ,"");
    dialog_simple_tree(configDialog);
}

void ST_StepGMM::compute()
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

        if(itemCpy_cloud_in!= NULL)
        {
            ConvertCTtoST ctst(itemCpy_cloud_in,16);
            ctst.convert();
            _cloud_in = ctst.get_cloud();

            if(_cloud_in->points.size()!=0)
            {
                qDebug() << "00";
                PointCloudS::Ptr cloud = pcl_voxel_grid_filter(_cloud_in, this, _range/5 , false);
                                qDebug() << "00021";
                pcl::NormalEstimationOMP<PointS, PointS> ne;
                ne.setInputCloud (cloud);
                pcl::search::KdTree<PointS>::Ptr tree (new pcl::search::KdTree<PointS> ());
                ne.setSearchMethod (tree);
                ne.setRadiusSearch(_range/2);
                ne.compute (*cloud);

qDebug() << "005";
                pcl::FPFHEstimationOMP<PointS,PointS, pcl::FPFHSignature33> fpfh;
                fpfh.setInputCloud(cloud);
                fpfh.setInputNormals(cloud);
                fpfh.setNumberOfThreads(11);
                pcl::search::KdTree<PointS>::Ptr tree2 (new pcl::search::KdTree<PointS>);

                fpfh.setSearchMethod (tree2);
                pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());
                fpfh.setRadiusSearch (_range*1);
                fpfh.compute (*fpfhs);
                qDebug() << "01";

                cv::Mat openCVPointCloud(fpfhs->points.size(), 33 ,  CV_32FC(1));
                for(size_t i = 0; i < fpfhs->points.size(); i++)
                {

                    pcl::FPFHSignature33 feature = fpfhs->points.at(i);
                    for(size_t j = 0; j < 33; j++)
                    {
                        openCVPointCloud.at<float>(i,j) = feature.histogram[j];
                    }
                }
                qDebug() << "02";
                cv::Ptr<cv::ml::EM> source_model = cv::ml::EM::create();
                source_model->setClustersNumber(_num);
                cv::Mat logs;
                cv::Mat labels;
                cv::Mat probs;
                qDebug() << "03";
                source_model->trainEM( openCVPointCloud,logs,labels,probs);
                qDebug() << "04";
                size_t counter = 0;
                const CT_AbstractPointCloudIndex *pointCloudIndex = itemCpy_cloud_in->getPointCloudIndex();

                CT_PointCloudIndexVector *extractedCloud1 = new CT_PointCloudIndexVector(); // 1) cretation of output pointcloud index
                CT_PointCloudIndexVector *extractedCloud2 = new CT_PointCloudIndexVector(); // 1) cretation of output pointcloud index
                CT_PointCloudIndexVector *extractedCloud3 = new CT_PointCloudIndexVector(); // 1) cretation of output pointcloud index
                CT_PointCloudIndexVector *extractedCloud4 = new CT_PointCloudIndexVector(); // 1) cretation of output pointcloud index
                CT_PointCloudIndexVector *extractedCloud5 = new CT_PointCloudIndexVector(); // 1) cretation of output pointcloud index
                CT_PointCloudIndexVector *extractedCloud6 = new CT_PointCloudIndexVector(); // 1) cretation of output pointcloud index
                CT_PointCloudIndexVector *extractedCloud7 = new CT_PointCloudIndexVector(); // 1) cretation of output pointcloud index
                CT_PointCloudIndexVector *extractedCloud8 = new CT_PointCloudIndexVector(); // 1) cretation of output pointcloud index
                CT_PointCloudIndexVector *extractedCloud9 = new CT_PointCloudIndexVector(); // 1) cretation of output pointcloud index
                CT_PointCloudIndexVector *extractedCloud10 = new CT_PointCloudIndexVector(); // 1) cretation of output pointcloud index

                pcl::search::KdTree<PointS>::Ptr tree3 (new pcl::search::KdTree<PointS>);
                tree3->setInputCloud(cloud);

                CT_PointIterator itP(pointCloudIndex);
                int ind = 0;
                while (itP.hasNext() && !isStopped())

                {
                    itP.next();
                    size_t index = itP.currentGlobalIndex();

                    PointS original = _cloud_in->points.at(ind);
                    ind++;
                    std::vector<int> pointIdxNKNSearch(1);
                    std::vector<float> pointNKNSquaredDistance(1);



                    if ( tree3->nearestKSearch(original, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
                    {
                        for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
                        {

                            cv::Mat probs(1, 33, CV_64FC1);
                            cv::Vec2d response = source_model->predict2(openCVPointCloud.row(pointIdxNKNSearch[0]), probs);
                            int label = cvRound(response[1]);
                            switch (label) {
                            case 0:
                                extractedCloud1->addIndex(index);
                                break;
                            case 1:
                                extractedCloud2->addIndex(index);
                                break;
                            case 2:
                                extractedCloud3->addIndex(index);
                                break;
                            case 3:
                                extractedCloud4->addIndex(index);
                                break;
                            case 4:
                                extractedCloud5->addIndex(index);
                                break;
                            case 5:
                                extractedCloud6->addIndex(index);
                                break;
                            case 6:
                                extractedCloud7->addIndex(index);
                                break;
                            case 7:
                                extractedCloud8->addIndex(index);
                                break;
                            case 8:
                                extractedCloud9->addIndex(index);
                                break;
                            case 9:
                                extractedCloud10->addIndex(index);
                                break;
                            default:
                                break;
                            }
                        }
                    }
                    counter++;
                }
                if (extractedCloud1->size() > 0)
                {
                    CT_Scene* outScene = new CT_Scene(_outScene_cluster1.completeName(),
                                                      resCpy_res, PS_REPOSITORY->registerPointCloudIndex(extractedCloud1)); // 3) create scene, registering the pointcloudindex
                    outScene->updateBoundingBox(); // 4) don't forget to update the bounding box, to be fitted to filtered points

                    grpCpy_grp->addItemDrawable(outScene);
                }
                if(extractedCloud2->size() > 0)
                {
                    CT_Scene* outScene = new CT_Scene(_outScene_cluster2.completeName(),
                                                      resCpy_res, PS_REPOSITORY->registerPointCloudIndex(extractedCloud2));
                    outScene->updateBoundingBox();
                    grpCpy_grp->addItemDrawable(outScene);
                }
                if (extractedCloud3->size() > 0)
                {
                    CT_Scene* outScene = new CT_Scene(_outScene_cluster3.completeName(),
                                                      resCpy_res, PS_REPOSITORY->registerPointCloudIndex(extractedCloud3)); // 3) create scene, registering the pointcloudindex
                    outScene->updateBoundingBox(); // 4) don't forget to update the bounding box, to be fitted to filtered points

                    grpCpy_grp->addItemDrawable(outScene);
                }
                if(extractedCloud4->size() > 0)
                {
                    CT_Scene* outScene = new CT_Scene(_outScene_cluster4.completeName(),
                                                      resCpy_res, PS_REPOSITORY->registerPointCloudIndex(extractedCloud4));
                    outScene->updateBoundingBox();
                    grpCpy_grp->addItemDrawable(outScene);
                }
                if (extractedCloud5->size() > 0)
                {
                    CT_Scene* outScene = new CT_Scene(_outScene_cluster5.completeName(),
                                                      resCpy_res, PS_REPOSITORY->registerPointCloudIndex(extractedCloud5)); // 3) create scene, registering the pointcloudindex
                    outScene->updateBoundingBox(); // 4) don't forget to update the bounding box, to be fitted to filtered points

                    grpCpy_grp->addItemDrawable(outScene);
                }
                if(extractedCloud6->size() > 0)
                {
                    CT_Scene* outScene = new CT_Scene(_outScene_cluster6.completeName(),
                                                      resCpy_res, PS_REPOSITORY->registerPointCloudIndex(extractedCloud6));
                    outScene->updateBoundingBox();
                    grpCpy_grp->addItemDrawable(outScene);
                }
                if(extractedCloud7->size() > 0)
                {
                    CT_Scene* outScene = new CT_Scene(_outScene_cluster7.completeName(),
                                                      resCpy_res, PS_REPOSITORY->registerPointCloudIndex(extractedCloud7));
                    outScene->updateBoundingBox();
                    grpCpy_grp->addItemDrawable(outScene);
                }
                if(extractedCloud8->size() > 0)
                {
                    CT_Scene* outScene = new CT_Scene(_outScene_cluster8.completeName(),
                                                      resCpy_res, PS_REPOSITORY->registerPointCloudIndex(extractedCloud8));
                    outScene->updateBoundingBox();
                    grpCpy_grp->addItemDrawable(outScene);
                }
                if(extractedCloud9->size() > 0)
                {
                    CT_Scene* outScene = new CT_Scene(_outScene_cluster9.completeName(),
                                                      resCpy_res, PS_REPOSITORY->registerPointCloudIndex(extractedCloud9));
                    outScene->updateBoundingBox();
                    grpCpy_grp->addItemDrawable(outScene);
                }
                if(extractedCloud10->size() > 0)
                {
                    CT_Scene* outScene = new CT_Scene(_outScene_cluster10.completeName(),
                                                      resCpy_res, PS_REPOSITORY->registerPointCloudIndex(extractedCloud10));
                    outScene->updateBoundingBox();
                    grpCpy_grp->addItemDrawable(outScene);
                }
            } else {
                groupsToBeRemoved.push_back(grpCpy_grp);
            }


        } else {
            groupsToBeRemoved.push_back(grpCpy_grp);
        }



    }
    while (!groupsToBeRemoved.isEmpty())
    {
        CT_AbstractItemGroup *group = groupsToBeRemoved.takeLast();
        recursiveRemoveGroupIfEmpty(group->parentGroup(), group);
    }
}















