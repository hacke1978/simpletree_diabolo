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
#include "st_step_eigen_ml.h"

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

//using namespace cv;
//using namespace cv::ml;
//using namespace std;


// Constructor : initialization of parameters
ST_StepEigenML::ST_StepEigenML(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{

}

ST_StepEigenML::~ST_StepEigenML()
{

}

// Step description (tooltip of contextual menu)
QString ST_StepEigenML::getStepDescription() const
{
    return tr("Denoising by Belton et al.() Runtimeoptimized.");
}

// Step detailled description
QString ST_StepEigenML::getStepDetailledDescription() const
{
    return tr("Denoising by Belton et al.() Runtimeoptimized." );
}

// Step URL
QString ST_StepEigenML::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
    // return CT_AbstractStep::getStepURL(); //by default URL of the plugin
}

// Step copy method
CT_VirtualAbstractStep* ST_StepEigenML::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepEigenML(dataInit);
}

//////////////////// PROTECTED METHODS //////////////////

// Creation and affiliation of IN models
void ST_StepEigenML::createInResultModelListProtected()
{



    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("cloud_in"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("grp_in"),  "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);

    resIn_res->addItemModel(DEFin_grp, DEFin_cloud_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Cloud In"));
}

// Creation and affiliation of OUT models
void ST_StepEigenML::createOutResultModelListProtected()
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
void ST_StepEigenML::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();
    configDialog->addInt(tr("number of clusters"), "", 2,10,_number_cluster,"");
    configDialog->addDouble(tr("range for first eigenvalue computation"),"",0.02,0.5,3,_range_1);
    configDialog->addBool(tr("check if second eigenvalue set should be computed"),"","compute",_eigen_2);
    configDialog->addDouble(tr("range for second eigenvalue computation"),"",0.02,0.5,3,_range_2);
    configDialog->addBool(tr("check if third eigenvalue set should be computed"),"","compute",_eigen_3);
    configDialog->addDouble(tr("range for third eigenvalue computation"),"",0.02,0.5,3,_range_3);
    configDialog->addBool(tr("check if forth eigenvalue set should be computed"),"","compute",_eigen_4);
    configDialog->addDouble(tr("range for forth eigenvalue computation"),"",0.02,0.5,3,_range_4);

    configDialog->addEmpty();
    configDialog->addText("Runtime improved frontend to (cite if you use):");
    configDialog->addText("Belton, D.; Moncrieff, S.; Chapman, J. Processing Tree Point Clouds Using Gaussian Mixture");
    configDialog->addText("Models. In Proceedings of the ISPRS annals of the photogrammetry, remote sensing and spatial");
    configDialog->addText("information sciences Antalya, Turkey, 11–13 November 2013; Volume II-5/W2, pp. 43–48.");
    dialog_simple_tree(configDialog);
}

void ST_StepEigenML::compute()
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
            _cloud_in1 = pcl_CT_to_PCL_cloud(itemCpy_cloud_in,this,16,false,true);
            if(_cloud_in1->points.size()!=0)
            {
                {
                    PointCloudS::Ptr downscaled_cloud1(new PointCloudS);
                    downscaled_cloud1 = pcl_voxel_grid_filter(_cloud_in1,this, _range_1/5.0f,true);
                    EnrichCloud (downscaled_cloud1, 15, _range_1, false);
                    pcl_transfer_attribute(downscaled_cloud1,_cloud_in1,PointAtrributeType::EIGENALL);
                }
                if(_eigen_2)
                {
                    _cloud_in2 = pcl_copy_PCL_cloud(_cloud_in1);
                    PointCloudS::Ptr downscaled_cloud2 (new PointCloudS);
                    downscaled_cloud2 = pcl_voxel_grid_filter(_cloud_in2,this,_range_2/5.0f,true);
                    EnrichCloud (downscaled_cloud2, 115, _range_2, false);
                    pcl_transfer_attribute(downscaled_cloud2,_cloud_in2,PointAtrributeType::EIGENALL);
                }
                if(_eigen_3)
                {
                    _cloud_in3 = pcl_copy_PCL_cloud(_cloud_in1);
                    PointCloudS::Ptr downscaled_cloud3 (new PointCloudS);
                    downscaled_cloud3 = pcl_voxel_grid_filter(_cloud_in3,this, _range_3/5.0f,true);
                    EnrichCloud (downscaled_cloud3, 15, _range_3, false);
                    pcl_transfer_attribute(downscaled_cloud3,_cloud_in3,PointAtrributeType::EIGENALL);
                }

                if(_eigen_4)
                {
                    _cloud_in4 = pcl_copy_PCL_cloud(_cloud_in1);
                    PointCloudS::Ptr downscaled_cloud4 (new PointCloudS);
                    downscaled_cloud4 = pcl_voxel_grid_filter(_cloud_in4,this, _range_4/5.0f,true);
                    EnrichCloud (downscaled_cloud4, 15, _range_4, false);
                    pcl_transfer_attribute(downscaled_cloud4,_cloud_in4,PointAtrributeType::EIGENALL);
                }

                int size = 3;
                if(_eigen_2)
                    size+=3;
                if(_eigen_3)
                    size+=3;
                if(_eigen_4)
                    size+=3;
                cv::Mat mat(_cloud_in1->points.size(), size,  CV_32FC(1));

                for(size_t i = 0; i< _cloud_in1->points.size(); i++)
                {

                    int pos = 0;
                    {
                        PointS p1 = _cloud_in1->points.at(i);
                        mat.at<float>(i,pos) = p1.eigen1;
                        pos++;
                        mat.at<float>(i,pos) = p1.eigen2;
                        pos++;
                        mat.at<float>(i,pos) = p1.eigen3;
                        pos++;
                    }

                    if(_eigen_2)
                    {
                        PointS p2 = _cloud_in2->points.at(i);
                        mat.at<float>(i,pos) = p2.eigen1;
                        pos++;
                        mat.at<float>(i,pos) = p2.eigen2;
                        pos++;
                        mat.at<float>(i,pos) = p2.eigen3;
                        pos++;
                    }
                    if(_eigen_3)
                    {
                        PointS p3 = _cloud_in3->points.at(i);
                        mat.at<float>(i,pos) = p3.eigen1;
                        pos++;
                        mat.at<float>(i,pos) = p3.eigen2;
                        pos++;
                        mat.at<float>(i,pos) = p3.eigen3;
                        pos++;
                    }
                    if(_eigen_4)
                    {
                        PointS p4 = _cloud_in4->points.at(i);
                        mat.at<float>(i,pos) = p4.eigen1;
                        pos++;
                        mat.at<float>(i,pos) = p4.eigen2;
                        pos++;
                        mat.at<float>(i,pos) = p4.eigen3;
                        pos++;
                    }

                }

                cv::Ptr<cv::ml::EM> source_model = cv::ml::EM::create();
                source_model->setClustersNumber(_number_cluster);
                cv::Mat logs;
                cv::Mat labels;
                cv::Mat probs;
                source_model->trainEM(mat,logs,labels,probs);
                //        source_model->save("D:/gmm_feature.xml");

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

                CT_PointIterator itP(pointCloudIndex);
                cv::MatIterator_<int> it(labels.begin<int>());
                size_t counter = 0;
                while (itP.hasNext() && !isStopped())
                {
                    int label = *it;
                    itP.next();
                    size_t index = itP.currentGlobalIndex();
                    //int label = labels.at<int>(counter);
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
                    counter++;
                    it++;
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
        }else {
            groupsToBeRemoved.push_back(grpCpy_grp);
        }
    }


    _cloud_in1.reset(new PointCloudS);
    _cloud_in2.reset(new PointCloudS);
    _cloud_in3.reset(new PointCloudS);
    _cloud_in4.reset(new PointCloudS);
    while (!groupsToBeRemoved.isEmpty())
    {
        CT_AbstractItemGroup *group = groupsToBeRemoved.takeLast();
        recursiveRemoveGroupIfEmpty(group->parentGroup(), group);
    }
}















